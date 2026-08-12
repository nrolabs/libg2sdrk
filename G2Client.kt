/*
 * libg2sdrk - Kotlin driver for the ANAN-G2 (Saturn) SDR transceiver
 *
 * Network transport (UDP), streaming and control lifecycle for openHPSDR
 * Protocol 2. The wire format itself lives in the Android-free [G2Protocol]
 * codec, ported byte-for-byte from the reference client (Thetis,
 * ChannelMaster/network.c) and the openHPSDR Ethernet Protocol specification.
 *
 * Kotlin port: Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

package com.isaklab.libg2sdrk
import com.isaklab.isdrdrivers.core.TxDriveCapable
import com.isaklab.isdrdrivers.core.TransmitCapable
import com.isaklab.isdrdrivers.core.RadioClient

import android.util.Log
import com.isaklab.isdrdrivers.core.DspThread
import com.isaklab.isdrdrivers.core.FFTProcessor
import com.isaklab.isdrdrivers.core.SpectrumWorker
import com.isaklab.isdrdrivers.core.FloatRing
import com.isaklab.isdrdrivers.core.SeqTracker
import kotlinx.coroutines.*
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketTimeoutException

/**
 * Full RX + TX driver for an ANAN-G2 / Saturn over the network.
 *
 * RX: receives DDC IQ frames (radio source ports 1035+), decodes 24-bit
 * interleaved IQ into a `FloatArray` (`i0,q0,…` in `[-1,1]`), accumulates
 * display-sized blocks, computes the power spectrum, and hands both to
 * [onDataReceived] — the same contract as the other clients so the app can
 * drive any device.
 *
 * TX: while [setPtt] is on, queued 48 kSps modulator IQ (fed via
 * [submitTxIq]) is polyphase-interpolated to the fixed 192 kSps DUC rate and
 * streamed to port 1029 at 800 packets/s, wall-clock paced. Control changes
 * re-send the high-priority packet; PA changes re-send the General packet.
 */
class G2Client(
    /** Radio IP, or "255.255.255.255" (default) to discover on the LAN. */
    private val host: String = BROADCAST,
    /** (power spectrum in dB, interleaved IQ i0,q0,… in [-1,1]) */
    private val onDataReceived: (FloatArray, FloatArray) -> Unit,
    private val onConnectionStatusChanged: (Boolean, String) -> Unit,
    private val onStatus: ((G2Protocol.Status) -> Unit)? = null,
    /**
     * Per-receiver IQ block (rx index, interleaved IQ), for receivers armed
     * via [setRxStreamMask]. Fired before [onDataReceived] of the same flush
     * over the same sample span (blocks are length-matched to the active
     * block), so the consumer pairs each EV_DATA_RX with the following
     * EV_DATA — the contract diversity and PureSignal ride on the HL2.
     */
    private val onDataRx: ((Int, FloatArray) -> Unit)? = null,
    /**
     * Added to every P2 UDP port (commands out AND stream source ports in).
     * 0 on real hardware; a bench harness points it at an emulator running
     * with the matching `--port-offset` so the real 1024+ range stays free.
     */
    private val portOffset: Int = 0,
) : RadioClient, TransmitCapable, TxDriveCapable {
    companion object {
        const val BROADCAST = "255.255.255.255"
        private const val TAG = "G2Client"
        /** DDC streams the Saturn hardware emits (ports 1035..1041). */
        private const val MAX_RECEIVERS = G2Protocol.MAX_DDC
        private val EMPTY_SPECTRUM = FloatArray(0)
        private const val PAIRS_PER_TX_PACKET = G2Protocol.TX_SAMPLES_PER_PACKET / 4  // 60 input pairs
    }

    /** When false, RX blocks skip the FFT and deliver an empty spectrum. */
    @Volatile override var spectrumEnabled: Boolean = true

    /**
     * Narrow the panadapter's span before the transform; returns the
     * decimation actually in force. With no session there is nothing to hold
     * it, and the answer is an honest 1 rather than the request echoed back.
     */
    fun setSpectrumZoom(decimation: Int, offsetHz: Long): Int =
        spectrumWorker?.setZoom(decimation, offsetHz.toDouble(), getSampleRate()) ?: 1


    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    // Hot loops on threads of their own (see DspThread): a shared coroutine
    // pool leaked their audio priority and hopped them between workers on
    // every paced beat.
    private var receiveThread: Thread? = null
    private var txSenderThread: Thread? = null
    /** One-shot latch: the teardown of one connection runs exactly once,
     *  whichever of the operator path, the timeout exit and the failure path
     *  reaches it first. Cleared by [connect]. */
    private val teardownDone = java.util.concurrent.atomic.AtomicBoolean(false)
    private var keepaliveJob: Job? = null
    private var socket: DatagramSocket? = null
    private var radio: InetAddress? = null
    @Volatile private var running = false

    private var fft: FFTProcessor? = null
    // Welch FFT off the receive thread: an inline FFT stalls socket reads and
    // drops DDC packets. The receive thread only submits on the display cadence
    // and delivers the last cached spectrum — audio never waits on the FFT.
    private var spectrumWorker: SpectrumWorker? = null

    private val stateLock = Any()
    private val state = G2Protocol.ControlState()
    private var txSeq = 0L

    // RX accumulation (delivered as display-sized blocks).
    private var lastFftTimeMs = 0L
    private val fftIntervalMs = 80L
    private var accum = FloatArray(0)
    private var accumPairs = 0
    private var flushPairs = 800

    // TX sample queue: interleaved I/Q in [-1,1] at 48 kSps, drop-oldest.
    // Primitive ring — ArrayDeque<Float> boxed every sample (~100k boxes/s
    // while keyed) and the resulting GC churn is exactly what the RX sinks
    // were rebuilt to avoid.
    private val txQueue = FloatRing(48_000 * 4)
    private val txLock = Any()
    private val interpolator = TxInterpolator()
    private val txPacketIq = FloatArray(G2Protocol.TX_SAMPLES_PER_PACKET * 2)

    // Which DDC feeds the spectrum/audio when several are configured.
    @Volatile private var activeReceiver = 0

    // Extra per-DDC streams (bit n = also deliver DDC n's IQ via onDataRx).
    @Volatile private var rxStreamMask = 0
    private val accumRx = arrayOfNulls<FloatArray>(MAX_RECEIVERS)
    private val accumRxPairs = IntArray(MAX_RECEIVERS)

    // DDC packet-sequence continuity, one tracker PER STREAM (each DDC source
    // port owns an independent sequence space). A gap is concealed with zeros
    // of the estimated hole size so downstream AGC/smoothing sees a dropout,
    // not a time-warp; a late reordered packet is dropped (its samples would
    // land out of order in the accumulator).
    private val ddcSeqAll = Array(MAX_RECEIVERS) { SeqTracker() }
    private val ddcSeq get() = ddcSeqAll[activeReceiver.coerceIn(0, MAX_RECEIVERS - 1)]

    /** RX discontinuity events (loss/reorder) since connect — telemetry. */
    val rxGapCount: Long get() = ddcSeqAll.sumOf { it.gapEvents }

    // Reused sinks (primitive params) so DDC IQ decode allocates nothing per
    // sample — a boxing lambda here caused a GC storm that popped the audio.
    private val ddcSink = G2Protocol.IqSink { i, q -> appendSample(i, q) }
    private val ddcSinksRx = Array(MAX_RECEIVERS) { rx ->
        G2Protocol.IqSink { i, q -> appendSampleRx(rx, i, q) }
    }

    // ========================================================================
    // Connection lifecycle
    // ========================================================================

    /**
     * Discovers the Saturn radio on the LAN (if broadcast), binds the UDP socket,
     * and sets up the Protocol-2 streaming topology. Allocates high-priority processing
     * threads for stream ingest and transmit pacing.
     *
     * @return true if successfully connected and initialized, false otherwise.
     */
    override suspend fun connect(): Boolean = withContext(Dispatchers.IO) {
        try {
            onConnectionStatusChanged(false, "Discovering…")
            // Arm the teardown latch before anything is allocated: the catch
            // below unwinds through teardown, and it must not be no-opped by
            // the latch the PREVIOUS session left set.
            teardownDone.set(false)
            val s = DatagramSocket()
            s.soTimeout = 1000
            s.broadcast = true
            // A 1444-byte packet arrives every ~1.5 ms per DDC; the platform
            // default (often 64–256 KiB) rides through only ~100 ms of GC or
            // scheduler stall. Ask for 2 MiB (kernel may clamp).
            try { s.receiveBufferSize = 2 * 1024 * 1024 } catch (_: Exception) {}
            socket = s
            radio = if (host == BROADCAST) discover(s) else InetAddress.getByName(host)
            if (radio == null) {
                onConnectionStatusChanged(false, "No Protocol-2 radio found")
                socket = null
                s.close()
                return@withContext false
            }
            fft = FFTProcessor(800)
            spectrumWorker = SpectrumWorker(fft!!)
            updateFlushThreshold()
            synchronized(stateLock) { state.run = true }
            sendStartSequence()
            running = true
            onConnectionStatusChanged(true, "Connected")
            startReceiving()
            startTxSender()
            startKeepalive()
            true
        } catch (e: Exception) {
            Log.e(TAG, "connect failed: ${e.message}")
            onConnectionStatusChanged(false, "Connection failed")
            disconnect()
            false
        }
    }

    /**
     * Releases the UDP socket and safely terminates the RX, TX, and keepalive background
     * coroutines/threads. The radio is commanded to unkey and stop sending DDC streams.
     */
    override fun disconnect() {
        scope.launch { teardown("Disconnected") }
    }

    /**
     * Unkeys the radio, stops the streams and releases the socket and the loop
     * threads, exactly once per connection.
     *
     * Shared by [disconnect], the receive loop's timeout/error exits and
     * [failLink], so every way out of a session ends with the radio told to
     * stop and the host told the link is gone. The unkey is first and on the
     * wire: [state].mox only lives in memory until the TX pacer sends it, and
     * on an abrupt exit the pacer may never get another turn — which would
     * leave the radio holding the MOX of its last received packet.
     *
     * Never joins a loop thread from that same thread (the failure and
     * timeout exits run on one of them).
     */
    private fun teardown(statusMessage: String) {
        if (!teardownDone.compareAndSet(false, true)) return
        running = false
        val self = Thread.currentThread()
        try {
            synchronized(stateLock) { state.run = false; state.mox = false }
            sendHighPriority()
        } catch (_: Exception) {}
        keepaliveJob?.cancel()
        // The socket is what unblocks the receive loop; close it first.
        socket?.close()
        receiveThread?.takeIf { it !== self }?.let { DspThread.stop(it) }
        txSenderThread?.takeIf { it !== self }?.let { DspThread.stop(it) }
        receiveThread = null
        txSenderThread = null
        spectrumWorker?.stop()
        socket = null
        onConnectionStatusChanged(false, statusMessage)
    }

    /**
     * Retire the connection after one of the loop threads died on an
     * unhandled throwable. DspThread has already logged it at ERROR with the
     * stack; this makes the death VISIBLE instead of leaving a dead loop
     * behind a live-looking connection.
     *
     * The transmit pacer is the dangerous one: it is the only sender of TX IQ,
     * so if it dies while keyed the host still shows "on air", the operator
     * keeps talking, and no RF is produced — [teardown] unkeys on the wire and
     * reports the failure. A throwable raised while a teardown is already
     * running is that teardown closing the socket under a send, and the latch
     * makes this a no-op.
     */
    private fun failLink() {
        teardown(if (running) "Error" else "Disconnected")
    }

    private fun discover(s: DatagramSocket): InetAddress? {
        val req = G2Protocol.discoveryRequest()
        val reply = ByteArray(256)
        repeat(4) {
            try {
                s.send(DatagramPacket(req, req.size, InetAddress.getByName(BROADCAST), G2Protocol.GENERAL_PORT + portOffset))
                val p = DatagramPacket(reply, reply.size)
                s.receive(p)
                val info = G2Protocol.parseDiscoveryReply(reply, p.length)
                if (info != null && !info.busy) {
                    Log.i(TAG, "found ${G2Protocol.Board.name(info.boardId)} at " +
                        "${p.address.hostAddress} mac=${info.mac} ddcs=${info.ddcCount}")
                    return p.address
                }
            } catch (_: SocketTimeoutException) { /* retry */ }
        }
        return null
    }

    /** General + RX-specific + TX-specific + high-priority, the P2 start order. */
    private fun sendStartSequence() {
        sendGeneral()
        sendRxSpecific()
        sendTxSpecific()
        sendHighPriority()
    }

    // ========================================================================
    // RX
    // ========================================================================

    private fun startReceiving() {
        // Audio priority: block delivery must not lose CPU to rendering.
        receiveThread = DspThread.start("g2-rx", DspThread.PRIORITY_RADIO, onFailure = { failLink() }) {
            spectrumWorker?.start()
            val buf = ByteArray(2048)
            val packet = DatagramPacket(buf, buf.size)
            var noData = 0
            while (running) {
                try {
                    socket!!.receive(packet)
                    noData = 0
                    when {
                        packet.port == G2Protocol.FROM_STATUS_PORT + portOffset &&
                            packet.length == G2Protocol.STATUS_LEN -> {
                            G2Protocol.parseStatus(buf, packet.length)?.let { onStatus?.invoke(it) }
                        }
                        G2Protocol.ddcIndexOf(packet.port - portOffset) >= 0 -> {
                            val rx = G2Protocol.ddcIndexOf(packet.port - portOffset)
                            if (packet.length == G2Protocol.BUFLEN) {
                                if (rx == activeReceiver) {
                                    // Sequence check BEFORE decoding: a stale
                                    // reordered packet must not be appended.
                                    when (val missing = ddcSeq.advance(G2Protocol.be32(buf, 0))) {
                                        -1L -> {}                   // late dup: drop
                                        0L -> G2Protocol.parseDdcIq(buf, packet.length, ddcSink)
                                        else -> {
                                            concealGap(missing * G2Protocol.IQ_SAMPLES_PER_PACKET)
                                            G2Protocol.parseDdcIq(buf, packet.length, ddcSink)
                                        }
                                    }
                                } else if ((rxStreamMask shr rx) and 1 == 1) {
                                    // Secondary DDC armed via the stream mask:
                                    // same sequence discipline, own accumulator.
                                    when (val missing = ddcSeqAll[rx].advance(G2Protocol.be32(buf, 0))) {
                                        -1L -> {}                   // late dup: drop
                                        0L -> G2Protocol.parseDdcIq(buf, packet.length, ddcSinksRx[rx])
                                        else -> {
                                            concealGapRx(rx, missing * G2Protocol.IQ_SAMPLES_PER_PACKET)
                                            G2Protocol.parseDdcIq(buf, packet.length, ddcSinksRx[rx])
                                        }
                                    }
                                }
                                // Unarmed non-active DDCs are dropped.
                            }
                            if (accumPairs >= flushPairs) flushRx()
                        }
                        // Mic samples: ignored (phone mic is used).
                    }
                } catch (e: SocketTimeoutException) {
                    if (++noData > 5) {
                        // Re-assert the start sequence — a lost run bit or a
                        // radio power-cycle both recover from this.
                        try { sendStartSequence() } catch (_: Exception) {}
                        if (noData > 15) {
                            // Full teardown, not just running=false: the radio
                            // has to be told to stop streaming and the socket
                            // and worker threads have to go, or they survive
                            // until the next connect.
                            teardown("Timeout")
                        }
                    }
                } catch (e: Exception) {
                    if (running) {
                        Log.e(TAG, "rx error: ${e.message}")
                        teardown("Error")
                    }
                }
            }
        }
    }

    /**
     * Insert [pairs] zero IQ pairs (capped at one display block) for a lost-
     * packet hole so the audio path gets a dropout of roughly the right
     * length instead of a seam, then restart the spectrum smoothing — the
     * IIR average must not blend across the discontinuity.
     */
    private fun concealGap(pairs: Long) {
        val n = pairs.coerceAtMost(flushPairs.toLong()).toInt()
        repeat(n) { appendSample(0f, 0f) }
        spectrumWorker?.resetSmoothing()
        Log.w(TAG, "ddc gap: $pairs pairs lost (events=${ddcSeq.gapEvents})")
    }

    /** Zero-conceal a hole on a SECONDARY armed stream (its own accumulator). */
    private fun concealGapRx(rx: Int, pairs: Long) {
        val n = pairs.coerceAtMost(flushPairs.toLong()).toInt()
        repeat(n) { appendSampleRx(rx, 0f, 0f) }
        Log.w(TAG, "ddc$rx gap: $pairs pairs lost (events=${ddcSeqAll[rx].gapEvents})")
    }

    private fun appendSample(i: Float, q: Float) {
        val need = (accumPairs + 1) * 2
        if (accum.size < need) accum = accum.copyOf(maxOf(need, flushPairs * 2))
        accum[accumPairs * 2] = i
        accum[accumPairs * 2 + 1] = q
        accumPairs++
    }

    private fun appendSampleRx(rx: Int, i: Float, q: Float) {
        if (rx !in 0 until MAX_RECEIVERS) return
        var a = accumRx[rx]
        val need = (accumRxPairs[rx] + 1) * 2
        if (a == null || a.size < need) {
            a = (a ?: FloatArray(0)).copyOf(maxOf(need, flushPairs * 2 + 2 * G2Protocol.IQ_SAMPLES_PER_PACKET))
            accumRx[rx] = a
        }
        a[accumRxPairs[rx] * 2] = i
        a[accumRxPairs[rx] * 2 + 1] = q
        accumRxPairs[rx]++
    }

    private fun flushRx() {
        val blockPairs = accumPairs
        // Armed per-DDC streams first: unlike the HL2 (one frame carries every
        // receiver), each G2 DDC is an independent UDP stream, so counts can
        // straddle a packet boundary. Emit EXACTLY the active block's span —
        // zero-pad a short stream, carry a long stream's excess to the next
        // flush — so every EV_DATA_RX pairs 1:1, length-matched, with the
        // EV_DATA that follows (diversity / PureSignal contract).
        if (rxStreamMask != 0 && onDataRx != null && blockPairs > 0) {
            for (rx in 0 until MAX_RECEIVERS) {
                if (rx == activeReceiver || (rxStreamMask shr rx) and 1 == 0) continue
                val have = accumRxPairs[rx]
                if (have == 0) continue          // stream idle: nothing to pair
                val a = accumRx[rx]!!
                val out = FloatArray(blockPairs * 2)
                val take = minOf(have, blockPairs)
                System.arraycopy(a, 0, out, 0, take * 2)
                val left = have - take
                if (left > 0) System.arraycopy(a, take * 2, a, 0, left * 2)
                accumRxPairs[rx] = left
                onDataRx.invoke(rx, out)
            }
        }
        val block = accum.copyOf(accumPairs * 2)
        accumPairs = 0
        if (!spectrumEnabled) {
            onDataReceived(EMPTY_SPECTRUM, block)
            return
        }
        // The display renders ~10 fps: on that cadence hand a copy of the block
        // to the off-thread worker (submit copies internally); the FFT never
        // runs on this receive thread. Deliver the IQ NOW with the last cached
        // spectrum — audio must never wait on the FFT (spectrum may lag one
        // frame, imperceptible at 10 fps).
        val now = System.currentTimeMillis()
        if (now - lastFftTimeMs >= fftIntervalMs) {
            lastFftTimeMs = now
            spectrumWorker?.submit(block, block.size / 2)
        }
        onDataReceived(spectrumWorker?.latest ?: EMPTY_SPECTRUM, block)
    }

    // ========================================================================
    // TX
    // ========================================================================

    /**
     * Streams TX IQ at exactly 800 packets/s while keyed (192 kSps / 240
     * samples). Wall-clock paced with a burst cap; on mic underrun the clock
     * is rebased instead of blasting a stale catch-up burst, which would put
     * a block of already-stale audio on the air and then starve the DUC again
     * on the next beat.
     */
    private fun startTxSender() {
        txSenderThread = DspThread.start("g2-tx", DspThread.PRIORITY_RADIO, onFailure = { failLink() }) {
            var txStartNs = 0L
            var packetsSent = 0L
            while (running) {
                val transmitting = synchronized(stateLock) { state.mox }
                if (!transmitting) {
                    // Idle: nothing is sent from here (the keepalive owns the
                    // watchdog), so there is no reason to wake 333 times a
                    // second just to look at a flag.
                    txStartNs = 0L
                    DspThread.pace(System.nanoTime() + 10_000_000L)
                    continue
                }
                val now = System.nanoTime()
                if (txStartNs == 0L) { txStartNs = now; packetsSent = 0L; interpolator.reset() }
                val targetPackets = (now - txStartNs) * 800L / 1_000_000_000L
                if (targetPackets - packetsSent > 32) {
                    txStartNs = now
                    packetsSent = 0L
                } else {
                    var burst = 0
                    while (packetsSent < targetPackets && burst < 16) {
                        if (!sendOneTxPacket()) break
                        packetsSent++
                        burst++
                    }
                }
                DspThread.pace(System.nanoTime() + 500_000L)
            }
        }
    }

    /** Interpolate 60 queued 48 k pairs into one 240-sample packet and send. */
    private fun sendOneTxPacket(): Boolean {
        val s = socket ?: return false
        val dst = radio ?: return false
        synchronized(txLock) {
            if (txQueue.size < PAIRS_PER_TX_PACKET * 2) return false
            var o = 0
            repeat(PAIRS_PER_TX_PACKET) {
                val i = txQueue.read()
                val q = txQueue.read()
                o = interpolator.process(i, q, txPacketIq, o)
            }
        }
        val p = G2Protocol.txIqPacket(txSeq++, txPacketIq, 0)
        s.send(DatagramPacket(p, p.size, dst, G2Protocol.TX_IQ_PORT + portOffset))
        return true
    }

    // ========================================================================
    // Command senders / keepalive
    // ========================================================================

    private fun sendTo(port: Int, payload: ByteArray) {
        val s = socket ?: return
        val dst = radio ?: return
        s.send(DatagramPacket(payload, payload.size, dst, port + portOffset))
    }

    private fun sendGeneral() =
        sendTo(G2Protocol.GENERAL_PORT, synchronized(stateLock) { G2Protocol.generalPacket(state) })

    private fun sendRxSpecific() =
        sendTo(G2Protocol.RX_SPECIFIC_PORT, synchronized(stateLock) { G2Protocol.rxSpecificPacket(state) })

    private fun sendTxSpecific() =
        sendTo(G2Protocol.TX_SPECIFIC_PORT, synchronized(stateLock) { G2Protocol.txSpecificPacket(state) })

    private fun sendHighPriority() =
        sendTo(G2Protocol.HIGH_PRIORITY_PORT, synchronized(stateLock) { G2Protocol.highPriorityPacket(state) })

    /**
     * State refresh loop: the high-priority packet every second (UDP loss
     * insurance) and, when the watchdog is armed, the General packet every
     * 500 ms — with it armed the radio stops unless petted on that interval.
     */
    private fun startKeepalive() {
        keepaliveJob = scope.launch {
            var tick = 0
            while (running && isActive) {
                delay(500)
                tick++
                try {
                    if (synchronized(stateLock) { state.watchdogEnabled }) sendGeneral()
                    if (tick % 2 == 0) sendHighPriority()
                } catch (_: Exception) {}
            }
        }
    }

    // ========================================================================
    // Public control API (mirrors the sibling clients)
    // ========================================================================

    /**
     * Tunes the primary DDC (DDC0) to the specified frequency in Hz.
     * Enqueues a high-priority command and resets the FFT smoothing filter.
     */
    override fun setFrequency(hz: Long) {
        synchronized(stateLock) { state.ddcFreqHz[0] = hz }
        sendHighPriority()
        spectrumWorker?.resetSmoothing()
    }

    fun setFrequency2(hz: Long) = setRxFrequency(1, hz)

    /** Set any DDC's NCO frequency (0..6) — high-priority phase word 9+4n. */
    fun setRxFrequency(index: Int, hz: Long) {
        if (index !in 0 until MAX_RECEIVERS) return
        synchronized(stateLock) { state.ddcFreqHz[index] = hz }
        sendHighPriority()
        if (index == activeReceiver) spectrumWorker?.resetSmoothing()
    }

    /**
     * Sets the sampling rate for the active DDCs. Supported Protocol-2 rates are 
     * 48, 96, 192, 384, 768, and 1536 kHz. Invalid values fallback to 48 kHz.
     */
    override fun setSampleRate(hz: Int) {
        synchronized(stateLock) {
            state.sampleRate = if (hz in G2Protocol.SAMPLE_RATES) hz else 48_000
        }
        sendRxSpecific()
        updateFlushThreshold()
        spectrumWorker?.resetSmoothing()
    }

    /**
     * Configures the number of parallel DDCs to stream from the radio (up to 7).
     * The hardware automatically multiplexes the streams to ports 1035+.
     */
    fun setReceiverCount(n: Int) {
        synchronized(stateLock) { state.receiverCount = n.coerceIn(1, MAX_RECEIVERS) }
        sendRxSpecific()
        sendHighPriority()                 // push the new DDCs' NCO words too
    }

    fun setActiveReceiver(index: Int) {
        activeReceiver = index.coerceIn(0, MAX_RECEIVERS - 1)
        ddcSeq.reset()                     // new stream, new sequence space
        spectrumWorker?.resetSmoothing()
        // PureSignal's feedback DDC is derived from the active receiver —
        // keep the DDC input routing in step.
        if (synchronized(stateLock) { state.pureSignal }) updatePsRouting()
    }

    /** Bit n = also stream DDC n's IQ via the onDataRx callback. */
    fun setRxStreamMask(mask: Int) {
        val old = rxStreamMask
        rxStreamMask = mask and ((1 shl MAX_RECEIVERS) - 1)
        if (rxStreamMask == 0) accumRxPairs.fill(0)
        // Newly armed streams start a fresh sequence space.
        for (rx in 0 until MAX_RECEIVERS) {
            if ((rxStreamMask shr rx) and 1 == 1 && (old shr rx) and 1 == 0) {
                ddcSeqAll[rx].reset()
                accumRxPairs[rx] = 0
            }
        }
    }

    /**
     * PureSignal TX-feedback routing: while on, the reference DDC (the pair
     * partner of the active receiver, same derivation as the app/HL2 —
     * active==1 ⇒ DDC0, else DDC1) is fed from the TX/DUC loopback instead of
     * the ADC, so the app can collect (feedback, reference) pairs while keyed.
     */
    fun setPureSignal(on: Boolean) {
        synchronized(stateLock) { state.pureSignal = on }
        updatePsRouting()
    }

    private fun updatePsRouting() {
        synchronized(stateLock) {
            state.psFeedbackDdc =
                if (state.pureSignal) (if (activeReceiver == 1) 0 else 1) else -1
        }
        sendRxSpecific()
    }

    fun getActiveReceiver(): Int = activeReceiver

    fun getSampleRate(): Double = synchronized(stateLock) { state.sampleRate.toDouble() }

    fun setSmoothingFactor(alpha: Float) { fft?.setSmoothingFactor(alpha) }

    override fun setTxFrequency(hz: Long) {
        synchronized(stateLock) { state.txFreqHz = hz }
        sendHighPriority()
    }

    override fun setPtt(on: Boolean) {
        synchronized(stateLock) { state.mox = on }
        if (!on) {
            synchronized(txLock) { txQueue.clear() }
            // Unkey: the spectrum jumps from TX leakage back to band noise —
            // restart the smoothing IIR instead of cross-fading the ghost.
            spectrumWorker?.resetSmoothing()
        }
        sendHighPriority()
    }

    override fun setTxDrive(level: Int) {
        synchronized(stateLock) { state.txDrive = level.coerceIn(0, 255) }
        sendHighPriority()
    }

    override fun setPaEnabled(on: Boolean) {
        synchronized(stateLock) { state.paEnabled = on }
        sendGeneral()                      // PA enable lives in the General packet
    }

    /** RX step attenuator, 0–31 dB (both ADCs mirrored). */
    fun setStepAttenuator(db: Int) {
        synchronized(stateLock) { state.stepAttenDb = db.coerceIn(0, 31) }
        sendHighPriority()
    }

    /** The 7 open-collector outputs (external band data / linear switching). */
    fun setOpenCollectorOutputs(mask: Int) {
        synchronized(stateLock) { state.ocOutputs = mask and 0x7F }
        sendHighPriority()
    }

    override fun isTransmitting(): Boolean = synchronized(stateLock) { state.mox }

    /** Queue interleaved transmit IQ (`i0,q0,…` in `[-1,1]`, 48 kSps). Drop-oldest. */
    override fun submitTxIq(iq: FloatArray) {
        synchronized(txLock) { txQueue.write(iq) }   // ring drops oldest itself
    }

    private fun updateFlushThreshold() {
        flushPairs = maxOf(800, synchronized(stateLock) { state.sampleRate } / 50)
    }
}
