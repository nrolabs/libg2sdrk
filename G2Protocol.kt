/*
 * libg2sdrk - ANAN-G2 (Saturn) driver, openHPSDR "Protocol 2" codec.
 *
 * Pure Kotlin, free of any Android dependency, so the wire format can be
 * unit-tested on the JVM and reused outside the app. Ported byte-for-byte
 * from the reference client (Thetis, ChannelMaster/network.c) and the
 * openHPSDR Ethernet Protocol specification:
 *   https://github.com/TAPR/OpenHPSDR-Firmware/tree/master/Protocol%202
 *   https://github.com/ramdor/Thetis
 *
 * Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>. GPL v2 or later.
 */

package com.isaklab.libg2sdrk

/**
 * Frame codec for openHPSDR Protocol 2 radios (ANAN-G2 / Saturn family).
 * Builds host→radio command/stream packets and parses radio→host status and
 * DDC IQ. Holds no state; the caller owns the sockets and sequence counters.
 *
 * Port model: commands go to the radio at 1024 (general), 1025 (RX specific),
 * 1026 (TX specific), 1027 (high priority), 1028 (RX audio), 1029 (TX IQ).
 * The radio answers from source ports 1025 (high-priority status), 1026 (mic)
 * and 1035..1041 (DDC0..6 IQ) — received streams are keyed on the SOURCE port.
 * All multi-byte fields are big-endian.
 */
object G2Protocol {

    const val GENERAL_PORT = 1024
    const val RX_SPECIFIC_PORT = 1025
    const val TX_SPECIFIC_PORT = 1026
    const val HIGH_PRIORITY_PORT = 1027
    const val RX_AUDIO_PORT = 1028
    const val TX_IQ_PORT = 1029

    /** Radio-side source ports of inbound streams. */
    const val FROM_STATUS_PORT = 1025
    const val FROM_MIC_PORT = 1026
    const val FROM_DDC0_PORT = 1035

    /** DDC streams a Saturn actually emits (source ports 1035..1041). */
    const val MAX_DDC = 7

    /**
     * Per-DDC input selector (DDC-specific packet, byte 17+6n) — p2app
     * saturnregisters.c SetDDCADC: 0 = ADC1, 1 = ADC2, 2 = TX/DUC feedback
     * (the DAC loopback PureSignal calibrates against).
     */
    const val DDC_INPUT_ADC0 = 0
    const val DDC_INPUT_ADC1 = 1
    const val DDC_INPUT_TX_FEEDBACK = 2

    const val BUFLEN = 1444
    const val STATUS_LEN = 60
    const val MIC_LEN = 132
    const val DDC_HEADER = 16
    const val IQ_SAMPLES_PER_PACKET = 238      // 1428 payload bytes / 6
    const val TX_SAMPLES_PER_PACKET = 240      // 4 + 240*6 = 1444 bytes
    const val TX_RATE = 192_000                // DUC input rate, fixed in P2

    /** DDC sample rates the protocol accepts (kHz value goes on the wire). */
    val SAMPLE_RATES = listOf(48_000, 96_000, 192_000, 384_000, 768_000, 1_536_000)

    /**
     * Numeric identities from the shared HPSDR discovery namespace
     * (Thetis enums.cs HPSDRHW). These are discovery bytes, never app
     * DeviceType/product selections; names such as HERMES only explain a
     * reply that [G2Client] must reject as non-Protocol-2.
     */
    object DiscoveryBoardId {
        const val ATLAS = 0
        const val HERMES = 1
        const val HERMES2 = 2
        const val ANGELIA = 3
        const val ORION = 4
        const val ORION2 = 5          // ANAN-7000DLE / 8000DLE
        const val HERMES_LITE = 6
        const val SATURN = 10         // ANAN-G2
        const val SATURN_MK2 = 11     // ANAN-G2 MkII
        const val G2E = 20            // ANAN-G2E (HermesC10)

        fun name(id: Int): String = when (id) {
            ATLAS -> "Atlas"; HERMES -> "Hermes"; HERMES2 -> "Hermes II"
            ANGELIA -> "Angelia"; ORION -> "Orion"; ORION2 -> "Orion MkII"
            HERMES_LITE -> "Hermes-Lite"; SATURN -> "Saturn (ANAN-G2)"
            SATURN_MK2 -> "Saturn MkII (ANAN-G2)"; G2E -> "ANAN-G2E"
            else -> "unknown ($id)"
        }

        /**
         * True only for products whose firmware implements the Saturn/G2
         * Protocol-2 control and stream contract used by [G2Client].
         *
         * Hermes, Hermes II and Hermes-Lite IDs occur in the wider HPSDR
         * discovery namespace, but that does not make those radios G2s.  In
         * particular, accepting every syntactically valid discovery reply
         * here would open a Protocol-1 product with Protocol-2 commands.
         * ORION2 is retained because Saturn firmware reports ID 5 on the
         * documented compatibility path.
         */
        fun isG2Product(id: Int): Boolean = id == ORION2 || id == SATURN ||
            id == SATURN_MK2 || id == G2E
    }

    /** Programmable radio state serialized into the command packets. */
    class ControlState {
        var run = false
        var mox = false
        /** Per-DDC NCO frequencies, Hz (high-priority phase words 9+4n). */
        val ddcFreqHz = LongArray(MAX_DDC) { 7_100_000L }
        /** DDC0 NCO — compatibility view over [ddcFreqHz]. */
        var rxFreqHz: Long
            get() = ddcFreqHz[0]
            set(v) { ddcFreqHz[0] = v }
        /** DDC1 NCO — compatibility view over [ddcFreqHz]. */
        var rx2FreqHz: Long
            get() = ddcFreqHz[1]
            set(v) { ddcFreqHz[1] = v }
        var txFreqHz = 7_100_000L
        // Antenna selection, Alex word bits 8..10 (1-based antenna number).
        var txAntenna = 1
        var rxAntenna = 1
        // Step attenuators DURING TX (DUC bytes 58/59) — the reference keys
        // the feedback ADC level for PureSignal with these.
        var txAttenDb = 0
        // CWX (host-keyed CW): high-priority byte 5 bits 0..2.
        var cwxEnabled = false
        var cwxDot = false
        var cwxDash = false
        // ADC conditioning (DDC-specific bytes 5/6): bit0 ADC1, bit1 ADC2.
        var adcDither = 0
        var adcRandom = 0
        // Synchronised (interleaved) DDC pair 0+1 — coherent diversity.
        // Route and sync are separate so the client can put DDC1 on ADC2,
        // frequency-lock the NCOs, and only then announce synchronisation.
        var diversityRoute01 = false
        var ddcSync01 = false
        // Hardware CW keyer (DUC bytes 5..17).
        var cwEnabled = false
        var cwReverse = false
        var cwIambic = false
        var cwModeB = false
        var cwSpacing = false
        var cwBreakin = false
        var cwSidetoneVol = 0
        var cwSidetoneFreq = 600
        var cwSpeedWpm = 25
        var cwWeight = 50
        var cwHangMs = 300
        var cwRfDelayMs = 20
        var cwRampMs = 5
        var sampleRate = 48_000
        var receiverCount = 1            // enabled DDCs (1..MAX_DDC)
        // PureSignal: which DDC's input is switched to the TX/DUC feedback
        // loopback (byte 17+6n = 2) while enabled; -1 = off.
        var pureSignal = false
        var psFeedbackDdc = -1
        var txDrive = 0                  // 0..255
        var paEnabled = false
        var stepAttenDb = 0              // step attenuator, 0..31 dB (both ADCs)
        var ocOutputs = 0                // 7 open-collector outputs
        var watchdogEnabled = false      // needs a General resend every 500 ms
        var cwSidetone = false
    }

    data class DiscoveryInfo(
        val boardId: Int,
        val mac: String,
        val protocolVersion: Int,
        val gatewareVersion: Int,
        val ddcCount: Int,
        val busy: Boolean,
    )

    /** High-priority status + telemetry from the radio (source port 1025). */
    data class Status(
        val pttIn: Boolean,
        val dot: Boolean,
        val dash: Boolean,
        val pllLocked: Boolean,
        val adcOverload: Boolean,
        val exciterPower: Int,
        val forwardPower: Int,
        val reversePower: Int,
        val supplyVolts: Double,
    )

    // ---- host -> radio ------------------------------------------------------

    fun discoveryRequest(): ByteArray = ByteArray(60).also { it[4] = 0x02 }

    /**
     * General packet (port 1024): advertises the port plan and global config.
     * Sent at start (and every 500 ms if [ControlState.watchdogEnabled]).
     *
     * @param state The global state object containing PA and watchdog configuration.
     * @return The 60-byte serialized general packet.
     */
    fun generalPacket(state: ControlState): ByteArray {
        val p = ByteArray(60)
        p[4] = 0x00                                   // command
        putBE16(p, 5, RX_SPECIFIC_PORT)
        putBE16(p, 7, TX_SPECIFIC_PORT)
        putBE16(p, 9, HIGH_PRIORITY_PORT)
        putBE16(p, 11, FROM_STATUS_PORT)
        putBE16(p, 13, RX_AUDIO_PORT)
        putBE16(p, 15, TX_IQ_PORT)
        putBE16(p, 17, FROM_DDC0_PORT)
        putBE16(p, 19, FROM_MIC_PORT)
        putBE16(p, 21, 1027)                          // wideband base (disabled)
        p[23] = 0                                     // wideband enable bitmap: off
        p[38] = if (state.watchdogEnabled) 1 else 0   // watchdog timer
        // p2app: SetPAEnabled(byte & 1) — bit0 SET enables the PA. (Thetis'
        // C source models this bit inverted; the radio side wins.)
        p[58] = if (state.paEnabled) 1 else 0
        return p
    }

    /**
     * High-priority command (port 1027, 1444 bytes): run/PTT, DDC and TX
     * frequencies (Hz), drive, OC outputs, preamp, step attenuators and the
     * Alex filter words. Resent on every control change.
     */
    /** Saturn conversion clock (saturnregisters.c VSAMPLERATE). */
    const val CONVERSION_CLOCK_HZ = 122_880_000L

    /**
     * NCO delta phase word: word = f · 2³² / 122.88 MHz. The wire ALWAYS
     * carries phase words — p2app passes them raw to the FPGA NCO with
     * IsDeltaPhase=true, and the "frequency in Hz" general-packet flag is a
     * stub in the Saturn firmware (audit: Hz tuned ~203 kHz for 7.1 MHz).
     */
    fun phaseWord(freqHz: Long): Long =
        Math.round(freqHz.toDouble() * 4294967296.0 / CONVERSION_CLOCK_HZ)

    /** Inverse, for tests: Hz from a phase word. */
    fun phaseWordToHz(word: Long): Double =
        word.toDouble() * CONVERSION_CLOCK_HZ / 4294967296.0

    /**
     * High-priority command (port 1027, 1444 bytes): run/PTT, DDC and TX
     * frequencies (Hz), drive, OC outputs, preamp, step attenuators and the
     * Alex filter words. Resent on every control change.
     *
     * @param state The global state configuration.
     * @return The 1444-byte high-priority command packet.
     */
    fun highPriorityPacket(state: ControlState): ByteArray {
        val p = ByteArray(BUFLEN)
        var b4 = 0
        if (state.run) b4 = b4 or 0x01
        if (state.mox) b4 = b4 or 0x02
        p[4] = b4.toByte()
        // CWX host keying: bit0 enable, bit1 dot, bit2 dash.
        var b5 = 0
        if (state.cwxEnabled) b5 = b5 or 0x01
        if (state.cwxDot) b5 = b5 or 0x02
        if (state.cwxDash) b5 = b5 or 0x04
        p[5] = b5.toByte()
        // All 7 DDC NCO phase words live at 9+4n — the radio latches every
        // one on each high-priority packet, enabled or not.
        for (n in 0 until MAX_DDC) {
            putBE32(p, 9 + 4 * n, phaseWord(state.ddcFreqHz[n]))
        }
        putBE32(p, 329, phaseWord(state.txFreqHz))    // DUC NCO
        p[345] = state.txDrive.toByte()
        p[1401] = ((state.ocOutputs shl 1) and 0xFE).toByte()
        // No preamp byte: p2app never reads 1403 — Saturn gain is the step
        // attenuator only.
        // The Saturn's on-board filters are DRIVEN by these words — the
        // gateware never auto-selects. Four independent BE 16-bit registers
        // (p2app InHighPriority.c): 1428 TX filt+ant (new-client path),
        // 1430 RX2 band-pass, 1432 TX filt+ant (always), 1434 RX1 band-pass.
        val tx = txFilterWord(state)
        putBE16(p, 1428, tx)
        putBE16(p, 1430, if (state.receiverCount >= 2) rxFilterWord(state.rx2FreqHz) else 0)
        // 1432 on the new-client path (FPGA >= 12, engaged by antenna bits in
        // 1428) is the RX ANTENNA word — which antenna listens in RX — not a
        // repeat of the TX word (audit InHighPriority.c:182-187).
        putBE16(p, 1432, rxAntennaWord(state))
        putBE16(p, 1434, rxFilterWord(state.rxFreqHz))
        p[1442] = (state.stepAttenDb and 0x1F).toByte()   // ADC2 (RX2, mirror)
        p[1443] = (state.stepAttenDb and 0x1F).toByte()   // ADC1 (RX1)
        return p
    }

    // ---- Alex filter words --------------------------------------------------
    // 16-bit Saturn register maps, verbatim from the radio-side reference
    // (p2app saturnregisters.c, Alex SPI core at 0x0B000).

    object Alex {
        // TX word (slots 1428/1432): LPF select + antenna + T/R.
        const val TX_LPF_30_20 = 1 shl 4
        const val TX_LPF_60_40 = 1 shl 5
        const val TX_LPF_80 = 1 shl 6
        const val TX_LPF_160 = 1 shl 7
        const val TX_ANT_1 = 1 shl 8
        const val TX_ANT_2 = 1 shl 9
        const val TX_ANT_3 = 1 shl 10
        const val TX_TR_RELAY = 1 shl 11      // 1 = TX (FPGA also strobes it)
        const val TX_LPF_6 = 1 shl 13
        const val TX_LPF_12_10 = 1 shl 14
        const val TX_LPF_17_15 = 1 shl 15

        // RX word (slots 1434 RX1 / 1430 RX2): band-pass select.
        const val RX_BPF_10_22 = 1 shl 1
        const val RX_BPF_22_35 = 1 shl 2
        const val RX_PREAMP_6M = 1 shl 3
        const val RX_BPF_6_10 = 1 shl 4
        const val RX_BPF_2_5_6 = 1 shl 5
        const val RX_BPF_1_2_5 = 1 shl 6
        const val RX_BYPASS = 1 shl 12
    }

    /** TX low-pass filter bit for a frequency (Alex antenna-band crossovers). */
    fun txLpfBit(freqHz: Long): Int {
        val mhz = freqHz / 1e6
        return when {
            mhz <= 2.75 -> Alex.TX_LPF_160
            mhz <= 4.665 -> Alex.TX_LPF_80
            mhz <= 8.7 -> Alex.TX_LPF_60_40
            mhz <= 16.209 -> Alex.TX_LPF_30_20
            mhz <= 23.17 -> Alex.TX_LPF_17_15
            mhz <= 39.85 -> Alex.TX_LPF_12_10
            else -> Alex.TX_LPF_6
        }
    }

    /** TX filter word: the band's LPF on antenna 1, T/R asserted while keyed. */
    fun txFilterWord(state: ControlState): Int {
        var w = txLpfBit(state.txFreqHz) or antennaBit(state.txAntenna)
        if (state.mox) w = w or Alex.TX_TR_RELAY
        return w
    }

    /** RX antenna select word (new-client path, offset 1432). */
    fun rxAntennaWord(state: ControlState): Int = antennaBit(state.rxAntenna)

    private fun antennaBit(n: Int): Int {
        require(n in 1..3) { "antenna $n is outside 1..3" }
        return when (n) {
            1 -> Alex.TX_ANT_1
            2 -> Alex.TX_ANT_2
            else -> Alex.TX_ANT_3
        }
    }

    /** RX band-pass word for a frequency (Saturn BPF bank edges). */
    fun rxFilterWord(freqHz: Long): Int {
        val mhz = freqHz / 1e6
        return when {
            mhz < 1.0 -> Alex.RX_BYPASS
            mhz < 2.5 -> Alex.RX_BPF_1_2_5
            mhz < 6.0 -> Alex.RX_BPF_2_5_6
            mhz < 10.0 -> Alex.RX_BPF_6_10
            mhz < 22.0 -> Alex.RX_BPF_10_22
            mhz < 35.0 -> Alex.RX_BPF_22_35
            else -> Alex.RX_PREAMP_6M
        }
    }

    /**
     * RX-specific command (port 1025, 1444 bytes): ADC config and per-DDC
     * enable/rate/size. Ordinary DDCs ride ADC1; the explicit diversity
     * transaction routes DDC1 to ADC2 before asserting the sync field.
     *
     * @param state State containing receiver counts, rates, and dither settings.
     * @return The 1444-byte RX-specific command packet.
     */
    fun rxSpecificPacket(state: ControlState): ByteArray {
        require(state.receiverCount in 1..MAX_DDC) {
            "receiver count ${state.receiverCount} is outside 1..$MAX_DDC"
        }
        require(state.sampleRate in SAMPLE_RATES) {
            "sample rate ${state.sampleRate} is not an exact Protocol-2 rate"
        }
        require(state.adcDither in 0..0b11) { "ADC dither mask exceeds 0b11" }
        require(state.adcRandom in 0..0b11) { "ADC random mask exceeds 0b11" }
        if (state.diversityRoute01) {
            require(state.receiverCount >= 2) { "RX1/RX2 route requires two DDCs" }
        }
        if (state.ddcSync01) {
            require(state.diversityRoute01) { "RX1/RX2 sync requires the typed ADC route" }
            require(state.ddcFreqHz[0] == state.ddcFreqHz[1]) {
                "RX1/RX2 sync requires frequency-locked NCOs"
            }
            require(!state.pureSignal || state.psFeedbackDdc != 1) {
                "RX2 cannot be diversity input and PureSignal feedback"
            }
        }
        val p = ByteArray(BUFLEN)
        p[4] = 2                                      // ADCs on a Saturn
        p[5] = state.adcDither.toByte()               // dither: bit0 ADC1, bit1 ADC2
        p[6] = state.adcRandom.toByte()               // random: bit0 ADC1, bit1 ADC2
        // DDC enable bits (LE16, low byte first): bit n = DDC n armed.
        val nRx = state.receiverCount
        val enable = (1 shl nRx) - 1
        p[7] = (enable and 0xFF).toByte()
        p[8] = ((enable ushr 8) and 0xFF).toByte()
        val rateKhz = state.sampleRate / 1000
        // Per-DDC config at 17+6n: [b] input select, [b+1..2] rate, [b+5] size.
        // With PureSignal on, the feedback DDC's input is the TX/DUC loopback.
        for (n in 0 until MAX_DDC) {
            val b = 17 + 6 * n
            p[b] = when {
                state.pureSignal && n == state.psFeedbackDdc -> DDC_INPUT_TX_FEEDBACK.toByte()
                state.diversityRoute01 && n == 1 -> DDC_INPUT_ADC1.toByte()
                else -> DDC_INPUT_ADC0.toByte()
            }
            putBE16(p, b + 1, rateKhz)
            p[b + 5] = 24
        }
        // Synchronised DDC pair 0+1 (IncomingDDCSpecific.c:130-139): value
        // 0b10 at 1363 interleaves DDC1 into DDC0's stream, phase-coherent —
        // the P2 foundation for diversity combining.
        if (state.ddcSync01) p[1363] = 0b10
        return p
    }

    /**
     * TX-specific command (port 1026, 60 bytes). For SSB/AM via the TX IQ
     * stream only two fields matter: one DAC and the fixed 192 kHz DUC rate.
     * CW/sidetone/mic bytes stay zero — voice reaches the radio pre-modulated
     * on port 1029, never through the radio's own mic path.
     *
     * @param state State containing attenuator and CW settings.
     * @return The 60-byte TX-specific command packet.
     */
    fun txSpecificPacket(state: ControlState): ByteArray {
        val p = ByteArray(60)
        p[4] = 1                                      // number of DACs
        // CW bools (IncomingDUCSpecific.c:85-90): bit1 enable, bit2 reverse,
        // bit3 iambic, bit4 sidetone, bit5 mode B, bit6 strict spacing,
        // bit7 breakin.
        var b5 = 0
        if (state.cwEnabled) b5 = b5 or 0x02
        if (state.cwReverse) b5 = b5 or 0x04
        if (state.cwIambic) b5 = b5 or 0x08
        if (state.cwSidetone) b5 = b5 or 0x10
        if (state.cwModeB) b5 = b5 or 0x20
        if (state.cwSpacing) b5 = b5 or 0x40
        if (state.cwBreakin) b5 = b5 or 0x80
        p[5] = b5.toByte()
        p[6] = (state.cwSidetoneVol and 0xFF).toByte()
        putBE16(p, 7, state.cwSidetoneFreq)
        p[9] = (state.cwSpeedWpm and 0xFF).toByte()
        p[10] = (state.cwWeight and 0xFF).toByte()
        putBE16(p, 11, state.cwHangMs)
        p[13] = (state.cwRfDelayMs and 0xFF).toByte()
        putBE16(p, 14, TX_RATE / 1000)                // DUC rate in kHz (192)
        p[17] = (state.cwRampMs and 0xFF).toByte()
        // Step attenuators DURING TX (bytes 58/59) — keys the feedback ADC
        // level for PureSignal instead of inheriting the RX attens.
        p[58] = (state.txAttenDb and 0x1F).toByte()   // ADC2
        p[59] = (state.txAttenDb and 0x1F).toByte()   // ADC1
        return p
    }

    /**
     * One TX IQ packet (port 1029, 1444 bytes): 4-byte sequence that
     * increments every packet, then 240 samples of 24-bit BE pairs (imaginary word first) taken
     * from [iq] (interleaved `[-1,1]` floats at 192 kSps) starting at
     * [offset] pairs. Missing samples are zero-filled.
     */
    fun txIqPacket(seq: Long, iq: FloatArray, offset: Int): ByteArray {
        val p = ByteArray(BUFLEN)
        putBE32(p, 0, seq)
        var o = 4
        for (k in 0 until TX_SAMPLES_PER_PACKET) {
            val idx = (offset + k) * 2
            if (idx + 1 < iq.size) {
                // Mirror of the DDC wire order: the radio expects the
                // IMAGINARY part first (p2app InDUCIQ.c swaps to the FPGA's
                // internal order on ingest) — sending real-first mirrors the
                // transmitted sideband.
                putS24(p, o, iq[idx + 1])
                putS24(p, o + 3, iq[idx])
            }
            o += 6
        }
        return p
    }

    // ---- radio -> host ------------------------------------------------------

    /** Parses a discovery reply; null if [buf] is not one. */
    fun parseDiscoveryReply(buf: ByteArray, length: Int): DiscoveryInfo? {
        if (length < 24) return null
        if (buf[0].toInt() != 0 || buf[1].toInt() != 0 ||
            buf[2].toInt() != 0 || buf[3].toInt() != 0
        ) return null
        val cmd = buf[4].toInt() and 0xFF
        if (cmd != 0x02 && cmd != 0x03) return null
        val mac = (5..10).joinToString(":") { "%02x".format(buf[it].toInt() and 0xFF) }
        return DiscoveryInfo(
            boardId = buf[11].toInt() and 0xFF,
            mac = mac,
            protocolVersion = buf[12].toInt() and 0xFF,
            gatewareVersion = buf[13].toInt() and 0xFF,
            ddcCount = if (length > 20) buf[20].toInt() and 0xFF else 0,
            busy = cmd == 0x03,
        )
    }

    /** Parses the 60-byte high-priority status packet (source port 1025). */
    fun parseStatus(buf: ByteArray, length: Int): Status? {
        if (length != STATUS_LEN) return null
        val b0 = buf[4].toInt() and 0xFF              // payload[0]
        // Supply volts ride payload[45..46]: 12-bit ADC, 3.3 V reference,
        // (4.7k + 0.82k) / 0.82k divider — the Saturn measurement chain.
        val vRaw = be16(buf, 4 + 45)
        val volts = vRaw / 4095.0 * 3.3 * ((4.7 + 0.82) / 0.82)
        return Status(
            pttIn = b0 and 0x01 != 0,
            dot = b0 and 0x02 != 0,
            dash = b0 and 0x04 != 0,
            pllLocked = b0 and 0x10 != 0,
            adcOverload = (buf[4 + 1].toInt() and 0xFF) != 0,
            exciterPower = be16(buf, 4 + 2),
            forwardPower = be16(buf, 4 + 10),
            reversePower = be16(buf, 4 + 18),
            supplyVolts = volts,
        )
    }

    /** Per-sample IQ sink with primitive params (no per-sample boxing). */
    fun interface IqSink {
        fun onSample(i: Float, q: Float)
    }

    /**
     * Parses a 1444-byte DDC IQ packet: 16-byte header (sequence, timestamp,
     * bits/sample, samples/packet) + 238 samples of 24-bit BE pairs, imaginary word first.
     *
     * @param buf The raw packet byte array.
     * @param length The total length of the packet.
     * @param onSample Callback for every parsed complex IQ pair.
     * @return The packet sequence, or -1 if the packet is malformed.
     */
    fun parseDdcIq(
        buf: ByteArray,
        length: Int,
        onSample: IqSink
    ): Long {
        if (length != BUFLEN) return -1
        val seq = be32(buf, 0)
        val nSamp = be16(buf, 14).coerceAtMost(IQ_SAMPLES_PER_PACKET)
        var o = DDC_HEADER
        for (k in 0 until nSamp) {
            // Wire order is IMAGINARY first, REAL second (same HPSDR FPGA
            // convention as Protocol 1): with the standard z = i + jq math the
            // first word must land in q, or the spectrum mirrors and USB/LSB
            // swap on real hardware.
            onSample.onSample(s24f(buf, o + 3), s24f(buf, o))
            o += 6
        }
        return seq
    }

    /** Whether a datagram from [srcPort] is a DDC IQ stream, and which DDC. */
    fun ddcIndexOf(srcPort: Int): Int {
        val idx = srcPort - FROM_DDC0_PORT
        return if (idx in 0..6) idx else -1
    }

    // ---- byte helpers -------------------------------------------------------

    fun putBE16(b: ByteArray, o: Int, v: Int) {
        b[o] = (v ushr 8).toByte(); b[o + 1] = v.toByte()
    }

    fun putBE32(b: ByteArray, o: Int, v: Long) {
        b[o] = (v ushr 24).toByte(); b[o + 1] = (v ushr 16).toByte()
        b[o + 2] = (v ushr 8).toByte(); b[o + 3] = v.toByte()
    }

    fun be16(b: ByteArray, o: Int) = ((b[o].toInt() and 0xFF) shl 8) or (b[o + 1].toInt() and 0xFF)

    fun be32(b: ByteArray, o: Int): Long =
        ((b[o].toLong() and 0xFF) shl 24) or ((b[o + 1].toLong() and 0xFF) shl 16) or
                ((b[o + 2].toLong() and 0xFF) shl 8) or (b[o + 3].toLong() and 0xFF)

    /** 24-bit BE two's complement, left-justified into 32 bits, scaled 1/2^31. */
    fun s24f(b: ByteArray, o: Int): Float {
        val v = ((b[o].toInt() and 0xFF) shl 24) or
                ((b[o + 1].toInt() and 0xFF) shl 16) or
                ((b[o + 2].toInt() and 0xFF) shl 8)
        return v / 2_147_483_648.0f
    }

    /** Packs a float in `[-1,1]` into 24-bit BE two's complement at [o]. */
    fun putS24(b: ByteArray, o: Int, f: Float) {
        val v = (f.coerceIn(-1f, 1f) * 8_388_607f).toInt()
        b[o] = (v shr 16).toByte(); b[o + 1] = (v shr 8).toByte(); b[o + 2] = v.toByte()
    }
}
