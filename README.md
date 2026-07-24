# libg2sdrk

Kotlin driver for the **ANAN-G2 / Saturn** amateur-radio SDR transceiver,
speaking the openHPSDR **Protocol 2** over UDP. Pure Kotlin — no NDK, no
native libraries, no root. Built for Android but the wire codec is
Android-free and JVM-testable.

Maintained by Isak — **PU3IAR**. Brought to you by [id.qsl.br](https://id.qsl.br),
a platform with tools for amateur radio operators.
YouTube: [@qraisak](https://www.youtube.com/@qraisak)

## Features

- **`G2Protocol`** — the openHPSDR Protocol 2 frame codec, free of any Android
  dependency and unit-tested on the JVM:
  - UDP **discovery** (board id, MAC, gateware, DDC count) and the General /
    RX-specific / TX-specific / high-priority command packets.
  - **RX**: DDC IQ parsing (24-bit big-endian, 238 samples/packet, source
    ports 1035+), for **1 or 2 receivers**, at 48/96/192/384 kSps.
  - **High-priority C&C**: run/MOX, DDC and DUC frequencies (Hz), TX drive,
    open-collector outputs, step attenuators, and the Saturn **Alex filter
    words** — four 16-bit registers (TX LPF + antenna + T/R; RX1/RX2
    band-pass) computed from frequency, since the gateware never
    auto-selects filters.
  - **TX**: 240-sample 24-bit IQ packets for the fixed 192 kSps DUC, and
    **telemetry** decode (PTT/dot/dash, PLL lock, ADC overload, exciter /
    forward / reverse power, supply voltage).
- **`TxInterpolator`** — polyphase FIR (Blackman-windowed sinc) expanding the
  48 kSps modulator IQ to the 192 kSps DUC rate with images below −60 dBc.
- **`G2Client`** — UDP transport and streaming lifecycle:
  - LAN discovery (or a fixed radio IP), RX accumulation into display blocks
    and a power spectrum, delivered through the same callback contract as the
    sibling clients so a host app can drive any device.
  - A wall-clock-paced TX sender (800 packets/s) that only consumes whole
    packets of queued audio, and a keepalive that satisfies the radio's
    1-second activity watchdog when armed.

## Validation

The wire contract is pinned by JVM tests written against **two independent
ground truths**: the Thetis reference client (`ChannelMaster/network.c`) and
the radio-side **p2app** server that runs inside the ANAN-G2
(`Saturn/sw_projects/P2_app`), including the p2app reference values for the
Alex filter registers.

## Credits and references

Faithful port of the reference implementations and the openHPSDR
documentation:

- **Saturn / ANAN-G2** project (hardware, FPGA, `p2app` radio-side server) —
  Laurence Barker (G8NJJ) and contributors:
  <https://github.com/laurencebarker/Saturn>
- **Thetis** console (reference Protocol 2 host client) — the Thetis
  maintainers and the openHPSDR community:
  <https://github.com/ramdor/Thetis>
- **openHPSDR Ethernet Protocol (Protocol 2)** specification — the openHPSDR
  project and TAPR.

## License

GNU General Public License **v2 or later** — see [`LICENSE`](LICENSE).
Kotlin port © 2026 Isak Ruas <isakruas@gmail.com>.
