# Copyright and credits — libg2sdrk

Kotlin driver for the ANAN-G2 / Saturn (openHPSDR Protocol 2 over UDP).

Kotlin port: Copyright (C) 2026 **Isak Ruas** <isakruas@gmail.com> (PU3IAR).

This program is free software; you can redistribute it and/or modify it under
the terms of the **GNU General Public License v2 or later** as published by the
Free Software Foundation. See [`LICENSE`](LICENSE) for the full text.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

## Based on the work of

The wire protocol is a faithful port of, and follows, the following upstream
work — with thanks to their authors:

- **Saturn / ANAN-G2** — Laurence Barker (G8NJJ) and contributors (hardware,
  FPGA gateware, and the `p2app` radio-side Protocol 2 server used as ground
  truth): <https://github.com/laurencebarker/Saturn>
- **Thetis** — the reference Protocol 2 host client, Thetis maintainers and
  the openHPSDR community: <https://github.com/ramdor/Thetis>
- **openHPSDR Ethernet Protocol (Protocol 2)** — the openHPSDR project / TAPR.

Trademarks and project names belong to their respective owners. This is an
independent Kotlin implementation and is not affiliated with or endorsed by
the openHPSDR, Apache Labs or Saturn projects.
