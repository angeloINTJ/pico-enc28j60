# Authorship and license

PicoENC28J60 is a fork of [pico-enc28j60](https://github.com/ljanyst/pico-enc28j60)
by Lukasz Janyst. It keeps his driver and adds what is needed to use it from the
Arduino IDE and with the networking stack of the arduino-pico core.

The whole project is available under the MIT license. [LICENSE](LICENSE) is the
license text of the original, unchanged, with the copyright notice of both authors
on top, which is what that license asks for when the work is passed on. This file
says who wrote what.

The fork starts from the upstream commit `f9f653b` (22 June 2025). The history of
Lukasz's work, 52 commits, is part of the history of this repository: a merge commit
records `f9f653b` as an ancestor without changing any file, so `git log` and
`git shortlog` show his commits. His files were first committed here as an import
(`ef509e9`), and the port is everything that came after.

## Lukasz Janyst, original work, 2024-2025

The foundation of this library. All of it comes from the upstream repository:

* The PIO program that speaks SPI to the chip (`src/enc28j60-spi.pio`), including
  the idea of extending each command with its transmit and receive sizes, so that
  the state machine frames every command on its own and DMA can feed it without
  handling the slave select by hand.
* The ENC28J60 driver in C: the initialization sequence, register access, the
  management of the on-chip transmit and receive buffers, interrupt handling, and
  the blocking and the DMA-driven (non-blocking) command execution
  (`src/enc28j60-driver.c`, `src/enc28j60-cmd.c`, `src/enc28j60-cmd.h`,
  `src/pico/enc28j60-driver.h`).
* The network interface driver for FreeRTOS+TCP (`freertos/`).
* The CMake build (`src/CMakeLists.txt`), the documentation of the C driver in
  `README.md` and the photographs in `imgs/`.

## Ângelo Moisés Alves, Arduino port, 2026

Written for this fork:

* `src/PicoENC28J60.h`, `src/PicoENC28J60.cpp`: the Arduino class. Pin and MAC
  configuration, sending and receiving whole frames, link and revision, the
  handling of the INT pin through the Arduino core, diagnostic counters.
* `src/PicoENC28J60lwIP.h`: the network interface that plugs the driver into the
  lwIP stack of the arduino-pico core: DHCP, TCP and UDP sockets, WebServer, with
  polling or driven by the INT pin.
* `examples/`: `BasicTest`, `AdvancedTest`, `WebServerLedToggle`.
* `extras/bench/`: the on-hardware test harness (`BenchTest.ino`, `bench.py`,
  `webtest.py`).
* `library.properties`, `keywords.txt`, `.gitattributes`, `NOTICE.md`,
  `CHANGELOG.md`, the Arduino sections of `README.md`.
* `src/enc28j60-spi.pio.h` is the output of `pioasm` for the PIO program above,
  committed because the Arduino IDE cannot run `pioasm`.

Changes to Lukasz's files, which keep his copyright notice and carry a
"Modifications copyright" line:

* `src/enc28j60-spi.pio`: the slave select is driven with `set` instead of being
  the second side-set pin. In the original it had to be the pin next to the clock;
  now SCK and CS can be any GPIO. Typos fixed.
* `src/pico/enc28j60-driver.h`: `cs_pin` in the configuration, and the
  declarations of the functions added to the driver.
* `src/enc28j60-driver.c`:
  * the separate chip select pin;
  * `enc28j60_init()` gives up instead of hanging when no chip answers (the
    oscillator-ready wait is bounded and the revision ID is checked) and returns
    `NULL`;
  * the PIO program is loaded once per PIO block, so that initialization can be
    retried;
  * fixes in the blocking receive and transmit paths for the silicon errata: the
    receive read pointer is always odd and the transmit logic is reset before each
    frame; and `enc28j60_frame_rx_info_blk()` kept the size of a frame but lost
    the pointer to the next one (an `=` that had to be `|=`);
  * new functions `enc28j60_frame_tx_wait_blk()`, `enc28j60_set_mac_blk()`,
    `enc28j60_rx_reset_blk()`, `enc28j60_irq_status_blk()` and
    `enc28j60_irq_enable_blk()`.
* `src/enc28j60-cmd.h`, `src/enc28j60-cmd.c`: the wait on the MII interface has a
  timeout, a few register bit definitions were added, `PHID1` was corrected, and a
  signedness warning was fixed.
* `freertos/enc28j60-freertos-driver.c`: stops with a message if the chip does
  not answer. Moved out of `src/` so that the Arduino IDE does not try to compile
  it; `freertos/pico/enc28j60-freertos-driver.h` was moved and is unchanged.
* `src/CMakeLists.txt`: paths adjusted for the files that moved.
* `.gitignore`, `README.md`.

Unchanged from the original: `.clang-format` and `imgs/*`.
