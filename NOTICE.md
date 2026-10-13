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
* The CMake build (`src/CMakeLists.txt`), the documentation of the C driver, which
  was the `README.md` of the original and is `docs/c-driver.md` here, and the
  photographs in `imgs/`.

## Ângelo Moisés Alves, Arduino port, 2026

Written for this fork:

* `src/PicoENC28J60.h`, `src/PicoENC28J60.cpp`: the Arduino class. Pin and MAC
  configuration, sending and receiving whole frames, link and revision, the
  handling of the INT pin through the Arduino core, diagnostic counters, the
  reporting of errors, `end()`, the recovery from a stalled PIO and from a chip
  that was reset.
* `src/PicoENC28J60lwIP.h`: the network interface that plugs the driver into the
  lwIP stack of the arduino-pico core: DHCP, TCP and UDP sockets, WebServer, with
  polling or driven by the INT pin.
* `examples/`: `BasicTest`, `AdvancedTest`, `WebServerLedToggle`.
* `extras/bench/`: the on-hardware test harness (`BenchTest.ino`, `bench.py`,
  `BenchLwip.ino`, `benchlwip.py`, `webtest.py`, `gate.sh`, `deploy.py`) and
  `extras/tools/`.
* `.github/`: the CI workflow and the form for bug reports.
* `library.properties`, `keywords.txt`, `.gitattributes`, `NOTICE.md`,
  `CHANGELOG.md`, `CONTRIBUTING.md`, `README.md`, which describes the Arduino library,
  and the section of `docs/c-driver.md` that lists what this fork added to the C driver.
* `src/enc28j60-spi.pio.h` is the output of `pioasm` for the PIO program above,
  committed because the Arduino IDE cannot run `pioasm`.

Changes to Lukasz's files, which keep his copyright notice and carry a
"Modifications copyright" line:

* `src/enc28j60-spi.pio`: the slave select is driven with `set` instead of being
  the second side-set pin. In the original it had to be the pin next to the clock;
  now SCK and CS can be any GPIO. Each SPI clock takes 4 PIO clocks, not 10, and the
  program keeps the chip select times of the data sheet (TCSS, TCSH, TCSD) at any
  clock up to 20 MHz. Typos fixed.
* `src/pico/enc28j60-driver.h`: `cs_pin` and `spi_hz` in the configuration, the
  statuses of `enc28j60_init_status()`, the declarations of the functions added to the
  driver, and comments updated for what changed in it.
* `src/enc28j60-driver.c`:
  * the separate chip select pin;
  * the SPI clock is a setting (`spi_hz`, 10 MHz by default, up to 20 MHz) where it was
    fixed at 3.5 MHz; it is never faster than asked for, and is lowered when the
    fractional divider of the PIO would make a period shorter than the 50 ns the chip
    allows;
  * `enc28j60_init()` gives up instead of hanging when no chip answers (the
    oscillator-ready wait is bounded and the revision ID is checked) and returns
    `NULL`; `enc28j60_init_status()` also says why it failed, after checking the
    configuration, and `enc28j60_status_string()` puts the reason in words;
  * the PIO program is loaded once per PIO block and shared, so that initialization
    can be retried, and its absence of room is detected (`pio_add_program()` returns
    a negative number);
  * the state machine is claimed, and can be left to the driver to choose;
  * `enc28j60_deinit()` stops the chip and gives everything back;
  * `enc28j60_io_failed()` and `enc28j60_recover()` deal with a state machine that
    stopped running;
  * fixes in the blocking receive and transmit paths for the silicon errata: the
    receive read pointer is always odd and the transmit logic is reset before each
    frame; the packet counter stands for the unreliable packet flag (issue 6); a
    frame aborted after a late collision is sent again (issue 13); and
    `enc28j60_frame_rx_info_blk()` kept the size of a frame but lost the pointer to
    the next one (an `=` that had to be `|=`);
  * the same errata in the asynchronous functions, which the FreeRTOS driver uses:
    `ERXRDPT` is kept odd (issue 14), the transmit logic is reset before each frame
    (issue 12), a frame aborted after a late collision counts as ended, with `TXERIF`
    as an interrupt source (issue 13), and the interrupt flags take `PKTIF` from the
    packet counter (issue 6);
  * in the asynchronous mode, `enc28j60_deinit()` gives back the DMA channels, the DMA
    interrupt handler and the command buffer, the handler is added once per DMA
    interrupt, and a lack of memory or of DMA channels at init is a status
    (`ENC28J60_ERR_NOMEM`, `ENC28J60_ERR_DMA`) instead of a panic;
  * the transmit area starts over from its beginning when nothing is waiting to be
    sent; without that, long frames were refused with a receive buffer of 0x1600
    bytes or more. A frame may also end where the oldest frame waiting begins, and a
    flag tells a full area from an empty one: before, the area refused room it had,
    and with a receive buffer of 0x1420 bytes or more it could put a frame over the
    one on the wire;
  * with `-DENC28J60_PROFILE`, the wait for the wire counts its reads of `EIR` and the
    time it takes;
  * new functions `enc28j60_frame_tx_wait_blk()`, `enc28j60_frame_send_blk()`,
    `enc28j60_frame_send_finish_blk()`, `enc28j60_frame_tx_ended_blk()`,
    `enc28j60_tx_status_blk()`, `enc28j60_alive_blk()`, `enc28j60_set_mac_blk()`,
    `enc28j60_rx_reset_blk()`, `enc28j60_irq_status_blk()`,
    `enc28j60_irq_enable_blk()` and `enc28j60_irq_is_tx_err()`.
* `src/enc28j60-cmd.h`, `src/enc28j60-cmd.c`: the wait on the MII interface has a
  timeout, a few register bit definitions were added, `PHID1` was corrected, a
  signedness warning was fixed, the pad setting of the MAC makes 64 byte frames, and
  the blocking transfer gives up when the state machine makes no progress. The
  arguments of the command macros are in parentheses, register bank 0 or 3 is chosen
  with one command instead of two, an asynchronous execution ends when both of its DMA
  transfers have, not only the reception, and with `-DENC28J60_PROFILE` the blocking
  execution counts its commands and their time, in counters that
  `enc28j60_profile_read()` copies and `enc28j60_profile_reset()` clears.
* `freertos/enc28j60-freertos-driver.c`: stops with a message that says why when the
  driver cannot start, releases a frame only when one is in progress, and lost an
  unused variable and two unused parameters. Moved out of `src/` so that the Arduino
  IDE does not try to compile it; `freertos/pico/enc28j60-freertos-driver.h` was
  moved and is unchanged.
* `src/CMakeLists.txt`: paths adjusted for the files that moved.
* `docs/c-driver.md`: his documentation of the C driver, moved from `README.md`.
  The text is his, updated where this fork changed what it describes (the length of
  the PIO program, the state machine that is claimed, the `pio`, `sm`, `cs_pin`,
  `spi_hz` and `rx_size` fields), with a list of the functions that were added, a
  section on the asynchronous functions and a note on the configuration that the
  FreeRTOS driver needs.
* `.gitignore`.

Unchanged from the original: `.clang-format` and `imgs/*`.
