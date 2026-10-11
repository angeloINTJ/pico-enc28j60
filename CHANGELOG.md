# Changelog

## 1.1.0 - 2026-10-11

The first version that works on hardware. 1.0.0 compiled, but on a real ENC28J60 it
received no frame correctly and stopped sending after a few.

Checked on a Raspberry Pi Pico with an ENC28J60 (revision B7) on a 10 Mbit/s port of
a home LAN: the 14 tests of `extras/bench` (17 with the INT pin, which was wired to
GP6), and the HTTP tests of `extras/bench/webtest.py` for the web server example.

### Added

* `PicoENC28J60lwIP.h`: the ENC28J60 as a network interface of the lwIP stack of the
  arduino-pico core. DHCP, TCP and UDP sockets, `WiFiClient`, `WiFiServer` and
  `WebServer` run on top of it. It does not rely on the polling of the core, which
  stops while a sketch keeps calling into lwIP.
* `examples/WebServerLedToggle`: a web page with a button that toggles the onboard
  LED, with a `/status` page.
* Interrupt-driven operation with the INT pin (`ENC28J60Config::irq_pin`):
  `interruptPending()`, `peekInterruptFlags()`, `interruptCount()`. With the lwIP
  interface, frames are served the moment they arrive and an idle network costs no
  SPI traffic.
* `Stats` counters (`stats()`, `driverStats()`), `getDriverHandle()`.
* `extras/bench`: on-hardware tests that generate real traffic from the PC (ping,
  UDP, soak, overload and recovery, the interrupt, the link going down and up).
* `NOTICE.md` (who wrote what), copyright notices in the files, `CHANGELOG.md`.
* C driver: `enc28j60_frame_tx_wait_blk()`, `enc28j60_set_mac_blk()`,
  `enc28j60_rx_reset_blk()`, `enc28j60_irq_status_blk()`, `enc28j60_irq_enable_blk()`.

### Fixed

* Receiving. The receive read pointer now follows the silicon errata (it must be
  odd), and `enc28j60_frame_rx_info_blk()` no longer loses the pointer to the next
  frame. Before this, no ping was answered and no frame was received correctly. A
  frame header that makes no sense now resets the receiver instead of being parsed.
* Frames are handed over without the 4-byte FCS that the chip counts in their length.
* Sending. Each frame is waited for and its buffer space released, and the transmit
  logic is reset before each frame; before, the transmit buffer filled up after about
  a dozen frames and sending failed from then on.
* `begin()` returns `false`, in about 120 ms, when no ENC28J60 answers, instead of
  hanging or reporting success, and can be called again. The wait on the MII
  interface has a timeout too.
* `setMACAddress()` programs the chip.
* The INT pin: the interrupt handler no longer talks to the chip. Before, with the
  pin wired, a handler that read the flags corrupted the transfers of the main
  program and the pin was never served. The GPIO interrupt is now attached through
  the Arduino core, so it coexists with your own `attachInterrupt()` calls.
* `BasicTest` waits for the link before reporting it.
* The C driver builds with the Pico SDK on its own and without warnings, and the
  include path of the FreeRTOS driver is right.

### Changed

* `hasPacket()`, `hasTXComplete()`, `hasLinkChange()` and `hasRXError()` only look at
  the flags; they no longer mask the INT pin.
* `setInterruptCallback()`: the function runs in interrupt context and must not call
  the driver. `poll()` runs it from `loop()` when the INT pin is not wired.
* The source layout: the public driver header is `src/pico/enc28j60-driver.h`, and
  the FreeRTOS driver is in `freertos/`, outside of what the Arduino IDE compiles.
* `library.properties`: `url` points to this repository and `maintainer` is a real
  address.

### Removed

* `examples/DhcpClient`, a DHCP client on raw frames. It never completed an exchange
  (it discarded every reply, and its UDP checksum and hostname option were wrong), and
  `PicoENC28J60lwIP` does DHCP properly.

## 1.0.0 - 2026-10-10

First version: the original driver with an Arduino class around it. Its tag was
withdrawn after 1.1.0, because this version could not receive or keep sending frames
on real hardware. Use 1.1.0 or later.
