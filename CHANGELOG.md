# Changelog

## Unreleased

Robustness of the driver: it no longer hangs when the PIO does not run, shares the PIO
with the rest of the sketch, reports why it could not start, and follows more of the
silicon errata. The lwIP interface without the INT pin answers faster.

### Added

* `PicoENC28J60::end()` stops the chip and gives back the state machine, the pins and
  the memory; `begin()` can start it again. The destructor calls it. `enc28j60_deinit()`
  did nothing before.
* `PicoENC28J60::lastError()`, the `Error` enum and `errorString()` say why `begin()`
  failed: a pin out of range or used twice, a state machine that is taken, no PIO with
  room for the program, no chip answering, out of memory. The C driver has
  `enc28j60_init_status()` and `enc28j60_status_string()`.
* `ENC28J60Config::pio_instance` and `pio_sm` default to -1, which lets the driver find
  a PIO with room and a free state machine. The C driver does the same with
  `pio = NULL` and `sm = ENC28J60_SM_ANY`, which are now the defaults of
  `enc28j60_get_default_config()`.
* Recovery from a PIO that stops running: `enc28j60_io_failed()` and
  `enc28j60_recover()` in the C driver, and in `PicoENC28J60` a restart of the state
  machine and of the chip, with `Stats::io_errors` and `Stats::recoveries`.
* `PicoENC28J60::checkChip()` finds out whether the chip lost its configuration, which a
  reset of the chip does silently (a glitch of its supply, say), and configures it
  again; `Stats::chip_resets` counts it. One SPI transaction. `PicoENC28J60lwIP` calls it
  once a second, with or without the INT pin. On the bench the chip was reset behind the
  back of the driver: the interface noticed within a second and ping and TCP worked
  afterwards.
* `Stats::tx_aborts` and `Stats::tx_retries`; in the C driver
  `enc28j60_frame_send_blk()` and `enc28j60_tx_status_blk()`, which gives the transmit
  status vector of a frame.
* `extras/bench`: tests for the configuration errors, a PIO without room, the transmit
  status, sharing the PIO with `SerialPIO`, an SPI transfer that stalls, `end()` followed
  by `begin()`, frames of all lengths with the longest receive buffer, and a chip that was
  reset behind the back of the driver.
* `extras/bench/BenchLwip` and `benchlwip.py`: tests of the lwIP interface on the
  hardware, with the PC on the other side: DHCP, ping both ways, TCP and UDP servers,
  a TCP client, DNS, a static address, `end()` followed by `begin()`, fragmented
  datagrams, chained buffers, a chip that was reset behind the back of the interface, and
  the cable being pulled out and put back.
* A GitHub Actions workflow: `arduino-lint`, the formatting of the sources, and a build
  of every example for the Pico, Pico W, Pico 2 and Pico 2 W.
* `extras/tools/regenerate-pio-header.sh` regenerates `src/enc28j60-spi.pio.h` with the
  pioasm of the core, and the CI checks that the program in the header is the one that
  pioasm makes of the `.pio` file. The CI also runs the static analyzer of GCC on the C
  driver.
* A message at compile time when the core is too old or is not arduino-pico: the raw
  class needs 4.0.0 or newer (checked with 4.0.0), the lwIP interface 5.6.0 or newer
  (checked with 5.6.0; before that the core has no `__getEthernetContext()`). The CI
  builds with both.

### Fixed

* The driver never claimed its PIO state machine. The Arduino core gives the first free
  one to `SerialPIO` and the like; it fills `pio1` first, so with five `SerialPIO`
  objects the fifth landed on the state machine of the driver and every call to the
  chip then waited for ever. The state machine is claimed now, and the others get
  another one.
* A blocking SPI transfer waited for ever when the PIO did not run. It now gives up
  after about 8 ms without progress, reads zeros, and the class recovers (see above).
* `pio_add_program()` returns a negative number when the instruction memory is full;
  that was taken for an offset.
* `hasPacket()` and `interruptPending()` without the INT pin read `EIR.PKTIF`, which
  the silicon errata (issue 6) says is unreliable, and could miss a frame. The
  packet counter is used now, also for `INT_PACKET_RX` in `getInterruptFlags()` and
  `peekInterruptFlags()`. The INT pin itself is reliable and was not affected.
* A frame that the MAC gave up on after a late collision (issue 13) was counted as sent
  and never retried. It is sent again, up to 15 times; any other abort makes
  `sendFrame()` return `false`.
* The pad setting of the MAC (`MACON3.PADCFG`) padded short frames to 64 bytes before the
  FCS, so they went out with 68 bytes. They are 64 bytes now.
* With a receive buffer of 0x1600 bytes or more (`rx_buffer_size`, up to 0x1A00), which
  leaves a transmit area shorter than two full frames, `sendFrame()` refused more and
  more frames and in the end every long one: the area was not started over when it
  was empty, so a frame that did not fit before the end of the RAM never found room
  after it. In a test with frames of all lengths, 238 of 240 failed with 0x1A00 and
  39 of 240 with 0x1600; a TCP connection whose stack tried to send a full-size
  segment never recovered. The default buffer was not affected.
* `BasicTest` and `AdvancedTest` sent forged ARP and ICMP packets claiming to be
  192.168.1.100, which could confuse a real network. They send a broadcast with the
  experimental EtherType 0x88B5 now.

### Changed

* `PicoENC28J60lwIP` without the INT pin looks at the chip every millisecond right after
  a frame went out or came in, backing off to the period of `setPollPeriod()` (5 ms by
  default) when nothing arrives, and a frame sent from your code brings the next look
  forward. A fixed period made the time of a request depend on where the timer fell
  relative to the answer of the peer: 13 ms or 18 ms for the same page on the bench,
  10 to 11 ms now. An idle line costs what it did.
* `PicoENC28J60lwIP` tells lwIP when the link goes down and up. When the cable comes
  back lwIP asks the DHCP server again, so a board moved to another network gets an
  address there at once and not when the lease comes up for renewal; before, lwIP
  was told nothing and went on with the lease it had. On the bench the PHY was
  powered down and up: the address was asked for again and the connections worked
  afterwards.
* The default MAC address of `ENC28J60Config` is made from the unique id of the board when
  `begin()` runs, and is no longer the same for every board (02:42:AC:11:00:01). Set
  `mac_addr` if you need a fixed one, for a reservation in the DHCP server of the router,
  say.
* `hasTXComplete()` is deprecated: `sendFrame()` already waits until the frame has left
  the wire, and the flag it read stayed set from one frame to the next.
* `PicoENC28J60lwIP` puts the pieces of a chain of buffers together before sending, instead
  of sending the first one only. The core never hands over a chain, so this is a guard.
* `hasPacket()` is one read of the packet counter.
* `README.md` has a section on what the library does when things go wrong and what it
  does not do.

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
