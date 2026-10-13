# Changelog

## Unreleased

The asynchronous functions of the C driver, the ones the FreeRTOS+TCP driver uses, had
never been run in this fork. The bench now runs them the way that driver does, and they
did not work: they corrupted the state of the chip within the first frames. They work
now, and follow the same silicon errata as the blocking functions. The tools that check
a change before it goes in (the hardware gate, a check of every commit, the timing of
the PIO program) are part of the repository now.

The lwIP interface sends full-size frames at 787 kB/s instead of 474: it uploads a frame
while the one before is on the wire. The transmit area of the chip, which the FreeRTOS
driver already used that way, refused room it had, and with a long receive buffer could
give a frame bytes of the one being sent.

### Fixed

* `spi_hz` near 20 MHz could make SPI periods shorter than the 50 ns that the chip
  allows. The PIO divides the system clock by a fraction, which stretches some of its
  clocks by one system clock, so with the system clock at 150 MHz (the default of the
  Pico 2) 20 MHz gave periods of 46.7 and 53.3 ns. On the bench, an RP2040 built for
  150 MHz got 9820 errors in 2000 rounds of data at 20 MHz, and none at 16 MHz or below.
  The driver now lowers the clock until every period is at least 50 ns: 18.75 MHz at a
  system clock of 150 MHz and 17.9 MHz at 125 MHz, where the same rounds came back
  clean. The clock is also never faster than asked for any more (the divider used to be
  rounded down). At 200 MHz, the default of arduino-pico for the Pico, nothing changes:
  20 MHz is exact there.
* An asynchronous call came back too early. `enc28j60_frame_rx()` reads the frame and
  then, in the same DMA list, moves the read pointers and decrements the packet counter;
  it returned as soon as the last byte of the frame was in, while those commands were
  still going out, and the next call started the DMA again under them. On the bench,
  3000 small frames received the way the FreeRTOS driver receives them gave 328 frame
  headers that made no sense, 138 DMA lists that never finished, and no frame intact;
  the chip received nothing afterwards. A call now returns when both the commands and
  what they read are through, and the same 3000 frames arrive intact.
* `enc28j60_frame_rx()` and `enc28j60_frame_discard()` wrote the next-packet pointer,
  which the hardware always makes even, to `ERXRDPT`, which the silicon errata (issue 14)
  say must be odd or the receive buffer may be corrupted. Read back from the chip after
  every frame, it was even 204 times out of 204; it is odd every time now.
* `enc28j60_frame_tx()` did not reset the transmit logic before a frame (issue 12); it
  does now, and also withdraws a request that an abort left pending.
* After a late collision the chip may raise TXERIF and never TXIF (issue 13, revisions B5
  and B7), and the FreeRTOS driver, which waited for TXIF, would never have sent another
  frame. `enc28j60_irq_is_tx()` is true for either flag now, and with an interrupt
  callback TXERIF drives the INT pin too. The frame itself is not sent again in this mode.
* `enc28j60_irq_flags()` returned PKTIF as the chip has it, which issue 6 says is not
  reliable, and the FreeRTOS driver reads a frame only when PKTIF is set. It comes from
  the packet counter now, as in `enc28j60_irq_flags_blk()`. After an overload of the
  receive buffer, 3 of 5 runs without it went wrong: once a frame of the stream that
  followed was missing, once frames of the overload turned up in the middle of it after
  more than a second of quiet. With it, 4 of 4 runs were clean.
* `enc28j60_deinit()` kept the three DMA channels, the shared DMA interrupt handler and
  the memory of the asynchronous mode, and the fourth start in a row stopped the
  firmware. Eight starts and stops now leave the same DMA channels claimed and the same
  heap free as before. `enc28j60_init_status()` returns `ENC28J60_ERR_DMA` instead of
  stopping the program when fewer than three DMA channels are free, and refuses a
  `dma_irq` other than 0 or 1.
* The FreeRTOS driver released the frame in progress whenever a transmit flag was set,
  even with none in progress, which would have broken the accounting of the transmit
  buffer. When the driver cannot start, it now says why (`enc28j60_status_string()`)
  instead of always blaming the wiring. This file is still not built by the CI; it was
  compiled for this change against the headers of FreeRTOS-Plus-TCP V4.3.4.
* The command macros of the C driver used their arguments without parentheses, so
  `BFC_CMD(ECON1, TXRTS | 0x3)` would have masked only the `0x3`. The firmware built from
  the code that uses them is the same, byte for byte.
* The transmit area of the chip gave a frame less room than there was when another frame
  was waiting or on the wire, which is how the FreeRTOS driver sends, and now the lwIP
  interface. A frame that would have ended right where the waiting one began was
  refused: with the default receive buffer, two of every three full-size frames waited
  for the wire. After a frame was refused at the end of the area, long frames were
  refused with nothing waiting, until a short one went out. And with a receive buffer of
  0x1420 or more, a frame could be given bytes of the one on the wire. On the bench
  (`txring`, 240 frames for each receive buffer, 2026-10-12): 159 uploads waited at
  0x1000; 3 frames were dropped at 0x1600 and at 0x1A00; at 0x145A, seven frames in a
  row made 34 uploads land on the frame on the wire, and with the SPI clock at 20 MHz
  those 34 frames went out corrupted (the chip sent them, the PC got 206 of 240). Now no
  upload waits at 0x1000, no frame is dropped and none lands on another; nor does any in
  a model of the area run with random frames at every even receive buffer size from
  0x0800 to 0x1A00 (32 million operations) and with every sequence of six operations at
  the edges (48 million).
* `extras/bench`: `--flash` and the web suites of `gate.sh` uploaded through the
  uploader of the core, which writes the image to every board in BOOTSEL: on 2026-10-12
  a second Pico that was put in BOOTSEL at that moment got the firmware of the bench
  instead of the board on `--port`. `extras/bench/deploy.py` restarts the board of the
  port and writes the image only to the BOOTSEL drive that appears on the same USB port
  of the PC.

### Added

* `enc28j60_irq_is_tx_err()` and the status `ENC28J60_ERR_DMA` in the C driver.
* `extras/bench`: an asynchronous mode of `BenchTest`, which gives the chip to the
  asynchronous functions and runs the interrupt branch of the FreeRTOS driver on them,
  and seven tests in `bench.py`: reception with `ERXRDPT` read back, frames dropped
  unseen, a stream of small frames with no pause between calls, UDP echo, a transmit
  stream, an overload, and eight starts and stops of the driver (under a watchdog, so
  that a driver that stops the firmware restarts the board).
* `extras/bench/gate.sh` runs every suite of the bench on a board, polled and with the
  INT pin, and writes a summary; a change to the driver, the PIO program or the lwIP
  interface goes in only after it passed.
* `extras/tools/check-commits.sh` checks every commit of a branch (formatting, links,
  four sketches built); `extras/tools/check-format.sh` is the formatting check of the CI,
  in one place.
* `extras/tools/pio-sim.py` simulates the SPI program of the PIO, as pioasm assembles
  it, and the CI checks with it that the chip select times and the clock period of the
  data sheet (table 16-6) hold at the fastest SPI clock. A copy of the program with a
  shorter chip select hold fails it.
* `udprate` in `extras/bench/benchlwip.py`: the board sends 500 UDP datagrams of 1472
  bytes with no pause and times them. Every one has to arrive intact and be counted by
  the driver, and at the default SPI clock a frame has to take less than one that is
  uploaded and then waited for.
* `enc28j60_frame_send_finish_blk()`, the second half of `enc28j60_frame_send_blk()`
  after `enc28j60_frame_tx_blk()`, so that the next frame can be uploaded while one is
  on the wire, and `enc28j60_frame_tx_ended_blk()`, which tells with one read whether it
  has ended.
* `txring` in `extras/bench/bench.py` sends through the C driver the way the lwIP
  interface does, each frame uploaded while the one before is on the wire, with receive
  buffers of several sizes.
* Built with `-DENC28J60_PROFILE`, the blocking functions of the C driver count their
  SPI commands and the microseconds they take (`enc28j60_profile_read()` in
  `src/enc28j60-cmd.h`). Without the define the code is the same as before.
* `cost` in `bench.py --flash --profile`, a suite of its own in `gate.sh`, fails when
  receiving, uploading or starting a frame takes more SPI commands than the code needs;
  `txfull` times 200 full-size frames through `sendFrame()`.
* `download` in `extras/bench/benchlwip.py`: the board reads 100 kB from the PC five
  times, and with the INT pin the PC must not wait for a timeout to send a segment again
  (its kernel counts them per connection from Linux 6.7).

### Changed

* `PicoENC28J60lwIP` puts a frame into the chip while the one before is still on the
  wire, and waits for that one only when the next frame is ready to start, or when the
  polling sees that it ended; `PicoENC28J60::sendFrame()` still waits for the wire. On
  the bench (SPI clock 10 MHz, 2026-10-12), 500 UDP datagrams of 1472 bytes with no
  pause went out at 787 kB/s polled and 804 kB/s with the INT pin (1.87 and 1.83 ms a
  frame), from 474 and 480 kB/s (3.10 and 3.07 ms), and a TCP upload of 100 kB from the
  board took 148-151 ms instead of 238-291 ms. `stats().tx_frames` counts such a frame
  once it has left the wire, and `tx_failures` one that fails after it was handed over.

## 1.2.0 - 2026-10-12

Robustness of the driver: it no longer hangs when the PIO does not run, shares the PIO
with the rest of the sketch, reports why it could not start, and follows more of the
silicon errata. The lwIP interface without the INT pin answers faster, and TCP no
longer stalls when a peer sends a window of full-size segments. The SPI link is more
than twice as fast.

Checked on the same bench as 1.1.0, with the arduino-pico core 6.3.0 and the SPI clock
at its new default of 10 MHz: the 28 tests of `bench.py` (31 with the INT pin), the 17
tests of `benchlwip.py` and the 8 HTTP tests of `webtest.py`, with and without the INT
pin, and 20 000 requests in a row to the web example in each mode, none of which failed.

### Added

* `ENC28J60Config::spi_hz` (`enc28j60_config.spi_hz` in the C driver) is the SPI clock,
  from 100 kHz to 20 MHz, the limit of the chip; 10 MHz by default. It used to be fixed
  at 3.5 MHz. The PIO program was rewritten for it: it takes 4 PIO clocks for each SPI
  clock instead of 10, and keeps the chip select timing that the data sheet asks for
  (table 16-6) at any clock. The old one let go of the chip select 143 ns after a read
  of the MAC and MII registers, where the data sheet asks for 210 ns, and less at a
  faster clock. On the bench a full-size frame is read in 1.3 ms at 10 MHz (3.7 ms at
  3.5 MHz with the new program, 0.7 ms at 20 MHz), and 2000 rounds of random data
  through every kind of access came back intact at every clock from 1 to 20 MHz.
* `PicoENC28J60lwIP::droppedNoBuffer()` counts the frames that lwIP had no buffer for and
  dropped.
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
  by `begin()`, frames of all lengths with the longest receive buffer, a chip that was
  reset behind the back of the driver, and the rest of the public API that nothing
  exercised: `setMACAddress()`, `discardFrame()` and the functions that only look,
  `hasLinkChange()`, the interrupt callback and `poll()`, an overflow of the receive buffer
  on purpose, and the SPI clock. `--spi-hz` runs every test at another clock.
* `extras/bench/BenchLwip` and `benchlwip.py`: tests of the lwIP interface on the
  hardware, with the PC on the other side: DHCP, ping both ways, TCP and UDP servers,
  a TCP client, bulk transfers through three connections at once, DNS, a static address,
  `end()` followed by `begin()`, fragmented datagrams, chained buffers, a chip that was
  reset behind the back of the interface, and the cable being pulled out and put back.
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

* TCP connections stalled when a peer sent a window of full-size segments. The core puts
  each frame it receives in one block of the heap of lwIP, which has 16 KB in the core.
  When a segment went missing, the ones behind it waited out of order and took the heap,
  and the missing one, sent again, found no block of 1514 bytes: it was dropped every
  time, and the connection stalled until a timeout of the peer ended it. The interface
  receives every frame in buffers from the pool of lwIP now, which are not taken from
  that heap. On the bench, three connections at once, each echoing 64 kB, stalled at
  least one of them in each of the four attempts (a round took 49 s in the end); now the
  slowest of two rounds takes 2 to 5 seconds, with a few frames that lwIP had no buffer
  for (`droppedNoBuffer()`) that the peer sends again. A single connection echoing
  200 kB went from 49 kB/s to about 180 kB/s. The slower clock hid the problem for a
  single connection (80 transfers of 64 kB in a row did not stall at 3.5 MHz): the
  buffer of the chip overflowed before the heap did. With a faster clock 1 of the first
  12 stalled at 8 MHz and 3 of the first 25 at 16 MHz.
* The driver never claimed its PIO state machine. The Arduino core gives the first free
  one to `SerialPIO` and the like; it fills `pio1` first, so with five `SerialPIO`
  objects the fifth landed on the state machine of the driver and every call to the
  chip then waited for ever. The state machine is claimed now, and the others get
  another one.
* A blocking SPI transfer waited for ever when the PIO did not run. It now gives up
  after about 8 ms without progress, reads zeros, and the class recovers (see above).
* `pio_add_program()` returns a negative number when the instruction memory is full;
  that was taken for an offset.
* Without the INT pin, `hasLinkChange()` and `INT_LINK_CHANGE` never reported a change of the link
  (so the link message of `BasicTest` never appeared), and `interruptPending()` ignored it. The chip
  raises its link flag only when the PHY interrupt is on (data sheet, section 12), and the library
  switched that on only when `irq_pin` was set. `begin()` and the recovery after a failure do it
  either way now. On the bench the PHY was powered down and up again: without the change
  `hasLinkChange()` stayed false both times, with it both changes were seen and cleared by
  `acknowledgeInterrupts()`.
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

* The SPI clock is 10 MHz by default, where it was 3.5 MHz. The errata (issue 1) ask for
  8 MHz or more: the MAC registers of the revisions B1 and B4 are unreliable below it.
  The PIO divides the system clock by a fraction, so at 125 MHz an 8 MHz clock has
  periods of 120 and 128 ns, mostly the longer one, and runs below 8 MHz most of the
  time; at 10 MHz they are 96 and 104 ns, all shorter than the 125 ns of 8 MHz. If your
  wiring cannot take it, set `ENC28J60Config::spi_hz` lower. On the bench the round trip
  of a UDP echo went from 4.4 to 2.4 ms, a page of the web example from 14 to 11 ms
  (median), and the receive flood of `bench.py` got 960 of its 3000 frames through the
  buffer of the chip instead of 384. At 20 MHz the whole of `bench.py` and
  `benchlwip.py` passed, the echo taking 1.8 ms.
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
* `PicoENC28J60::INT_GLOBAL` is deprecated: 0x80 is the enable bit of the INT output
  (`EIE.INTIE`), `EIR` has no such bit and no function returned it.
* `PicoENC28J60lwIP` puts the pieces of a chain of buffers together before sending, instead
  of sending the first one only. The core never hands over a chain, so this is a guard.
* `hasPacket()` is one read of the packet counter.
* `README.md` is about the Arduino library from the first line on: a wiring table, the
  configuration, a section on what the library does when things go wrong and what it
  does not do, and one on testing. The documentation of the C driver and of the FreeRTOS
  driver, which Lukasz Janyst wrote and which used to take half of it, is in
  `docs/c-driver.md` now, with the functions this fork added to the driver.

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
