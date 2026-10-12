# pico-enc28j60

The code in this repository implements an ENC28J60 driver for Raspberry Pi Pico.
The implementation is split into two libraries. A base library that provides the
infrastructure necessary for communication with the device, and a driver for the
FreeRTOS TCP stack. If you happen to use a different stack, you can easily write
your own driver using the base library and taking the FreeRTOS driver as an
example.

**This repository also includes an Arduino IDE compatible library wrapper** (PicoENC28J60)
that provides a C++ class interface while preserving all the original C driver functionality.

---

## RP2040 resources used
---------------------

The base driver uses:

 * 5 GPIOs for communication with the device: 4 for SPI and 1 for the interrupt
   signal.
 * The SPI GPIOs are driven by a PIO program that is 18 instructions long and
   uses 1 state machine, which the driver claims so that other users of the PIO
   (SerialPIO, tone, Servo, ...) are given another one. The hardware SPI is not
   used because the device commands are variable length.
 * In non-blocking mode, the driver uses 3 DMA channels: 1 for data reception
   and two for scatter-gather transmission.

The FreeRTOS driver spawns a task and requires two task notifications to
function: one for device interrupt and one for DMA completion interrupt.

---

## ENC28J60 Hardware
-----------------

ENC28J60 is a 3.3V chip and there's a bunch of breakout boards available for it
that can be used directly with Pico.

![ENC28J60 Board](imgs/enc28j60-board.webp)

There's also an Arduino Nano shield designed for a 5V operation. Coincidentally,
that's the board that has been used for the development of this driver, but it
needs to be modified before it can be used with Pico.

![ENC28J60 Arduino Nano Shield](imgs/enc28j60-nano.webp)

The AMS1117 voltage regulator (marked blue) used to drop the supply voltage to
3.3V needs to be removed, as we are going to use the Pico's 3.3V output to
supply the device directly. The MM74HCT08M AND gate (marked green) used as a
buffer to raise the voltage of the MOSI and interrupt signals to 5V needs to be
removed as well because Pico can't tolerate GPIO input voltage this high. The
connectors for Vin and Vout pins of the voltage regulator need to be shorted
(marked blue on the picture below) and so do the inputs and outputs of the
AND gates responsible for the MOSI and IRQ signals (marked green).

![Modified ENC28J60 Arduino Nano Shield](imgs/enc28j60-nano-moded.webp)

The relevant pins on this board are:

 * 5V - where we connect the Pico's 3.3V output
 * GND - for ground, all of them are connected and will work fine
 * D13 - SPI clock
 * D12 - MISO
 * D11 - MOSI
 * D10 - Slave Select
 * D2 - Interrupt

---

## Arduino IDE Support
--------------------

The Arduino library (`PicoENC28J60`) targets the Raspberry Pi Pico (RP2040) with the
[arduino-pico](https://github.com/earlephilhower/arduino-pico) core. It has two layers:

 * `PicoENC28J60.h` - a raw Ethernet frame driver. You send and receive whole
   frames. The SPI link is a PIO program, so any free pins will do.
 * `PicoENC28J60lwIP.h` - the same driver as a network interface of the lwIP
   stack in the core: DHCP, TCP and UDP sockets, `WiFiClient`, `WiFiServer`,
   `WebServer` and everything else built on lwIP. Header only.

### Installation

Install it from this repository; the Arduino Library Manager does not list it yet:

 * Arduino IDE: download `PicoENC28J60-<version>.zip` from the
   [latest release](https://github.com/angeloINTJ/pico-enc28j60/releases/latest),
   then *Sketch > Include Library > Add .ZIP Library*.
 * From a clone, which makes it easy to stay up to date: clone the repository
   into your Arduino libraries folder (usually `~/Arduino/libraries`).
 * PlatformIO with the arduino-pico core:
   `lib_deps = https://github.com/angeloINTJ/pico-enc28j60.git#v1.1.0`

What changed in each version is in [CHANGELOG.md](CHANGELOG.md).

### Quick start: a web page that toggles the LED

`examples/WebServerLedToggle` gets an address from DHCP and serves a page with a
*Toggle LED* button. The network part is just:

```cpp
#include <PicoENC28J60lwIP.h>

PicoENC28J60lwIP eth;  // default pins: SCK=GP2 MOSI=GP3 MISO=GP4 CS=GP5 RST=GP7

void setup() {
    eth.begin();                        // DHCP
    while (!eth.connected()) delay(100);
    // from here on use WiFiClient, WiFiServer, WebServer... as on any interface
}
```

### Quick start: raw frames

```cpp
#include <PicoENC28J60.h>

PicoENC28J60 eth;  // default pins

void setup() {
    Serial.begin(115200);
    if (!eth.begin()) {
        // say why: wrong pins, no free PIO state machine, no chip answering...
        Serial.println(PicoENC28J60::errorString(eth.lastError()));
        while (true) delay(1000);
    }
    Serial.printf("revision 0x%02X\n", eth.getRevision());
}

void loop() {
    uint8_t frame[PicoENC28J60::MAX_FRAME_SIZE];
    while (eth.available()) {
        uint16_t len = eth.receiveFrame(frame, sizeof(frame));  // length without the FCS
        if (len) Serial.printf("received %u bytes\n", len);
    }
}
```

Other pins or MAC address: fill in an `ENC28J60Config` and pass it to
`eth.begin(config)`. By default the library picks any PIO block with room for its
program and a free state machine; `pio_instance` and `pio_sm` choose them yourself.

### Using the INT pin

Wire the chip's INT pin to a free GPIO and give its number as `irq_pin`. The chip
pulls the line low when a frame arrives, the link changes or its receive buffer
overflows. `loop()` then asks `interruptPending()`, which only looks at that line
and costs no SPI traffic, and talks to the chip when the answer is yes:

```cpp
ENC28J60Config config;
config.irq_pin = 6;                       // INT on GP6
PicoENC28J60 eth(config);                 // eth.begin() in setup()

void loop() {
    if (eth.interruptPending()) {
        uint8_t flags = eth.getInterruptFlags();      // also masks the INT pin
        while (eth.available()) { /* eth.receiveFrame(...) */ }
        if (flags & PicoENC28J60::INT_LINK_CHANGE) { /* eth.linkStatus() */ }
        eth.acknowledgeInterrupts(flags);             // clears them, unmasks the pin
    }
}
```

The same loop works without the wire (`irq_pin` left at -1): `interruptPending()`
then reads the chip's flags instead. The library sets the pin up with the
internal pull-up and attaches the interrupt through the Arduino core, so your own
`attachInterrupt()` calls keep working.

An interrupt handler registered with `setInterruptCallback()` runs in interrupt
context and must not call this class: the chip is reached over one PIO state
machine, and a transfer started from the handler in the middle of one of the main
program corrupts both. Set a flag there and use `interruptPending()` in `loop()`.
See `examples/AdvancedTest`. With `PicoENC28J60lwIP` the same `irq_pin` makes the
network stack serve a frame the moment it arrives, see below.

### Hardware test

`extras/bench` has a test that runs against a real board and your LAN: ping,
receive and echo streams, transmit streams, a long soak and a receiver overload.
See `extras/bench/README.md`.

---

## Using in a CMake Project (Original)
------------------

Simply add this repository as a submodule and include it in your master
CMakeLists.txt as follows:

```
set(ENABLE_ENC28J60_FREERTOS_DRIVER FALSE)
add_subdirectory(lib/pico-enc28j60/src)
```

The FreeRTOS driver may be switched on or off.

---

## Base driver setup (Original C API)
-----------------

Driver can be configured through an enc28j60_config instance. It can be
created and initialized to defaults using enc28j60_get_default_config(). The
struct has the following fields:

 * pio - a PIO instance to be used for SPI: pio0 or pio1, or NULL (the default) to
         let the driver look for any one with room and a free state machine
 * sm - index to the state machine to run the PIO program, or ENC28J60_SM_ANY (the
        default); the state machine is claimed
 * miso_pin - defaults to 16
 * mosi_pin - defaults to 17
 * sclk_pin - defaults to 18
 * cs_pin - slave select, defaults to 19
 * irq_pin - defaults to 20
 * dma_irq - interrupt channel to handle DMA IRQs, defaults to 0
 * rx_size - the device has 8kB big buffer that is used for RX and TX queues
               the range from 0 to rx_size - 1 will be used for RX, the
               reminder for TX; defaults to 0x1000
 * mac_addr - MAC address of the device
 * irq_cb - callback function for the device interrupts
 * irq_data - data for the callback
 * notify - notification helper described below

The DMA notification struct looks as follows:

```c
typedef struct {
    void (*notify)(void *);
    void (*wait)(void *);
    void *data;
} enc28j60_notification;
```

The notify function is called in an interrupt context to wake a task that is
going to service the interrupt, while wait blocks that task until an interrupt
has been raised. data is passed to both functions when they are called.

---

## FreeRTOS setup (Original)
--------------

When using the FreeRTOS driver, especially in a multicore setup, it is important
to ensure the consistency of heap operations. Consider adding
PICO_USE_MALLOC_MUTEX to your global defines:

```cmake
add_definitions(
  -DPICO_USE_MALLOC_MUTEX
)
```

and configuring the FreeRTOS-Pico sync interop in your FreeRTOSConfig.h:

```c
#define configSUPPORT_PICO_SYNC_INTEROP 1
```

The FreeRTOS driver is normally initialized the following way assuming that most
of the driver configuration options are left at their defaults:

```c
#include <pico/enc28j60-driver.h>
#include <pico/enc28j60-freertos-driver.h>

enc28j60_freertos_interface iface;
static NetworkEndPoint_t endpoint_v4;
static NetworkEndPoint_t endpoint_v6_link;

void setup_network()
{
    memset(&iface, 0, sizeof(enc28j60_freertos_interface));
    enc28j60_config *drv_cfg = enc28j60_get_default_config();

    // Choose a MAC address
    memcpy(drv_cfg->mac_addr, mac_addr, 6);

    // Interface setup
    iface.iface.pcName = "e0";
    iface.iface.pvArgument = drv_cfg;
    iface.iface.pfInitialise = enc28j60_freertos_initialize;
    iface.iface.pfOutput = enc28j60_freertos_output_frame;
    iface.iface.pfGetPhyLinkStatus = enc28j60_freertos_get_phy_link_status;
    FreeRTOS_AddNetworkInterface(&iface.iface);

    // IPv4
    memset(&endpoint_v4, 0, sizeof(endpoint_v4));
    // The ip, net_mask, and gateway below may just be zeros if DHCP is used
    FreeRTOS_FillEndPoint(&iface.iface, &endpoint_v4, (uint8_t *)&ip,
                          (uint8_t *)&net_mask, (uint8_t *)&gateway,
                          (uint8_t *)&any_ip, cfg->mac_addr);

    endpoint_v4.bits.bWantDHCP = pdTRUE;

    // IPv6 Link Scope endpoint. A global unicast address requires another
    // endpoint.
    memset(&endpoint_v6_link, 0, sizeof(endpoint_v6_link));
    IPv6_Address_t prefix;
    IPv6_Address_t address;
    FreeRTOS_inet_pton6("fe80::", &prefix);
    memcpy(&address, cfg->ipv6_addr_link, 16);

    FreeRTOS_FillEndPoint_IPv6(&iface.iface, &endpoint_v6_link, &address,
                               &prefix, 64, NULL, NULL, cfg->mac_addr);

    FreeRTOS_IPInit_Multi();
}
```

---

## Arduino API Reference
-------------------

### PicoENC28J60

 * `PicoENC28J60(const ENC28J60Config& config = ENC28J60Config())` - pins, MAC
   address and buffer split live in `ENC28J60Config`.
 * `bool begin()`, `bool begin(const ENC28J60Config&)` - initialize. Returns
   `false` when it cannot, and can be called again; `lastError()` says why (an
   `Error` value) and `PicoENC28J60::errorString(error)` turns it into text: a pin
   that is out of range or used twice, a state machine that is taken, no PIO with
   room for the program, no chip answering, no memory.
 * `void end()` - stops the chip and gives back the state machine, the pins and the
   memory; `begin()` starts it again. The destructor calls it.
 * `bool linkStatus()` - the link takes a second or two to come up after `begin()`.
 * `uint8_t getRevision()`
 * `void getMACAddress(uint8_t* mac) const`, `void setMACAddress(const uint8_t* mac)`
 * `bool sendFrame(const uint8_t* frame, uint16_t len)` - blocking, up to
   `MAX_FRAME_SIZE` (1514) bytes, without the FCS.
 * `uint8_t available()` - frames waiting in the receive buffer.
 * `uint16_t receiveFrame(uint8_t* buffer, uint16_t max_len)` - length without
   the FCS, 0 if there is none. `void discardFrame()` drops the next frame.
 * INT pin (`irq_pin`): `interruptPending()` (the pin, no SPI traffic),
   `getInterruptFlags()` (reads the flags and masks the pin),
   `acknowledgeInterrupts(flags)` (clears them and unmasks the pin),
   `interruptCount()`. Looking without touching the pin: `peekInterruptFlags()`,
   `hasPacket()`, `hasLinkChange()`, `hasRXError()` (`hasTXComplete()` is of
   little use: `sendFrame()` already waits for the frame to leave the wire).
   `setInterruptCallback()` and `poll()` are for a handler that only sets a flag.
   `hasPacket()` and `INT_PACKET_RX` come from the packet counter of the chip,
   because its packet flag is not reliable (silicon errata, issue 6).
 * `const Stats& stats()` - frame counters, for diagnostics: frames in and out,
   `tx_failures`, `tx_aborts` (the MAC gave up), `tx_retries` (a late collision
   was retried), `io_errors` and `recoveries` (see below).
 * `void* getDriverHandle()` - the handle of the C driver.

### PicoENC28J60lwIP

Same API as the other wired Ethernet interfaces of the core (`begin()`,
`config()`, `connected()`, `localIP()`, `linkStatus()`, ...), plus
`setPollPeriod(ms)` and `driverStats()`. Do not call the raw driver methods
while it runs. Tested on a Pico (RP2040) only. When the cable is unplugged and
plugged back, lwIP is told and asks the DHCP server again.

Give it a config with `irq_pin` set and it is driven by the INT line: the
interrupt handler only wakes the lwIP context up, which then reads the frames, so
the chip is never touched from interrupt context. On the bench that took the median
time of a `GET /` from 16 ms to 11 ms and made an idle network free of SPI
traffic. The timer of `setPollPeriod()` remains as a safety net (20 ms then).

Without the INT pin the chip is polled: every 5 ms while the line is quiet, and
every millisecond right after a frame went out or came in, backing off to 5 ms
again when nothing arrives. The peer's answer to a frame is usually on its way,
and a fixed period made the time of a request depend on where the timer happened to
fall (13 ms or 18 ms for the same page on the bench); with the adaptive period it
is 10 ms.

---

## Robustness and limitations
---------------------------

What the library does when things go wrong:

 * **A PIO that does not run.** Another piece of code taking the state machine, or
   stopping it, would leave a blocking SPI transfer waiting forever. Transfers give
   up after about 8 ms without progress and read zeros; the class counts it
   (`stats().io_errors`), restarts the state machine and the chip, and counts that
   too (`stats().recoveries`), trying again every second while the chip does not
   answer. Frames in the chip's buffers are lost in the process.
 * **A frame the MAC gives up on** after a late collision, which some link partners
   provoke in half duplex (silicon errata, issue 13), is sent again, up to 15
   times (`stats().tx_retries`). Any other abort is final and `sendFrame()`
   returns `false` (`stats().tx_aborts`).
 * **A receive buffer that overflows** loses frames; `INT_RX_ERROR` tells you.

What it does not do:

 * It is 10 Mbit/s and **half duplex** only, as the chip is. The link partner has
   to detect that by itself, which every switch and router does.
 * SPI runs at 3.5 MHz, so a full-size frame takes about 3.5 ms to move between the
   chip and the Pico, and the throughput is well below the 10 Mbit/s of the wire.
 * `sendFrame()` and the receiving functions block while they talk to the chip;
   they are not meant to be called from interrupt handlers.
 * **Tested on** a Raspberry Pi Pico (RP2040) with an ENC28J60 revision B7 and the
   arduino-pico core 6.3.0. The sketches also compile for the Pico W, Pico 2 and
   Pico 2 W, but have not been run on them. The FreeRTOS+TCP driver is the one of
   the original project, and was not built or run here.
 * The receive buffer of the chip is small (4 KB, 6.5 KB at most with
   `rx_buffer_size = 0x1A00`; the transmit side only needs one frame) and reading a
   frame from it takes 3.5 ms, while one can arrive every 1.2 ms. A peer that sends
   a window of data at once, like several bulk TCP transfers at the same time,
   overflows it, and TCP recovers slowly: one connection echoes 200 kB at about
   50 kB/s on the bench, and requests and answers are fine.

---

## Examples
---------

See the examples/ folder:
- WebServerLedToggle - a web page with a Toggle LED button (lwIP, DHCP). Checked
  with a real browser on the bench.
- BasicTest - raw frames: initialization, link, receive and print, send a test frame
  (a broadcast with the experimental EtherType 0x88B5, never an ARP or ICMP that
  could confuse a real network)
- AdvancedTest - interrupt mode (needs INT wired to GP6), custom MAC, frame parsing.
  Checked on the bench.

---

## Repository layout
-----------------

| Folder | Contents |
|--------|----------|
| `src/` | The library: the Arduino classes (`PicoENC28J60.*`, `PicoENC28J60lwIP.h`) and the C driver they are built on (`enc28j60-*`, `pico/`) |
| `examples/` | Arduino sketches |
| `extras/bench/` | Tests that run on the hardware; not compiled by the Arduino IDE |
| `freertos/` | The FreeRTOS+TCP driver of the original project, outside `src/` so that the Arduino IDE does not compile it |
| `imgs/` | Photographs used by this README |

---

## Reporting problems
-------------------

Open an [issue](https://github.com/angeloINTJ/pico-enc28j60/issues). Say which board
and which version of the arduino-pico core you use, how the chip is wired (the
`ENC28J60Config` you pass), and, if you can run it, what `extras/bench/bench.py` prints.

---

## References
----------

 * [ENC28J60 Data Sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/39662e.pdf)
 * [ENC28J60 Errata](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/Errata/80349c.pdf)

---

## Credits and license
-------------------

PicoENC28J60 is a fork of [pico-enc28j60](https://github.com/ljanyst/pico-enc28j60)
by **Lukasz Janyst**, and his work is the foundation of everything here: the PIO
program that speaks SPI to the chip, the ENC28J60 driver in C, the FreeRTOS+TCP
driver, the documentation of that driver in this README and the photographs in
`imgs/`. They are included with his copyright notice and under the MIT license he
published them with. His 52 commits are part of the history of this repository too.

**Ângelo Moisés Alves** ([@angeloINTJ](https://github.com/angeloINTJ)) adapted it
to the Arduino IDE: the C++ classes, the lwIP network interface, the examples, the
on-hardware test bench, and the changes to the original driver that these needed
(a chip select pin of its own, a driver that fails instead of hanging when no chip
answers, fixes in the blocking receive and transmit paths, the INT pin).

[NOTICE.md](NOTICE.md) says who wrote what, file by file. The project is released
under the [MIT license](LICENSE), which carries the copyright notice of both.
