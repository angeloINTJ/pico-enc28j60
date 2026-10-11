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
 * The SPI GPIOs are driven by a PIO program that is 17 instructions long and
   uses 1 state machine. The hardware SPI is not used because the device
   commands are variable length.
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

The library is not in the Arduino Library Manager and there is no GitHub Release
yet, so install it from this repository:

 * Arduino IDE: *Code > Download ZIP* on GitHub, then *Sketch > Include Library >
   Add .ZIP Library*.
 * From a clone, which makes it easy to stay up to date: clone the repository
   into your Arduino libraries folder (usually `~/Arduino/libraries`).
 * PlatformIO with the arduino-pico core:
   `lib_deps = https://github.com/angeloINTJ/pico-enc28j60`

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
        Serial.println("No ENC28J60 found: check the wiring and the 3.3 V supply");
        while (true) delay(1000);
    }
    Serial.printf("revision 0x%02X, link %s\n", eth.getRevision(), eth.linkStatus() ? "up" : "down");
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
`eth.begin(config)`. Interrupt mode (INT wired to a GPIO) is shown in
`examples/AdvancedTest`; it has not been exercised on the bench.

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

 * pio - a PIO instance to be used for SPI: pio0 or pio1
 * sm - index to the state machine to run the PIO program
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
   `false` if no ENC28J60 answers, and can be called again.
 * `bool linkStatus()`, `uint8_t getRevision()`
 * `void getMACAddress(uint8_t* mac) const`, `void setMACAddress(const uint8_t* mac)`
 * `bool sendFrame(const uint8_t* frame, uint16_t len)` - blocking, up to
   `MAX_FRAME_SIZE` (1514) bytes, without the FCS.
 * `uint8_t available()` - frames waiting in the receive buffer.
 * `uint16_t receiveFrame(uint8_t* buffer, uint16_t max_len)` - length without
   the FCS, 0 if there is none. `void discardFrame()` drops the next frame.
 * Interrupt flags: `getInterruptFlags()`, `hasPacket()`, `hasTXComplete()`,
   `hasLinkChange()`, `hasRXError()`, `acknowledgeInterrupts()`,
   `setInterruptCallback()`, `poll()`.
 * `const Stats& stats()` - frame counters, for diagnostics.
 * `void* getDriverHandle()` - the handle of the C driver.

### PicoENC28J60lwIP

Same API as the other wired Ethernet interfaces of the core (`begin()`,
`config()`, `connected()`, `localIP()`, `linkStatus()`, ...), plus
`setPollPeriod(ms)` and `driverStats()`. Do not call the raw driver methods
while it runs. Tested on a Pico (RP2040) only. Not handled yet: reacting to the
cable being unplugged and plugged back (no DHCP renewal on a link change).

---

## Examples
---------

See the examples/ folder:
- WebServerLedToggle - a web page with a Toggle LED button (lwIP, DHCP). Checked
  with a real browser on the bench.
- BasicTest - raw frames: initialization, link, receive and print, send a test frame
- AdvancedTest - interrupt mode (needs INT wired), custom MAC, frame parsing
- DhcpClient - experimental raw-frame DHCP client written before the receive
  path was fixed and not tested since. Prefer PicoENC28J60lwIP for DHCP.

---

## Credits
-------

This library is based on pico-enc28j60 by Lukasz Janyst (lukasz@jany.st)
- Original repository: https://github.com/ljanyst/pico-enc28j60
- Licensed under MIT License

The PIO SPI implementation and ENC28J60 register-level driver are entirely
Lukasz's work. This repository adds:
- Arduino-style C++ class wrapper (PicoENC28J60)
- Flexible pin configuration including separate CS pin support
- Arduino build system integration
- Examples and documentation for Arduino users

**All original C driver code, PIO programs, FreeRTOS integration, and documentation are preserved.**

---

## References
==========

 * [ENC28J60 Data Sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/39662e.pdf)
 * [ENC28J60 Errata](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/Errata/80349c.pdf)

---

## License
-------

MIT License - See LICENSE file for details.
