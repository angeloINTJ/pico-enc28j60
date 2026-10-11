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

## Arduino IDE Support (New)
-------------------------

This repository includes an **Arduino library wrapper** (src/PicoENC28J60.h/.cpp)
that provides a C++ class interface while using the original C driver underneath.
All original functionality is preserved.

### Installation

#### Arduino IDE Library Manager
Search for "PicoENC28J60" in the Library Manager.

#### Manual Installation
1. Download this repository as ZIP from GitHub Releases
2. In Arduino IDE: Sketch -> Include Library -> Add .ZIP Library
3. Select the downloaded ZIP file

#### PlatformIO
lib_deps = 
    https://github.com/angeloINTJ/pico-enc28j60

#### Arduino CLI
arduino-cli lib install PicoENC28J60

### Quick Start (Arduino)

```cpp
#include <PicoENC28J60.h>

// Optional: custom pin configuration
ENC28J60Config config;
config.mosi_pin = 3;   // GP3
config.miso_pin = 4;   // GP4
config.sclk_pin = 2;   // GP2
config.cs_pin = 5;     // GP5 (separate CS pin support)
config.rst_pin = 7;    // GP7
config.irq_pin = -1;   // -1 = polling mode

PicoENC28J60 eth(config);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("ENC28J60 Test");
    
    if (!eth.begin()) {
        Serial.println("Failed to initialize ENC28J60!");
        while (1) delay(1000);
    }
    
    Serial.print("Revision: 0x");
    Serial.println(eth.getRevision(), HEX);
    
    uint8_t mac[6];
    eth.getMACAddress(mac);
    Serial.print("MAC: ");
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial.print("0");
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    Serial.print("Link: ");
    Serial.println(eth.linkStatus() ? "UP" : "DOWN");
}

void loop() {
    // Poll for interrupts (required in polling mode)
    eth.poll();
    
    // Check for received packets
    if (eth.hasPacket()) {
        uint8_t buffer[1520];
        uint16_t len = eth.receiveFrame(buffer, sizeof(buffer));
        if (len > 0) {
            Serial.printf("Received %d bytes\n", len);
            // Process frame...
        }
    }
    
    // Send a test frame periodically
    static uint32_t last_send = 0;
    if (millis() - last_send > 5000) {
        last_send = millis();
        
        // Simple ARP request frame
        uint8_t frame[60] = {0};
        memset(frame, 0xFF, 6);  // Broadcast destination
        eth.getMACAddress(frame + 6);  // Source MAC
        frame[12] = 0x08; frame[13] = 0x06;  // ARP
        
        if (eth.sendFrame(frame, 60)) {
            Serial.println("Frame sent");
        }
    }
    
    delay(10);
}
```

### Advanced Arduino Usage

#### Interrupt Mode
```cpp
volatile bool packetReceived = false;

void irqHandler(void* userData) {
    PicoENC28J60* eth = static_cast<PicoENC28J60*>(userData);
    uint8_t flags = eth->getInterruptFlags();
    if (flags & PicoENC28J60::INT_PACKET_RX) packetReceived = true;
    eth->acknowledgeInterrupts(flags);
}

void setup() {
    // ... initialization ...
    
    ENC28J60Config config;
    config.irq_pin = 6;  // Connect ENC28J60 INT to GP6
    PicoENC28J60 eth(config);
    eth.setInterruptCallback(irqHandler, &eth);
    eth.begin();
}

void loop() {
    if (packetReceived) {
        packetReceived = false;
        // Process packets...
    }
}
```

#### Custom MAC Address
```cpp
ENC28J60Config config;
uint8_t my_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
memcpy(config.mac_addr, my_mac, 6);
PicoENC28J60 eth(config);
```

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
 * sclk_pin - defaults to 18, and sclk_pin + 1 = 19 is used for Slave Select
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

### PicoENC28J60(ENC28J60Config config = ENC28J60Config())
Constructor with optional configuration.

### bool begin()
Initialize the ENC28J60. Returns true on success.

### bool linkStatus()
Check if Ethernet link is up.

### uint8_t getRevision()
Get hardware revision ID (0x06 for ENC28J60).

### void getMACAddress(uint8_t* mac)
Get current MAC address (6 bytes).

### void setMACAddress(const uint8_t* mac)
Set MAC address (6 bytes, must be locally administered).

### bool sendFrame(const uint8_t* frame, uint16_t len)
Send an Ethernet frame. Returns true if queued successfully.

### uint8_t available()
Get number of frames in RX buffer.

### uint16_t receiveFrame(uint8_t* buffer, uint16_t max_len)
Receive next frame. Returns number of bytes read, 0 if none available.

### void discardFrame()
Discard the next frame in RX buffer.

### uint8_t getInterruptFlags()
Get raw interrupt flags.

### bool hasPacket() / hasTXComplete() / hasLinkChange() / hasRXError()
Convenience methods to check specific interrupts.

### void acknowledgeInterrupts(uint8_t flags)
Acknowledge and clear interrupt flags.

### void setInterruptCallback(void (*callback)(void*), void* user_data = nullptr)
Set interrupt callback function. Pass NULL to disable.

### void poll()
Poll for interrupts. Must be called regularly in polling mode.

---

## Examples
---------

See the examples/ folder:
- BasicTest - Simple initialization and packet loop (polling mode)
- AdvancedTest - Interrupt mode, custom MAC, frame parsing

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
