# pico-enc28j60

High-performance ENC28J60 Ethernet driver for Raspberry Pi Pico (RP2040) using PIO and DMA.

This library provides both a **low-level C API** for Pico SDK/CMake projects and an **Arduino-compatible C++ wrapper** for use with Arduino IDE.

## Features

- **Hardware-accelerated SPI** via PIO (Programmable I/O) - no bit-banging overhead
- **DMA transfers** for zero-copy packet transmission/reception
- **Interrupt-driven** operation support
- **FreeRTOS+TCP** integration (via pico_enc28j60_freertos_driver)
- **Arduino IDE compatible** with C++ wrapper class
- **Configurable pins** - use any GPIOs for SPI/IRQ

## Hardware Requirements

- Raspberry Pi Pico (RP2040) or compatible board
- ENC28J60 Ethernet module (3.3V version)
- 5 GPIO pins: 4 for SPI (MISO, MOSI, SCK, CS) + 1 for Interrupt

### Default Pin Mapping (Pico)

| Function | GPIO | Notes |
|----------|------|-------|
| MISO | 16 | |
| MOSI | 17 | |
| SCK | 18 | |
| CS | 19 | SCK + 1 |
| IRQ | 20 | Connect to ENC28J60 INT pin |

### Wiring Diagram

```
ENC28J60 Module          Raspberry Pi Pico
-------------           ------------------
VCC      ------------>  3V3 (Pin 36)
GND      ------------>  GND
SCK      ------------>  GP18 (Pin 24)
MISO     ------------>  GP16 (Pin 21)
MOSI     ------------>  GP17 (Pin 22)
CS       ------------>  GP19 (Pin 25)
INT      ------------>  GP20 (Pin 26)
```

> **Note:** ENC28J60 is a 3.3V device. Do NOT connect to 5V. If using a 5V Arduino Nano shield, you must modify it (remove voltage regulator and level shifters).

## Installation

### Arduino IDE (Recommended)

1. **Install Arduino Core for RP2040:**
   - Open Arduino IDE -> Preferences -> Additional Boards Manager URLs
   - Add: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Tools -> Board -> Boards Manager -> Search "pico" -> Install "Raspberry Pi Pico/RP2040"

2. **Install Library:**
   - **Option A:** Download ZIP -> Sketch -> Include Library -> Add .ZIP Library
   - **Option B:** Clone to `Documents/Arduino/libraries/pico-enc28j60`
   - **Option C:** Library Manager (coming soon)

3. **Select Board:** Tools -> Board -> Raspberry Pi Pico/RP2040 -> Raspberry Pi Pico

### Pico SDK / CMake

For native Pico SDK projects:

```cmake
# In your CMakeLists.txt
set(ENABLE_ENC28J60_FREERTOS_DRIVER FALSE)  # or TRUE for FreeRTOS
add_subdirectory(lib/pico-enc28j60/src)
target_link_libraries(your_target PRIVATE pico_enc28j60_driver)
```

## Quick Start (Arduino)

```cpp
#include <Arduino.h>
#include <enc28j60.h>

// MAC address (use locally administered address)
uint8_t mac[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};

ENC28J60 eth;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Initialize with default pins
    if (!eth.begin(mac)) {
        Serial.println("Failed to initialize!");
        return;
    }

    Serial.print("Revision: 0x");
    Serial.println(eth.getRevision(), HEX);
    Serial.println(eth.linkStatus() ? "Link: UP" : "Link: DOWN");
}

void loop() {
    // Check for packets
    if (eth.packetAvailable() > 0) {
        uint8_t buffer[1518];
        uint16_t len = eth.receivePacket(buffer, sizeof(buffer));
        if (len > 0) {
            // Process packet...
        }
    }
    
    // Send packet
    // eth.sendPacket(data, size);
    
    delay(10);
}
```

## Arduino API Reference

### Class: ENC28J60

#### Initialization

```cpp
// Default pins (GP16,17,18,19,20)
bool begin(const uint8_t* mac);

// Custom pins
bool begin(const uint8_t* mac, 
           int miso_pin, int mosi_pin, int sclk_pin, int irq_pin,
           PIO pio_instance = pio0, uint sm_index = 0);

// Advanced: use existing C config
bool begin(enc28j60_config* config);

// Cleanup
void end();
bool isInitialized() const;
```

#### Status & Configuration

```cpp
uint8_t getRevision() const;           // Hardware revision (0-7)
bool linkStatus() const;               // Blocking link check
bool linkStatusCached() const;         // Cached link status
void getMacAddress(uint8_t* mac) const;
void setMacAddress(const uint8_t* mac); // Requires re-init
uint16_t getRxBufferSize() const;
void setRxBufferSize(uint16_t size);   // Call before begin()
```

#### Packet Transmission

```cpp
// Blocking send
bool sendPacket(const void* data, size_t size);

// Non-blocking with callback
bool sendPacket(const void* data, size_t size, 
                void (*callback)(void*), void* user_data);
```

#### Packet Reception

```cpp
uint8_t packetAvailable() const;       // Packets in RX buffer
uint16_t receivePacket(void* buffer, uint16_t max_size);        // Blocking
uint16_t receivePacketNonBlocking(void* buffer, uint16_t max_size);
bool discardPacket();                  // Discard next packet
```

#### Interrupts

```cpp
void enableInterrupts(bool enable, 
                      void (*callback)(void*) = nullptr, 
                      void* user_data = nullptr);
void handleInterrupt();  // Call from loop() when using interrupts
```

#### FreeRTOS Integration

```cpp
#ifdef FREERTOS_H
bool beginFreeRTOS(const char* interface_name,
                   uint32_t ip = 0, uint32_t netmask = 0, uint32_t gateway = 0);
NetworkInterface_t* getFreeRTOSInterface() const;
#endif
```

## Examples

| Example | Description |
|---------|-------------|
| **Basic** | Simple initialization and link monitoring |
| **SendPacket** | Sending ARP requests and receiving responses |
| **InterruptDriven** | Using GPIO interrupts for efficient packet handling |
| **CustomPins** | Using non-default GPIO pins |

Open via: File -> Examples -> pico-enc28j60

## Low-Level C API (Pico SDK)

For direct Pico SDK usage, include the C headers:

```c
#include "pico/enc28j60-driver.h"
#include "pico/enc28j60-freertos-driver.h"  // Optional

enc28j60_config* cfg = enc28j60_get_default_config();
// Configure pins, MAC, etc.
enc28j60* eth = enc28j60_init(cfg);

// Blocking operations
uint8_t rev = enc28j60_revision_blk(eth);
bool link = enc28j60_link_status_blk(eth);
uint32_t frame_id = enc28j60_frame_upload_blk(eth, size, data);
enc28j60_frame_tx_blk(eth, frame_id);

// Non-blocking (DMA) operations
uint32_t frame_id = enc28j60_frame_upload(eth, size, data);
enc28j60_frame_tx(eth, frame_id);

// FreeRTOS+TCP
enc28j60_freertos_initialize(&iface);
```

See `src/include/pico/enc28j60-driver.h` for full C API.

## Architecture

```
+---------------------------------------------+
|           Arduino C++ Wrapper               |  <- enc28j60.h/cpp
|           (ENC28J60 class)                  |
+---------------------------------------------+
|           Low-Level C Driver                |  <- enc28j60-driver.c
|   - PIO SPI Program                         |
|   - DMA Engine                              |
|   - ENC28J60 Register Access                |
|   - Buffer Management                       |
+---------------------------------------------+
|           Hardware (RP2040)                 |
|   - PIO State Machine                       |
|   - DMA Channels (3)                        |
|   - GPIO / Interrupts                       |
+---------------------------------------------+
```

### RP2040 Resources Used

- **PIO**: 1 state machine (17 instructions) for SPI
- **DMA**: 3 channels (1 RX, 2 TX scatter-gather)
- **GPIO**: 5 pins (4 SPI + 1 IRQ)
- **IRQ**: 1 DMA IRQ channel + 1 GPIO IRQ

## Performance

- SPI Clock: ~3.5 MHz (reliable operation per ENC28J60 errata)
- Throughput: ~1-2 Mbps typical (limited by ENC28J60 10BASE-T)
- Latency: Sub-millisecond with interrupts
- CPU Overhead: Minimal (DMA handles bulk transfers)

## License

MIT License - see [LICENSE](LICENSE) for details.

## References

- [ENC28J60 Data Sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/39662e.pdf)
- [ENC28J60 Errata](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/Errata/80349c.pdf)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Arduino Core for RP2040](https://github.com/earlephilhower/arduino-pico)
