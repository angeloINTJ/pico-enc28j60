# PicoENC28J60

Arduino library for the ENC28J60 Ethernet controller on Raspberry Pi Pico (RP2040).

Based on the excellent work by **Lukasz Janyst** - https://github.com/ljanyst/pico-enc28j60

## Features

- **PIO-based SPI**: Uses Raspberry Pi Pico's Programmable I/O (PIO) for SPI communication, allowing flexible pin assignments
- **Custom pin mapping**: Configure MOSI, MISO, SCK, CS, RST, and IRQ pins freely
- **Arduino-friendly API**: Simple C++ class interface with familiar Arduino patterns
- **Interrupt support**: Optional callback-based interrupt handling
- **Polling mode**: Works without interrupts for simpler applications
- **Full Ethernet frame control**: Send/receive raw Ethernet frames

## Hardware Connections

Default pinout (matches the original library):

| Signal | GPIO | Pico Pin | ENC28J60 Pin |
|--------|------|----------|--------------|
| VCC | 3V3 | 36 | VCC |
| GND | GND | 38 | GND |
| SI (MOSI) | GP3 | 5 | SI |
| SO (MISO) | GP4 | 6 | SO |
| SCK | GP2 | 4 | SCK |
| CS | GP5 | 7 | CS |
| RST | GP7 | 10 | RST |
| INT | (optional) | - | INT |

All pins are configurable via ENC28J60Config structure.

## Installation

### Arduino IDE Library Manager (Coming Soon)
Search for "PicoENC28J60" in the Library Manager.

### Manual Installation
1. Download this repository as ZIP
2. In Arduino IDE: Sketch -> Include Library -> Add .ZIP Library
3. Select the downloaded ZIP file

### PlatformIO
lib_deps = 
    https://github.com/yourusername/PicoENC28J60

## Quick Start

```cpp
#include <PicoENC28J60.h>

// Optional: custom pin configuration
ENC28J60Config config;
config.mosi_pin = 3;   // GP3
config.miso_pin = 4;   // GP4
config.sclk_pin = 2;   // GP2
config.cs_pin = 5;     // GP5
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
        memcpy(frame + 6, eth.getMACAddress(), 6);  // Source MAC
        frame[12] = 0x08; frame[13] = 0x06;  // ARP
        
        if (eth.sendFrame(frame, 60)) {
            Serial.println("Frame sent");
        }
    }
    
    delay(10);
}
```

## Advanced Usage

### Interrupt Mode

```cpp
void irqHandler(void* userData) {
    PicoENC28J60* eth = static_cast<PicoENC28J60*>(userData);
    // Handle interrupt - set flag for main loop
    packetReceived = true;
}

void setup() {
    // ... initialization ...
    
    // Enable interrupt on GP6 (for example)
    config.irq_pin = 6;
    eth.setInterruptCallback(irqHandler, &eth);
}

void loop() {
    if (packetReceived) {
        packetReceived = false;
        // Process packets...
    }
}
```

### Custom MAC Address

```cpp
ENC28J60Config config;
uint8_t my_mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
memcpy(config.mac_addr, my_mac, 6);
PicoENC28J60 eth(config);
```

## API Reference

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

## Examples

See the examples/ folder:
- BasicTest - Simple initialization and packet loop
- AdvancedTest - Interrupt mode, custom MAC, frame parsing

## Credits

This library is a port of pico-enc28j60 by Lukasz Janyst (lukasz@jany.st)
- Original repository: https://github.com/ljanyst/pico-enc28j60
- Licensed under MIT License

The PIO SPI implementation and ENC28J60 register-level driver are entirely
Lukasz's work. This Arduino port adds:
- Arduino-style C++ class wrapper
- Flexible pin configuration
- Arduino build system integration
- Examples and documentation

## License

MIT License - See LICENSE file for details.
