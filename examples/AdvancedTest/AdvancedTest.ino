/*
 * AdvancedTest - interrupt-driven reception, custom pins and a custom MAC.
 *
 * Wire the ENC28J60 INT pin to GP6 (ENC28J60Config::irq_pin). The chip pulls
 * that line low when a frame arrives, the link changes or the receive buffer
 * overflows. loop() asks interruptPending() - a read of that pin, no SPI
 * traffic - and talks to the chip only when the answer is yes.
 *
 * The same sketch works without the wire: set irq_pin to -1 and
 * interruptPending() looks at the chip instead.
 *
 * Frames are printed as they come, so on a busy network this is chatty. Every
 * 5 seconds the sketch also sends a test frame.
 */
#include <PicoENC28J60.h>

PicoENC28J60* eth = nullptr;  // created in setup() with a custom config

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && millis() - t < 3000;) {
        delay(10);  // give a serial monitor a moment, but do not depend on one
    }

    Serial.println("\n=== PicoENC28J60 Advanced Test ===");
    Serial.println("Interrupt-driven mode with custom MAC");

    // Create custom configuration
    ENC28J60Config config;
    config.mosi_pin = 3;   // GP3 - SI (MOSI)
    config.miso_pin = 4;   // GP4 - SO (MISO)
    config.sclk_pin = 2;   // GP2 - SCK
    config.cs_pin = 5;     // GP5 - CS
    config.rst_pin = 7;    // GP7 - RST
    config.irq_pin = 6;    // GP6 - INT (connect ENC28J60 INT to GP6)

    // Custom MAC address (locally administered)
    uint8_t my_mac[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x02};
    memcpy(config.mac_addr, my_mac, 6);

    eth = new PicoENC28J60(config);

    Serial.println("Initializing ENC28J60...");
    if (!eth->begin()) {
        Serial.println("ERROR: Failed to initialize ENC28J60!");
        while (1) delay(1000);
    }

    Serial.println("ENC28J60 initialized successfully!");

    // Show info
    uint8_t rev = eth->getRevision();
    Serial.printf("Revision: 0x%02X\n", rev);

    uint8_t mac[6];
    eth->getMACAddress(mac);
    Serial.print("MAC: ");
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial.print("0");
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    Serial.printf("Link: %s\n", eth->linkStatus() ? "UP" : "DOWN");
    Serial.println("Interrupt mode enabled on GP6");
    Serial.println("\nMain loop running...");
}

void loop() {
    if (eth->interruptPending()) {
        // Reads the flags and masks the INT pin until they are acknowledged, so
        // nothing that arrives while we work is lost
        uint8_t flags = eth->getInterruptFlags();

        if (flags & PicoENC28J60::INT_LINK_CHANGE) {
            Serial.printf("\n[LINK] Link status: %s\n", eth->linkStatus() ? "UP" : "DOWN");
        }
        if (flags & PicoENC28J60::INT_RX_ERROR) {
            Serial.println("\n[ERROR] Receive buffer overflow: frames were lost");
        }

        processPackets();

        eth->acknowledgeInterrupts(flags);
    }

    // Send periodic test frames
    static uint32_t last_send = 0;
    if (millis() - last_send > 5000) {
        last_send = millis();
        sendTestFrame();
    }
}

void processPackets() {
    uint8_t buffer[1520];
    
    while (eth->available()) {
        uint16_t len = eth->receiveFrame(buffer, sizeof(buffer));
        if (len == 0) break;
        
        Serial.printf("\n[RX] %d bytes", len);
        
        if (len >= 14) {
            // Parse Ethernet header
            uint16_t ethertype = (buffer[12] << 8) | buffer[13];
            
            Serial.printf(" | Type: 0x%04X", ethertype);
            
            switch (ethertype) {
                case 0x0800: // IPv4
                    Serial.print(" (IPv4)");
                    if (len >= 34) {
                        uint8_t protocol = buffer[23];
                        Serial.printf(" Proto: %d", protocol);
                        if (protocol == 1) Serial.print(" ICMP");
                        else if (protocol == 6) Serial.print(" TCP");
                        else if (protocol == 17) Serial.print(" UDP");
                    }
                    break;
                case 0x0806: // ARP
                    Serial.print(" (ARP)");
                    if (len >= 42) {
                        uint16_t opcode = (buffer[20] << 8) | buffer[21];
                        Serial.printf(" Op: %d", opcode);
                    }
                    break;
                case 0x86DD: // IPv6
                    Serial.print(" (IPv6)");
                    break;
                default:
                    break;
            }
            Serial.println();
        }
    }
}

void sendTestFrame() {
    // Alternate between different frame types
    static int frame_type = 0;
    frame_type = (frame_type + 1) % 3;
    
    uint8_t frame[1500] = {0};
    uint16_t frame_len = 60;
    
    // Broadcast destination
    memset(frame, 0xFF, 6);
    
    // Source MAC
    eth->getMACAddress(frame + 6);
    
    switch (frame_type) {
        case 0: // ARP Request
            frame[12] = 0x08; frame[13] = 0x06;
            frame[14] = 0x00; frame[15] = 0x01;
            frame[16] = 0x08; frame[17] = 0x00;
            frame[18] = 0x06; frame[19] = 0x04;
            frame[20] = 0x00; frame[21] = 0x01;
            eth->getMACAddress(frame + 22);
            frame[28] = 192; frame[29] = 168; frame[30] = 1; frame[31] = 100;
            memset(frame + 32, 0, 6);
            frame[38] = 192; frame[39] = 168; frame[40] = 1; frame[41] = 1;
            Serial.println("[TX] Sending ARP request");
            break;
            
        case 1: // IPv4 ICMP Echo Request (ping)
            frame[12] = 0x08; frame[13] = 0x00;
            // Minimal IPv4 header
            frame[14] = 0x45; frame[15] = 0x00; frame[16] = 0x00; frame[17] = 0x3C;
            frame[18] = 0x00; frame[19] = 0x01; frame[20] = 0x00; frame[21] = 0x00;
            frame[22] = 0x40; frame[23] = 0x01; frame[24] = 0x00; frame[25] = 0x00;
            frame[26] = 192; frame[27] = 168; frame[28] = 1; frame[29] = 100;
            frame[30] = 192; frame[31] = 168; frame[32] = 1; frame[33] = 1;
            // ICMP header
            frame[34] = 0x08; frame[35] = 0x00; frame[36] = 0x00; frame[37] = 0x00;
            frame[38] = 0x00; frame[39] = 0x01; frame[40] = 0x00; frame[41] = 0x01;
            frame_len = 60;
            Serial.println("[TX] Sending ICMP echo request");
            break;
            
        case 2: // Custom test pattern
            frame[12] = 0x88; frame[13] = 0xB5; // Custom ethertype
            for (int i = 14; i < 60; i++) {
                frame[i] = i & 0xFF;
            }
            Serial.println("[TX] Sending custom test pattern");
            break;
    }
    
    if (eth->sendFrame(frame, frame_len)) {
        // Frame queued successfully
    } else {
        Serial.println("[TX] Failed to queue frame");
    }
}
