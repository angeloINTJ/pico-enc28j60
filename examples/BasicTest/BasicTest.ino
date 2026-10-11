// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#include <PicoENC28J60.h>

// ENC28J60 configuration with default pins
// Modify these if your wiring is different
ENC28J60Config config;
// config.mosi_pin = 3;   // GP3 - SI (MOSI)
// config.miso_pin = 4;   // GP4 - SO (MISO)
// config.sclk_pin = 2;   // GP2 - SCK
// config.cs_pin = 5;     // GP5 - CS
// config.rst_pin = 7;    // GP7 - RST
// config.irq_pin = -1;   // -1 = polling mode

PicoENC28J60 eth(config);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n=== PicoENC28J60 Basic Test ===");
    Serial.println("Based on pico-enc28j60 by Lukasz Janyst");
    
    // Initialize
    Serial.println("Initializing ENC28J60...");
    if (!eth.begin()) {
        Serial.println("ERROR: Failed to initialize ENC28J60!");
        Serial.println("Check wiring and power supply.");
        while (1) delay(1000);
    }
    
    Serial.println("ENC28J60 initialized successfully!");
    
    // Show revision
    uint8_t rev = eth.getRevision();
    Serial.print("Revision ID: 0x");
    Serial.println(rev, HEX);
    
    // Show MAC address
    uint8_t mac[6];
    eth.getMACAddress(mac);
    Serial.print("MAC Address: ");
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial.print("0");
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    // The link takes a second or two to come up after begin()
    Serial.print("Waiting for link");
    for (uint32_t t = millis(); !eth.linkStatus() && millis() - t < 5000;) {
        Serial.print('.');
        delay(250);
    }
    Serial.print("\nLink Status: ");
    Serial.println(eth.linkStatus() ? "UP" : "DOWN (is the cable plugged in?)");
    
    Serial.println("\nStarting main loop...");
    Serial.println("Connect Ethernet cable and watch for packets.");
    Serial.println("Press Reset to restart.");
}

void loop() {
    // Must call poll() regularly in polling mode
    eth.poll();
    
    // Check for received packets
    if (eth.hasPacket()) {
        uint8_t buffer[1520];
        uint16_t len = eth.receiveFrame(buffer, sizeof(buffer));
        
        if (len > 0) {
            Serial.printf("\n[RX] Packet received: %d bytes\n", len);
            
            // Print first 64 bytes in hex
            uint16_t print_len = min(len, (uint16_t)64);
            for (uint16_t i = 0; i < print_len; i++) {
                if (buffer[i] < 0x10) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
                if ((i + 1) % 16 == 0) Serial.println();
            }
            if (print_len % 16 != 0) Serial.println();
            
            // Basic frame analysis
            if (len >= 14) {
                Serial.print("  Dst MAC: ");
                for (int i = 0; i < 6; i++) {
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    if (i < 5) Serial.print(":");
                }
                Serial.println();
                
                Serial.print("  Src MAC: ");
                for (int i = 6; i < 12; i++) {
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    if (i < 11) Serial.print(":");
                }
                Serial.println();
                
                uint16_t ethertype = (buffer[12] << 8) | buffer[13];
                Serial.printf("  EtherType: 0x%04X", ethertype);
                if (ethertype == 0x0800) Serial.print(" (IPv4)");
                else if (ethertype == 0x0806) Serial.print(" (ARP)");
                else if (ethertype == 0x86DD) Serial.print(" (IPv6)");
                Serial.println();
            }
        }
    }
    
    // Check for other interrupts
    if (eth.hasLinkChange()) {
        Serial.print("\n[LINK] Link status changed: ");
        Serial.println(eth.linkStatus() ? "UP" : "DOWN");
        eth.acknowledgeInterrupts(PicoENC28J60::INT_LINK_CHANGE);
    }
    
    if (eth.hasTXComplete()) {
        Serial.println("[TX] Transmission complete");
        eth.acknowledgeInterrupts(PicoENC28J60::INT_TX_COMPLETE);
    }
    
    if (eth.hasRXError()) {
        Serial.println("[ERROR] Receive error");
        eth.acknowledgeInterrupts(PicoENC28J60::INT_RX_ERROR);
    }
    
    // Send a test frame every 10 seconds
    static uint32_t last_send = 0;
    if (millis() - last_send > 10000) {
        last_send = millis();
        
        // Create a simple ARP request frame
        uint8_t frame[60] = {0};
        
        // Destination MAC (broadcast)
        memset(frame, 0xFF, 6);
        
        // Source MAC
        eth.getMACAddress(frame + 6);
        
        // EtherType: ARP (0x0806)
        frame[12] = 0x08;
        frame[13] = 0x06;
        
        // ARP payload
        frame[14] = 0x00; frame[15] = 0x01;  // Hardware type: Ethernet
        frame[16] = 0x08; frame[17] = 0x00;  // Protocol type: IPv4
        frame[18] = 0x06;                     // Hardware size: 6
        frame[19] = 0x04;                     // Protocol size: 4
        frame[20] = 0x00; frame[21] = 0x01;  // Opcode: Request
        
        // Sender MAC (our MAC)
        eth.getMACAddress(frame + 22);
        
        // Sender IP (example: 192.168.1.100)
        frame[28] = 192; frame[29] = 168; frame[30] = 1; frame[31] = 100;
        
        // Target MAC (zero for ARP request)
        memset(frame + 32, 0, 6);
        
        // Target IP (example: 192.168.1.1)
        frame[38] = 192; frame[39] = 168; frame[40] = 1; frame[41] = 1;
        
        // Pad to minimum frame size (60 bytes)
        // Hardware adds FCS
        
        if (eth.sendFrame(frame, 60)) {
            Serial.println("[TX] ARP request sent");
        } else {
            Serial.println("[TX] Failed to send frame");
        }
    }
    
    delay(10);
}
