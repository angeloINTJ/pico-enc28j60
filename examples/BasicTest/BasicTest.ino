// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

/*
 * BasicTest - bring the chip up, print every frame that arrives and send a test
 * frame every 10 seconds, without the INT pin.
 *
 * The test frame is a broadcast with the "local experimental" EtherType 0x88B5
 * of IEEE 802. No host reacts to it, and it says nothing about anybody's IP
 * address, unlike an ARP or an ICMP packet would. To see it, run
 *
 *     sudo tcpdump -i <interface> -e -X ether proto 0x88b5
 *
 * on a computer on the same network.
 */
#include <PicoENC28J60.h>

// Default pins; change them if your wiring is different
//   SCK=GP2, MOSI(SI)=GP3, MISO(SO)=GP4, CS=GP5, RST=GP7, INT=not used
ENC28J60Config config;
// config.mosi_pin = 3;
// config.miso_pin = 4;
// config.sclk_pin = 2;
// config.cs_pin = 5;
// config.rst_pin = 7;
// config.irq_pin = -1;   // -1: the chip is polled

PicoENC28J60 eth(config);

static const uint16_t TEST_ETHERTYPE = 0x88B5;

static void printMac(const uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial.print('0');
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(':');
    }
}

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && millis() - t < 3000;) {
        delay(10);  // give a serial monitor a moment, but do not depend on one
    }

    Serial.println("\n=== PicoENC28J60 Basic Test ===");
    Serial.println("Based on pico-enc28j60 by Lukasz Janyst");

    Serial.println("Initializing ENC28J60...");
    if (!eth.begin()) {
        Serial.printf("ERROR: %s\n", PicoENC28J60::errorString(eth.lastError()));
        while (1) delay(1000);
    }
    Serial.println("ENC28J60 initialized successfully!");

    Serial.printf("Revision ID: 0x%02X\n", eth.getRevision());

    uint8_t mac[6];
    eth.getMACAddress(mac);
    Serial.print("MAC Address: ");
    printMac(mac);
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
    Serial.println("Frames are printed as they arrive; press Reset to restart.");
}

void loop() {
    // Check for received packets
    if (eth.hasPacket()) {
        uint8_t buffer[PicoENC28J60::MAX_FRAME_SIZE];
        uint16_t len = eth.receiveFrame(buffer, sizeof(buffer));

        if (len > 0) {
            Serial.printf("\n[RX] Packet received: %d bytes\n", len);

            // Print first 64 bytes in hex
            uint16_t print_len = min(len, (uint16_t)64);
            for (uint16_t i = 0; i < print_len; i++) {
                if (buffer[i] < 0x10) Serial.print('0');
                Serial.print(buffer[i], HEX);
                Serial.print(' ');
                if ((i + 1) % 16 == 0) Serial.println();
            }
            if (print_len % 16 != 0) Serial.println();

            // Basic frame analysis
            if (len >= 14) {
                Serial.print("  Dst MAC: ");
                printMac(buffer);
                Serial.println();

                Serial.print("  Src MAC: ");
                printMac(buffer + 6);
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

    // Check for other events
    if (eth.hasLinkChange()) {
        Serial.print("\n[LINK] Link status changed: ");
        Serial.println(eth.linkStatus() ? "UP" : "DOWN");
        eth.acknowledgeInterrupts(PicoENC28J60::INT_LINK_CHANGE);
    }

    if (eth.hasRXError()) {
        Serial.println("[ERROR] Receive buffer overflow: frames were lost");
        eth.acknowledgeInterrupts(PicoENC28J60::INT_RX_ERROR);
    }

    // Send a test frame every 10 seconds. sendFrame() returns when the frame
    // has left the wire, so there is no "transmit complete" event to wait for.
    static uint32_t last_send = 0;
    static uint32_t counter = 0;
    if (millis() - last_send > 10000) {
        last_send = millis();

        uint8_t frame[60] = {0};
        memset(frame, 0xFF, 6);  // destination: broadcast
        eth.getMACAddress(frame + 6);  // source: us
        frame[12] = TEST_ETHERTYPE >> 8;
        frame[13] = TEST_ETHERTYPE & 0xFF;
        int n = snprintf((char *)frame + 14, sizeof(frame) - 14,
                         "PicoENC28J60 test frame #%lu", (unsigned long)counter++);

        // The hardware pads a short frame to 60 bytes and appends the FCS
        if (eth.sendFrame(frame, 14 + n)) {
            Serial.println("[TX] Test frame sent");
        } else {
            Serial.println("[TX] Failed to send frame");
        }
    }

    delay(10);
}
