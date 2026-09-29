#include <Arduino.h>
#include <enc28j60.h>

// MAC address
uint8_t mac[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};

ENC28J60 eth;

// Simple Ethernet frame for testing (ARP request)
uint8_t arp_request[42] = {
    // Ethernet header
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Destination MAC (broadcast)
    0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A,  // Source MAC (our MAC)
    0x08, 0x06,                           // EtherType: ARP
    // ARP payload
    0x00, 0x01,  // Hardware type: Ethernet
    0x08, 0x00,  // Protocol type: IPv4
    0x06,        // Hardware size: 6
    0x04,        // Protocol size: 4
    0x00, 0x01,  // Opcode: Request
    0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A,  // Sender MAC
    192, 168, 1, 100,                     // Sender IP
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Target MAC (unknown)
    192, 168, 1, 1                        // Target IP (gateway)
};

uint32_t last_send = 0;
uint32_t send_interval = 5000;  // 5 seconds

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("ENC28J60 Send Packet Example");

    if (!eth.begin(mac)) {
        Serial.println("Falha ao inicializar!");
        while (1) delay(1000);
    }

    Serial.print("Revisao: 0x");
    Serial.println(eth.getRevision(), HEX);
    
    if (eth.linkStatus()) {
        Serial.println("Link: Conectado");
    } else {
        Serial.println("Link: Desconectado (cabo desconectado?)");
    }
}

void loop() {
    // Periodic link status check
    static uint32_t last_link_check = 0;
    if (millis() - last_link_check > 10000) {
        last_link_check = millis();
        bool link = eth.linkStatus();
        Serial.print("Link: ");
        Serial.println(link ? "Ativo" : "Inativo");
    }

    // Send ARP request periodically
    if (eth.linkStatus() && (millis() - last_send > send_interval)) {
        last_send = millis();
        
        Serial.println("Enviando ARP request...");
        if (eth.sendPacket(arp_request, sizeof(arp_request))) {
            Serial.println("Pacote enviado com sucesso!");
        } else {
            Serial.println("Falha ao enviar (buffer cheio?)");
        }
    }

    // Check for received packets
    if (eth.packetAvailable() > 0) {
        uint8_t buffer[1518];
        uint16_t len = eth.receivePacket(buffer, sizeof(buffer));
        if (len > 0) {
            Serial.print("Recebido: ");
            Serial.print(len);
            Serial.println(" bytes");
            
            // Print first 32 bytes as hex
            Serial.print("Dados: ");
            for (int i = 0; i < min(len, (uint16_t)32); i++) {
                if (buffer[i] < 0x10) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println("...");
        }
    }

    delay(10);
}
