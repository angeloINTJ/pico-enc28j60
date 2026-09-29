#include <Arduino.h>
#include <enc28j60.h>

// MAC address for the ENC28J60
// Use a locally administered address (2nd bit of first byte = 1)
uint8_t mac[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};

ENC28J60 eth;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial port to connect
    }

    Serial.println("Inicializando ENC28J60...");

    // Initialize with default pins (Pico GPIOs 16, 17, 18, 19, 20)
    if (!eth.begin(mac)) {
        Serial.println("Falha ao inicializar o ENC28J60!");
        return;
    }

    uint8_t rev = eth.getRevision();
    Serial.print("ENC28J60 Revisao: 0x");
    Serial.println(rev, HEX);

    bool link = eth.linkStatus();
    Serial.print("Status do link: ");
    Serial.println(link ? "Conectado" : "Desconectado");
    
    // Print MAC address
    uint8_t mac_addr[6];
    eth.getMacAddress(mac_addr);
    Serial.print("MAC: ");
    for (int i = 0; i < 6; i++) {
        if (mac_addr[i] < 0x10) Serial.print("0");
        Serial.print(mac_addr[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    Serial.print("RX Buffer Size: ");
    Serial.print(eth.getRxBufferSize());
    Serial.println(" bytes");
}

void loop() {
    if (eth.isInitialized()) {
        bool link = eth.linkStatus();
        Serial.print("Link: ");
        Serial.println(link ? "Ativo" : "Inativo");
        
        // Check for received packets
        if (eth.packetAvailable() > 0) {
            uint8_t buffer[1518];
            uint16_t len = eth.receivePacket(buffer, sizeof(buffer));
            if (len > 0) {
                Serial.print("Pacote recebido: ");
                Serial.print(len);
                Serial.println(" bytes");
                // Process packet here...
            }
        }
    }
    delay(2000);
}
