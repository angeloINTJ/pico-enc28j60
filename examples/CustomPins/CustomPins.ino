#include <Arduino.h>
#include <enc28j60.h>

// MAC address
uint8_t mac[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};

ENC28J60 eth;

// Custom pin configuration for different board layouts
// Example: Using GPIO 4, 5, 6, 7, 8 for SPI + IRQ
const int CUSTOM_MISO = 4;
const int CUSTOM_MOSI = 5;
const int CUSTOM_SCLK = 6;
const int CUSTOM_IRQ = 8;   // CS will be SCLK + 1 = 7

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("ENC28J60 Custom Pins Example");
    Serial.println("Usando GPIOs personalizados:");
    Serial.print("  MISO: GPIO "); Serial.println(CUSTOM_MISO);
    Serial.print("  MOSI: GPIO "); Serial.println(CUSTOM_MOSI);
    Serial.print("  SCLK: GPIO "); Serial.println(CUSTOM_SCLK);
    Serial.print("  CS:   GPIO "); Serial.println(CUSTOM_SCLK + 1);
    Serial.print("  IRQ:  GPIO "); Serial.println(CUSTOM_IRQ);

    // Initialize with custom pins
    if (!eth.begin(mac, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_SCLK, CUSTOM_IRQ)) {
        Serial.println("Falha ao inicializar!");
        while (1) delay(1000);
    }

    Serial.print("Revisao: 0x");
    Serial.println(eth.getRevision(), HEX);
    
    if (eth.linkStatus()) {
        Serial.println("Link: Conectado");
    } else {
        Serial.println("Link: Desconectado");
    }
}

void loop() {
    if (eth.linkStatus()) {
        if (eth.packetAvailable() > 0) {
            uint8_t buffer[1518];
            uint16_t len = eth.receivePacket(buffer, sizeof(buffer));
            if (len > 0) {
                Serial.print("Pacote: ");
                Serial.print(len);
                Serial.println(" bytes");
            }
        }
    } else {
        Serial.println("Aguardando link...");
    }
    delay(2000);
}
