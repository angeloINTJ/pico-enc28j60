#include <Arduino.h>
#include <enc28j60.h>

// MAC address
uint8_t mac[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};

ENC28J60 eth;

// Interrupt flag - set in ISR, processed in loop
volatile bool irq_triggered = false;
volatile uint8_t irq_flags = 0;

// ISR callback - called from interrupt context
void irqCallback(void* user_data) {
    irq_triggered = true;
    // Note: Can't do much here - keep it minimal!
    // We just set a flag and process in loop()
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("ENC28J60 Interrupt-Driven Example");

    // Initialize with custom pins if needed
    // eth.begin(mac, 16, 17, 18, 20, pio0, 0);
    
    if (!eth.begin(mac)) {
        Serial.println("Falha ao inicializar!");
        while (1) delay(1000);
    }

    Serial.print("Revisao: 0x");
    Serial.println(eth.getRevision(), HEX);

    // Enable interrupt-driven operation
    eth.enableInterrupts(true, irqCallback, nullptr);
    Serial.println("Interrupcoes habilitadas");
    Serial.println("Pino IRQ: GPIO 20 (padrao)");
    Serial.println("Conecte o pino INT do ENC28J60 ao GPIO 20");
}

void loop() {
    // Check if interrupt was triggered
    if (irq_triggered) {
        irq_triggered = false;
        
        // Process interrupts - this reads and clears flags
        eth.handleInterrupt();
        
        // Check for received packets
        while (eth.packetAvailable() > 0) {
            uint8_t buffer[1518];
            uint16_t len = eth.receivePacketNonBlocking(buffer, sizeof(buffer));
            if (len > 0) {
                Serial.print("[IRQ] Pacote recebido: ");
                Serial.print(len);
                Serial.println(" bytes");
                
                // Print packet info
                Serial.print("  Dest MAC: ");
                for (int i = 0; i < 6; i++) {
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    if (i < 5) Serial.print(":");
                }
                Serial.println();
                
                Serial.print("  Src MAC:  ");
                for (int i = 6; i < 12; i++) {
                    if (buffer[i] < 0x10) Serial.print("0");
                    Serial.print(buffer[i], HEX);
                    if (i < 11) Serial.print(":");
                }
                Serial.println();
                
                // EtherType
                uint16_t ethertype = (buffer[12] << 8) | buffer[13];
                Serial.print("  EtherType: 0x");
                Serial.println(ethertype, HEX);
            }
        }
    }

    // Periodic status
    static uint32_t last_status = 0;
    if (millis() - last_status > 10000) {
        last_status = millis();
        bool link = eth.linkStatusCached();
        Serial.print("Status: Link ");
        Serial.print(link ? "Ativo" : "Inativo");
        Serial.print(" | Pacotes na fila: ");
        Serial.println(eth.packetAvailable());
    }

    // Small delay to not hog CPU
    delay(1);
}
