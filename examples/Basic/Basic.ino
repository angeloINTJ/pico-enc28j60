#include <Arduino.h>
#include <enc28j60.h>

enc28j60_config *cfg;
enc28j60 *eth;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Aguarda a porta serial conectar
    }

    Serial.println("Inicializando ENC28J60...");

    cfg = enc28j60_get_default_config();
    if (!cfg) {
        Serial.println("Falha ao obter configuracao padrao!");
        return;
    }

    eth = enc28j60_init(cfg);
    if (!eth) {
        Serial.println("Falha ao inicializar o ENC28J60!");
        return;
    }

    uint8_t rev = enc28j60_revision_blk(eth);
    Serial.print("ENC28J60 Revisao: ");
    Serial.println(rev, HEX);

    bool link = enc28j60_link_status_blk(eth);
    Serial.print("Status do link: ");
    Serial.println(link ? "Conectado" : "Desconectado");
}

void loop() {
    if (eth) {
        bool link = enc28j60_link_status_blk(eth);
        Serial.print("Link: ");
        Serial.println(link ? "Ativo" : "Inativo");
    }
    delay(2000);
}
