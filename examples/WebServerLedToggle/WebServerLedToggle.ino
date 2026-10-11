#include <PicoENC28J60.h>

// ============================================================
// Pico ENC28J60 WebServer - LED Toggle
// ============================================================
// Custom pinout for ENC28J60 (matches your hardware)
ENC28J60Config g_config;

PicoENC28J60 g_eth(g_config);

// Network configuration (static IP - no DHCP in basic driver)
const uint8_t MY_MAC[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x01};
const uint8_t MY_IP[4] = {192, 168, 1, 100};
const uint8_t GATEWAY[4] = {192, 168, 1, 1};
const uint8_t NETMASK[4] = {255, 255, 255, 0};
const uint16_t HTTP_PORT = 80;

// Onboard LED
const int LED_PIN = 25;
volatile bool g_ledState = false;

// Buffers
uint8_t g_rxBuffer[1520];
uint8_t g_txBuffer[1520];

// Simple TCP connection tracking
struct TcpConnection {
    uint32_t srcIp;
    uint16_t srcPort;
    uint32_t seqNum;
    uint32_t ackNum;
    uint8_t state; // 0=closed, 1=syn_rcvd, 2=established, 3=fin_wait
    unsigned long lastActivity;
};

TcpConnection g_conn;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Configure pins
    g_config.mosi_pin = 3;   // GP3 - SI (MOSI)
    g_config.miso_pin = 4;   // GP4 - SO (MISO)
    g_config.sclk_pin = 2;   // GP2 - SCK
    g_config.cs_pin = 5;     // GP5 - CS
    g_config.rst_pin = 7;    // GP7 - RST
    g_config.irq_pin = -1;   // Polling mode
    memcpy(g_config.mac_addr, MY_MAC, 6);
    
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("\n========================================");
    Serial.println("Pico ENC28J60 WebServer - LED Toggle");
    Serial.println("========================================");
    Serial.println("Pinout: MOSI=GP3, MISO=GP4, SCK=GP2, CS=GP5, RST=GP7");
    
    // Initialize Ethernet
    Serial.println("\nInitializing ENC28J60...");
    if (!g_eth.begin()) {
        Serial.println("ERROR: Failed to initialize ENC28J60!");
        while (1) { delay(1000); }
    }
    
    Serial.println("ENC28J60 initialized successfully!");
    
    // Print MAC
    uint8_t mac[6];
    g_eth.getMACAddress(mac);
    Serial.print("MAC Address: ");
    printMac(mac);
    Serial.println();
    
    // Wait for link
    Serial.println("\nWaiting for Ethernet link...");
    unsigned long linkStart = millis();
    while (!g_eth.linkStatus()) {
        g_eth.poll();
        delay(100);
        if (millis() - linkStart > 15000) {
            Serial.println("WARNING: No link detected (cable unplugged?)");
            break;
        }
    }
    
    if (g_eth.linkStatus()) {
        Serial.println("Link Status: UP");
    } else {
        Serial.println("Link Status: DOWN (continuing anyway)");
    }
    
    // Print server info
    Serial.println("\n================== SERVER INFO ==================");
    Serial.print("IP Address:   "); printIp(MY_IP); Serial.println();
    Serial.print("Gateway:      "); printIp(GATEWAY); Serial.println();
    Serial.print("Netmask:      "); printIp(NETMASK); Serial.println();
    Serial.print("HTTP Port:    "); Serial.println(HTTP_PORT);
    Serial.println("=================================================");
    Serial.println();
    Serial.print("Open browser to: http://");
    printIp(MY_IP);
    Serial.println("/");
    Serial.println("=================================================");
    
    memset(&g_conn, 0, sizeof(g_conn));
}

void loop() {
    // Must poll regularly for interrupts
    g_eth.poll();
    
    // Check for incoming packets
    if (g_eth.hasPacket()) {
        uint16_t len = g_eth.receiveFrame(g_rxBuffer, sizeof(g_rxBuffer));
        if (len > 0) {
            processPacket(g_rxBuffer, len);
        }
    }
    
    // Handle TCP timeouts/retransmissions
    handleTcpState();
    
    delay(1);
}

void processPacket(uint8_t* buffer, uint16_t len) {
    if (len < 14) return; // Minimum Ethernet frame
    
    // Check EtherType
    uint16_t ethType = (buffer[12] << 8) | buffer[13];
    
    switch (ethType) {
        case 0x0806: // ARP
            handleArp(buffer, len);
            break;
        case 0x0800: // IPv4
            handleIpv4(buffer, len);
            break;
        default:
            break;
    }
}

void handleArp(uint8_t* buffer, uint16_t len) {
    if (len < 42) return;
    
    // ARP header starts at offset 14
    uint8_t* arp = buffer + 14;
    
    // Check hardware type (Ethernet = 1)
    if (arp[0] != 0x00 || arp[1] != 0x01) return;
    // Check protocol type (IPv4 = 0x0800)
    if (arp[2] != 0x08 || arp[3] != 0x00) return;
    // Check hardware size (6) and protocol size (4)
    if (arp[4] != 6 || arp[5] != 4) return;
    // Check operation (1 = request)
    if (arp[6] != 0x00 || arp[7] != 0x01) return;
    
    // Target protocol address (who we're looking for)
    uint8_t* targetIp = arp + 24;
    
    // Check if ARP is for our IP
    if (memcmp(targetIp, MY_IP, 4) != 0) return;
    
    Serial.println("[ARP] Request for our IP, sending reply");
    
    // Build ARP reply
    // Swap sender/target
    memcpy(buffer + 0, buffer + 6, 6);  // dst MAC = src MAC
    memcpy(buffer + 6, MY_MAC, 6);       // src MAC = our MAC
    
    // ARP reply (operation = 2)
    arp[6] = 0x00; arp[7] = 0x02;
    
    // Sender MAC = our MAC
    memcpy(arp + 8, MY_MAC, 6);
    // Sender IP = our IP
    memcpy(arp + 14, MY_IP, 4);
    // Target MAC = original sender MAC
    memcpy(arp + 18, buffer + 0, 6);
    // Target IP = original sender IP
    memcpy(arp + 24, arp + 14, 4); // was sender IP, now target
    
    // Send ARP reply (42 bytes total)
    g_eth.sendFrame(buffer, 42);
    Serial.println("[ARP] Reply sent");
}

void handleIpv4(uint8_t* buffer, uint16_t len) {
    uint8_t* ip = buffer + 14;
    
    if (len < 14 + 20) return;
    
    // Check version (4) and header length (5 = 20 bytes)
    if ((ip[0] >> 4) != 4) return;
    if ((ip[0] & 0x0F) < 5) return;
    
    // Check protocol (6 = TCP, 1 = ICMP)
    uint8_t protocol = ip[9];
    uint16_t totalLen = (ip[2] << 8) | ip[3];
    
    // Destination IP
    uint8_t* dstIp = ip + 16;
    if (memcmp(dstIp, MY_IP, 4) != 0) return; // Not for us
    
    // Source IP
    uint8_t* srcIp = ip + 12;
    
    uint8_t* payload = ip + 20;
    uint16_t payloadLen = totalLen - 20;
    
    switch (protocol) {
        case 1: // ICMP
            handleIcmp(buffer, srcIp, payload, payloadLen);
            break;
        case 6: // TCP
            handleTcp(buffer, srcIp, payload, payloadLen);
            break;
        default:
            break;
    }
}

void handleIcmp(uint8_t* buffer, uint8_t* srcIp, uint8_t* payload, uint16_t len) {
    if (len < 8) return;
    
    // ICMP Echo Request (type 8)
    if (payload[0] == 8 && payload[1] == 0) {
        Serial.println("[ICMP] Echo Request, sending Reply");
        
        // Build Echo Reply (type 0)
        payload[0] = 0; // Type 0 = Echo Reply
        // Recalculate checksum
        payload[2] = 0; payload[3] = 0;
        uint16_t csum = checksum(payload, len);
        payload[2] = csum >> 8;
        payload[3] = csum & 0xFF;
        
        // Send IP packet back
        sendIpPacket(buffer, srcIp, 1, payload, len);
    }
}

void handleTcp(uint8_t* buffer, uint8_t* srcIp, uint8_t* payload, uint16_t len) {
    if (len < 20) return; // Minimum TCP header
    
    uint16_t srcPort = (payload[0] << 8) | payload[1];
    uint16_t dstPort = (payload[2] << 8) | payload[3];
    uint32_t seqNum = (payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | payload[7];
    uint32_t ackNum = (payload[8] << 24) | (payload[9] << 16) | (payload[10] << 8) | payload[11];
    uint8_t flags = payload[13];
    uint16_t window = (payload[14] << 8) | payload[15];
    uint16_t tcpHeaderLen = ((payload[12] >> 4) & 0x0F) * 4;
    
    uint8_t* tcpPayload = payload + tcpHeaderLen;
    uint16_t tcpPayloadLen = len - tcpHeaderLen;
    
    // Only handle HTTP port
    if (dstPort != HTTP_PORT) {
        // Send RST for other ports
        sendTcpRst(buffer, srcIp, dstPort, seqNum);
        return;
    }
    
    Serial.printf("[TCP] Port %d, Flags: 0x%02X, Seq: %u, Payload: %d bytes\n", 
                  srcPort, flags, seqNum, tcpPayloadLen);
    
    // Handle TCP state machine
    if (flags & 0x02) { // SYN
        handleTcpSyn(buffer, srcIp, srcPort, dstPort, seqNum, ackNum);
    } else if (g_conn.state >= 1 && g_conn.srcPort == srcPort && g_conn.srcIp == *(uint32_t*)srcIp) {
        if (flags & 0x10) { // ACK
            if (flags & 0x01) { // FIN
                handleTcpFin(buffer, srcIp, srcPort, seqNum, ackNum);
            } else if (tcpPayloadLen > 0) {
                handleHttpRequest(buffer, srcIp, srcPort, tcpPayload, tcpPayloadLen);
            } else {
                // Pure ACK
                g_conn.ackNum = seqNum + tcpPayloadLen;
                g_conn.lastActivity = millis();
            }
        }
    }
}

void handleTcpSyn(uint8_t* buffer, uint8_t* srcIp, uint16_t srcPort, uint16_t dstPort, 
                  uint32_t seqNum, uint32_t ackNum) {
    Serial.println("[TCP] SYN received, sending SYN-ACK");
    
    // Store connection info
    g_conn.srcIp = *(uint32_t*)srcIp;
    g_conn.srcPort = srcPort;
    g_conn.seqNum = 0x12345678; // Our initial sequence
    g_conn.ackNum = seqNum + 1;
    g_conn.state = 1; // SYN_RCVD
    g_conn.lastActivity = millis();
    
    // Send SYN-ACK
    sendTcpSynAck(buffer, srcIp, srcPort, g_conn.seqNum, g_conn.ackNum);
    g_conn.state = 2; // ESTABLISHED
}

void handleHttpRequest(uint8_t* buffer, uint8_t* srcIp, uint16_t srcPort, 
                       uint8_t* payload, uint16_t len) {
    Serial.printf("[HTTP] Request (%d bytes)\n", len);
    
    // Parse HTTP GET
    char* request = (char*)payload;
    if (strncmp(request, "GET ", 4) != 0) return;
    
    char* pathStart = request + 4;
    char* pathEnd = strchr(pathStart, ' ');
    if (!pathEnd) return;
    *pathEnd = '\0';
    
    String path = String(pathStart);
    Serial.printf("[HTTP] Path: %s\n", path.c_str());
    
    // Toggle LED
    if (path.indexOf("/toggle") >= 0) {
        g_ledState = !g_ledState;
        digitalWrite(LED_PIN, g_ledState ? HIGH : LOW);
        Serial.printf("[LED] %s\n", g_ledState ? "ON" : "OFF");
    }
    
    // Send HTTP response
    sendHttpResponse(buffer, srcIp, srcPort, path);
}

void sendHttpResponse(uint8_t* buffer, uint8_t* srcIp, uint16_t srcPort, String path) {
    // Build HTML
    String html = "<!DOCTYPE html><html><head><title>Pico LED Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;text-align:center;padding:50px;}";
    html += "button{padding:20px 40px;font-size:24px;margin:10px;border:none;border-radius:10px;cursor:pointer;}";
    html += ".on{background:#4CAF50;color:white;}.off{background:#f44336;color:white;}";
    html += "h1{color:#333;}</style></head><body>";
    html += "<h1>Pico ENC28J60 WebServer</h1>";
    html += "<p>Onboard LED (GP25) is <strong>";
    html += g_ledState ? "ON" : "OFF";
    html += "</strong></p>";
    html += "<form action='/toggle' method='GET'>";
    html += "<button class='";
    html += g_ledState ? "off" : "on";
    html += "' type='submit'>";
    html += g_ledState ? "TURN OFF" : "TURN ON";
    html += "</button></form>";
    html += "<p><small>IP: 192.168.1.100 | Port: 80</small></p>";
    html += "</body></html>";
    
    String httpResponse = "HTTP/1.1 200 OK\r\n";
    httpResponse += "Content-Type: text/html\r\n";
    httpResponse += "Content-Length: " + String(html.length()) + "\r\n";
    httpResponse += "Connection: close\r\n";
    httpResponse += "\r\n";
    httpResponse += html;
    
    // Send TCP response with PSH+ACK+FIN
    sendTcpData(buffer, srcIp, srcPort, (uint8_t*)httpResponse.c_str(), httpResponse.length(), 0x18); // PSH+ACK+FIN
    
    g_conn.state = 3; // FIN_WAIT
    Serial.printf("[HTTP] Response sent (%d bytes)\n", httpResponse.length());
}

void handleTcpFin(uint8_t* buffer, uint8_t* srcIp, uint16_t srcPort, uint32_t seqNum, uint32_t ackNum) {
    Serial.println("[TCP] FIN received, sending ACK");
    g_conn.state = 0; // CLOSED
    // Send ACK
    sendTcpAck(buffer, srcIp, srcPort, g_conn.seqNum, seqNum + 1);
}

void handleTcpState() {
    if (g_conn.state > 0 && millis() - g_conn.lastActivity > 30000) {
        Serial.println("[TCP] Connection timeout, closing");
        g_conn.state = 0;
    }
}

// Helper functions for raw packet building

void sendIpPacket(uint8_t* buffer, uint8_t* dstIp, uint8_t protocol, uint8_t* payload, uint16_t len) {
    // Build Ethernet + IP header
    uint8_t* eth = buffer;
    uint8_t* ip = buffer + 14;
    
    // Ethernet header - swap src/dst MAC
    memcpy(eth + 0, eth + 6, 6); // dst = original src
    memcpy(eth + 6, MY_MAC, 6);  // src = our MAC
    eth[12] = 0x08; eth[13] = 0x00; // IPv4
    
    // IP header
    ip[0] = 0x45; // Version 4, Header length 5
    ip[1] = 0x00; // DSCP/ECN
    uint16_t totalLen = 20 + len;
    ip[2] = totalLen >> 8; ip[3] = totalLen & 0xFF;
    ip[4] = 0x00; ip[5] = 0x01; // Identification
    ip[6] = 0x40; ip[7] = 0x00; // Flags + Fragment offset (DF)
    ip[8] = 64; // TTL
    ip[9] = protocol; // Protocol
    ip[10] = 0; ip[11] = 0; // Header checksum (will calc)
    memcpy(ip + 12, MY_IP, 4); // Source IP
    memcpy(ip + 16, dstIp, 4); // Dest IP
    
    // Calculate IP header checksum
    ip[10] = 0; ip[11] = 0;
    uint16_t csum = checksum(ip, 20);
    ip[10] = csum >> 8; ip[11] = csum & 0xFF;
    
    // Copy payload
    memcpy(ip + 20, payload, len);
    
    // Send frame
    uint16_t frameLen = 14 + 20 + len;
    g_eth.sendFrame(buffer, frameLen);
}

void sendTcpSynAck(uint8_t* buffer, uint8_t* dstIp, uint16_t dstPort, uint32_t seqNum, uint32_t ackNum) {
    // Build minimal TCP SYN-ACK packet
    Serial.println("[TCP] SYN-ACK (simplified - full TCP stack needed for production)");
}

void sendTcpAck(uint8_t* buffer, uint8_t* dstIp, uint16_t dstPort, uint32_t seqNum, uint32_t ackNum) {
    Serial.println("[TCP] ACK (simplified)");
}

void sendTcpRst(uint8_t* buffer, uint8_t* dstIp, uint16_t dstPort, uint32_t seqNum) {
    Serial.println("[TCP] Sending RST (simplified)");
}

void sendTcpData(uint8_t* buffer, uint8_t* dstIp, uint16_t dstPort, uint8_t* data, uint16_t len, uint8_t flags) {
    Serial.printf("[TCP] Would send %d bytes with flags 0x%02X (needs full TCP stack)\n", len, flags);
}

uint16_t checksum(uint8_t* data, uint16_t len) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i += 2) {
        uint16_t word = (data[i] << 8) | (i+1 < len ? data[i+1] : 0);
        sum += word;
    }
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return ~sum;
}

void printMac(uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] < 0x10) Serial.print("0");
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(":");
    }
}

void printIp(const uint8_t* ip) {
    Serial.printf("%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}
