#include <PicoENC28J60.h>

// ============================================================
// Pico ENC28J60 DHCP Client Example
// ============================================================
// Custom pinout for ENC28J60
ENC28J60Config g_config;

PicoENC28J60 g_eth(g_config);

// Network configuration
const uint8_t MY_MAC[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x01};

// DHCP state machine
enum DhcpState {
    ST_IDLE = 0,
    ST_DISCOVER = 1,
    ST_OFFER = 2,
    ST_REQUEST = 3,
    ST_ACK = 4,
    ST_BOUND = 5
};

DhcpState g_dhcpState = ST_IDLE;
uint32_t g_xid = 0x12345678;
uint32_t g_serverIp = 0;
uint32_t g_offeredIp = 0;
uint32_t g_leaseTime = 0;
uint8_t g_serverMac[6] = {0};
unsigned long g_lastDhcpTime = 0;
int g_dhcpRetries = 0;
const int MAX_DHCP_RETRIES = 3;

// Onboard LED
const int LED_PIN = 25;

// Buffers
uint8_t g_rxBuffer[1520];
uint8_t g_txBuffer[1520];

// DHCP message types
#define DHCP_BOOTREQUEST 1
#define DHCP_BOOTREPLY 2

// DHCP option codes
#define DHCP_OPT_SUBNET_MASK 1
#define DHCP_OPT_ROUTER 3
#define DHCP_OPT_DNS 6
#define DHCP_OPT_HOSTNAME 12
#define DHCP_OPT_REQUESTED_IP 50
#define DHCP_OPT_LEASE_TIME 51
#define DHCP_OPT_MSG_TYPE 53
#define DHCP_OPT_SERVER_ID 54
#define DHCP_OPT_PARAM_REQ 55
#define DHCP_OPT_END 255

// UDP ports
#define DHCP_CLIENT_PORT 68
#define DHCP_SERVER_PORT 67

uint16_t g_checksum = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Configure pins
    g_config.mosi_pin = 3;
    g_config.miso_pin = 4;
    g_config.sclk_pin = 2;
    g_config.cs_pin = 5;
    g_config.rst_pin = 7;
    g_config.irq_pin = -1;
    memcpy(g_config.mac_addr, MY_MAC, 6);
    
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("\n========================================");
    Serial.println("Pico ENC28J60 DHCP Client");
    Serial.println("========================================");
    Serial.println("Pinout: MOSI=GP3, MISO=GP4, SCK=GP2, CS=GP5, RST=GP7");
    
    // Initialize Ethernet
    Serial.println("\nInitializing ENC28J60...");
    if (!g_eth.begin()) {
        Serial.println("ERROR: Failed to initialize ENC28J60!");
        while (1) { delay(1000); }
    }
    
    Serial.println("ENC28J60 initialized!");
    
    uint8_t mac[6];
    g_eth.getMACAddress(mac);
    Serial.print("MAC: ");
    printMac(mac);
    Serial.println();
    
    // Wait for link
    Serial.println("Waiting for link...");
    unsigned long linkStart = millis();
    while (!g_eth.linkStatus()) {
        g_eth.poll();
        delay(100);
        if (millis() - linkStart > 15000) {
            Serial.println("WARNING: No link");
            break;
        }
    }
    
    if (g_eth.linkStatus()) Serial.println("Link: UP");
    else Serial.println("Link: DOWN");
    
    // Start DHCP
    g_dhcpState = ST_DISCOVER;
    g_lastDhcpTime = millis();
    g_dhcpRetries = 0;
    
    Serial.println("\nStarting DHCP...");
}

void loop() {
    g_eth.poll();
    
    // Handle DHCP state machine
    handleDhcp();
    
    // Process incoming packets
    if (g_eth.hasPacket()) {
        uint16_t len = g_eth.receiveFrame(g_rxBuffer, sizeof(g_rxBuffer));
        if (len > 0) {
            processPacket(g_rxBuffer, len);
        }
    }
    
    // Blink LED based on DHCP state
    static unsigned long ledBlink = 0;
    if (millis() - ledBlink > (g_dhcpState == ST_BOUND ? 2000 : 200)) {
        ledBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delay(1);
}

void handleDhcp() {
    unsigned long now = millis();
    
    switch (g_dhcpState) {
        case ST_DISCOVER:
            if (now - g_lastDhcpTime > 5000 || g_lastDhcpTime == 0) {
                if (g_dhcpRetries >= MAX_DHCP_RETRIES) {
                    Serial.println("DHCP: Max retries, using fallback IP");
                    useFallbackIp();
                    return;
                }
                sendDhcpDiscover();
                g_lastDhcpTime = now;
                g_dhcpRetries++;
                g_dhcpState = ST_OFFER;
            }
            break;
            
        case ST_OFFER:
            if (now - g_lastDhcpTime > 10000) {
                g_dhcpState = ST_DISCOVER;
                g_lastDhcpTime = 0;
            }
            break;
            
        case ST_REQUEST:
            if (now - g_lastDhcpTime > 5000) {
                if (g_dhcpRetries >= MAX_DHCP_RETRIES) {
                    g_dhcpState = ST_DISCOVER;
                    g_lastDhcpTime = 0;
                    g_dhcpRetries = 0;
                } else {
                    sendDhcpRequest();
                    g_lastDhcpTime = now;
                    g_dhcpRetries++;
                }
            }
            break;
            
        case ST_ACK:
            if (now - g_lastDhcpTime > 10000) {
                g_dhcpState = ST_DISCOVER;
                g_lastDhcpTime = 0;
                g_dhcpRetries = 0;
            }
            break;
            
        case ST_BOUND:
            // Renew at 50% lease time
            if (g_leaseTime > 0 && now > (g_leaseTime * 500)) {
                Serial.println("DHCP: Renewing lease");
                g_dhcpState = ST_REQUEST;
                g_dhcpRetries = 0;
                g_lastDhcpTime = 0;
            }
            break;
    }
}

void processPacket(uint8_t* buffer, uint16_t len) {
    if (len < 14) return;
    uint16_t ethType = (buffer[12] << 8) | buffer[13];
    
    switch (ethType) {
        case 0x0806: handleArp(buffer, len); break;
        case 0x0800: handleIpv4(buffer, len); break;
    }
}

void handleArp(uint8_t* buffer, uint16_t len) {
    if (len < 42) return;
    uint8_t* arp = buffer + 14;
    
    if (arp[0]!=0 || arp[1]!=1 || arp[2]!=8 || arp[3]!=0 || arp[4]!=6 || arp[5]!=4) return;
    if (arp[6]!=0 || arp[7]!=1) return;
    
    uint8_t* targetIp = arp + 24;
    if (g_dhcpState != ST_BOUND) return;
    if (memcmp(targetIp, (uint8_t*)&g_offeredIp, 4) != 0) return;
    
    // ARP reply
    memcpy(buffer + 0, buffer + 6, 6);
    memcpy(buffer + 6, MY_MAC, 6);
    arp[6] = 0; arp[7] = 2;
    memcpy(arp + 8, MY_MAC, 6);
    memcpy(arp + 14, (uint8_t*)&g_offeredIp, 4);
    memcpy(arp + 18, buffer + 0, 6);
    memcpy(arp + 24, arp + 14, 4);
    
    g_eth.sendFrame(buffer, 42);
}

void handleIpv4(uint8_t* buffer, uint16_t len) {
    uint8_t* ip = buffer + 14;
    if (len < 34 || (ip[0]>>4)!=4 || (ip[0]&0xF)<5) return;
    
    uint8_t protocol = ip[9];
    uint16_t totalLen = (ip[2]<<8) | ip[3];
    uint8_t* srcIp = ip + 12;
    uint8_t* dstIp = ip + 16;
    uint8_t* payload = ip + 20;
    uint16_t payloadLen = totalLen - 20;
    
    if (protocol == 17) { // UDP
        handleUdp(buffer, srcIp, dstIp, payload, payloadLen);
    } else if (protocol == 1) { // ICMP
        handleIcmp(buffer, srcIp, payload, payloadLen);
    }
}

void handleUdp(uint8_t* buffer, uint8_t* srcIp, uint8_t* dstIp, uint8_t* payload, uint16_t len) {
    if (len < 8) return;
    
    uint16_t srcPort = (payload[0]<<8) | payload[1];
    uint16_t dstPort = (payload[2]<<8) | payload[3];
    uint16_t udpLen = (payload[4]<<8) | payload[5];
    
    if (dstPort != DHCP_CLIENT_PORT) return;
    
    uint8_t* dhcpPayload = payload + 8;
    uint16_t dhcpLen = udpLen - 8;
    
    if (srcPort != DHCP_SERVER_PORT) return;
    
    handleDhcpResponse(buffer, srcIp, dhcpPayload, dhcpLen);
}

void handleIcmp(uint8_t* buffer, uint8_t* srcIp, uint8_t* payload, uint16_t len) {
    if (len < 8 || payload[0] != 8) return;
    
    payload[0] = 0;
    payload[2] = 0; payload[3] = 0;
    uint16_t csum = checksum(payload, len);
    payload[2] = csum >> 8; payload[3] = csum & 0xFF;
    
    sendIpPacket(buffer, srcIp, 1, payload, len);
}

// ============================================================
// DHCP Implementation
// ============================================================

void sendDhcpDiscover() {
    Serial.println("[DHCP] Sending DISCOVER");
    
    memset(g_txBuffer, 0, sizeof(g_txBuffer));
    uint8_t* eth = g_txBuffer;
    uint8_t* ip = g_txBuffer + 14;
    uint8_t* udp = g_txBuffer + 34;
    uint8_t* dhcp = g_txBuffer + 42;
    
    // Ethernet: broadcast
    memset(eth, 0xFF, 6);
    memcpy(eth + 6, MY_MAC, 6);
    eth[12] = 0x08; eth[13] = 0x00;
    
    // IP header
    ip[0] = 0x45; ip[1] = 0x00;
    uint16_t ipTotalLen = 20 + 8 + 300;
    ip[2] = ipTotalLen >> 8; ip[3] = ipTotalLen & 0xFF;
    ip[4] = g_xid >> 8; ip[5] = g_xid & 0xFF;
    ip[6] = 0; ip[7] = 0;
    ip[8] = 64; ip[9] = 17;
    ip[10] = 0; ip[11] = 0;
    memset(ip + 12, 0, 4);
    memset(ip + 16, 0xFF, 4);
    ip[10] = 0; ip[11] = 0;
    g_checksum = checksum(ip, 20);
    ip[10] = g_checksum >> 8; ip[11] = g_checksum & 0xFF;
    
    // UDP header
    udp[0] = DHCP_CLIENT_PORT >> 8; udp[1] = DHCP_CLIENT_PORT & 0xFF;
    udp[2] = DHCP_SERVER_PORT >> 8; udp[3] = DHCP_SERVER_PORT & 0xFF;
    uint16_t udpLen = 8 + 300;
    udp[4] = udpLen >> 8; udp[5] = udpLen & 0xFF;
    udp[6] = 0; udp[7] = 0;
    
    // DHCP payload
    buildDhcpMessage(dhcp, 1, 0, 0, g_xid); // DISCOVER = 1
    
    uint16_t frameLen = 14 + 20 + 8 + 300;
    g_eth.sendFrame(g_txBuffer, frameLen);
    
    Serial.println("[DHCP] DISCOVER sent (broadcast)");
    g_xid++;
}

void handleDhcpResponse(uint8_t* buffer, uint8_t* srcIp, uint8_t* payload, uint16_t len) {
    if (len < 240) return;
    
    if (payload[0] != DHCP_BOOTREPLY) return;
    
    uint32_t xid = (payload[4]<<24) | (payload[5]<<16) | (payload[6]<<8) | payload[7];
    if (xid != g_xid - 1) return;
    
    uint32_t offeredIp = (payload[16]<<24) | (payload[17]<<16) | (payload[18]<<8) | payload[19];
    uint32_t serverIp = (payload[20]<<24) | (payload[21]<<16) | (payload[22]<<8) | payload[23];
    
    // Parse options
    uint8_t* opts = payload + 240;
    uint8_t dhcpMsgType = 0;
    uint32_t serverId = 0;
    uint32_t leaseTime = 0;
    uint32_t subnetMask = 0;
    uint32_t router = 0;
    
    while (*opts != DHCP_OPT_END) {
        uint8_t opt = *opts++;
        uint8_t optLen = *opts++;
        
        switch (opt) {
            case DHCP_OPT_MSG_TYPE: dhcpMsgType = *opts; break;
            case DHCP_OPT_SERVER_ID: serverId = (opts[0]<<24)|(opts[1]<<16)|(opts[2]<<8)|opts[3]; break;
            case DHCP_OPT_LEASE_TIME: leaseTime = (opts[0]<<24)|(opts[1]<<16)|(opts[2]<<8)|opts[3]; break;
            case DHCP_OPT_SUBNET_MASK: subnetMask = (opts[0]<<24)|(opts[1]<<16)|(opts[2]<<8)|opts[3]; break;
            case DHCP_OPT_ROUTER: router = (opts[0]<<24)|(opts[1]<<16)|(opts[2]<<8)|opts[3]; break;
        }
        opts += optLen;
    }
    
    Serial.printf("[DHCP] Msg type: %d, Offered IP: %d.%d.%d.%d\n", 
                  dhcpMsgType, offeredIp>>24, (offeredIp>>16)&0xFF, (offeredIp>>8)&0xFF, offeredIp&0xFF);
    
    if (g_dhcpState == ST_OFFER && dhcpMsgType == 2) { // OFFER
        g_offeredIp = offeredIp;
        g_serverIp = serverId ? serverId : *(uint32_t*)srcIp;
        g_leaseTime = leaseTime ? leaseTime : 3600;
        memcpy(g_serverMac, buffer + 6, 6);
        
        Serial.printf("[DHCP] OFFER: %d.%d.%d.%d\n", 
                      offeredIp>>24, (offeredIp>>16)&0xFF, (offeredIp>>8)&0xFF, offeredIp&0xFF);
        
        g_dhcpState = ST_REQUEST;
        g_dhcpRetries = 0;
        sendDhcpRequest();
    }
    else if (g_dhcpState == ST_REQUEST && dhcpMsgType == 5) { // ACK
        g_offeredIp = offeredIp;
        g_serverIp = serverId ? serverId : *(uint32_t*)srcIp;
        g_leaseTime = leaseTime ? leaseTime : 3600;
        
        Serial.println("[DHCP] ACK received!");
        Serial.printf("IP: %d.%d.%d.%d\n", offeredIp>>24, (offeredIp>>16)&0xFF, (offeredIp>>8)&0xFF, offeredIp&0xFF);
        Serial.printf("Mask: %d.%d.%d.%d\n", subnetMask>>24, (subnetMask>>16)&0xFF, (subnetMask>>8)&0xFF, subnetMask&0xFF);
        Serial.printf("Gateway: %d.%d.%d.%d\n", router>>24, (router>>16)&0xFF, (router>>8)&0xFF, router&0xFF);
        Serial.printf("Lease: %u seconds\n", g_leaseTime);
        
        g_dhcpState = ST_BOUND;
        g_dhcpRetries = 0;
        digitalWrite(LED_PIN, HIGH);
    }
    else if (dhcpMsgType == 6) { // NAK
        Serial.println("[DHCP] NAK, restarting");
        g_dhcpState = ST_DISCOVER;
        g_lastDhcpTime = 0;
        g_dhcpRetries = 0;
    }
}

void sendDhcpRequest() {
    Serial.println("[DHCP] Sending REQUEST");
    
    memset(g_txBuffer, 0, sizeof(g_txBuffer));
    uint8_t* eth = g_txBuffer;
    uint8_t* ip = g_txBuffer + 14;
    uint8_t* udp = g_txBuffer + 34;
    uint8_t* dhcp = g_txBuffer + 42;
    
    // Ethernet: broadcast
    memset(eth, 0xFF, 6);
    memcpy(eth + 6, MY_MAC, 6);
    eth[12] = 0x08; eth[13] = 0x00;
    
    // IP header
    ip[0] = 0x45; ip[1] = 0x00;
    uint16_t ipTotalLen = 20 + 8 + 300;
    ip[2] = ipTotalLen >> 8; ip[3] = ipTotalLen & 0xFF;
    ip[4] = g_xid >> 8; ip[5] = g_xid & 0xFF;
    ip[6] = 0; ip[7] = 0;
    ip[8] = 64; ip[9] = 17;
    ip[10] = 0; ip[11] = 0;
    memset(ip + 12, 0, 4);
    memset(ip + 16, 0xFF, 4);
    g_checksum = checksum(ip, 20);
    ip[10] = g_checksum >> 8; ip[11] = g_checksum & 0xFF;
    
    // UDP
    udp[0] = DHCP_CLIENT_PORT >> 8; udp[1] = DHCP_CLIENT_PORT & 0xFF;
    udp[2] = DHCP_SERVER_PORT >> 8; udp[3] = DHCP_SERVER_PORT & 0xFF;
    udp[4] = (8 + 300) >> 8; udp[5] = (8 + 300) & 0xFF;
    udp[6] = 0; udp[7] = 0;
    
    // DHCP Request
    buildDhcpMessage(dhcp, 3, g_offeredIp, g_serverIp, g_xid); // REQUEST = 3
    
    uint16_t frameLen = 14 + 20 + 8 + 300;
    g_eth.sendFrame(g_txBuffer, frameLen);
    
    Serial.println("[DHCP] REQUEST sent");
    g_xid++;
    g_dhcpState = ST_ACK;
    g_lastDhcpTime = millis();
}

void buildDhcpMessage(uint8_t* dhcp, uint8_t msgType, uint32_t reqIp, uint32_t serverId, uint32_t xid) {
    dhcp[0] = DHCP_BOOTREQUEST; dhcp[1] = 1; dhcp[2] = 6; dhcp[3] = 0;
    dhcp[4] = xid >> 24; dhcp[5] = xid >> 16; dhcp[6] = xid >> 8; dhcp[7] = xid;
    dhcp[8] = 0; dhcp[9] = 0;
    dhcp[10] = 0x80; dhcp[11] = 0;
    dhcp[12] = 0; dhcp[13] = 0; dhcp[14] = 0; dhcp[15] = 0;
    dhcp[16] = 0; dhcp[17] = 0; dhcp[18] = 0; dhcp[19] = 0;
    dhcp[20] = 0; dhcp[21] = 0; dhcp[23] = 0; dhcp[23] = 0;
    dhcp[24] = 0; dhcp[25] = 0; dhcp[26] = 0; dhcp[27] = 0;
    memcpy(dhcp + 28, MY_MAC, 6);
    
    // Magic cookie
    dhcp[236] = 99; dhcp[237] = 130; dhcp[238] = 83; dhcp[239] = 99;
    
    uint8_t* opt = dhcp + 240;
    
    *opt++ = DHCP_OPT_MSG_TYPE; *opt++ = 1; *opt++ = msgType;
    *opt++ = 61; *opt++ = 7; *opt++ = 1;
    memcpy(opt, MY_MAC, 6); opt += 6;
    
    if (msgType == 3 && reqIp) { // REQUEST
        *opt++ = DHCP_OPT_REQUESTED_IP; *opt++ = 4;
        *opt++ = reqIp >> 24; *opt++ = (reqIp>>16)&0xFF; *opt++ = (reqIp>>8)&0xFF; *opt++ = reqIp&0xFF;
    }
    
    if (msgType == 3 && serverId) {
        *opt++ = DHCP_OPT_SERVER_ID; *opt++ = 4;
        *opt++ = serverId >> 24; *opt++ = (serverId>>16)&0xFF; *opt++ = (serverId>>8)&0xFF; *opt++ = serverId&0xFF;
    }
    
    *opt++ = DHCP_OPT_PARAM_REQ; *opt++ = 4;
    *opt++ = DHCP_OPT_SUBNET_MASK; *opt++ = DHCP_OPT_ROUTER; *opt++ = DHCP_OPT_DNS; *opt++ = DHCP_OPT_HOSTNAME;
    
    *opt++ = DHCP_OPT_HOSTNAME; *opt++ = 9;
    memcpy(opt, "pico-enc", 8); opt[8] = 0; opt += 9;
    
    *opt++ = DHCP_OPT_END;
}

void useFallbackIp() {
    g_offeredIp = (169<<24) | (254<<16) | (1<<8) | 100;
    g_dhcpState = ST_BOUND;
    g_leaseTime = 3600;
    Serial.println("[DHCP] Fallback IP: 169.254.1.100");
}

// ============================================================
// Packet Building
// ============================================================

void sendIpPacket(uint8_t* buffer, uint8_t* dstIp, uint8_t protocol, uint8_t* payload, uint16_t len) {
    uint8_t* eth = buffer;
    uint8_t* ip = buffer + 14;
    
    memcpy(eth + 0, eth + 6, 6);
    memcpy(eth + 6, MY_MAC, 6);
    eth[12] = 0x08; eth[13] = 0x00;
    
    ip[0] = 0x45; ip[1] = 0x00;
    uint16_t totalLen = 20 + len;
    ip[2] = totalLen >> 8; ip[3] = totalLen & 0xFF;
    ip[4] = 0; ip[5] = 1;
    ip[6] = 0x40; ip[7] = 0;
    ip[8] = 64; ip[9] = protocol;
    ip[10] = 0; ip[11] = 0;
    memcpy(ip + 12, (uint8_t*)&g_offeredIp, 4);
    memcpy(ip + 16, dstIp, 4);
    ip[10] = 0; ip[11] = 0;
    g_checksum = checksum(ip, 20);
    ip[10] = g_checksum >> 8; ip[11] = g_checksum & 0xFF;
    
    memcpy(ip + 20, payload, len);
    
    uint16_t frameLen = 14 + 20 + len;
    g_eth.sendFrame(buffer, frameLen);
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

void printIp(uint32_t ip) {
    Serial.printf("%d.%d.%d.%d", (ip>>24)&0xFF, (ip>>16)&0xFF, (ip>>8)&0xFF, ip&0xFF);
}
