#include <PicoENC28J60.h>

// ============================================================
// Pico ENC28J60 DHCP Client - Debug Version
// ============================================================
ENC28J60Config g_config;
PicoENC28J60 g_eth(g_config);

const uint8_t MY_MAC[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x01};

enum DhcpState {
    ST_DISCOVER = 1,
    ST_OFFER    = 2,
    ST_REQUEST  = 3,
    ST_ACK      = 4,
    ST_BOUND    = 5
};

DhcpState g_dhcpState = ST_DISCOVER;
uint32_t g_xid = 0x12345678;
uint32_t g_serverIp = 0;
uint32_t g_offeredIp = 0;
uint32_t g_leaseTime = 0;
unsigned long g_lastDhcpTime = 0;
int g_dhcpRetries = 0;
const int MAX_DHCP_RETRIES = 5;

const int LED_PIN = 25;
uint8_t g_rxBuffer[1520];
uint8_t g_txBuffer[600];
uint16_t g_checksum = 0;

#define DHCP_CLIENT_PORT 68
#define DHCP_SERVER_PORT 67

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    g_config.mosi_pin = 3; g_config.miso_pin = 4; g_config.sclk_pin = 2;
    g_config.cs_pin = 5; g_config.rst_pin = 7; g_config.irq_pin = -1;
    memcpy(g_config.mac_addr, MY_MAC, 6);
    
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("\n=== Pico ENC28J60 DHCP Debug ===");
    Serial.println("Pinout: MOSI=GP3, MISO=GP4, SCK=GP2, CS=GP5, RST=GP7");
    
    if (!g_eth.begin()) { Serial.println("ERROR: ENC28J60 init failed!"); while(1) delay(1000); }
    Serial.println("ENC28J60 OK");
    
    uint8_t mac[6]; g_eth.getMACAddress(mac);
    Serial.print("MAC: "); printMac(mac); Serial.println();
    
    Serial.println("Waiting for link...");
    unsigned long t = millis();
    while (!g_eth.linkStatus()) { g_eth.poll(); delay(100); if (millis()-t>10000) break; }
    Serial.println(g_eth.linkStatus() ? "Link: UP" : "Link: DOWN");
    
    g_dhcpState = ST_DISCOVER;
    g_lastDhcpTime = 0;
    g_dhcpRetries = 0;
    Serial.println("\n=== Starting DHCP ===");
}

void loop() {
    g_eth.poll();
    
    if (g_eth.hasPacket()) {
        uint16_t len = g_eth.receiveFrame(g_rxBuffer, sizeof(g_rxBuffer));
        if (len > 0) {
            debugPacket(g_rxBuffer, len);
            processPacket(g_rxBuffer, len);
        }
    }
    
    handleDhcp();
    
    static unsigned long ledT = 0;
    if (millis() - ledT > 200) { ledT = millis(); digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }
    delay(1);
}

// ============================================================
// Packet Debug
// ============================================================

void debugPacket(uint8_t* buf, uint16_t len) {
    uint16_t ethType = (buf[12]<<8) | buf[13];
    Serial.printf("\n[RX] len=%d ethType=0x%04X\n", len, ethType);
    Serial.print("  DST MAC: "); printMac(buf); Serial.println();
    Serial.print("  SRC MAC: "); printMac(buf+6); Serial.println();
    Serial.printf("  EtherType: 0x%04X\n", ethType);
    
    if (len >= 14) {
        uint8_t* ip = buf + 14;
        if ((ip[0]>>4) == 4) {
            uint8_t proto = ip[9];
            uint8_t* srcIp = ip+12;
            uint8_t* dstIp = ip+16;
            Serial.printf("  IPv4: %d.%d.%d.%d -> %d.%d.%d.%d proto=%d\n",
                srcIp[0],srcIp[1],srcIp[2],srcIp[3], dstIp[0],dstIp[1],dstIp[2],dstIp[3], ip[9]);
            
            if (ip[9] == 17 && len >= 34) {
                uint16_t srcPort = (buf[34]<<8)|buf[35];
                uint16_t dstPort = (buf[36]<<8)|buf[37];
                Serial.printf("  UDP: %d -> %d len=%d\n", srcPort, dstPort, (buf[38]<<8)|buf[39]);
                
                if (dstPort == 68 || dstPort == 67) {
                    uint8_t* dhcp = buf + 42;
                    if (len >= 42+240) {
                        Serial.printf("  DHCP: op=%d xid=0x%08X yiaddr=%d.%d.%d.%d\n",
                            buf[42], 
                            (buf[46]<<24)|(buf[47]<<16)|(buf[48]<<8)|buf[49],
                            buf[58],buf[59],buf[60],buf[61]);
                        
                        uint8_t* opt = buf + 42 + 240;
                        while (*opt != 255 && opt < buf + len) {
                            uint8_t o = *opt++; uint8_t ol = *opt++;
                            if (o == 53) Serial.printf("    DHCP Opt 53 (msg type): %d\n", *opt);
                            else if (o == 54) Serial.printf("    DHCP Opt 54 (server): %d.%d.%d.%d\n", opt[0],opt[1],opt[2],opt[3]);
                            else if (o == 51) Serial.printf("    DHCP Opt 51 (lease): %u\n", (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3]);
                            else if (o == 1) Serial.printf("    DHCP Opt 1 (mask): %d.%d.%d.%d\n", opt[0],opt[1],opt[2],opt[3]);
                            else if (o == 3) Serial.printf("    DHCP Opt 3 (router): %d.%d.%d.%d\n", opt[0],opt[1],opt[2],opt[3]);
                            else if (o == 58) Serial.printf("    DHCP Opt 58 (T1): %u\n", (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3]);
                            else if (o == 59) Serial.printf("    DHCP Opt 59 (T2): %u\n", (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3]);
                            else if (o == 6) Serial.printf("    DHCP Opt 6 (DNS): %d.%d.%d.%d\n", opt[0],opt[1],opt[2],opt[3]);
                            else Serial.printf("    DHCP Opt %d (len=%d)\n", o, ol);
                            opt += ol;
                        }
                    }
                }
            }
        }
    }
    
    Serial.print("  Raw: ");
    for (int i=0; i<min(len,64); i++) {
        if (buf[i]<16) Serial.print("0");
        Serial.print(buf[i],HEX); Serial.print(" ");
        if ((i+1)%16==0) Serial.print(" ");
    }
    Serial.println();
}

void processPacket(uint8_t* buf, uint16_t len) {
    if (len < 14) return;
    uint16_t ethType = (buf[12]<<8) | buf[13];
    
    if (ethType == 0x0806) handleArp(buf, len);
    else if (ethType == 0x0800) handleIpv4(buf, len);
}

void handleArp(uint8_t* buf, uint16_t len) {
    if (len < 42) return;
    uint8_t* arp = buf + 14;
    if (arp[0]!=0||arp[1]!=1||arp[2]!=8||arp[3]!=0||arp[4]!=6||arp[5]!=4||arp[6]!=0||arp[7]!=1) return;
    
    uint8_t* tip = arp + 24;
    if (g_dhcpState != ST_BOUND) return;
    if (memcmp(tip, (uint8_t*)&g_offeredIp, 4) != 0) return;
    
    memcpy(buf, buf+6, 6); memcpy(buf+6, MY_MAC, 6);
    arp[6]=0; arp[7]=2;
    memcpy(arp+8, MY_MAC, 6);
    memcpy(arp+14, (uint8_t*)&g_offeredIp, 4);
    memcpy(arp+18, buf, 6);
    memcpy(arp+24, arp+14, 4);
    g_eth.sendFrame(buf, 42);
    Serial.println("[ARP] Replied");
}

void handleIpv4(uint8_t* buf, uint16_t len) {
    uint8_t* ip = buf + 14;
    if (len<34 || (ip[0]>>4)!=4 || (ip[0]&0xF)<5) return;
    
    uint8_t proto = ip[9];
    uint16_t totLen = (ip[2]<<8)|ip[3];
    uint8_t* srcIp = ip+12;
    uint8_t* dstIp = ip+16;
    uint8_t* pay = ip+20;
    uint16_t payLen = totLen - 20;
    
    if (proto == 17) handleUdp(buf, srcIp, dstIp, buf+34, payLen);
    else if (proto == 1) handleIcmp(buf, srcIp, pay, payLen);
}

void handleUdp(uint8_t* buf, uint8_t* srcIp, uint8_t* dstIp, uint8_t* udp, uint16_t len) {
    if (len < 8) return;
    uint16_t srcPort = (udp[0]<<8)|udp[1];
    uint16_t dstPort = (udp[2]<<8)|udp[3];
    uint16_t udpLen = (udp[4]<<8)|udp[5];
    
    if (dstPort != DHCP_CLIENT_PORT) return;
    if (srcPort != DHCP_SERVER_PORT) return;
    
    uint8_t* dhcp = udp + 8;
    uint16_t dhcpLen = udpLen - 8;
    if (dhcpLen < 240) return;
    
    handleDhcpResponse(buf, buf+12, dhcp, dhcpLen);
}

// ============================================================
// DHCP Core
// ============================================================

void sendDhcpDiscover() {
    Serial.println("[DHCP] >>> DISCOVER xid=0x" + String(g_xid, HEX));
    
    memset(g_txBuffer, 0, 600);
    uint8_t* eth = g_txBuffer;
    uint8_t* ip = g_txBuffer + 14;
    uint8_t* udp = g_txBuffer + 34;
    
    // Ethernet broadcast
    memset(eth, 0xFF, 6); memcpy(eth+6, MY_MAC, 6); eth[12]=8; eth[13]=0;
    
    // IP
    ip[0]=0x45; ip[1]=0;
    uint16_t ipTot = 20+8+300; ip[2]=ipTot>>8; ip[3]=ipTot&0xFF;
    ip[4]=g_xid>>8; ip[5]=g_xid&0xFF; ip[6]=0; ip[7]=0;
    ip[8]=64; ip[9]=17; ip[10]=0; ip[11]=0;
    memset(ip+12, 0, 4); memset(ip+16, 0xFF, 4);
    g_checksum = checksum(ip,20); ip[10]=g_checksum>>8; ip[11]=g_checksum&0xFF;
    
    // UDP
    udp[0]=68>>8; udp[1]=68; udp[2]=67>>8; udp[3]=67;
    uint16_t ulen = 8+300; udp[4]=ulen>>8; udp[5]=ulen&0xFF; udp[6]=0; udp[7]=0;
    
    // DHCP DISCOVER
    buildDhcp(udp+8, 1, 0, 0, g_xid);
    
    uint16_t flen = 14+20+8+300;
    g_eth.sendFrame(g_txBuffer, flen);
    Serial.println("[DHCP] >>> DISCOVER sent xid=0x" + String(g_xid, HEX));
    // DO NOT increment g_xid - keep same xid for entire DHCP exchange
    g_dhcpState = ST_OFFER;
    g_lastDhcpTime = millis(); g_dhcpRetries++;
}

void handleDhcpResponse(uint8_t* buf, uint8_t* srcIp, uint8_t* dhcp, uint16_t len) {
    if (len < 240) return;
    if (dhcp[0] != 2) return; // BOOTREPLY
    
    uint32_t xid = (dhcp[4]<<24)|(dhcp[5]<<16)|(dhcp[6]<<8)|dhcp[7];
    if (xid != g_xid - 1) return;
    
    uint32_t yiaddr = (dhcp[16]<<24)|(dhcp[17]<<16)|(dhcp[18]<<8)|dhcp[19];
    
    uint8_t* opt = dhcp + 240;
    uint8_t msgType = 0; uint32_t lease = 0, mask = 0, router = 0, servIp = 0;
    
    while (*opt != 255) {
        uint8_t o = *opt++; uint8_t ol = *opt++;
        if (o == 53) msgType = *opt;
        else if (o == 51) lease = (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3];
        else if (o == 1) mask = (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3];
        else if (o == 3) router = (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3];
        else if (o == 54) servIp = (opt[0]<<24)|(opt[1]<<16)|(opt[2]<<8)|opt[3];
        opt += ol;
    }
    
    Serial.printf("[DHCP] RX type=%d xid=0x%08X yiaddr=%d.%d.%d.%d\n", 
                  msgType, g_xid-1, yiaddr>>24, (yiaddr>>16)&0xFF, (yiaddr>>8)&0xFF, yiaddr&0xFF);
    
    if (g_dhcpState == ST_OFFER && msgType == 2) { // OFFER
        g_offeredIp = yiaddr; 
        g_serverIp = servIp ? servIp : *(uint32_t*)srcIp;
        g_leaseTime = lease ? lease : 3600;
        Serial.printf("[DHCP] OFFER: %d.%d.%d.%d\n", yiaddr>>24, (yiaddr>>16)&0xFF, (yiaddr>>8)&0xFF, yiaddr&0xFF);
        g_dhcpState = ST_REQUEST; g_dhcpRetries = 0; sendDhcpRequest();
    }
    else if (g_dhcpState == ST_REQUEST && msgType == 5) { // ACK
        g_offeredIp = yiaddr;
        Serial.println("[DHCP] ACK received!");
        g_dhcpState = ST_BOUND; digitalWrite(LED_PIN, HIGH);
    }
    else if (msgType == 6) { // NAK
        Serial.println("[DHCP] NAK - restarting");
        g_dhcpState = ST_DISCOVER; g_lastDhcpTime = 0; g_dhcpRetries = 0;
    }
}

void sendDhcpRequest() {
    Serial.println("[DHCP] >>> REQUEST for " + String(g_offeredIp>>24) + "." + String((g_offeredIp>>16)&0xFF) + "." + String((g_offeredIp>>8)&0xFF) + "." + String(g_offeredIp&0xFF));
    
    memset(g_txBuffer, 0, 600);
    uint8_t* eth = g_txBuffer;
    uint8_t* ip = g_txBuffer + 14;
    uint8_t* udp = g_txBuffer + 34;
    
    memset(eth, 0xFF, 6); memcpy(eth+6, MY_MAC, 6); eth[12]=8; eth[13]=0;
    
    ip[0]=0x45; ip[1]=0;
    uint16_t ipTot = 20+8+300; ip[2]=ipTot>>8; ip[3]=ipTot&0xFF;
    ip[4]=g_xid>>8; ip[5]=g_xid&0xFF; ip[6]=0; ip[7]=0;
    ip[8]=64; ip[9]=17; ip[10]=0; ip[11]=0;
    memset(ip+12, 0, 4); memset(ip+16, 0xFF, 4);
    g_checksum = checksum(ip,20); ip[10]=g_checksum>>8; ip[11]=g_checksum&0xFF;
    
    udp[0]=68>>8; udp[1]=68; udp[2]=67>>8; udp[3]=67;
    uint16_t ulen = 8+300; udp[4]=ulen>>8; udp[5]=ulen&0xFF; udp[6]=0; udp[7]=0;
    
    // Build DHCP REQUEST with Server ID from OFFER
    uint8_t* dhcp = udp+8;
    memset(dhcp, 0, 300);
    dhcp[0]=1; dhcp[1]=1; dhcp[2]=6; dhcp[3]=0;
    dhcp[4]=g_xid>>24; dhcp[5]=g_xid>>16; dhcp[6]=g_xid>>8; dhcp[7]=g_xid;
    dhcp[8]=0; dhcp[9]=0; dhcp[10]=0x80; dhcp[11]=0;
    // ciaddr = offered IP (some servers require this in REQUEST)
    memcpy(dhcp+12, (uint8_t*)&g_offeredIp, 4);
    memset(dhcp+16, 0, 12); // yiaddr, siaddr, giaddr = 0
    memcpy(dhcp+28, MY_MAC, 6);
    dhcp[236]=99; dhcp[237]=130; dhcp[238]=83; dhcp[239]=99;
    
    uint8_t* opt = dhcp + 240;
    *opt++=53; *opt++=1; *opt++=3; // REQUEST=3
    *opt++=61; *opt++=7; *opt++=1; memcpy(opt, MY_MAC, 6); opt+=6;
    // Requested IP (option 50)
    *opt++=50; *opt++=4; *opt++=g_offeredIp>>24; *opt++=(g_offeredIp>>16)&0xFF; *opt++=(g_offeredIp>>8)&0xFF; *opt++=g_offeredIp&0xFF;
    // Server ID (option 54) - CRITICAL: must match the server that sent OFFER
    *opt++=54; *opt++=4; *opt++=g_serverIp>>24; *opt++=(g_serverIp>>16)&0xFF; *opt++=(g_serverIp>>8)&0xFF; *opt++=g_serverIp&0xFF;
    // Parameter request list
    *opt++=55; *opt++=4; *opt++=1; *opt++=3; *opt++=6; *opt++=12;
    // Hostname
    *opt++=12; *opt++=9; memcpy(opt, "pico-enc", 8); opt[8]=0; opt+=9;
    *opt++=255;
    
    uint16_t flen = 14+20+8+300;
    g_eth.sendFrame(g_txBuffer, flen);
    Serial.println("[DHCP] REQUEST sent with Server ID: " + String(g_serverIp>>24) + "." + String((g_serverIp>>16)&0xFF) + "." + String((g_serverIp>>8)&0xFF) + "." + String(g_serverIp&0xFF));
    // DO NOT increment g_xid - keep same xid for entire DHCP exchange
    g_dhcpState = ST_ACK; g_lastDhcpTime = millis(); g_dhcpRetries++;
}

void buildDhcp(uint8_t* dhcp, uint8_t msgType, uint32_t reqIp, uint32_t servId, uint32_t xid) {
    dhcp[0]=1; dhcp[1]=1; dhcp[2]=6; dhcp[3]=0;
    dhcp[4]=xid>>24; dhcp[5]=xid>>16; dhcp[6]=xid>>8; dhcp[7]=xid;
    dhcp[8]=0; dhcp[9]=0; dhcp[10]=0x80; dhcp[11]=0;
    memset(dhcp+12, 0, 16);
    memcpy(dhcp+28, MY_MAC, 6);
    dhcp[236]=99; dhcp[237]=130; dhcp[238]=83; dhcp[239]=99;
    
    uint8_t* opt = dhcp + 240;
    *opt++=53; *opt++=1; *opt++=msgType;
    *opt++=61; *opt++=7; *opt++=1; memcpy(opt, MY_MAC, 6); opt+=6;
    
    if (msgType == 3) { // REQUEST
        *opt++=50; *opt++=4; *opt++=reqIp>>24; *opt++=(reqIp>>16)&0xFF; *opt++=(reqIp>>8)&0xFF; *opt++=reqIp&0xFF;
        *opt++=54; *opt++=4; *opt++=servId>>24; *opt++=(servId>>16)&0xFF; *opt++=(servId>>8)&0xFF; *opt++=servId&0xFF;
    }
    
    *opt++=55; *opt++=4; *opt++=1; *opt++=3; *opt++=6; *opt++=12;
    *opt++=12; *opt++=9; memcpy(opt, "pico-enc", 8); opt[8]=0; opt+=9;
    *opt++=255;
}

void sendIpPacket(uint8_t* buf, uint8_t* dstIp, uint8_t proto, uint8_t* pay, uint16_t len) {
    uint8_t* eth = buf; uint8_t* ip = buf+14;
    memcpy(eth, eth+6, 6); memcpy(eth+6, MY_MAC, 6); eth[12]=8; eth[13]=0;
    ip[0]=0x45; ip[1]=0;
    uint16_t tlen=20+len; ip[2]=tlen>>8; ip[3]=tlen&0xFF;
    ip[4]=0; ip[5]=1; ip[6]=0x40; ip[7]=0; ip[8]=64; ip[9]=proto; ip[10]=0; ip[11]=0;
    memcpy(ip+12, (uint8_t*)&g_offeredIp, 4); memcpy(ip+16, dstIp, 4);
    ip[10]=0; ip[11]=0; g_checksum = checksum(ip,20); ip[10]=g_checksum>>8; ip[11]=g_checksum&0xFF;
    memcpy(ip+20, pay, len);
    g_eth.sendFrame(buf, 14+20+len);
}

void handleIcmp(uint8_t* buf, uint8_t* srcIp, uint8_t* pay, uint16_t len) {
    if (len<8 || pay[0]!=8) return;
    pay[0]=0; pay[2]=0; pay[3]=0;
    uint16_t c = checksum(pay, len);
    pay[2]=c>>8; pay[3]=c&0xFF;
    sendIpPacket(buf+14, srcIp, 1, pay, len);
    Serial.println("[ICMP] Echo reply");
}

void handleDhcp() {
    unsigned long now = millis();
    
    switch (g_dhcpState) {
        case ST_DISCOVER:
            if (now - g_lastDhcpTime > 5000 || g_lastDhcpTime == 0) {
                if (g_dhcpRetries >= MAX_DHCP_RETRIES) { useFallbackIp(); return; }
                sendDhcpDiscover(); g_lastDhcpTime = now; g_dhcpRetries++;
            }
            break;
        case ST_OFFER:
            if (now - g_lastDhcpTime > 10000) { g_dhcpState = ST_DISCOVER; g_lastDhcpTime = 0; }
            break;
        case ST_REQUEST:
            if (now - g_lastDhcpTime > 5000) {
                if (g_dhcpRetries >= 5) { g_dhcpState = ST_DISCOVER; g_lastDhcpTime = 0; g_dhcpRetries = 0; }
                else { sendDhcpRequest(); g_lastDhcpTime = now; g_dhcpRetries++; }
            }
            break;
        case ST_ACK:
            if (now - g_lastDhcpTime > 10000) { g_dhcpState = ST_DISCOVER; g_lastDhcpTime = 0; g_dhcpRetries = 0; }
            break;
        case ST_BOUND:
            digitalWrite(LED_PIN, HIGH);
            break;
    }
}

void useFallbackIp() {
    g_offeredIp = (169<<24)|(254<<16)|(1<<8)|100;
    g_dhcpState = ST_BOUND;
    g_leaseTime = 3600;
    Serial.println("[DHCP] Fallback IP: 169.254.1.100");
}

uint16_t checksum(uint8_t* d, uint16_t len) {
    uint32_t s=0; for(uint16_t i=0;i<len;i+=2){ uint16_t w=(d[i]<<8)|(i+1<len?d[i+1]:0); s+=w; }
    while(s>>16) s=(s&0xFFFF)+(s>>16); return ~s;
}

void printMac(uint8_t* m) { for(int i=0;i<6;i++){ if(m[i]<16)Serial.print(0); Serial.print(m[i],HEX); if(i<5)Serial.print(":"); } }
void printIp(uint32_t ip) { Serial.printf("%d.%d.%d.%d", ip>>24, (ip>>16)&0xFF, (ip>>8)&0xFF, ip&0xFF); }
