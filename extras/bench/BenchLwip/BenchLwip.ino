// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

/*
 * BenchLwip - on-hardware self-test for PicoENC28J60lwIP, the network
 * interface of the lwIP stack.
 *
 * Driven over USB serial by extras/bench/benchlwip.py, which plays the other
 * side of the network: it pings the board, opens TCP and UDP sockets to it,
 * serves a TCP connection that the board opens, and checks what comes back.
 *
 * The board runs echo servers (TCP 7007, UDP 7008) once the interface is up.
 * The commands (one per line, the reply starts with the name of the command):
 *
 *   ID                              READY
 *   START dhcp                      get an address from the DHCP server
 *   START static <ip> <gw> <mask>   or use this one
 *   STOP                            end() the interface
 *   INFO                            link, address, DHCP state, counters, heap
 *   PING <ip>                       ICMP echo from the board
 *   CLIENT <ip> <port> <down> <up>  open a TCP connection to the host, read
 *                                   <down> bytes, send <up> bytes
 *   UDPSEND <ip> <port> <n> <size> <gap_ms>
 *   DNS <name>                      resolve a name
 *   FLAP                            power the PHY down and up, as if the cable
 *                                   was pulled out and put back
 *   CHAIN <port> <n>                broadcast <n> UDP datagrams sent as chains of pbufs
 *   LOSTCHIP                        reset the chip behind the back of the interface
 *   DIAG                            registers of the chip and pointers of the driver
 *   HOLD <0|1>                      1: the TCP echo server stops reading, so what the peer sends waits
 *                                   in lwIP
 *   HEAPBLOCK                       the biggest block that the heap of lwIP can give right now
 *   RESET                           zero the counters of the echo servers
 *
 * Wiring: ENC28J60Config defaults (SCK=GP2 MOSI=GP3 MISO=GP4 CS=GP5 RST=GP7).
 * Compile with -DBENCH_IRQ_PIN=6 for the interrupt-driven build, with the chip's
 * INT wired to that GPIO (benchlwip.py --irq 6 does it). -DBENCH_SPI_HZ=n sets the SPI clock
 * of the chip to n Hz instead of the default of the library (benchlwip.py --spi-hz n).
 */
#include <LwipEthernet.h>
#include <LwipIntfDev.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/dhcp.h>
#include <lwip/mem.h>
#include <lwip/netif.h>
#include <lwip/prot/dhcp.h>

// The C driver, for FLAP, which has to reach the PHY of the chip
extern "C" {
#include "enc28j60-cmd.h"
#include "pico/enc28j60-driver.h"
}

// FLAP goes below the interface, to the driver of the chip
#define private public
#include <PicoENC28J60lwIP.h>
#undef private

#ifndef BENCH_IRQ_PIN
#define BENCH_IRQ_PIN -1
#endif

static ENC28J60Config makeConfig()
{
    ENC28J60Config c;  // default pins
    c.irq_pin = BENCH_IRQ_PIN;
#ifdef BENCH_SPI_HZ
    c.spi_hz = BENCH_SPI_HZ;
#endif
    return c;
}
PicoENC28J60lwIP eth(makeConfig());

static const uint16_t PORT_TCP_ECHO = 7007;
static const uint16_t PORT_UDP_ECHO = 7008;
static const uint16_t PHY_PWRSV = 0x0800;  // PHCON1.PPWRSV

WiFiServer tcpEcho(PORT_TCP_ECHO);
WiFiUDP udpEcho;
static WiFiClient conns[4];
static bool serving = false;

static struct {
    uint32_t accepted, closed, bytes, rejected;  // TCP echo
    uint32_t dgrams, dbytes;                      // UDP echo
} srv;

// How lwIP told us about the link (netif_set_link_callback)
static struct {
    uint32_t up, down;
} linkcb;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static String ipText(const IPAddress& ip)
{
    return ip.toString();
}

static bool parseIp(const char* s, IPAddress& ip)
{
    return ip.fromString(s);
}

static struct dhcp* dhcpData()
{
    netif* n = eth.getNetIf();
    return n ? netif_dhcp_data(n) : nullptr;
}

static uint32_t dhcpXid()
{
    struct dhcp* d = dhcpData();
    return d ? d->xid : 0;
}

static int dhcpState()
{
    struct dhcp* d = dhcpData();
    return d ? d->state : -1;
}

static void linkCallback(netif* n)
{
    if (netif_is_link_up(n)) {
        linkcb.up++;
    } else {
        linkcb.down++;
    }
}

// The pattern of the bytes of the CLIENT test; the host uses the same ones
static inline uint8_t downByte(uint32_t i) { return (uint8_t)(i * 31 + 7); }
static inline uint8_t upByte(uint32_t i) { return (uint8_t)(i * 17 + 3); }

// ---------------------------------------------------------------------------
// Servers
// ---------------------------------------------------------------------------
static void startServers()
{
    tcpEcho.begin();
    tcpEcho.setNoDelay(true);
    udpEcho.begin(PORT_UDP_ECHO);
    serving = true;
}

static void stopServers()
{
    serving = false;
    for (auto& c : conns) {
        c.stop();
        c = WiFiClient();
    }
    tcpEcho.stop();
    udpEcho.stop();
}

static bool holdTcp = false;  // HOLD 1: what the peer sends stays in lwIP

static void serveTcp()
{
    WiFiClient c = tcpEcho.accept();
    if (c) {
        bool placed = false;
        for (auto& slot : conns) {
            if (!(slot.connected() || slot.available())) {
                slot = c;
                srv.accepted++;
                placed = true;
                break;
            }
        }
        if (!placed) {
            srv.rejected++;
            c.stop();
        }
    }
    for (auto& cl : conns) {
        if (!(cl.connected() || cl.available())) {
            continue;
        }
        int n = cl.available();
        if (n > 0 && holdTcp) {
            continue;
        }
        if (n > 0) {
            static uint8_t buf[512];  // not on the stack: it has 2 KB in the core, and lwIP runs on top of it
            if (n > (int)sizeof buf) {
                n = sizeof buf;
            }
            int r = cl.read(buf, n);
            size_t sent = 0;
            uint32_t t0 = millis();
            while (r > 0 && sent < (size_t)r && millis() - t0 < 3000) {
                size_t w = cl.write(buf + sent, r - sent);
                sent += w;
                if (w == 0) {
                    delay(1);  // the send buffer is full
                }
            }
            srv.bytes += sent;
        } else if (!cl.connected()) {
            cl.stop();  // closed by the peer and everything was read
            srv.closed++;
        }
    }
}

static void serveUdp()
{
    static uint8_t buf[4096];  // longer than the MTU: lwIP puts fragments together, and takes them apart again
    int n = udpEcho.parsePacket();
    while (n > 0) {
        int r = udpEcho.read(buf, sizeof buf);
        IPAddress ip = udpEcho.remoteIP();
        uint16_t port = udpEcho.remotePort();
        if (r > 0) {
            udpEcho.beginPacket(ip, port);
            udpEcho.write(buf, r);
            udpEcho.endPacket();
            srv.dgrams++;
            srv.dbytes += r;
        }
        n = udpEcho.parsePacket();
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static bool started = false;  // START worked and STOP has not been given since

static void cmdStart(char* args)
{
    if (started) {
        stopServers();
        eth.end();  // a START after a START begins again
        started = false;
    }
    char* mode = strtok(args, " ");
    bool ok = false;
    IPAddress ip, gw, mask;
    if (mode && strcmp(mode, "static") == 0) {
        char* a = strtok(nullptr, " ");
        char* b = strtok(nullptr, " ");
        char* c = strtok(nullptr, " ");
        if (!a || !b || !c || !parseIp(a, ip) || !parseIp(b, gw) || !parseIp(c, mask)) {
            Serial.println("START ok=0 err=args");
            return;
        }
        eth.config(ip, gw, mask);
    } else {
        eth.config(IPAddress(0, 0, 0, 0));  // DHCP
    }
    eth.hostname("benchlwip");

    uint32_t t0 = millis();
    ok = eth.begin();
    if (!ok) {
        Serial.printf("START ok=0 err=%s\n", PicoENC28J60::errorString(eth.lastError()));
        return;
    }
    netif_set_link_callback(eth.getNetIf(), linkCallback);
    linkcb.up = linkcb.down = 0;

    while (!eth.connected() && millis() - t0 < 30000) {
        delay(10);
    }
    uint32_t ms = millis() - t0;
    if (!eth.connected()) {
        Serial.printf("START ok=0 err=noaddress ms=%lu\n", (unsigned long)ms);
        return;
    }
    startServers();
    started = true;
    Serial.printf("START ok=1 ms=%lu ip=%s gw=%s mask=%s dns=%s xid=%lu\n", (unsigned long)ms,
                  ipText(eth.localIP()).c_str(), ipText(eth.gatewayIP()).c_str(),
                  ipText(eth.subnetMask()).c_str(), ipText(eth.dnsIP()).c_str(),
                  (unsigned long)dhcpXid());
}

static void cmdStop()
{
    uint32_t heap0 = rp2040.getFreeHeap();
    stopServers();
    eth.end();
    started = false;
    uint32_t heap1 = rp2040.getFreeHeap();
    Serial.printf("STOP heap_before=%lu heap_after=%lu\n", (unsigned long)heap0, (unsigned long)heap1);
}

static void cmdInfo()
{
    const PicoENC28J60::Stats& s = eth.driverStats();
    Serial.printf(
        "INFO link=%d connected=%d ip=%s dhcp_state=%d xid=%lu heap=%lu rx=%lu tx=%lu txfail=%lu io=%lu "
        "rec=%lu linkcb_up=%lu linkcb_down=%lu tcp_accepted=%lu tcp_closed=%lu tcp_bytes=%lu tcp_rejected=%lu "
        "udp_dgrams=%lu udp_bytes=%lu irqpin=%d nobuf=%lu\n",
        eth.linkStatus() == LinkON, eth.connected(), ipText(eth.localIP()).c_str(), dhcpState(),
        (unsigned long)dhcpXid(), (unsigned long)rp2040.getFreeHeap(), (unsigned long)s.rx_frames,
        (unsigned long)s.tx_frames, (unsigned long)s.tx_failures, (unsigned long)s.io_errors,
        (unsigned long)s.recoveries, (unsigned long)linkcb.up, (unsigned long)linkcb.down,
        (unsigned long)srv.accepted, (unsigned long)srv.closed, (unsigned long)srv.bytes,
        (unsigned long)srv.rejected, (unsigned long)srv.dgrams, (unsigned long)srv.dbytes, BENCH_IRQ_PIN,
        (unsigned long)eth.droppedNoBuffer());
}

static void cmdPing(char* args)
{
    IPAddress ip;
    if (!args || !parseIp(args, ip)) {
        Serial.println("PING ok=0 err=args");
        return;
    }
    int ok = 0;
    uint32_t t0 = micros();
    for (int i = 0; i < 5; i++) {
        if (eth.ping(ip, 64, 1000) > 0) {
            ok++;
        }
    }
    Serial.printf("PING ok=%d of=5 us=%lu\n", ok, (unsigned long)(micros() - t0));
}

static void cmdClient(char* args)
{
    char* a = strtok(args, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    IPAddress ip;
    if (!a || !b || !c || !d || !parseIp(a, ip)) {
        Serial.println("CLIENT ok=0 err=args");
        return;
    }
    uint16_t port = atoi(b);
    uint32_t down = strtoul(c, nullptr, 10), up = strtoul(d, nullptr, 10);

    WiFiClient cl;
    cl.setTimeout(5000);
    uint32_t t0 = millis();
    if (!cl.connect(ip, port)) {
        Serial.println("CLIENT ok=0 err=connect");
        return;
    }
    uint32_t connectMs = millis() - t0;

    // Download: the host sends <down> bytes of a known pattern
    t0 = millis();
    uint32_t got = 0, bad = 0;
    static uint8_t buf[512];  // see serveTcp()
    uint32_t idle = millis();
    while (got < down && millis() - idle < 5000) {
        int n = cl.available();
        if (n <= 0) {
            if (!cl.connected()) {
                break;
            }
            delay(1);
            continue;
        }
        if (n > (int)sizeof buf) {
            n = sizeof buf;
        }
        if ((uint32_t)n > down - got) {
            n = down - got;
        }
        int r = cl.read(buf, n);
        for (int i = 0; i < r; i++) {
            if (buf[i] != downByte(got + i)) {
                bad++;
            }
        }
        got += r;
        idle = millis();
    }
    uint32_t downMs = millis() - t0;

    // Upload: <up> bytes of another pattern
    t0 = millis();
    uint32_t sent = 0;
    idle = millis();
    while (sent < up && millis() - idle < 5000) {
        uint32_t n = up - sent;
        if (n > sizeof buf) {
            n = sizeof buf;
        }
        for (uint32_t i = 0; i < n; i++) {
            buf[i] = upByte(sent + i);
        }
        size_t w = cl.write(buf, n);
        if (w == 0) {
            delay(1);
            continue;
        }
        // a short write: the rest of the buffer goes first next time
        sent += w;
        idle = millis();
    }
    cl.flush();
    uint32_t upMs = millis() - t0;
    cl.stop();
    Serial.printf("CLIENT ok=%d connect_ms=%lu down=%lu bad=%lu down_ms=%lu up=%lu up_ms=%lu\n",
                  (got == down && bad == 0 && sent == up), (unsigned long)connectMs, (unsigned long)got,
                  (unsigned long)bad, (unsigned long)downMs, (unsigned long)sent, (unsigned long)upMs);
}

static void cmdUdpSend(char* args)
{
    char* a = strtok(args, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    char* e = strtok(nullptr, " ");
    IPAddress ip;
    if (!a || !b || !c || !d || !e || !parseIp(a, ip)) {
        Serial.println("UDPSEND ok=0 err=args");
        return;
    }
    uint16_t port = atoi(b);
    uint32_t n = strtoul(c, nullptr, 10), size = strtoul(d, nullptr, 10), gap = strtoul(e, nullptr, 10);
    if (size < 8 || size > 1472) {
        Serial.println("UDPSEND ok=0 err=size");
        return;
    }
    WiFiUDP u;
    static uint8_t buf[1472];
    uint32_t sent = 0, failed = 0;
    const PicoENC28J60::Stats& s = eth.driverStats();
    uint32_t frames0 = s.tx_frames, fails0 = s.tx_failures;
    uint32_t t0 = micros();  // with gap 0 the time says how fast frames go out through lwIP
    for (uint32_t i = 0; i < n; i++) {
        buf[0] = 'U';
        buf[1] = 'D';
        buf[2] = 'P';
        buf[3] = 'B';
        buf[4] = i >> 24;
        buf[5] = i >> 16;
        buf[6] = i >> 8;
        buf[7] = i;
        for (uint32_t j = 8; j < size; j++) {
            buf[j] = (uint8_t)(i * 13 + j);
        }
        if (u.beginPacket(ip, port) && u.write(buf, size) == size && u.endPacket()) {
            sent++;
        } else {
            failed++;
        }
        if (gap) {
            delay(gap);
        }
    }
    uint32_t us = micros() - t0;
    // The counters of the driver: a frame counts once it has left the wire, and the last one is seen by the polling
    for (uint32_t w0 = millis(); s.tx_frames - frames0 + s.tx_failures - fails0 < sent && millis() - w0 < 20;) {
        delay(1);
    }
    Serial.printf("UDPSEND ok=%d sent=%lu failed=%lu us=%lu spihz=%lu txframes=%lu txfail=%lu\n", failed == 0,
                  (unsigned long)sent, (unsigned long)failed, (unsigned long)us, (unsigned long)makeConfig().spi_hz,
                  (unsigned long)(s.tx_frames - frames0), (unsigned long)(s.tx_failures - fails0));
}

static void cmdDns(char* args)
{
    if (!args || !*args) {
        Serial.println("DNS ok=0 err=args");
        return;
    }
    IPAddress ip;
    uint32_t t0 = millis();
    int r = eth.hostByName(args, ip, 5000);
    Serial.printf("DNS ok=%d ip=%s ms=%lu\n", r == 1, ipText(ip).c_str(), (unsigned long)(millis() - t0));
}

// Pull the cable out and put it back: power the PHY down and up
static void cmdFlap()
{
    enc28j60* drv = static_cast<enc28j60*>(eth._eth.getDriverHandle());
    uint32_t xid0 = dhcpXid();
    uint32_t up0 = linkcb.up, down0 = linkcb.down;

    uint32_t t0 = millis();
    {
        LWIPMutex m;
        reg_write_p_blk(drv, PHCON1, PHY_PWRSV);
    }
    while (eth.linkStatus() != LinkOFF && millis() - t0 < 3000) {
        delay(5);
    }
    uint32_t downMs = millis() - t0;
    bool sawDown = eth.linkStatus() == LinkOFF;
    uint32_t cbDown = linkcb.down - down0;

    delay(1000);  // a cable that is pulled out stays out for a while

    t0 = millis();
    {
        LWIPMutex m;
        reg_write_p_blk(drv, PHCON1, 0);
    }
    while (eth.linkStatus() != LinkON && millis() - t0 < 10000) {
        delay(5);
    }
    uint32_t upMs = millis() - t0;
    bool sawUp = eth.linkStatus() == LinkON;

    // With DHCP, lwIP asks the server again when the link comes back
    bool dhcp = dhcpData() != nullptr && dhcpXid() != 0;
    uint32_t rebindMs = 0;
    bool rebound = !dhcp;
    if (dhcp) {
        t0 = millis();
        while (millis() - t0 < 20000) {
            if (dhcpXid() != xid0 && dhcpState() == DHCP_STATE_BOUND && eth.connected()) {
                rebound = true;
                break;
            }
            delay(10);
        }
        rebindMs = millis() - t0;
    }
    Serial.printf(
        "FLAP down=%d down_ms=%lu up=%d up_ms=%lu cb_down=%lu cb_up=%lu dhcp=%d rebound=%d rebind_ms=%lu "
        "xid0=%lu xid1=%lu ip=%s\n",
        sawDown, (unsigned long)downMs, sawUp, (unsigned long)upMs, (unsigned long)cbDown,
        (unsigned long)(linkcb.up - up0), dhcp, rebound, (unsigned long)rebindMs, (unsigned long)xid0,
        (unsigned long)dhcpXid(), ipText(eth.localIP()).c_str());
}

// CHAIN <port> <count>: broadcast UDP datagrams, sent as chains of three pbufs straight to the interface. lwIP
// hands over a single buffer, because the core asks it to, but the interface does not depend on that.
static uint16_t ipChecksum(const uint8_t* p, size_t n)
{
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < n; i += 2) {
        sum += (p[i] << 8) | p[i + 1];
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return ~sum;
}

static void cmdChain(char* args)
{
    char* a = strtok(args, " ");
    char* b = strtok(nullptr, " ");
    if (!a || !b) {
        Serial.println("CHAIN ok=0 err=args");
        return;
    }
    uint16_t port = atoi(a);
    uint32_t count = strtoul(b, nullptr, 10);
    const uint16_t PAYLOAD = 300;
    const uint16_t HEADERS = 14 + 20 + 8;
    static uint8_t frame[HEADERS + PAYLOAD];
    uint32_t sent = 0, failed = 0;
    for (uint32_t i = 0; i < count; i++) {
        memset(frame, 0xFF, 6);  // broadcast
        memcpy(frame + 6, eth.getNetIf()->hwaddr, 6);
        frame[12] = 0x08;
        frame[13] = 0x00;
        uint8_t* ip = frame + 14;
        ip[0] = 0x45;
        ip[1] = 0;
        ip[2] = (20 + 8 + PAYLOAD) >> 8;
        ip[3] = (20 + 8 + PAYLOAD) & 0xFF;
        ip[4] = i >> 8;
        ip[5] = i;
        ip[6] = ip[7] = 0;
        ip[8] = 64;
        ip[9] = 17;
        ip[10] = ip[11] = 0;
        uint32_t src = (uint32_t)eth.localIP();
        memcpy(ip + 12, &src, 4);
        memset(ip + 16, 0xFF, 4);
        uint16_t sum = ipChecksum(ip, 20);
        ip[10] = sum >> 8;
        ip[11] = sum & 0xFF;
        uint8_t* udp = ip + 20;
        udp[0] = PORT_UDP_ECHO >> 8;
        udp[1] = PORT_UDP_ECHO & 0xFF;
        udp[2] = port >> 8;
        udp[3] = port & 0xFF;
        udp[4] = (8 + PAYLOAD) >> 8;
        udp[5] = (8 + PAYLOAD) & 0xFF;
        udp[6] = udp[7] = 0;  // no checksum
        uint8_t* data = udp + 8;
        memcpy(data, "CHAN", 4);
        data[4] = i >> 24;
        data[5] = i >> 16;
        data[6] = i >> 8;
        data[7] = i;
        for (uint16_t j = 8; j < PAYLOAD; j++) {
            data[j] = (uint8_t)(i * 7 + j);
        }

        // three pieces: part of the headers, the rest of them with some data, the rest of the data
        pbuf* p1 = pbuf_alloc(PBUF_RAW, 20, PBUF_RAM);
        pbuf* p2 = pbuf_alloc(PBUF_RAW, 100, PBUF_RAM);
        pbuf* p3 = pbuf_alloc(PBUF_RAW, HEADERS + PAYLOAD - 120, PBUF_RAM);
        if (!p1 || !p2 || !p3) {
            pbuf_free(p1);
            pbuf_free(p2);
            pbuf_free(p3);
            failed++;
            continue;
        }
        pbuf_take(p1, frame, 20);
        pbuf_take(p2, frame + 20, 100);
        pbuf_take(p3, frame + 120, HEADERS + PAYLOAD - 120);
        pbuf_cat(p1, p2);
        pbuf_cat(p1, p3);
        uint16_t r;
        {
            LWIPMutex m;
            r = eth.sendFrame(p1);
        }
        if (r != 0) {
            sent++;
        } else {
            failed++;
        }
        pbuf_free(p1);  // frees the whole chain
        delay(5);
    }
    Serial.printf("CHAIN ok=%d sent=%lu failed=%lu\n", failed == 0, (unsigned long)sent, (unsigned long)failed);
}

// LOSTCHIP: reset the chip behind the back of the interface, as a glitch of its supply would. The interface
// has to find out by itself, which takes up to a second, and bring the chip back.
static void cmdLostChip()
{
    enc28j60* drv = static_cast<enc28j60*>(eth._eth.getDriverHandle());
    uint32_t before = eth.driverStats().chip_resets;
    {
        LWIPMutex m;
        system_reset(drv);
    }
    uint32_t t0 = millis();
    while (eth.driverStats().chip_resets == before && millis() - t0 < 10000) {
        delay(10);
    }
    uint32_t foundMs = millis() - t0;
    bool found = eth.driverStats().chip_resets != before;
    bool alive;
    {
        LWIPMutex m;
        alive = enc28j60_alive_blk(drv);
    }
    Serial.printf("LOSTCHIP found=%d found_ms=%lu alive=%d resets=%lu recoveries=%lu\n", found,
                  (unsigned long)foundMs, alive, (unsigned long)(eth.driverStats().chip_resets - before),
                  (unsigned long)eth.driverStats().recoveries);
}

// The registers of the chip and the pointers of the driver, for finding out why a transfer failed
static void cmdDiag()
{
    enc28j60* drv = static_cast<enc28j60*>(eth._eth.getDriverHandle());
    uint8_t econ1, econ2, eir, eie, estat, pktcnt, macon1, macon3, macon4;
    uint16_t txst, txnd, wrpt, rdpt, rxrdpt, rxwrpt, phstat1, phstat2;
    {
        LWIPMutex m;
        econ1 = reg_read_e_blk(drv, ECON1);
        econ2 = reg_read_e_blk(drv, ECON2);
        eir = reg_read_e_blk(drv, EIR);
        eie = reg_read_e_blk(drv, EIE);
        estat = reg_read_e_blk(drv, ESTAT);
        bank_set_blk(drv, 0);
        auto r16 = [&](uint8_t lo) { return (uint16_t)(reg_read_e_blk(drv, lo) | reg_read_e_blk(drv, lo + 1) << 8); };
        txst = r16(ETXSTL);
        txnd = r16(ETXNDL);
        wrpt = r16(EWRPTL);
        rdpt = r16(ERDPTL);
        rxrdpt = r16(ERXRDPTL);
        rxwrpt = r16(ERXWRPTL);
        bank_set_blk(drv, 1);
        pktcnt = reg_read_e_blk(drv, EPKTCNT);
        bank_set_blk(drv, 2);
        macon1 = reg_read_m_blk(drv, MACON1);
        macon3 = reg_read_m_blk(drv, MACON3);
        macon4 = reg_read_m_blk(drv, MACON4);
        phstat1 = reg_read_p_blk(drv, PHSTAT1);
        phstat2 = reg_read_p_blk(drv, 0x11);
        bank_set_blk(drv, 0);
    }
    const PicoENC28J60::Stats& s = eth.driverStats();
    Serial.printf(
        "DIAG econ1=%02x econ2=%02x eir=%02x eie=%02x estat=%02x pktcnt=%u macon1=%02x macon3=%02x macon4=%02x "
        "phstat1=%04x phstat2=%04x txst=%u txnd=%u wrpt=%u rdpt=%u rxrdpt=%u rxwrpt=%u drv_wptr=%u drv_rptr=%u "
        "tx_frames=%lu tx_failures=%lu tx_aborts=%lu tx_retries=%lu rx_frames=%lu\n",
        econ1, econ2, eir, eie, estat, pktcnt, macon1, macon3, macon4, phstat1, phstat2, txst, txnd, wrpt, rdpt, rxrdpt,
        rxwrpt, drv->wptr, drv->dev_rptr, (unsigned long)s.tx_frames, (unsigned long)s.tx_failures,
        (unsigned long)s.tx_aborts, (unsigned long)s.tx_retries, (unsigned long)s.rx_frames);
}

// HEAPBLOCK: the biggest block that the heap of lwIP (16 KB in the core) can give right now. A frame that
// waits in lwIP must not take its place there.
static void cmdHeapBlock()
{
    static const uint16_t SIZES[] = {16384, 14336, 12288, 10240, 8192, 6144, 4096, 3072, 2048, 1514, 1024, 512};
    uint16_t largest = 0;
    ethernet_arch_lwip_begin();
    for (uint16_t size : SIZES) {
        void* block = mem_malloc(size);
        if (block) {
            mem_free(block);
            largest = size;
            break;
        }
    }
    ethernet_arch_lwip_end();
    Serial.printf("HEAPBLOCK largest=%u\n", largest);
}

static void handleLine(char* line)
{
    char* args = strchr(line, ' ');
    if (args) {
        *args++ = 0;
    }
    if (strcmp(line, "ID") == 0) {
        Serial.println("READY lwip");
    } else if (strcmp(line, "START") == 0) {
        cmdStart(args ? args : (char*)"dhcp");
    } else if (strcmp(line, "STOP") == 0) {
        cmdStop();
    } else if (strcmp(line, "INFO") == 0) {
        cmdInfo();
    } else if (strcmp(line, "PING") == 0) {
        cmdPing(args);
    } else if (strcmp(line, "CLIENT") == 0) {
        cmdClient(args);
    } else if (strcmp(line, "UDPSEND") == 0) {
        cmdUdpSend(args);
    } else if (strcmp(line, "DNS") == 0) {
        cmdDns(args);
    } else if (strcmp(line, "FLAP") == 0) {
        cmdFlap();
    } else if (strcmp(line, "CHAIN") == 0) {
        cmdChain(args);
    } else if (strcmp(line, "LOSTCHIP") == 0) {
        cmdLostChip();
    } else if (strcmp(line, "DIAG") == 0) {
        cmdDiag();
    } else if (strcmp(line, "HOLD") == 0) {
        holdTcp = args && args[0] == '1';
        Serial.println("HOLD ok=1");
    } else if (strcmp(line, "HEAPBLOCK") == 0) {
        cmdHeapBlock();
    } else if (strcmp(line, "RESET") == 0) {
        memset(&srv, 0, sizeof srv);
        Serial.println("RESET ok=1");
    } else {
        Serial.printf("ERR unknown command '%s'\n", line);
    }
}

void setup()
{
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && millis() - t < 3000;) {
        delay(10);
    }
}

void loop()
{
    static char line[96];
    static size_t len = 0;
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len) {
                line[len] = 0;
                handleLine(line);
                len = 0;
            }
        } else if (len < sizeof line - 1) {
            line[len++] = c;
        }
    }
    if (serving) {
        serveTcp();
        serveUdp();
    }
}
