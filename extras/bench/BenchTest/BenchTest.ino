/*
 * BenchTest - on-hardware self-test for PicoENC28J60.
 *
 * Driven over USB serial by extras/bench/bench.py, which also generates the
 * LAN traffic that this sketch checks (ping, UDP broadcast, UDP unicast).
 * The sketch implements just enough ARP, ICMP echo and UDP to be a test
 * target. It is NOT a network stack; see examples/WebServerLedToggle for
 * real networking on top of lwIP.
 *
 * Wiring: ENC28J60Config defaults (SCK=GP2 MOSI=GP3 MISO=GP4 CS=GP5 RST=GP7).
 *
 * Interrupt build: compile with -DBENCH_IRQ_PIN=6 (bench.py --irq 6 does that)
 * with the chip's INT wired to that GPIO. loop() then touches the chip only
 * when the INT line asks for it, so every test also proves that the interrupt
 * alone drives reception. The LINK command is available in that build.
 */
#include <PicoENC28J60.h>

// Register access for the PHY power cycle of the LINK command
extern "C" {
#include "pico/enc28j60-driver.h"
#include "enc28j60-cmd.h"
}

// ---------------------------------------------------------------------------
// Test protocol (must match bench.py)
// ---------------------------------------------------------------------------
static const uint16_t PORT_RX = 9999;    // host -> Pico broadcast stream, counted and verified
static const uint16_t PORT_TX = 9998;    // Pico -> host broadcast stream (TXB command)
static const uint16_t PORT_ECHO = 9997;  // host -> Pico unicast, echoed back; sender becomes the TX peer
static const uint16_t HDR_LEN = 10;      // "PBNC" + seq(4) + payload length(2)

static const uint8_t MAC[6] = {0x02, 0x50, 0x49, 0x43, 0x4F, 0x01};
static uint8_t myIp[4];  // set with the IP command; 0.0.0.0 = unset

static ENC28J60Config makeConfig()
{
    ENC28J60Config c;  // default pins
    memcpy(c.mac_addr, MAC, 6);
#ifdef BENCH_IRQ_PIN
    c.irq_pin = BENCH_IRQ_PIN;
#endif
    return c;
}
PicoENC28J60 eth(makeConfig());

static uint8_t rxbuf[1600];
static uint8_t txbuf[1600];

// Result of the error-path probe that runs before the real initialization
static bool probeOk = true;
static uint32_t probeMs = 0;

struct Stats {
    uint32_t frames, bench, dup, stale, gap, bad;
    uint32_t arp, icmp, echo, other, runt;
    uint32_t extra0, extra4, extraX;  // bytes after the IP datagram in a full-size frame
    uint32_t rxz;                     // receiveFrame() returned 0 although available() > 0
    uint32_t tx, txf;
    uint32_t irqSvc, irqLink, irqRxErr;  // interrupt build: service passes, link and overflow events
    uint32_t irqFlags;                   // OR of every flag byte read
} st;
static uint32_t expectedSeq;
static uint32_t irqBase;  // interrupt count at the last RESET

// LINK command: power the PHY down and up again and watch the interrupt report it
static struct {
    uint8_t phase;  // 0 idle, 1 PHY powered down, 2 powered up again and waiting for the link
    uint32_t t0, t1, downMs, upMs;
} lk;
static const uint16_t PHY_PWRSV = 0x0800;  // PHCON1.PPWRSV

struct Peer {
    bool valid;
    uint8_t mac[6];
    uint8_t ip[4];
    uint16_t port;
} peer;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t rd32(const uint8_t* p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline void wr16(uint8_t* p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}
static inline void wr32(uint8_t* p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static inline uint8_t fill(uint32_t seq, uint16_t i) { return (uint8_t)(seq * 131u + i * 7u + 13u); }

static uint32_t sum16(const uint8_t* p, uint16_t n, uint32_t s = 0)
{
    while (n > 1) {
        s += (uint32_t)((p[0] << 8) | p[1]);
        p += 2;
        n -= 2;
    }
    if (n) s += (uint32_t)(p[0] << 8);
    return s;
}
static uint16_t fold(uint32_t s)
{
    while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
    return (uint16_t)~s;
}

static uint16_t udpChecksum(const uint8_t* ip, const uint8_t* udp, uint16_t udpLen)
{
    uint32_t s = sum16(ip + 12, 8);  // source + destination address
    s += 17;                         // protocol
    s += udpLen;
    s = sum16(udp, udpLen, s);
    uint16_t c = fold(s);
    return c ? c : 0xFFFF;
}

static bool sendTracked(const uint8_t* f, uint16_t len)
{
    bool ok = eth.sendFrame(f, len);
    if (ok) st.tx++;
    else st.txf++;
    return ok;
}

// Writes Ethernet + IPv4 + UDP headers for a payload of plen bytes that the
// caller already stored at f + 42. Returns the frame length.
static uint16_t finishUdp(uint8_t* f, const uint8_t* dmac, const uint8_t* dip, uint16_t sport,
                          uint16_t dport, uint16_t plen, uint16_t id)
{
    memcpy(f, dmac, 6);
    memcpy(f + 6, MAC, 6);
    wr16(f + 12, 0x0800);
    uint8_t* ip = f + 14;
    ip[0] = 0x45;
    ip[1] = 0;
    wr16(ip + 2, 28 + plen);
    wr16(ip + 4, id);
    wr16(ip + 6, 0x4000);
    ip[8] = 64;
    ip[9] = 17;
    wr16(ip + 10, 0);
    memcpy(ip + 12, myIp, 4);
    memcpy(ip + 16, dip, 4);
    wr16(ip + 10, fold(sum16(ip, 20)));
    uint8_t* u = ip + 20;
    wr16(u, sport);
    wr16(u + 2, dport);
    wr16(u + 4, 8 + plen);
    wr16(u + 6, 0);
    wr16(u + 6, udpChecksum(ip, u, 8 + plen));
    return 42 + plen;
}

// ---------------------------------------------------------------------------
// Frame handlers
// ---------------------------------------------------------------------------
static void handleArp(uint8_t* f, uint16_t len)
{
    st.arp++;
    if (len < 42 || (myIp[0] | myIp[1] | myIp[2] | myIp[3]) == 0) return;
    uint8_t* a = f + 14;
    if (rd16(a) != 1 || rd16(a + 2) != 0x0800 || a[4] != 6 || a[5] != 4 || rd16(a + 6) != 1) return;
    if (memcmp(a + 24, myIp, 4) != 0) return;

    uint8_t r[60] = {0};
    memcpy(r, a + 8, 6);
    memcpy(r + 6, MAC, 6);
    wr16(r + 12, 0x0806);
    uint8_t* ra = r + 14;
    wr16(ra, 1);
    wr16(ra + 2, 0x0800);
    ra[4] = 6;
    ra[5] = 4;
    wr16(ra + 6, 2);
    memcpy(ra + 8, MAC, 6);
    memcpy(ra + 14, myIp, 4);
    memcpy(ra + 18, a + 8, 6);
    memcpy(ra + 24, a + 14, 4);
    sendTracked(r, 60);
}

static void handleIcmp(uint8_t* f, uint8_t* ip, uint16_t ihl, uint16_t ipTot)
{
    uint8_t* ic = ip + ihl;
    uint16_t icLen = ipTot - ihl;
    if (icLen < 8 || ic[0] != 8 || ic[1] != 0) return;  // echo request only
    if (memcmp(ip + 16, myIp, 4) != 0) return;
    st.icmp++;

    uint8_t t6[6];
    memcpy(t6, f, 6);
    memcpy(f, f + 6, 6);
    memcpy(f + 6, t6, 6);
    uint8_t t4[4];
    memcpy(t4, ip + 12, 4);
    memcpy(ip + 12, ip + 16, 4);
    memcpy(ip + 16, t4, 4);
    ip[8] = 64;
    wr16(ip + 10, 0);
    wr16(ip + 10, fold(sum16(ip, ihl)));
    ic[0] = 0;
    wr16(ic + 2, 0);
    wr16(ic + 2, fold(sum16(ic, icLen)));
    sendTracked(f, 14 + ipTot);
}

static void rxBench(const uint8_t* p, uint16_t plen, uint16_t frameLen, uint16_t ipTot)
{
    if (plen < HDR_LEN || memcmp(p, "PBNC", 4) != 0 || rd16(p + 8) != plen) {
        st.bad++;
        return;
    }
    uint32_t seq = rd32(p + 4);
    for (uint16_t i = HDR_LEN; i < plen; i++) {
        if (p[i] != fill(seq, i)) {
            st.bad++;
            return;
        }
    }
    // For frames without Ethernet padding, anything after the IP datagram is
    // the FCS, so this reveals whether receiveFrame() includes it.
    if (ipTot >= 46) {
        uint16_t extra = frameLen - (14 + ipTot);
        if (extra == 0) st.extra0++;
        else if (extra == 4) st.extra4++;
        else st.extraX++;
    }
    if (seq == expectedSeq) {
        st.bench++;
        expectedSeq++;
    } else if (seq < expectedSeq) {
        if (seq + 1 == expectedSeq) st.dup++;
        else st.stale++;
    } else {
        st.gap += seq - expectedSeq;
        st.bench++;
        expectedSeq = seq + 1;
    }
}

static void echoBack(uint8_t* f, uint8_t* ip, uint16_t ihl, uint16_t ipTot)
{
    uint8_t* u = ip + ihl;
    // Remember the sender so that the TX command can stream back to it.
    memcpy(peer.mac, f + 6, 6);
    memcpy(peer.ip, ip + 12, 4);
    peer.port = rd16(u);
    peer.valid = true;
    st.echo++;

    uint8_t t6[6];
    memcpy(t6, f, 6);
    memcpy(f, f + 6, 6);
    memcpy(f + 6, t6, 6);
    uint8_t t4[4];
    memcpy(t4, ip + 12, 4);
    memcpy(ip + 12, ip + 16, 4);
    memcpy(ip + 16, t4, 4);
    uint8_t t2[2];
    memcpy(t2, u, 2);
    memcpy(u, u + 2, 2);
    memcpy(u + 2, t2, 2);
    // Swapping addresses and ports does not change either checksum.
    sendTracked(f, 14 + ipTot);
}

static void handleUdp(uint8_t* f, uint16_t len, uint8_t* ip, uint16_t ihl, uint16_t ipTot)
{
    if (ipTot < ihl + 8) return;
    uint8_t* u = ip + ihl;
    uint16_t dport = rd16(u + 2);
    uint16_t ulen = rd16(u + 4);
    if (ulen < 8 || ulen > ipTot - ihl) return;
    if (dport == PORT_RX) {
        rxBench(u + 8, ulen - 8, len, ipTot);
    } else if (dport == PORT_ECHO && memcmp(ip + 16, myIp, 4) == 0) {
        echoBack(f, ip, ihl, ipTot);
    } else {
        st.other++;
    }
}

static void handleFrame(uint8_t* f, uint16_t len)
{
    st.frames++;
    if (len < 14) {
        st.runt++;
        return;
    }
    uint16_t type = rd16(f + 12);
    if (type == 0x0806) {
        handleArp(f, len);
        return;
    }
    if (type != 0x0800 || len < 34) {
        st.other++;
        return;
    }
    uint8_t* ip = f + 14;
    uint16_t ihl = (ip[0] & 0x0F) * 4;
    uint16_t ipTot = rd16(ip + 2);
    if ((ip[0] >> 4) != 4 || ihl < 20 || ipTot < ihl || 14 + ipTot > len) {
        st.other++;
        return;
    }
    if (ip[9] == 1) handleIcmp(f, ip, ihl, ipTot);
    else if (ip[9] == 17) handleUdp(f, len, ip, ihl, ipTot);
    else st.other++;
}

// ---------------------------------------------------------------------------
// Interrupt build: link cycle and service routine
// ---------------------------------------------------------------------------
static void startLinkCycle()
{
#ifdef BENCH_IRQ_PIN
    if (lk.phase) {
        Serial.println("LINKERR busy");
        return;
    }
    enc28j60* h = static_cast<enc28j60*>(eth.getDriverHandle());
    // Stop the receiver first, as the datasheet asks before powering down
    reg_bits_clear_blk(h, ECON1, RXEN);
    delay(2);
    reg_write_p_blk(h, PHCON1, reg_read_p_blk(h, PHCON1) | PHY_PWRSV);
    lk.phase = 1;
    lk.t0 = millis();
    lk.downMs = lk.upMs = 0;
    Serial.println("LINKSTART");
#else
    Serial.println("LINKERR needs-irq-build");
#endif
}

#ifdef BENCH_IRQ_PIN
// Called when the link-change flag was read; decides what the change means for the LINK command
static void onLinkChange()
{
    bool up = eth.linkStatus();
    if (lk.phase == 1 && !up && lk.downMs == 0) {
        lk.downMs = millis() - lk.t0;
    } else if (lk.phase == 2 && up) {
        lk.upMs = millis() - lk.t1;
        lk.phase = 0;
        Serial.printf("LINKDONE down_ms=%lu up_ms=%lu lnk=%lu\n", (unsigned long)lk.downMs,
                      (unsigned long)lk.upMs, (unsigned long)st.irqLink);
    }
}

static void linkCycleStep()
{
    if (lk.phase == 1 && millis() - lk.t0 >= 1500) {
        enc28j60* h = static_cast<enc28j60*>(eth.getDriverHandle());
        reg_write_p_blk(h, PHCON1, reg_read_p_blk(h, PHCON1) & ~PHY_PWRSV);
        delay(2);  // the PHY needs a moment before it takes more commands
        reg_bits_set_blk(h, ECON1, RXEN);
        lk.phase = 2;
        lk.t1 = millis();
    } else if (lk.phase == 2 && millis() - lk.t1 >= 15000) {
        lk.phase = 0;
        Serial.printf("LINKFAIL down_ms=%lu lnk=%lu\n", (unsigned long)lk.downMs, (unsigned long)st.irqLink);
    }
}

static void serviceInterrupt()
{
    st.irqSvc++;
    uint8_t flags = eth.getInterruptFlags();  // masks the INT pin until the acknowledge below
    st.irqFlags |= flags;
    if (flags & PicoENC28J60::INT_RX_ERROR) st.irqRxErr++;
    if (flags & PicoENC28J60::INT_LINK_CHANGE) {
        st.irqLink++;
        onLinkChange();
    }
    for (int n = 0; n < 64 && eth.available(); n++) {
        uint16_t len = eth.receiveFrame(rxbuf, sizeof(rxbuf));
        if (len == 0) {
            st.rxz++;
            continue;
        }
        handleFrame(rxbuf, len);
    }
    eth.acknowledgeInterrupts(flags);
}
#endif

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
static int irqPin()
{
#ifdef BENCH_IRQ_PIN
    return BENCH_IRQ_PIN;
#else
    return -1;
#endif
}

static void printInfo()
{
    uint8_t m[6];
    eth.getMACAddress(m);
    Serial.printf("INFO init=%d rev=0x%02X mac=%02x:%02x:%02x:%02x:%02x:%02x link=%d ip=%u.%u.%u.%u pend=%u "
                  "probe_ok=%d probe_ms=%lu irqpin=%d\n",
                  eth.isInitialized(), eth.getRevision(), m[0], m[1], m[2], m[3], m[4], m[5],
                  eth.linkStatus(), myIp[0], myIp[1], myIp[2], myIp[3], eth.available(), probeOk,
                  (unsigned long)probeMs, irqPin());
}

static void printStats()
{
    Serial.printf("STAT frames=%lu bench=%lu dup=%lu stale=%lu gap=%lu bad=%lu arp=%lu icmp=%lu echo=%lu "
                  "other=%lu runt=%lu x0=%lu x4=%lu xo=%lu rxz=%lu tx=%lu txf=%lu pend=%u rxrst=%lu rxdrop=%lu "
                  "irq=%lu svc=%lu lnk=%lu rxe=%lu irqfl=0x%02X int=%d\n",
                  (unsigned long)st.frames, (unsigned long)st.bench, (unsigned long)st.dup,
                  (unsigned long)st.stale, (unsigned long)st.gap, (unsigned long)st.bad,
                  (unsigned long)st.arp, (unsigned long)st.icmp, (unsigned long)st.echo,
                  (unsigned long)st.other, (unsigned long)st.runt, (unsigned long)st.extra0,
                  (unsigned long)st.extra4, (unsigned long)st.extraX, (unsigned long)st.rxz,
                  (unsigned long)st.tx, (unsigned long)st.txf, eth.available(),
                  (unsigned long)eth.stats().rx_resets, (unsigned long)eth.stats().rx_dropped,
                  (unsigned long)(eth.interruptCount() - irqBase), (unsigned long)st.irqSvc, (unsigned long)st.irqLink,
                  (unsigned long)st.irqRxErr, (unsigned)st.irqFlags,
                  irqPin() >= 0 ? digitalRead(irqPin()) : -1);
}

// TX  <count> <payload length> <gap in microseconds>: unicast stream to the last echo peer.
// TXB <count> <payload length> <gap in microseconds>: broadcast stream to 255.255.255.255:PORT_TX.
static void cmdTx(bool broadcast, uint32_t count, uint16_t plen, uint32_t gapUs)
{
    static const uint8_t bcMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static const uint8_t bcIp[4] = {255, 255, 255, 255};
    if (!broadcast && !peer.valid) {
        Serial.println("TXERR nopeer");
        return;
    }
    if (plen < HDR_LEN) plen = HDR_LEN;
    if (plen > 1472) plen = 1472;
    uint32_t ok = 0, fail = 0;
    for (uint32_t s = 0; s < count; s++) {
        uint8_t* p = txbuf + 42;
        memcpy(p, "PBNC", 4);
        wr32(p + 4, s);
        wr16(p + 8, plen);
        for (uint16_t i = HDR_LEN; i < plen; i++) p[i] = fill(s, i);
        uint16_t flen = broadcast
            ? finishUdp(txbuf, bcMac, bcIp, PORT_ECHO, PORT_TX, plen, (uint16_t)s)
            : finishUdp(txbuf, peer.mac, peer.ip, PORT_ECHO, peer.port, plen, (uint16_t)s);
        if (eth.sendFrame(txbuf, flen)) ok++;
        else fail++;
        if (gapUs) delayMicroseconds(gapUs);
    }
    st.tx += ok;
    st.txf += fail;
    Serial.printf("TXDONE sent=%lu fail=%lu\n", (unsigned long)ok, (unsigned long)fail);
}

static void handleCommand(char* line)
{
    unsigned a, b, c, d;
    unsigned long n, g;
    if (strncmp(line, "IP ", 3) == 0 && sscanf(line + 3, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
        myIp[0] = a;
        myIp[1] = b;
        myIp[2] = c;
        myIp[3] = d;
        Serial.println("OK");
    } else if (strcmp(line, "INFO") == 0) {
        printInfo();
    } else if (strcmp(line, "STAT") == 0) {
        printStats();
    } else if (strcmp(line, "RESET") == 0) {
        memset(&st, 0, sizeof(st));
        irqBase = eth.interruptCount();
        expectedSeq = 0;
        peer.valid = false;
        Serial.println("OK");
    } else if (strcmp(line, "ID") == 0) {
        Serial.println("READY bench=1");
    } else if (strcmp(line, "LINK") == 0) {
        startLinkCycle();
    } else if (sscanf(line, "TXB %lu %u %lu", &n, &a, &g) == 3) {
        cmdTx(true, n, (uint16_t)a, g);
    } else if (sscanf(line, "TX %lu %u %lu", &n, &a, &g) == 3) {
        cmdTx(false, n, (uint16_t)a, g);
    } else if (line[0]) {
        Serial.println("ERR unknown");
    }
}

static void pollSerial()
{
    static char line[64];
    static uint8_t len = 0;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            line[len] = 0;
            len = 0;
            handleCommand(line);
        } else if (len < sizeof(line) - 1) {
            line[len++] = ch;
        }
    }
}

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    // Error path: with the chip select on a pin that goes nowhere no ENC28J60
    // can answer, so begin() has to fail, quickly, instead of hanging. The
    // real initialization below then shows that begin() can be retried.
    {
        ENC28J60Config wrong;
        wrong.cs_pin = 15;
        PicoENC28J60 probe(wrong);
        uint32_t t0 = millis();
        probeOk = probe.begin();
        probeMs = millis() - t0;
    }

    bool ok = eth.begin();
    Serial.println("READY bench=1");
    if (!ok) Serial.println("INITFAIL");
}

void loop()
{
    pollSerial();
#ifdef BENCH_IRQ_PIN
    // The chip is touched only when its INT pin asks for it
    if (eth.isInitialized() && eth.interruptPending()) serviceInterrupt();
    linkCycleStep();
#else
    for (int n = 0; n < 8 && eth.isInitialized() && eth.available(); n++) {
        uint16_t len = eth.receiveFrame(rxbuf, sizeof(rxbuf));
        if (len == 0) {
            st.rxz++;
            continue;
        }
        handleFrame(rxbuf, len);
    }
#endif
}
