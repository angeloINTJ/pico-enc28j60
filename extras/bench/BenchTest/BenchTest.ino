// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

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
 * loop() uses the pattern that the library documents: it asks
 * interruptPending(), and talks to the chip only when the answer is yes.
 *
 * Interrupt build: compile with -DBENCH_IRQ_PIN=6 (bench.py --irq 6 does that)
 * with the chip's INT wired to that GPIO. interruptPending() then looks at the
 * pin, so every test also proves that the interrupt alone drives reception.
 * The LINK command is available in that build.
 *
 * -DBENCH_SPI_HZ=n sets the SPI clock of the chip to n Hz instead of the default of the
 * library (bench.py --spi-hz n does that), to run every test at another clock.
 *
 * -DENC28J60_PROFILE (bench.py --flash --profile does that, in the C flags and in the C++ flags) makes the C driver
 * count the commands of the SPI that it runs, and the microseconds they take (src/enc28j60-cmd.h). COST reports them for
 * the hot paths of the driver. A build without the define, which is what --flash alone makes and the build that users
 * get, answers COSTERR noprofile.
 *
 * Before the chip is initialized for real, setup() checks that begin() fails,
 * fast and for the right reason, with no chip behind the pins, with settings
 * that make no sense and with no room left in the PIO. The commands PIOSHARE,
 * STALL, LIFECYCLE, TSV, TXMIX, TXRING and LOSTCHIP check what the driver does
 * about the other users of the PIO, a state machine that stops, begin() after
 * end(), the status of a transmitted frame, sending with the longest receive
 * buffer, uploading a frame while the one before is on the wire, and a chip that
 * was reset behind its back.
 *
 * The rest of the public API is covered by HOLD, PEEK and DRAIN (the functions that
 * only look, and discardFrame(), on frames that wait in the receive buffer),
 * LINKEVENT (hasLinkChange()), SETMAC (setMACAddress()) and CBMODE (the interrupt
 * callback, and poll() when the INT pin is not wired).
 *
 * The asynchronous functions of the C driver (DMA, with notify callbacks), which only the FreeRTOS+TCP driver
 * uses, are covered by ASYNC 1, which hands the chip to them in place of the class, and ASYNC 0, which gives it
 * back. In that mode loop() runs the interrupt branch of driver_task() (freertos/enc28j60-freertos-driver.c) on
 * those functions, and the TX and TXB commands send through them. AOPT sets whether ERXRDPT is read back after
 * every frame and whether frames are dropped unseen. ASYNCCYCLE starts and stops the driver over and over and
 * reports what it did not give back (DMA channels, heap).
 */
#include <PicoENC28J60.h>
#include <SerialPIO.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/watchdog.h>
#include <new>

// The C driver, for the commands that go below the Arduino class
extern "C" {
#include "enc28j60-cmd.h"
#include "enc28j60-spi.pio.h"
#include "pico/enc28j60-driver.h"
}

// ---------------------------------------------------------------------------
// Test protocol (must match bench.py)
// ---------------------------------------------------------------------------
static const uint16_t PORT_RX = 9999;    // host -> Pico broadcast stream, counted and verified
static const uint16_t PORT_TX = 9998;    // Pico -> host broadcast stream (TXB command)
static const uint16_t PORT_ECHO = 9997;  // host -> Pico unicast, echoed back; sender becomes the TX peer
static const uint16_t HDR_LEN = 10;      // "PBNC" + seq(4) + payload length(2)

static uint8_t MAC[6] = {0x02, 0x50, 0x49, 0x43, 0x4F, 0x01};  // the one in use; SETMAC changes it
static uint8_t myIp[4];  // set with the IP command; 0.0.0.0 = unset

static ENC28J60Config makeConfig()
{
    ENC28J60Config c;  // default pins
    memcpy(c.mac_addr, MAC, 6);
#ifdef BENCH_IRQ_PIN
    c.irq_pin = BENCH_IRQ_PIN;
#endif
#ifdef BENCH_SPI_HZ
    c.spi_hz = BENCH_SPI_HZ;
#endif
    return c;
}
PicoENC28J60 eth(makeConfig());

static uint8_t rxbuf[1600];
static uint8_t txbuf[1600];

// Results of the probes that run before the real initialization. Each begin()
// has to fail, quickly, and say why.
static bool probeOk = true;  // begin() with no chip behind the pins
static uint32_t probeMs = 0;
static PicoENC28J60::Error probeErr = PicoENC28J60::Error::None;
static int cfgTried = 0, cfgRejected = 0;  // begin() with settings that make no sense
static uint32_t cfgMaxUs = 0;
static bool pioFilled = false;  // begin() with the instruction memory of every PIO full
static bool pioFullOk = true;
static uint32_t pioFullMs = 0;
static PicoENC28J60::Error pioFullErr = PicoENC28J60::Error::None;

static const char* errName(PicoENC28J60::Error e)
{
    switch (e) {
    case PicoENC28J60::Error::None: return "none";
    case PicoENC28J60::Error::Config: return "config";
    case PicoENC28J60::Error::PioBusy: return "piobusy";
    case PicoENC28J60::Error::PioSpace: return "piospace";
    case PicoENC28J60::Error::NoChip: return "nochip";
    case PicoENC28J60::Error::NoMemory: return "nomem";
    case PicoENC28J60::Error::Io: return "io";
    }
    return "?";
}

struct Stats {
    uint32_t frames, bench, dup, stale, gap, bad;
    uint32_t arp, icmp, echo, other, runt;
    uint32_t extra0, extra4, extraX;  // bytes after the IP datagram in a full-size frame
    uint32_t rxz;                     // receiveFrame() returned 0 although available() > 0
    uint32_t tx, txf;
    uint32_t irqSvc, irqLink, irqRxErr;  // interrupt build: service passes, link and overflow events
    uint32_t irqFlags;                   // OR of every flag byte read
    uint32_t discarded;                  // frames dropped unseen with discardFrame()
    uint32_t peekBad;                    // a function that only looks said something the flags then denied
    uint32_t peekPkt, peekLnk, peekRxe;  // times hasPacket(), hasLinkChange() and hasRXError() said yes
    // Asynchronous mode (ASYNC 1)
    uint32_t rdptEven, rdptChecked;  // ERXRDPT read back after a frame, and how often it was even (errata, issue 14)
    uint32_t rdptLast, nextLast;     // the last ERXRDPT read back, and the next-packet pointer of that frame
    uint32_t hdrBad;                 // frame headers that made no sense
    uint32_t waitTimeouts;           // wait() gave up: the DMA interrupt did not come
    uint32_t txUploadFail;           // enc28j60_frame_upload() found no room
    uint32_t txTimeouts;             // TX and TXB gave up on a frame: the upload slot did not become free
    uint32_t txConfirmed;            // frames the transmit interrupt reported as sent
} st;
static volatile uint32_t cbCount;  // calls of the interrupt callback
static volatile bool cbFlag;       // set by it, taken by the main loop in CBMODE
static bool cbMode;                // CBMODE 1: serve the chip when the callback asks, not when interruptPending() does
static bool hold;                  // HOLD 1: leave the frames in the receive buffer
static uint32_t expectedSeq;
static uint32_t irqBase;  // interrupt count at the last RESET

// Asynchronous mode (ASYNC 1): the chip belongs to the C driver, which runs its commands through DMA
static enc28j60* ah;            // its handle; null while the class owns the chip
static enc28j60_config acfg;    // the driver keeps a pointer to it, so it must outlive the driver
static volatile bool notified;  // set by the DMA interrupt (notify), taken by wait
static volatile bool intFlag;   // set by the interrupt on the INT pin (the interrupt build)
static bool check;              // AOPT: read ERXRDPT back after every frame
static uint32_t discardEvery;   // AOPT: drop every n-th frame unseen, 0 = none
static uint32_t rxSeen;         // frames taken since AOPT, to know which is the n-th
static uint32_t uploadedId = ENC28J60_INVALID_FRAME_ID;    // a frame in the chip, waiting for the transmitter
static uint32_t txInProgress = ENC28J60_INVALID_FRAME_ID;  // the frame the transmitter has, until its interrupt
static const uint32_t WAIT_US = 50000;                     // longest wait for a DMA list to finish
// The size of a frame in the header of the chip includes the FCS; PicoENC28J60.cpp has the same two constants
static const uint16_t ETH_HEADER_LEN = 14;
static const uint16_t ETH_FCS_LEN = 4;

static const uint16_t PHY_PWRSV = 0x0800;  // PHCON1.PPWRSV

#ifdef BENCH_IRQ_PIN
// LINK command: power the PHY down and up again and watch the interrupt report it
static struct {
    uint8_t phase;  // 0 idle, 1 PHY powered down, 2 powered up again and waiting for the link
    uint32_t t0, t1, downMs, upMs;
} lk;
#endif

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

static bool asyncSend(const uint8_t* f, uint16_t len);  // asynchronous mode, below

static bool sendTracked(const uint8_t* f, uint16_t len)
{
    if (ah) return asyncSend(f, len);  // it counts tx and txf by itself
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
// Service routine, and the link cycle of the interrupt build
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
#endif

// What the library documents: the chip asked for attention, so read the flags
// (which masks the INT pin), do what they say, and acknowledge them.
static void serviceInterrupt()
{
    st.irqSvc++;
    // The functions that only look must not say what the flags, read right after, deny: nothing clears a flag in
    // between. A flag may well appear in between, so the other way round proves nothing.
    uint8_t peek = eth.peekInterruptFlags();
    bool pkt = eth.hasPacket(), lnk = eth.hasLinkChange(), rxe = eth.hasRXError();
    uint8_t flags = eth.getInterruptFlags();
    if ((peek & ~flags) || (pkt && !(flags & PicoENC28J60::INT_PACKET_RX)) ||
        (lnk && !(flags & PicoENC28J60::INT_LINK_CHANGE)) || (rxe && !(flags & PicoENC28J60::INT_RX_ERROR))) {
        st.peekBad++;
    }
    st.peekPkt += pkt;
    st.peekLnk += lnk;
    st.peekRxe += rxe;
    st.irqFlags |= flags;
    if (flags & PicoENC28J60::INT_RX_ERROR) st.irqRxErr++;
    if (flags & PicoENC28J60::INT_LINK_CHANGE) {
        st.irqLink++;
#ifdef BENCH_IRQ_PIN
        onLinkChange();
#endif
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

// ---------------------------------------------------------------------------
// Asynchronous mode: the DMA functions of the C driver, used the way the FreeRTOS driver uses them
// ---------------------------------------------------------------------------

// Interrupt context: the DMA list is done
static void asyncNotify(void*) { notified = true; }

// The driver calls this right after it starts a DMA list: spin until the interrupt says that the list is done.
// Giving up after WAIT_US is what makes a list that never finishes (a corrupted command stream, say) show up in
// waitTimeouts instead of hanging the sketch.
static void asyncWait(void*)
{
    uint32_t t0 = time_us_32();
    while (!notified) {
        if (time_us_32() - t0 > WAIT_US) {
            st.waitTimeouts++;
            break;
        }
    }
    notified = false;
}

// The configuration of the C driver for this mode: the pins, the clock and the MAC address of the class, and the
// callbacks above, which are what turns the asynchronous functions on. No irq_cb: the core of Arduino-Pico owns the
// one GPIO interrupt callback of the SDK (see PicoENC28J60::begin()), so the INT pin goes through attachInterrupt().
// Called before every init, because init fills in pio and sm.
static void asyncConfig()
{
    ENC28J60Config c = makeConfig();
    memset(&acfg, 0, sizeof(acfg));
    acfg.pio = NULL;
    acfg.sm = ENC28J60_SM_ANY;
    acfg.sclk_pin = c.sclk_pin;
    acfg.mosi_pin = c.mosi_pin;
    acfg.miso_pin = c.miso_pin;
    acfg.cs_pin = c.cs_pin;
    acfg.spi_hz = c.spi_hz;
    acfg.dma_irq = 0;
    enc28j60_config* defaults = enc28j60_get_default_config();  // allocated
    acfg.rx_size = defaults->rx_size;  // the FreeRTOS driver uses the default of the C driver, not of the class
    free(defaults);
    memcpy(acfg.mac_addr, c.mac_addr, 6);
    acfg.irq_cb = NULL;
    acfg.notify.notify = asyncNotify;
    acfg.notify.wait = asyncWait;
    acfg.notify.data = NULL;
}

// Like the FreeRTOS driver: one frame uploaded and waiting, one in progress. When the transmitter is free the one
// that waits goes to it.
static void asyncKick()
{
    if (uploadedId != ENC28J60_INVALID_FRAME_ID && txInProgress == ENC28J60_INVALID_FRAME_ID) {
        enc28j60_frame_tx(ah, uploadedId);
        txInProgress = uploadedId;
        uploadedId = ENC28J60_INVALID_FRAME_ID;
    }
}

// Never waits, because the service pass calls it (handleFrame -> sendTracked): a frame that finds the slot taken
// is counted in txf and dropped.
static bool asyncSend(const uint8_t* f, uint16_t len)
{
    if (uploadedId != ENC28J60_INVALID_FRAME_ID) {
        st.txf++;
        return false;
    }
    uint32_t id = enc28j60_frame_upload(ah, len, f);
    if (id == ENC28J60_INVALID_FRAME_ID) {
        st.txUploadFail++;
        st.txf++;
        return false;
    }
    uploadedId = id;
    asyncKick();
    st.tx++;
    return true;
}

// One frame from the chip, taken the way rx_frame() of the FreeRTOS driver takes it, and handed to handleFrame()
// where that one hands it to the network stack. The header is checked the way PicoENC28J60::_readRxInfo() checks it;
// the FreeRTOS driver takes it as it comes.
static void asyncRxFrame()
{
    uint32_t info = enc28j60_frame_rx_info(ah);
    uint16_t size = ENC28J60_RX_SIZE(info);
    uint16_t next = ENC28J60_RX_NEXT(info);
    bool valid = size >= ETH_HEADER_LEN + ETH_FCS_LEN && size <= PicoENC28J60::MAX_FRAME_SIZE + ETH_FCS_LEN &&
                 next < acfg.rx_size && (next & 1) == 0;
    if (!valid) {
        st.hdrBad++;
        enc28j60_rx_reset_blk(ah);
        return;
    }

    bool taken = true;
    if (discardEvery && ++rxSeen % discardEvery == 0) {
        enc28j60_frame_discard(ah, info);
        st.discarded++;
        taken = false;
    } else {
        enc28j60_frame_rx(ah, info, rxbuf);
    }

    if (check) {
        // The blocking reads below must not cut into a DMA list that is still going out, and a call can come back
        // before the last commands of its list have been sent. So wait until the list is certainly over: 64 bytes of
        // SPI is more than the commands that follow the data in any list. Without check there is no wait and no SPI
        // between one call and the next, which is how the FreeRTOS driver goes on (and what asyncfast tests).
        delayMicroseconds(64u * 8u * 1000000u / acfg.spi_hz + 50u);
        bank_set_blk(ah, 0);
        uint16_t rdpt = reg_read_e_blk(ah, ERXRDPTL);
        rdpt |= (uint16_t)(reg_read_e_blk(ah, ERXRDPTH) << 8);
        st.rdptChecked++;
        if (!(rdpt & 1)) st.rdptEven++;  // errata, issue 14: the chip needs ERXRDPT odd
        st.rdptLast = rdpt;
        st.nextLast = next;
    }
    // The size in the header includes the FCS, which receiveFrame() leaves out: pass on what it would return, so that
    // x0 and x4 mean the same here
    if (taken) handleFrame(rxbuf, size - ETH_FCS_LEN);
}

// The interrupt branch of driver_task() in freertos/enc28j60-freertos-driver.c: it mirrors that function, call for
// call and in the same order (one frame for PKTIF, every frame that waits for RXERIF), with handleFrame() where the
// network stack is. Its transmit half is asyncSend() and asyncKick(). The link change is only counted, and a
// transmit flag is taken only when a frame is in progress (the FreeRTOS driver confirms the id it has).
static void asyncService()
{
    uint8_t flags = enc28j60_irq_flags(ah);
    st.irqSvc++;
    st.irqFlags |= flags;
    if (enc28j60_irq_is_link(flags)) st.irqLink++;
    if (enc28j60_irq_is_tx(flags) && txInProgress != ENC28J60_INVALID_FRAME_ID) {
        enc28j60_frame_tx_confirm(ah, txInProgress);
        txInProgress = ENC28J60_INVALID_FRAME_ID;
        st.txConfirmed++;
    }
    if (enc28j60_irq_is_rx(flags)) asyncRxFrame();
    if (enc28j60_irq_is_rx_err(flags)) {
        st.irqRxErr++;
        uint8_t n = enc28j60_rx_count(ah);
        for (uint8_t i = 0; i < n; i++) asyncRxFrame();
    }
    enc28j60_irq_ack(ah, flags);
    asyncKick();
}

// A service pass when one is due: at every turn without the INT pin, when the pin asked with it
static void asyncPoll()
{
#ifdef BENCH_IRQ_PIN
    if (!intFlag) return;
    intFlag = false;  // before the pass, so that an edge during it asks for another
#endif
    asyncService();
}

// What the streams of TX and TXB use in this mode. The upload slot is freed by the service pass (the transmit
// interrupt is what moves the frame in it to the transmitter), so serve the chip while waiting for it. A full-size
// frame takes 1.2 ms on the wire: 20 ms leaves room for deferral and retries, and then the frame is given up.
static bool asyncSendWait(const uint8_t* f, uint16_t len)
{
    for (uint32_t t0 = millis(); uploadedId != ENC28J60_INVALID_FRAME_ID;) {
        if (millis() - t0 >= 20) {
            st.txTimeouts++;
            st.txf++;
            return false;
        }
        asyncPoll();
    }
    return asyncSend(f, len);
}

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

static enc28j60* driverHandle() { return static_cast<enc28j60*>(eth.getDriverHandle()); }

static void printInfo()
{
    uint8_t m[6];
    eth.getMACAddress(m);
    enc28j60* h = driverHandle();
    int pio = h ? (int)pio_get_index(h->cfg->pio) : -1;
    int sm = h ? (int)h->cfg->sm : -1;
    // The class is stopped in the asynchronous mode and cannot tell: ask the chip through the C driver. Nothing is in
    // flight between two commands, so the blocking call is safe.
    bool link = ah ? enc28j60_link_status_blk(ah) : eth.linkStatus();
    Serial.printf("INFO init=%d rev=0x%02X mac=%02x:%02x:%02x:%02x:%02x:%02x link=%d ip=%u.%u.%u.%u pend=%u "
                  "probe_ok=%d probe_ms=%lu probe_err=%s irqpin=%d pio=%d sm=%d claimed=%d "
                  "cfg_rejected=%d cfg_tried=%d cfg_us=%lu pioful_filled=%d pioful_ok=%d pioful_ms=%lu pioful_err=%s err=%s "
                  "spihz=%lu\n",
                  eth.isInitialized(), eth.getRevision(), m[0], m[1], m[2], m[3], m[4], m[5],
                  link, myIp[0], myIp[1], myIp[2], myIp[3], eth.available(), probeOk,
                  (unsigned long)probeMs, errName(probeErr), irqPin(), pio, sm,
                  h ? pio_sm_is_claimed(h->cfg->pio, h->cfg->sm) : 0, cfgRejected, cfgTried,
                  (unsigned long)cfgMaxUs, pioFilled, pioFullOk, (unsigned long)pioFullMs, errName(pioFullErr),
                  errName(eth.lastError()), h ? (unsigned long)h->cfg->spi_hz : 0UL);
}

static void printStats()
{
    const PicoENC28J60::Stats& e = eth.stats();
    Serial.printf("STAT frames=%lu bench=%lu dup=%lu stale=%lu gap=%lu bad=%lu arp=%lu icmp=%lu echo=%lu "
                  "other=%lu runt=%lu x0=%lu x4=%lu xo=%lu rxz=%lu tx=%lu txf=%lu pend=%u rxrst=%lu rxdrop=%lu "
                  "irq=%lu svc=%lu lnk=%lu rxe=%lu irqfl=0x%02X int=%d io=%lu rec=%lu txab=%lu txre=%lu "
                  "dis=%lu peekbad=%lu peekpkt=%lu peeklnk=%lu peekrxe=%lu cb=%lu "
                  "async=%d rdpteven=%lu rdptchk=%lu rdptlast=0x%04x nextlast=0x%04x hdrbad=%lu waitto=%lu "
                  "txupf=%lu txto=%lu txconf=%lu\n",
                  (unsigned long)st.frames, (unsigned long)st.bench, (unsigned long)st.dup,
                  (unsigned long)st.stale, (unsigned long)st.gap, (unsigned long)st.bad,
                  (unsigned long)st.arp, (unsigned long)st.icmp, (unsigned long)st.echo,
                  (unsigned long)st.other, (unsigned long)st.runt, (unsigned long)st.extra0,
                  (unsigned long)st.extra4, (unsigned long)st.extraX, (unsigned long)st.rxz,
                  (unsigned long)st.tx, (unsigned long)st.txf, ah ? enc28j60_rx_count_blk(ah) : eth.available(),
                  (unsigned long)e.rx_resets, (unsigned long)e.rx_dropped,
                  (unsigned long)(eth.interruptCount() - irqBase), (unsigned long)st.irqSvc, (unsigned long)st.irqLink,
                  (unsigned long)st.irqRxErr, (unsigned)st.irqFlags,
                  irqPin() >= 0 ? digitalRead(irqPin()) : -1, (unsigned long)e.io_errors,
                  (unsigned long)e.recoveries, (unsigned long)e.tx_aborts, (unsigned long)e.tx_retries,
                  (unsigned long)st.discarded, (unsigned long)st.peekBad, (unsigned long)st.peekPkt,
                  (unsigned long)st.peekLnk, (unsigned long)st.peekRxe, (unsigned long)cbCount, ah != nullptr,
                  (unsigned long)st.rdptEven, (unsigned long)st.rdptChecked, (unsigned)st.rdptLast,
                  (unsigned)st.nextLast, (unsigned long)st.hdrBad, (unsigned long)st.waitTimeouts,
                  (unsigned long)st.txUploadFail, (unsigned long)st.txTimeouts, (unsigned long)st.txConfirmed);
}

// TX  <count> <payload length> <gap in microseconds>: unicast stream to the last echo peer.
// TXB <count> <payload length> <gap in microseconds>: broadcast stream to 255.255.255.255:PORT_TX.
// In the asynchronous mode both send through the C driver, waiting for the upload slot (asyncSendWait()).
// TXDONE gives the frames that went out and the ones that did not, and in us= the time of the whole stream, from before the
// first frame is made to after the last one is sent: the making of the frames and the gaps are in it.
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
    uint32_t t0 = micros();
    for (uint32_t s = 0; s < count; s++) {
        uint8_t* p = txbuf + 42;
        memcpy(p, "PBNC", 4);
        wr32(p + 4, s);
        wr16(p + 8, plen);
        for (uint16_t i = HDR_LEN; i < plen; i++) p[i] = fill(s, i);
        uint16_t flen = broadcast
            ? finishUdp(txbuf, bcMac, bcIp, PORT_ECHO, PORT_TX, plen, (uint16_t)s)
            : finishUdp(txbuf, peer.mac, peer.ip, PORT_ECHO, peer.port, plen, (uint16_t)s);
        if (ah ? asyncSendWait(txbuf, flen) : eth.sendFrame(txbuf, flen)) ok++;
        else fail++;
        if (gapUs) delayMicroseconds(gapUs);
    }
    uint32_t us = micros() - t0;
    if (!ah) {  // asyncSend() has counted its frames by itself
        st.tx += ok;
        st.txf += fail;
    }
    Serial.printf("TXDONE sent=%lu fail=%lu us=%lu\n", (unsigned long)ok, (unsigned long)fail, (unsigned long)us);
}

// TXMIX <receive buffer size> <count>: begin() again with that receive buffer, which leaves the rest of the RAM of
// the chip to send from, and broadcast frames of all lengths in turn, the longest first. With a transmit area of
// less than two full frames every frame has to be found room for after the pointers wrapped. The default
// configuration is back afterwards.
static void cmdTxMix(uint16_t rxSize, uint32_t count)
{
    static const uint8_t bcMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static const uint8_t bcIp[4] = {255, 255, 255, 255};
    static const uint16_t PLEN[] = {1472, 10, 1472, 400, 1472, 1000, 18, 1472, 700, 1472, 100, 1300};
    eth.end();
    ENC28J60Config c = makeConfig();
    c.rx_buffer_size = rxSize;
    bool began = eth.begin(c);
    for (uint32_t t0 = millis(); began && !eth.linkStatus() && millis() - t0 < 10000;) {
        delay(50);  // the link takes a moment to come up after begin()
    }
    uint32_t ok = 0, fail = 0;
    for (uint32_t s = 0; began && s < count; s++) {
        uint16_t plen = PLEN[s % (sizeof(PLEN) / sizeof(PLEN[0]))];
        uint8_t* p = txbuf + 42;
        memcpy(p, "PBNC", 4);
        wr32(p + 4, s);
        wr16(p + 8, plen);
        for (uint16_t i = HDR_LEN; i < plen; i++) p[i] = fill(s, i);
        uint16_t flen = finishUdp(txbuf, bcMac, bcIp, PORT_ECHO, PORT_TX, plen, (uint16_t)s);
        if (eth.sendFrame(txbuf, flen)) ok++;
        else fail++;
    }
    eth.end();
    bool restored = eth.begin(makeConfig());
    Serial.printf("TXMIXDONE rx=0x%04X began=%d sent=%lu fail=%lu restored=%d\n", rxSize, began, (unsigned long)ok,
                  (unsigned long)fail, restored);
}

// TXRING <receive buffer size> <count> <pattern>: begin() again with that receive buffer and broadcast frames through
// the C driver the way the FreeRTOS driver sends them: the next frame is uploaded while the one before is still on the
// wire, and that one is waited for right before the next one starts, or first if there is no room for the next one.
// The payload lengths come in turn from pattern 0 (all 1472 bytes, full-size frames), pattern 1 (a mix, a short
// frame and a long one first) or pattern 2 (seven frames after which the transmit area of a receive buffer of 0x145A
// used to give the next frame the bytes of the one on the wire). TXRINGDONE gives the frames sent and failed, the ones
// that found no room even with nothing else waiting (dropped), the uploads that found no room while a frame was on the
// wire (waited), and the uploads that got bytes of the frame on the wire (overlaps), from the ids, which hold where
// each frame is. The default configuration is back afterwards.
static bool txRingFinish(enc28j60* h, uint32_t id)
{
    return enc28j60_frame_send_finish_blk(h, id, 20000, 15, nullptr) == ENC28J60_TX_OK;
}

// Do two frames share a byte of the chip? A frame id holds the address of its control byte and of its last byte; the
// chip writes the status vector in the 7 bytes after it.
static bool txRingOverlap(uint32_t a, uint32_t b)
{
    uint32_t aStart = a >> 16, aEnd = (a & 0xFFFF) + 7, bStart = b >> 16, bEnd = (b & 0xFFFF) + 7;
    return aStart <= bEnd && bStart <= aEnd;
}

static void cmdTxRing(uint16_t rxSize, uint32_t count, uint32_t pattern)
{
    static const uint8_t bcMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static const uint8_t bcIp[4] = {255, 255, 255, 255};
    static const uint16_t FULL[] = {1472};
    static const uint16_t MIX[] = {58, 1058, 1472, 1472, 1472, 18, 1472, 600, 1472, 1000, 1472, 1300};
    static const uint16_t WRAP[] = {1424, 1004, 879, 13, 470, 1463, 1447};  // frames of 1466, 1046 ... 1489 bytes
    const uint16_t* plens = pattern == 2 ? WRAP : pattern == 1 ? MIX : FULL;
    size_t n = pattern == 2   ? sizeof(WRAP) / sizeof(WRAP[0])
               : pattern == 1 ? sizeof(MIX) / sizeof(MIX[0])
                              : sizeof(FULL) / sizeof(FULL[0]);
    eth.end();
    ENC28J60Config c = makeConfig();
    c.rx_buffer_size = rxSize;
    bool began = eth.begin(c);
    for (uint32_t t0 = millis(); began && !eth.linkStatus() && millis() - t0 < 10000;) {
        delay(50);  // the link takes a moment to come up after begin()
    }
    enc28j60* h = driverHandle();
    uint32_t onWire = ENC28J60_INVALID_FRAME_ID;
    uint32_t ok = 0, fail = 0, dropped = 0, waited = 0, overlaps = 0;
    for (uint32_t s = 0; began && s < count; s++) {
        uint16_t plen = plens[s % n];
        uint8_t* p = txbuf + 42;
        memcpy(p, "PBNC", 4);
        wr32(p + 4, s);
        wr16(p + 8, plen);
        for (uint16_t i = HDR_LEN; i < plen; i++) p[i] = fill(s, i);
        uint16_t flen = finishUdp(txbuf, bcMac, bcIp, PORT_ECHO, PORT_TX, plen, (uint16_t)s);
        uint32_t id = enc28j60_frame_upload_blk(h, flen, txbuf);
        if (id == ENC28J60_INVALID_FRAME_ID && onWire != ENC28J60_INVALID_FRAME_ID) {
            waited++;
            txRingFinish(h, onWire) ? ok++ : fail++;
            onWire = ENC28J60_INVALID_FRAME_ID;
            id = enc28j60_frame_upload_blk(h, flen, txbuf);
        }
        if (id == ENC28J60_INVALID_FRAME_ID) {
            dropped++;
            continue;
        }
        if (onWire != ENC28J60_INVALID_FRAME_ID) {
            if (txRingOverlap(id, onWire)) overlaps++;
            txRingFinish(h, onWire) ? ok++ : fail++;
        }
        enc28j60_frame_tx_blk(h, id);
        onWire = id;
    }
    if (onWire != ENC28J60_INVALID_FRAME_ID) {
        txRingFinish(h, onWire) ? ok++ : fail++;
    }
    eth.end();
    bool restored = eth.begin(makeConfig());
    Serial.printf("TXRINGDONE rx=0x%04X pattern=%lu began=%d sent=%lu fail=%lu dropped=%lu waited=%lu overlaps=%lu "
                  "restored=%d\n",
                  rxSize, (unsigned long)pattern, began, (unsigned long)ok, (unsigned long)fail, (unsigned long)dropped,
                  (unsigned long)waited, (unsigned long)overlaps, restored);
}

// PIOSHARE: start PIO users of the core, TX only serial ports, until there is no state machine left.
// The one of the driver has to stay with the driver, and the chip has to go on answering.
static void cmdPioShare()
{
    enc28j60* h = driverHandle();
    static const int MAX_USERS = 12;  // more than the state machines of any board
    alignas(SerialPIO) static uint8_t storage[MAX_USERS][sizeof(SerialPIO)];
    SerialPIO* users[MAX_USERS];
    int started = 0;
    for (int i = 0; i < MAX_USERS; i++) {
        users[i] = new (storage[i]) SerialPIO(10 + i, NOPIN);
        users[i]->begin(9600);
        if (!*users[i]) {
            users[i]->~SerialPIO();
            break;
        }
        started++;
    }
    bool claimed = pio_sm_is_claimed(h->cfg->pio, h->cfg->sm);
    uint8_t rev = eth.getRevision();
    for (int i = 0; i < started; i++) {
        users[i]->end();
        users[i]->~SerialPIO();
    }
    uint8_t after = eth.getRevision();
    Serial.printf("PIOSHARE started=%d claimed=%d rev=0x%02X after=0x%02X io=%lu\n", started, claimed, rev, after,
                  (unsigned long)eth.stats().io_errors);
}

// STALL: stop the state machine of the driver, as another piece of code could. The next call has to come
// back, failing, instead of hanging; the one after that finds the chip working again.
static void cmdStall()
{
    enc28j60* h = driverHandle();
    pio_sm_set_enabled(h->cfg->pio, h->cfg->sm, false);
    uint32_t t0 = micros();
    uint8_t first = eth.getRevision();
    uint32_t firstUs = micros() - t0;
    t0 = micros();
    uint8_t second = eth.getRevision();
    uint32_t secondMs = (micros() - t0) / 1000;
    Serial.printf("STALLDONE first_us=%lu first=0x%02X second_ms=%lu second=0x%02X io=%lu rec=%lu\n",
                  (unsigned long)firstUs, first, (unsigned long)secondMs, second, (unsigned long)eth.stats().io_errors,
                  (unsigned long)eth.stats().recoveries);
}

// LIFECYCLE: end() and begin() again, a few times. Each end() has to give the state machine back.
static void cmdLifecycle()
{
    static const int CYCLES = 3;
    bool freed = true, began = true;
    for (int i = 0; i < CYCLES; i++) {
        enc28j60* h = driverHandle();
        PIO pio = h->cfg->pio;
        uint sm = h->cfg->sm;
        eth.end();
        freed = freed && !pio_sm_is_claimed(pio, sm) && !eth.isInitialized() && eth.getDriverHandle() == nullptr;
        began = eth.begin() && began;
    }
    enc28j60* h = driverHandle();
    Serial.printf("LIFECYCLE cycles=%d freed=%d began=%d rev=0x%02X claimed=%d\n", CYCLES, freed, began,
                  eth.getRevision(), h ? pio_sm_is_claimed(h->cfg->pio, h->cfg->sm) : 0);
}

// LOSTCHIP: reset the chip behind the back of the driver, as a glitch of its supply would, and see whether
// checkChip() finds out and configures it again
static void cmdLostChip()
{
    enc28j60* h = driverHandle();
    uint32_t before = eth.stats().chip_resets;
    system_reset(h);  // every register goes back to its default: the receiver is off
    delay(2);
    bool alive = enc28j60_alive_blk(h);
    uint32_t t0 = micros();
    bool fixed = eth.checkChip();
    uint32_t us = micros() - t0;
    bool aliveAfter = enc28j60_alive_blk(h);
    Serial.printf("LOSTCHIP alive_before=%d fixed=%d alive_after=%d ms=%lu resets=%lu rev=0x%02X\n", alive, fixed,
                  aliveAfter, (unsigned long)(us / 1000), (unsigned long)(eth.stats().chip_resets - before),
                  eth.getRevision());
}

// The functions that only look, and discardFrame(), on frames that wait in the receive buffer.
//   HOLD 1   leave the frames where they are: loop() stops serving the chip. HOLD 0 serves it again.
//   PEEK     what hasPacket(), available() and peekInterruptFlags() say about the buffer right now
//   DRAIN    take every frame that waits, dropping every third one unseen with discardFrame()
static void cmdPeek()
{
    bool pkt = eth.hasPacket();
    uint8_t avail = eth.available();
    uint8_t flags = eth.peekInterruptFlags();
    Serial.printf("PEEK pkt=%d avail=%u flags=0x%02X\n", pkt, avail, flags);
}

static void cmdDrain()
{
    uint32_t total = 0, dropped = 0;
    while (eth.available() && total < 200) {
        if (total % 3 == 1) {
            eth.discardFrame();
            dropped++;
        } else {
            uint16_t len = eth.receiveFrame(rxbuf, sizeof(rxbuf));
            if (len) handleFrame(rxbuf, len);
            else st.rxz++;
        }
        total++;
    }
    st.discarded += dropped;
    // What the functions that only look say right after the drain, in the same command: a frame of the LAN that
    // comes in before a PEEK could ask would count against them otherwise
    bool pkt = eth.hasPacket();
    uint8_t avail = eth.available();
    uint8_t flags = eth.peekInterruptFlags();
    Serial.printf("DRAINDONE total=%lu dropped=%lu left=%u pkt=%d avail=%u flags=0x%02X\n", (unsigned long)total,
                  (unsigned long)dropped, avail, pkt, avail, flags);
}

// LINKEVENT: power the PHY down and up again, and watch hasLinkChange() report each change and
// acknowledgeInterrupts(INT_LINK_CHANGE) clear it. Works with or without the INT pin: it only looks at the flag.
static void cmdLinkEvent()
{
    enc28j60* h = driverHandle();
    eth.acknowledgeInterrupts(PicoENC28J60::INT_LINK_CHANGE);  // start from a clean slate
    bool clean = !eth.hasLinkChange();

    reg_bits_clear_blk(h, ECON1, RXEN);  // stop the receiver first, as the datasheet asks before powering down
    delay(2);
    reg_write_p_blk(h, PHCON1, reg_read_p_blk(h, PHCON1) | PHY_PWRSV);
    bool downSeen = false;
    for (uint32_t t0 = millis(); !downSeen && millis() - t0 < 1500; delay(5)) downSeen = eth.hasLinkChange();
    bool linkDown = !eth.linkStatus();
    eth.acknowledgeInterrupts(PicoENC28J60::INT_LINK_CHANGE);
    bool downCleared = !eth.hasLinkChange();

    reg_write_p_blk(h, PHCON1, reg_read_p_blk(h, PHCON1) & ~PHY_PWRSV);
    delay(2);  // the PHY needs a moment before it takes more commands
    reg_bits_set_blk(h, ECON1, RXEN);
    bool upSeen = false;
    for (uint32_t t0 = millis(); !upSeen && millis() - t0 < 15000; delay(20)) upSeen = eth.hasLinkChange();
    bool linkUp = eth.linkStatus();
    eth.acknowledgeInterrupts(PicoENC28J60::INT_LINK_CHANGE);
    bool upCleared = !eth.hasLinkChange();
    Serial.printf("LINKEVENT clean=%d down_seen=%d link_down=%d down_cleared=%d up_seen=%d link_up=%d up_cleared=%d\n",
                  clean, downSeen, linkDown, downCleared, upSeen, linkUp, upCleared);
}

// SETMAC <aa:bb:cc:dd:ee:ff> <a.b.c.d>: setMACAddress() while the chip works, and answer for that IP address from
// now on. The chip's own registers are read back, and getMACAddress() has to agree.
static void cmdSetMac(const char* args)
{
    unsigned m[6], a, b, c, d;
    if (sscanf(args, "%x:%x:%x:%x:%x:%x %u.%u.%u.%u", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &a, &b, &c, &d) != 10) {
        Serial.println("SETMACERR syntax");
        return;
    }
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
    memcpy(MAC, mac, 6);
    myIp[0] = a;
    myIp[1] = b;
    myIp[2] = c;
    myIp[3] = d;
    eth.setMACAddress(mac);

    uint8_t back[6];
    eth.getMACAddress(back);
    enc28j60* h = driverHandle();
    bank_set_blk(h, 3);
    uint8_t hw[6] = {reg_read_m_blk(h, MAADR1), reg_read_m_blk(h, MAADR2), reg_read_m_blk(h, MAADR3),
                     reg_read_m_blk(h, MAADR4), reg_read_m_blk(h, MAADR5), reg_read_m_blk(h, MAADR6)};
    Serial.printf("SETMAC get=%d hw=%d mac=%02x:%02x:%02x:%02x:%02x:%02x\n", memcmp(back, mac, 6) == 0,
                  memcmp(hw, mac, 6) == 0, hw[0], hw[1], hw[2], hw[3], hw[4], hw[5]);
}

// SPITEST <Hz> <rounds>: begin() again with that SPI clock, and move data through every kind of access the driver
// makes: an ETH register, the MAC registers, PHY registers through the MII interface, and the buffer memory. What
// comes back is compared with what was sent, or with what the chip holds as a constant (the PHY identifier). Then
// the time of a frame's worth of reads and of a register read. The default configuration is back afterwards.
static void cmdSpiTest(uint32_t hz, uint32_t rounds)
{
    static const uint8_t MAADR[6] = {MAADR1, MAADR2, MAADR3, MAADR4, MAADR5, MAADR6};
    eth.end();
    ENC28J60Config c = makeConfig();
    c.spi_hz = hz;
    bool began = eth.begin(c);
    uint32_t errEth = 0, errMac = 0, errMii = 0, errMem = 0, frameUs = 0, regUs = 0;
    uint16_t phid1 = 0, phid2 = 0;
    uint32_t clkdiv = 0;  // the divider of the state machine as the driver set it: INT in 31:16, FRAC in 15:8
    if (began) {
        enc28j60* h = driverHandle();
        clkdiv = h->cfg->pio->sm[h->cfg->sm].clkdiv;
        uint32_t rng = 0x2545F491u ^ hz;
        auto next = [&rng]() {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            return rng;
        };
        phid1 = reg_read_p_blk(h, PHID1);
        phid2 = reg_read_p_blk(h, PHID2);
        for (uint32_t i = 0; i < rounds; i++) {
            uint8_t v = (uint8_t)next();
            bank_set_blk(h, 1);  // EPMM0, a pattern match mask that the driver does not use
            reg_write_blk(h, EPMM0, v);
            if (reg_read_e_blk(h, EPMM0) != v) errEth++;

            uint8_t m[6];
            bank_set_blk(h, 3);  // MAC registers: reading them takes a dummy byte and the longest CS hold
            for (int k = 0; k < 6; k++) {
                m[k] = (uint8_t)next();
                reg_write_blk(h, MAADR[k], m[k]);
            }
            for (int k = 0; k < 6; k++) {
                if (reg_read_m_blk(h, MAADR[k]) != m[k]) errMac++;
            }

            if (reg_read_p_blk(h, PHID1) != phid1 || reg_read_p_blk(h, PHID2) != phid2) errMii++;

            uint8_t out[30], back[30];
            for (int k = 0; k < 30; k++) out[k] = (uint8_t)next();
            uint16_t addr = 0x1800 + (i % 8) * 32;  // in the transmit area, which nothing uses here
            mem_write_at_blk(h, addr, sizeof(out), out);
            mem_read_at_blk(h, addr, sizeof(back), back);
            if (memcmp(out, back, sizeof(out)) != 0) errMem++;
        }
        // what a received frame costs: 49 reads of 31 bytes make 1519 bytes. And one register read.
        uint8_t sink[31];
        bank_set_blk(h, 0);
        uint32_t t0 = micros();
        for (int k = 0; k < 49; k++) mem_read_blk(h, sizeof(sink), sink);
        frameUs = micros() - t0;
        t0 = micros();
        for (int k = 0; k < 100; k++) (void)reg_read_e_blk(h, ESTAT);
        regUs = (micros() - t0) / 100;
    }
    uint32_t io = eth.stats().io_errors;
    const char* err = errName(eth.lastError());
    eth.end();
    bool restored = eth.begin(makeConfig());  // the MAC address and the registers the test used start over
    Serial.printf("SPITEST hz=%lu began=%d err=%s rounds=%lu eth=%lu mac=%lu mii=%lu mem=%lu phid1=0x%04X phid2=0x%04X "
                  "frame_us=%lu reg_us=%lu io=%lu restored=%d div_int=%lu div_frac=%lu sys_hz=%lu\n",
                  (unsigned long)hz, began, err, (unsigned long)rounds, (unsigned long)errEth, (unsigned long)errMac,
                  (unsigned long)errMii, (unsigned long)errMem, phid1, phid2, (unsigned long)frameUs,
                  (unsigned long)regUs, (unsigned long)io, restored, (unsigned long)(clkdiv >> 16),
                  (unsigned long)((clkdiv >> 8) & 0xFF), (unsigned long)clock_get_hz(clk_sys));
}

// TSV <length> <broadcast 0/1>: send one frame through the C driver and read its transmit status vector.
static void cmdTsv(uint16_t len, bool broadcast)
{
    static const uint8_t other[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x99};
    if (len < 22) len = 22;  // the header and the marker
    if (len > PicoENC28J60::MAX_FRAME_SIZE) len = PicoENC28J60::MAX_FRAME_SIZE;
    memset(txbuf, 0, len);
    memcpy(txbuf, broadcast ? (const uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF" : other, 6);
    memcpy(txbuf + 6, MAC, 6);
    wr16(txbuf + 12, 0x88B5);  // IEEE 802 local experimental EtherType: nothing on the LAN answers it
    memcpy(txbuf + 14, "PBNC-TSV", 8);

    enc28j60* h = driverHandle();
    uint32_t id = enc28j60_frame_upload_blk(h, len, txbuf);
    if (id == ENC28J60_INVALID_FRAME_ID) {
        Serial.println("TSVERR upload");
        return;
    }
    uint8_t retries = 0;
    enc28j60_tx_status result = enc28j60_frame_send_blk(h, id, 20000, 15, &retries);
    uint8_t v[7];
    enc28j60_tx_status_blk(h, id, v);
    // Table 7-3 of the data sheet, least significant byte first
    Serial.printf("TSVDONE len=%u status=%d retries=%u bytes=%u coll=%u done=%d bcast=%d mcast=%d late=%d xcoll=%d "
                  "xdefer=%d defer=%d underrun=%d wire=%u raw=%02x%02x%02x%02x%02x%02x%02x\n",
                  len, (int)result, retries, v[0] | (v[1] << 8), v[2] & 0x0F, (v[2] >> 7) & 1, (v[3] >> 1) & 1,
                  v[3] & 1, (v[3] >> 5) & 1, (v[3] >> 4) & 1, (v[3] >> 3) & 1, (v[3] >> 2) & 1, (v[3] >> 7) & 1,
                  v[4] | (v[5] << 8), v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
}

// COST <frame length>: what the hot paths of the driver cost in commands of the SPI and in microseconds, from the counters
// that the C driver keeps when it is built with -DENC28J60_PROFILE (src/enc28j60-cmd.h). Every step zeroes the counters,
// makes its call and reads them, so a step holds only what that call did:
//   avail  available() with the receive buffer empty: the frames that wait are dropped first, and it is asked again at
//          once if one came in meanwhile (empty=1 says that the answer was 0)
//   rx     receiveFrame() of the next frame. The board says COSTARM and bench.py sends the frame; rxlen is the length
//          that receiveFrame() returned, which is the frame without its FCS
//   up     enc28j60_frame_upload_blk() of a frame of <frame length> bytes (uplen), as sendFrame() does it
//   start  enc28j60_frame_tx_blk(), as sendFrame() does it
//   wait   enc28j60_frame_tx_wait_blk(): eir reads of EIR, as many as the wire takes, then one command that clears
//          TXRTS, so wait is eir + 1 when the frame was sent (sent=1); wait_us is the time of the loop of the reads
// The times are those of the commands (the counters of the driver), not of the calls, so the Arduino class around them
// is not in them. The frame that goes out is a broadcast with the EtherType 0x88B5, like the one of TSV.
static void cmdCost(uint16_t len)
{
#ifdef ENC28J60_PROFILE
    static const uint8_t bcMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (len < 24) len = 24;  // the header and the marker
    if (len > PicoENC28J60::MAX_FRAME_SIZE) len = PicoENC28J60::MAX_FRAME_SIZE;
    enc28j60* h = driverHandle();
    enc28j60_profile availC, rxC, upC, startC, waitC;

    // An empty buffer. A frame of the LAN can come in between the last one dropped and the question: ask again
    uint8_t left = 1;
    for (int tries = 0; tries < 8 && left; tries++) {
        for (int n = 0; n < 200 && eth.available(); n++) eth.discardFrame();
        enc28j60_profile_reset();
        left = eth.available();
        enc28j60_profile_read(&availC);
    }

    // The frame to take: bench.py sends it when it reads COSTARM
    Serial.println("COSTARM");
    for (uint32_t t0 = millis(); !eth.available() && millis() - t0 < 3000;) delay(1);
    enc28j60_profile_reset();
    uint16_t rxlen = eth.receiveFrame(rxbuf, sizeof(rxbuf));
    enc28j60_profile_read(&rxC);

    // The frame to send: nothing on the LAN answers the EtherType 0x88B5
    memset(txbuf, 0, len);
    memcpy(txbuf, bcMac, 6);
    memcpy(txbuf + 6, MAC, 6);
    wr16(txbuf + 12, 0x88B5);
    memcpy(txbuf + 14, "PBNC-COST", 9);
    enc28j60_profile_reset();
    uint32_t id = enc28j60_frame_upload_blk(h, len, txbuf);
    enc28j60_profile_read(&upC);
    if (id == ENC28J60_INVALID_FRAME_ID) {
        Serial.println("COSTERR upload");
        return;
    }
    enc28j60_profile_reset();
    enc28j60_frame_tx_blk(h, id);
    enc28j60_profile_read(&startC);
    enc28j60_profile_reset();
    bool sent = enc28j60_frame_tx_wait_blk(h, id, 20000);
    enc28j60_profile_read(&waitC);

    Serial.printf("COST avail=%lu rx=%lu rxlen=%u up=%lu uplen=%u start=%lu eir=%lu wait=%lu empty=%d sent=%d spihz=%lu "
                  "avail_us=%lu rx_us=%lu up_us=%lu start_us=%lu wait_us=%lu\n",
                  (unsigned long)availC.commands, (unsigned long)rxC.commands, rxlen, (unsigned long)upC.commands, len,
                  (unsigned long)startC.commands, (unsigned long)waitC.wait_reads, (unsigned long)waitC.commands,
                  left == 0, sent, (unsigned long)h->cfg->spi_hz, (unsigned long)availC.command_us,
                  (unsigned long)rxC.command_us, (unsigned long)upC.command_us, (unsigned long)startC.command_us,
                  (unsigned long)waitC.wait_us);
#else
    (void)len;
    Serial.println("COSTERR noprofile");
#endif
}

// ASYNC 1: hand the chip to the C driver in its asynchronous mode, in place of the class. ASYNC 0 gives it back.
static void cmdAsyncOn()
{
    if (ah) {
        Serial.println("ASYNCERR active");
        return;
    }
    eth.end();  // gives back the chip, the state machine and the INT pin
    asyncConfig();
    enc28j60_status status;
    ah = enc28j60_init_status(&acfg, &status);
    if (!ah) {
        Serial.printf("ASYNCERR %s\n", enc28j60_status_string(status));
        eth.begin(makeConfig());  // the class has the chip again
        return;
    }
    // The interrupt sources that the FreeRTOS driver enables (initialize_enc28j60() with irq_cb); without irq_cb the
    // C driver leaves them all off
    enc28j60_irq_enable_blk(ah, PKTIE | LINKIE | TXIE | TXERIE | RXERIE);
    uploadedId = txInProgress = ENC28J60_INVALID_FRAME_ID;
    notified = false;
    check = false;
    discardEvery = 0;
    rxSeen = 0;
#ifdef BENCH_IRQ_PIN
    pinMode(BENCH_IRQ_PIN, INPUT_PULLUP);
    attachInterrupt(BENCH_IRQ_PIN, []() { intFlag = true; }, FALLING);
    intFlag = true;  // the line may be low already: the first pass runs whatever happens
#endif
    Serial.println("OK");
}

static void cmdAsyncOff()
{
    if (!ah) {
        Serial.println("ASYNCERR inactive");
        return;
    }
#ifdef BENCH_IRQ_PIN
    detachInterrupt(BENCH_IRQ_PIN);
#endif
    enc28j60_deinit(ah);
    ah = nullptr;
    uploadedId = txInProgress = ENC28J60_INVALID_FRAME_ID;
    if (eth.begin(makeConfig())) Serial.println("OK");
    else Serial.println("ASYNCERR begin");
}

// AOPT <check 0/1> <n>: read ERXRDPT back after every frame (check), and drop every n-th frame unseen (0 = none)
static void cmdAsyncOpt(bool readBack, uint32_t every)
{
    if (!ah) {
        Serial.println("ASYNCERR inactive");
        return;
    }
    check = readBack;
    discardEvery = every;
    rxSeen = 0;
    Serial.println("OK");
}

static int claimedDmaChannels()
{
    int n = 0;
    for (uint i = 0; i < NUM_DMA_CHANNELS; i++) n += dma_channel_is_claimed(i);
    return n;
}

// ASYNCCYCLE <n>: with the class stopped, start the C driver in its asynchronous mode and stop it again, n times,
// asking it something that goes through DMA each time. What stopping has to give back (DMA channels, heap) is counted
// before and after. A driver that keeps the 3 channels it takes runs out of them (the RP2040 has 12) and panics, so
// a line goes out before each start to show how far a run got. The class is started again at the end.
static void cmdAsyncCycle(uint32_t n)
{
    if (ah) {
        Serial.println("ASYNCERR active");
        return;
    }
    eth.end();
    int dmaBefore = claimedDmaChannels();
    int heapBefore = rp2040.getFreeHeap();
    uint32_t waitBefore = st.waitTimeouts, ok = 0;
    // A driver that runs out of DMA channels panics, and the panic stops the USB too: the board would stay dead until
    // it is unplugged. The watchdog starts it again instead, and bench.py sees a board that stopped answering.
    watchdog_enable(4000, true);
    for (uint32_t i = 1; i <= n; i++) {
        watchdog_update();
        Serial.printf("ASYNCCYCLE_STEP %lu\n", (unsigned long)i);
        Serial.flush();
        asyncConfig();
        enc28j60* h = enc28j60_init_status(&acfg, nullptr);
        if (!h) continue;
        uint32_t w0 = st.waitTimeouts;
        enc28j60_rx_count(h);  // a list through DMA: it only comes back in time if the callbacks work
        uint8_t rev = enc28j60_revision_blk(h);
        if (st.waitTimeouts == w0 && rev >= 1 && rev <= 0x1f) ok++;
        enc28j60_deinit(h);
    }
    watchdog_disable();
    int dmaAfter = claimedDmaChannels();
    int heapAfter = rp2040.getFreeHeap();
    bool began = eth.begin(makeConfig());
    Serial.printf("ASYNCCYCLE n=%lu ok=%lu dma_before=%d dma_after=%d heap_before=%d heap_after=%d waitto=%lu "
                  "began=%d\n",
                  (unsigned long)n, (unsigned long)ok, dmaBefore, dmaAfter, heapBefore, heapAfter,
                  (unsigned long)(st.waitTimeouts - waitBefore), began);
}

// While the C driver has the chip the class is stopped, and the commands that go through it would use a null driver
// handle: only these are served in that mode.
static bool worksInAsyncMode(const char* line)
{
    static const char* const names[] = {"IP ", "INFO", "STAT", "RESET", "ID", "ASYNC ", "AOPT ", "TX ", "TXB "};
    for (const char* name : names) {
        if (strncmp(line, name, strlen(name)) == 0) return true;
    }
    return false;
}

static void handleCommand(char* line)
{
    unsigned a, b, c, d;
    unsigned long n, g;
    if (ah && line[0] && !worksInAsyncMode(line)) {
        Serial.println("ASYNCERR active");
        return;
    }
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
        cbCount = 0;
        irqBase = eth.interruptCount();
        expectedSeq = 0;
        peer.valid = false;
        Serial.println("OK");
    } else if (strcmp(line, "ID") == 0) {
        Serial.println("READY bench=1");
    } else if (strcmp(line, "LINK") == 0) {
        startLinkCycle();
    } else if (strcmp(line, "PIOSHARE") == 0) {
        cmdPioShare();
    } else if (strcmp(line, "STALL") == 0) {
        cmdStall();
    } else if (strcmp(line, "LIFECYCLE") == 0) {
        cmdLifecycle();
    } else if (strcmp(line, "LOSTCHIP") == 0) {
        cmdLostChip();
    } else if (strncmp(line, "HOLD ", 5) == 0) {
        hold = line[5] == '1';
        Serial.println("OK");
    } else if (strcmp(line, "PEEK") == 0) {
        cmdPeek();
    } else if (strcmp(line, "DRAIN") == 0) {
        cmdDrain();
    } else if (strncmp(line, "CBMODE ", 7) == 0) {
        cbMode = line[7] == '1';
        cbFlag = true;  // serve the chip once, whatever raised the line before the callback was the way in
        Serial.println("OK");
    } else if (strcmp(line, "LINKEVENT") == 0) {
        cmdLinkEvent();
    } else if (strncmp(line, "SETMAC ", 7) == 0) {
        cmdSetMac(line + 7);
    } else if (sscanf(line, "SPITEST %lu %lu", &g, &n) == 2) {
        cmdSpiTest(g, n);
    } else if (sscanf(line, "TXMIX %u %lu", &a, &n) == 2) {
        cmdTxMix(a, n);
    } else if (sscanf(line, "TXRING %u %lu %lu", &a, &n, &g) == 3) {
        cmdTxRing((uint16_t)a, n, g);
    } else if (sscanf(line, "TSV %u %u", &a, &b) == 2) {
        cmdTsv((uint16_t)a, b != 0);
    } else if (sscanf(line, "COST %u", &a) == 1) {
        cmdCost((uint16_t)a);
    } else if (sscanf(line, "TXB %lu %u %lu", &n, &a, &g) == 3) {
        cmdTx(true, n, (uint16_t)a, g);
    } else if (sscanf(line, "TX %lu %u %lu", &n, &a, &g) == 3) {
        cmdTx(false, n, (uint16_t)a, g);
    } else if (strcmp(line, "ASYNC 1") == 0) {
        cmdAsyncOn();
    } else if (strcmp(line, "ASYNC 0") == 0) {
        cmdAsyncOff();
    } else if (sscanf(line, "AOPT %u %u", &a, &b) == 2) {
        cmdAsyncOpt(a != 0, b);
    } else if (sscanf(line, "ASYNCCYCLE %lu", &n) == 1) {
        cmdAsyncCycle(n);
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
// Probes that run before the real initialization
// ---------------------------------------------------------------------------

// With the chip select on a pin that goes nowhere no ENC28J60 can answer, so
// begin() has to fail, quickly, saying that there is no chip, instead of
// hanging. The real initialization afterwards shows that begin() can be tried
// again, and that the failure gave back what it had taken.
static void probeNoChip()
{
    ENC28J60Config wrong;
    wrong.cs_pin = 15;
    PicoENC28J60 probe(wrong);
    uint32_t t0 = millis();
    probeOk = probe.begin();
    probeMs = millis() - t0;
    probeErr = probe.lastError();
}

// Settings that make no sense have to be refused before the pins are touched
static void probeConfigs()
{
    struct Case {
        void (*spoil)(ENC28J60Config&);
    };
    static const Case cases[] = {
        {[](ENC28J60Config& c) { c.sclk_pin = c.mosi_pin; }},   // two lines on one pin
        {[](ENC28J60Config& c) { c.miso_pin = 99; }},           // a pin that does not exist
        {[](ENC28J60Config& c) { c.cs_pin = -1; }},             // the chip select is not optional
        {[](ENC28J60Config& c) { c.irq_pin = c.rst_pin; }},     // INT on the reset line
        {[](ENC28J60Config& c) { c.irq_pin = c.cs_pin; }},      // INT on an SPI line
        {[](ENC28J60Config& c) { c.pio_instance = 9; }},        // a PIO that does not exist
        {[](ENC28J60Config& c) { c.pio_sm = 7; }},              // a state machine that does not exist
    };
    for (const Case& k : cases) {
        ENC28J60Config cfg;
        k.spoil(cfg);
        PicoENC28J60 probe(cfg);
        uint32_t t0 = micros();
        bool ok = probe.begin();
        uint32_t us = micros() - t0;
        cfgTried++;
        if (!ok && probe.lastError() == PicoENC28J60::Error::Config) cfgRejected++;
        if (us > cfgMaxUs) cfgMaxUs = us;
    }
}

// With the instruction memory of every PIO full the SPI program has nowhere to
// go: begin() has to say so, instead of running a program from the wrong place.
static void probePioFull()
{
    static uint16_t nop;
    static pio_program_t filler;  // one instruction, loaded wherever there is room
    nop = pio_encode_nop();
    filler.instructions = &nop;
    filler.length = 1;
    filler.origin = -1;

    static uint8_t where[NUM_PIOS][32];
    static uint8_t count[NUM_PIOS];
    for (uint i = 0; i < NUM_PIOS; i++) {
        PIO pio = pio_get_instance(i);
        count[i] = 0;
        while (count[i] < 32 && pio_can_add_program(pio, &enc28j60_spi_program)) {
            int offset = pio_add_program(pio, &filler);
            if (offset < 0) break;
            where[i][count[i]++] = (uint8_t)offset;
        }
    }
    pioFilled = true;
    for (uint i = 0; i < NUM_PIOS; i++) pioFilled = pioFilled && !pio_can_add_program(pio_get_instance(i), &enc28j60_spi_program);

    {
        ENC28J60Config cfg;
        PicoENC28J60 probe(cfg);
        uint32_t t0 = millis();
        pioFullOk = probe.begin();
        pioFullMs = millis() - t0;
        pioFullErr = probe.lastError();
    }

    for (uint i = 0; i < NUM_PIOS; i++) {
        for (int k = 0; k < count[i]; k++) pio_remove_program(pio_get_instance(i), &filler, where[i][k]);
    }
}

// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    probeNoChip();
    probeConfigs();
    probePioFull();

    // Called from the interrupt handler when the INT pin is wired, from poll() otherwise, and only
    // counts and raises a flag: the chip is talked to from loop()
    eth.setInterruptCallback([](void*) {
        cbCount = cbCount + 1;
        cbFlag = true;
    });
    bool ok = eth.begin();
    Serial.println("READY bench=1");
    if (!ok) Serial.printf("INITFAIL %s\n", PicoENC28J60::errorString(eth.lastError()));
}

void loop()
{
    pollSerial();
    if (ah) {  // asynchronous mode; HOLD and CBMODE do not apply
        asyncPoll();
        return;
    }
    // The chip is touched only when it asks for it: through its INT pin if it is
    // wired, by looking at its flags otherwise. In CBMODE the callback asks.
    if (eth.isInitialized() && !hold) {
        if (cbMode) {
            eth.poll();  // without the INT pin this calls the callback when interruptPending() says so
            if (cbFlag) {
                cbFlag = false;
                serviceInterrupt();
            }
        } else if (eth.interruptPending()) {
            serviceInterrupt();
        }
    }
#ifdef BENCH_IRQ_PIN
    linkCycleStep();
#endif
}
