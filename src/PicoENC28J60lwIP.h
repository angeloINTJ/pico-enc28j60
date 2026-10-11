#ifndef PICO_ENC28J60_LWIP_H
#define PICO_ENC28J60_LWIP_H

/**
 * @file PicoENC28J60lwIP.h
 * @brief The ENC28J60 as a network interface of the lwIP stack in the arduino-pico core
 *
 * Gives you DHCP, TCP and UDP sockets, WiFiClient / WiFiServer, WebServer and
 * everything else that runs on top of lwIP, with the PIO-driven ENC28J60 and
 * any pin assignment:
 *
 *     #include <PicoENC28J60lwIP.h>
 *     #include <WebServer.h>
 *
 *     PicoENC28J60lwIP eth;      // default pins, see ENC28J60Config
 *     WebServer server(80);
 *
 *     void setup() {
 *         eth.begin();           // DHCP; or eth.config(ip, gateway, mask) first
 *         while (!eth.connected()) delay(100);
 *         server.begin();
 *     }
 *     void loop() { server.handleClient(); }
 *
 * The header is self-contained: nothing is added to the build of sketches that
 * only use PicoENC28J60.h. Once the interface is started, leave the chip to
 * lwIP and do not call the PicoENC28J60 methods yourself.
 *
 * The chip is checked for frames every 5 ms (setPollPeriod() changes that) by a
 * timer of this class. The polling of the core cannot be relied on for a device
 * without an interrupt line: it is rescheduled every time user code calls into
 * lwIP, so a sketch that keeps calling it, like a WebServer loop with a client
 * connected, stops the reception of frames altogether.
 */

#include <LwipEthernet.h>
#include <LwipIntfDev.h>
#include <WiFi.h>

#ifndef __FREERTOS
#include <lwip/timeouts.h>
#include <pico/async_context.h>
#endif

#include "PicoENC28J60.h"

/**
 * @brief The raw device that LwipIntfDev drives; not used directly.
 */
class PicoENC28J60LwipDriver {
public:
    // LwipIntfDev passes the arguments of the SPI based drivers; the pins come
    // from the ENC28J60Config instead.
    PicoENC28J60LwipDriver(int8_t = -1, SPIClass& = SPI, int8_t = -1) {}

    void setConfig(const ENC28J60Config& config) { _config = config; }

    bool begin(const uint8_t* mac, netif* net) {
        (void)net;
        ENC28J60Config config = _config;
        memcpy(config.mac_addr, mac, 6);
        if (!_eth.begin(config)) {
            return false;
        }
        _frameLen = 0;
        _linked = _eth.linkStatus();
        _lastLinkCheck = millis();
        return true;
    }

    // The driver cannot be torn down, see PicoENC28J60::~PicoENC28J60().
    void end() {}

    // Called by lwIP with its lock held. LWIP_NETIF_TX_SINGLE_PBUF is set in the
    // core, so p is one contiguous buffer.
    uint16_t sendFrame(struct pbuf* p) {
        return _eth.sendFrame((const uint8_t*)p->payload, p->len) ? p->len : 0;
    }

    // Everything below runs from the lwIP polling worker with the lwIP lock held,
    // which is what keeps the PIO state machine to one user at a time.
    uint16_t readFrameSize() {
        if (_frameLen == 0) {
            uint32_t now = millis();
            if (now - _lastLinkCheck >= LINK_CHECK_MS) {
                _lastLinkCheck = now;
                _linked = _eth.linkStatus();
            }
            _frameLen = _eth.receiveFrame(_frame, sizeof(_frame));
        }
        return _frameLen;
    }

    uint16_t readFrameData(uint8_t* buffer, uint16_t size) {
        uint16_t n = (size < _frameLen) ? size : _frameLen;
        memcpy(buffer, _frame, n);
        _frameLen = 0;
        return n;
    }

    void discardFrame(uint16_t) { _frameLen = 0; }

    // Refreshed by the polling worker, so this is safe to call from loop().
    bool isLinked() { return _linked; }

    //! Frame counters of the driver, for diagnostics
    const PicoENC28J60::Stats& driverStats() const { return _eth.stats(); }

    constexpr bool isLinkDetectable() const { return true; }
    constexpr bool needsSPI() const { return false; }  // the SPI link is a PIO program
    static constexpr bool interruptIsPossible() { return false; }
    static constexpr PinStatus interruptMode() { return LOW; }

private:
    static constexpr uint32_t LINK_CHECK_MS = 250;

    PicoENC28J60 _eth;
    ENC28J60Config _config;
    uint8_t _frame[PicoENC28J60::MAX_FRAME_SIZE];
    uint16_t _frameLen = 0;
    bool _linked = false;
    uint32_t _lastLinkCheck = 0;
};

/**
 * @brief ENC28J60 network interface for lwIP
 *
 * Has the same API as the other wired Ethernet interfaces of the core
 * (ENC28J60lwIP, Wiznet5500lwIP, ...): begin(), config(), connected(),
 * localIP(), linkStatus(), ...
 */
class PicoENC28J60lwIP : public LwipIntfDev<PicoENC28J60LwipDriver> {
public:
    explicit PicoENC28J60lwIP(const ENC28J60Config& config = ENC28J60Config()) { setConfig(config); }

    //! Start the interface (DHCP unless config() was called first); false if no ENC28J60 answers
    bool begin(const uint8_t* macAddress = nullptr, const uint16_t mtu = DEFAULT_MTU) {
        if (!LwipIntfDev<PicoENC28J60LwipDriver>::begin(macAddress, mtu)) {
            return false;
        }
#ifndef __FREERTOS
        memset(&_pollWorker, 0, sizeof(_pollWorker));
        _pollWorker.do_work = poll;
        _pollWorker.user_data = this;
        async_context_add_at_time_worker_in_ms(__getEthernetContext(), &_pollWorker, _pollMs);
#endif
        return true;
    }

    void end() {
#ifndef __FREERTOS
        async_context_remove_at_time_worker(__getEthernetContext(), &_pollWorker);
#endif
        LwipIntfDev<PicoENC28J60LwipDriver>::end();
    }

    //! How often to check the chip for frames, in milliseconds (default 5)
    void setPollPeriod(uint32_t ms) {
        if (ms > 0) {
            _pollMs = ms;
        }
    }

private:
#ifndef __FREERTOS
    // Runs in the lwIP context with its lock held. It schedules itself again, so
    // calls into lwIP from user code cannot push it back like they do with the
    // polling worker of the core.
    static void poll(async_context_t* context, async_at_time_worker_t* worker) {
        PicoENC28J60lwIP* self = static_cast<PicoENC28J60lwIP*>(worker->user_data);
        self->handlePackets();
        sys_check_timeouts();
        async_context_add_at_time_worker_in_ms(context, worker, self->_pollMs);
    }

    async_at_time_worker_t _pollWorker;
#endif
    uint32_t _pollMs = 5;
};

#endif  // PICO_ENC28J60_LWIP_H
