// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#ifndef PICO_ENC28J60_LWIP_H
#define PICO_ENC28J60_LWIP_H

/**
 * @file PicoENC28J60lwIP.h
 * @brief The ENC28J60 as a network interface of the lwIP stack in the
 * arduino-pico core
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
 *         eth.begin();           // DHCP, or eth.config(...) before it
 *         while (!eth.connected()) delay(100);
 *         server.begin();
 *     }
 *     void loop() { server.handleClient(); }
 *
 * The header is self-contained: nothing is added to the build of sketches that
 * only use PicoENC28J60.h. Once the interface is started, leave the chip to
 * lwIP and do not call the PicoENC28J60 methods yourself.
 *
 * Without an interrupt line the chip is checked for frames by a timer of this
 * class: every 5 ms while the line is quiet (setPollPeriod() changes that), and
 * sooner, starting at every millisecond, after a frame went out or came in,
 * when the answer of the peer is most likely about to arrive. The polling of
 * the core cannot be relied on for a device without an interrupt line: it is
 * rescheduled every time user code calls into lwIP, so a sketch that keeps
 * calling it, like a WebServer loop with a client connected, stops the
 * reception of frames altogether.
 *
 * With the chip's INT pin wired and given as ENC28J60Config::irq_pin, a frame
 * is served the moment it arrives and an idle network costs no SPI traffic at
 * all; the timer stays as a safety net (every 20 ms by default). The interrupt
 * handler only wakes lwIP up: the chip is always talked to from the lwIP
 * context, never from the handler.
 *
 *     ENC28J60Config cfg;
 *     cfg.irq_pin = 6;                // INT on GP6
 *     PicoENC28J60lwIP eth(cfg);
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
    PicoENC28J60LwipDriver(int8_t = -1, SPIClass & = SPI, int8_t = -1)
    {
    }

    void setConfig(const ENC28J60Config &config)
    {
        _config = config;
    }

    //! Function to call from the interrupt handler of the INT pin; see
    //! PicoENC28J60::setInterruptCallback()
    void setWake(void (*wake)(void *), void *arg)
    {
        _wake = wake;
        _wakeArg = arg;
    }

    //! Function to call after each frame that goes out, from where lwIP sent
    //! it, with the lwIP lock held
    void setSentHook(void (*hook)(void *), void *arg)
    {
        _sent = hook;
        _sentArg = arg;
    }

    bool begin(const uint8_t *mac, netif *net)
    {
        ENC28J60Config config = _config;
        memcpy(config.mac_addr, mac, 6);
        _eth.setInterruptCallback(_wake, _wakeArg);
        if (!_eth.begin(config)) {
            return false;
        }
        _frameLen = 0;
        _burst = false;
        _netif = nullptr; // no call into lwIP for the state the link starts in
        _linked = false;
        _setLinked(_eth.linkStatus());
        _netif = net;
        _lastLinkCheck = millis();
        return true;
    }

    // Stops waking anybody up, releases the PIO state machine and the pins
    void end()
    {
        _eth.setInterruptCallback(nullptr, nullptr);
        _eth.end();
        _frameLen = 0;
        _burst = false;
        _linked = false;
        _netif = nullptr; // the core removes it
    }

    //! Why begin() failed, see PicoENC28J60::lastError()
    PicoENC28J60::Error lastError() const
    {
        return _eth.lastError();
    }

    // Called by lwIP with its lock held. LWIP_NETIF_TX_SINGLE_PBUF is set in
    // the core, so p is one contiguous buffer.
    uint16_t sendFrame(struct pbuf *p)
    {
        bool sent = _eth.sendFrame((const uint8_t *)p->payload, p->len);
        _traffic++;
        if (_sent) {
            _sent(_sentArg);
        }
        return sent ? p->len : 0;
    }

    // Everything below runs from the lwIP polling worker with the lwIP lock
    // held, which is what keeps the PIO state machine to one user at a time.
    uint16_t readFrameSize()
    {
        if (_frameLen != 0) {
            return _frameLen;
        }

        if (_config.irq_pin >= 0) {
            // A call that finds the INT line high returns without touching the
            // chip
            if (!_burst) {
                if (!_eth.interruptPending()) {
                    return 0;
                }
                _serveEvents();
                _burst = true;
            }
        } else {
            _checkLink();
        }

        _frameLen = _eth.receiveFrame(_frame, sizeof(_frame));
        if (_frameLen == 0) {
            _burst = false; // the receive buffer is empty
        }
        return _frameLen;
    }

    uint16_t readFrameData(uint8_t *buffer, uint16_t size)
    {
        uint16_t n = (size < _frameLen) ? size : _frameLen;
        memcpy(buffer, _frame, n);
        _frameLen = 0;
        _delivered++;
        _traffic++;
        return n;
    }

    //! Frames handed to lwIP so far
    uint32_t delivered() const
    {
        return _delivered;
    }

    //! Frames that went out or came in so far
    uint32_t traffic() const
    {
        return _traffic;
    }

    //! More frames may be waiting: lwIP reads a limited number of them at a
    //! time
    bool moreWaiting() const
    {
        return _burst;
    }

    void discardFrame(uint16_t)
    {
        _frameLen = 0;
    }

    // Refreshed by the polling worker, so this is safe to call from loop().
    bool isLinked()
    {
        return _linked;
    }

    //! Frame counters of the driver, for diagnostics
    const PicoENC28J60::Stats &driverStats() const
    {
        return _eth.stats();
    }

    constexpr bool isLinkDetectable() const
    {
        return true;
    }
    constexpr bool needsSPI() const
    {
        return false;
    } // the SPI link is a PIO program
    static constexpr bool interruptIsPossible()
    {
        return false;
    }
    static constexpr PinStatus interruptMode()
    {
        return LOW;
    }

  private:
    static constexpr uint32_t LINK_CHECK_MS = 250;

    void _checkLink()
    {
        uint32_t now = millis();
        if (now - _lastLinkCheck >= LINK_CHECK_MS) {
            _lastLinkCheck = now;
            _setLinked(_eth.linkStatus());
        }
    }

    // What the chip reports besides frames. The flags are cleared one by one: a
    // frame is not an event to clear, the receive buffer being empty is.
    void _serveEvents()
    {
        uint8_t events =
            _eth.peekInterruptFlags() &
            (PicoENC28J60::INT_LINK_CHANGE | PicoENC28J60::INT_RX_ERROR);
        if (events) {
            _eth.acknowledgeInterrupts(events);
        }
        // A missed event must not leave the state wrong for long: look as well
        // when the flag is set, and every now and then when it is not
        if ((events & PicoENC28J60::INT_LINK_CHANGE) ||
            millis() - _lastLinkCheck >= LINK_CHECK_MS) {
            _lastLinkCheck = millis();
            _setLinked(_eth.linkStatus());
        }
    }

    // Keeps lwIP's idea of the link in step with the chip's. When the cable
    // comes back lwIP asks the DHCP server again, in case the board was moved
    // to another network, and restarts what depends on the link.
    void _setLinked(bool up)
    {
        if (up == _linked) {
            return;
        }
        _linked = up;
        if (_netif) {
            if (up) {
                netif_set_link_up(_netif);
            } else {
                netif_set_link_down(_netif);
            }
        }
    }

    PicoENC28J60 _eth;
    ENC28J60Config _config;
    uint8_t _frame[PicoENC28J60::MAX_FRAME_SIZE];
    uint16_t _frameLen = 0;
    bool _linked = false;
    netif *_netif = nullptr; // the interface of lwIP, from begin() to end()
    bool _burst =
        false; // inside a run of frames that started with an interrupt
    uint32_t _delivered = 0;
    uint32_t _traffic = 0;
    uint32_t _lastLinkCheck = 0;
    void (*_wake)(void *) = nullptr;
    void *_wakeArg = nullptr;
    void (*_sent)(void *) = nullptr;
    void *_sentArg = nullptr;
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
    explicit PicoENC28J60lwIP(const ENC28J60Config &config = ENC28J60Config())
    {
        setConfig(config);
        _irq = config.irq_pin >= 0;
        _pollMs = _irq ? 20 : 5;
        setWake(wake, this);
        setSentHook(sent, this);
    }

    //! Start the interface (DHCP unless config() was called first); false if no
    //! ENC28J60 answers
    bool begin(const uint8_t *macAddress = nullptr,
               const uint16_t mtu = DEFAULT_MTU)
    {
#ifndef __FREERTOS
        async_context_t *context = __getEthernetContext();
        if (_irq) {
            // Before the chip can raise the line, the worker that it wakes must
            // exist
            memset(&_irqWorker, 0, sizeof(_irqWorker));
            _irqWorker.do_work = irqWork;
            _irqWorker.user_data = this;
            async_context_add_when_pending_worker(context, &_irqWorker);
        }
#endif
        if (!LwipIntfDev<PicoENC28J60LwipDriver>::begin(macAddress, mtu)) {
#ifndef __FREERTOS
            if (_irq) {
                async_context_remove_when_pending_worker(context, &_irqWorker);
            }
#endif
            return false;
        }
#ifndef __FREERTOS
        memset(&_pollWorker, 0, sizeof(_pollWorker));
        _pollWorker.do_work = poll;
        _pollWorker.user_data = this;
        async_context_add_at_time_worker_in_ms(context, &_pollWorker, _pollMs);
        _pollQueued = true;
#endif
        return true;
    }

    void end()
    {
#ifndef __FREERTOS
        async_context_t *context = __getEthernetContext();
        _pollQueued = false;
        async_context_remove_at_time_worker(context, &_pollWorker);
        if (_irq) {
            LwipIntfDev<PicoENC28J60LwipDriver>::end(); // stops the wake-ups
                                                        // first
            async_context_remove_when_pending_worker(context, &_irqWorker);
            return;
        }
#endif
        LwipIntfDev<PicoENC28J60LwipDriver>::end();
    }

    /**
     * How often to check the chip for frames while the line is quiet, in
     * milliseconds. Defaults to 5, or to 20 with an INT pin, where it only has
     * to catch what the interrupt missed. Without an INT pin the chip is looked
     * at sooner after each frame, see nextPollMs(), whatever this is set to.
     */
    void setPollPeriod(uint32_t ms)
    {
        if (ms > 0) {
            _pollMs = ms;
        }
    }

  private:
#ifndef __FREERTOS
    // One round of work, in the lwIP context with its lock held. Returns true
    // if lwIP stopped reading frames because of its limit per call, so that
    // there may be more.
    bool service()
    {
        uint32_t before = delivered();
        handlePackets();
        sys_check_timeouts();
        return delivered() - before >= RX_BURST_LIMIT || moreWaiting();
    }

    // Runs in the lwIP context. It schedules itself again, so calls into lwIP
    // from user code cannot push it back like they do with the polling worker
    // of the core.
    static void poll(async_context_t *context, async_at_time_worker_t *worker)
    {
        PicoENC28J60lwIP *self =
            static_cast<PicoENC28J60lwIP *>(worker->user_data);
        bool more = self->service();
        async_context_add_at_time_worker_in_ms(context, worker,
                                               more ? 0 : self->nextPollMs());
    }

    // How long to wait before looking at the chip again. Without the INT pin
    // the peer's answer to a frame that went out, and the next frame of a
    // burst, are usually about to arrive: after traffic the chip is looked at
    // every millisecond, then at growing intervals up to the relaxed period
    // while nothing comes. The peer is served within a millisecond instead of
    // up to a full period, and an idle line costs no more than before. With a
    // fixed period, where it happens to fall relative to the answer would
    // decide whether a reply takes a few milliseconds or a full period more.
    uint32_t nextPollMs()
    {
        if (_irq || _pollMs <= ACTIVE_POLL_MS) {
            _waitMs = _pollMs;
        } else if (traffic() != _seenTraffic) {
            _seenTraffic = traffic();
            _waitMs = ACTIVE_POLL_MS;
        } else if (_waitMs < _pollMs) {
            _waitMs = _waitMs * 2 < _pollMs ? _waitMs * 2 : _pollMs;
        }
        return _waitMs;
    }

    // A frame just went out, so its answer is about to arrive: if the timer is
    // far off, bring it closer. Called with the lwIP lock held, from user code
    // or from the worker itself, which is not queued while it runs and
    // schedules itself when it is done.
    void pollSoon()
    {
        if (_irq || !_pollQueued) {
            return;
        }
        int64_t left =
            absolute_time_diff_us(get_absolute_time(), _pollWorker.next_time);
        if (left <= (int64_t)ACTIVE_POLL_MS * 1000) {
            return;
        }
        async_context_t *context = __getEthernetContext();
        if (async_context_remove_at_time_worker(context, &_pollWorker)) {
            async_context_add_at_time_worker_in_ms(context, &_pollWorker,
                                                   ACTIVE_POLL_MS);
        }
    }

    static void sent(void *arg)
    {
        static_cast<PicoENC28J60lwIP *>(arg)->pollSoon();
    }

    // Runs in the lwIP context after the INT pin fell
    static void irqWork(async_context_t *context,
                        async_when_pending_worker_t *worker)
    {
        PicoENC28J60lwIP *self =
            static_cast<PicoENC28J60lwIP *>(worker->user_data);
        if (self->service()) {
            async_context_set_work_pending(context, worker);
        }
    }

    // Called from the interrupt handler of the INT pin: nothing but a request
    static void wake(void *arg)
    {
        PicoENC28J60lwIP *self = static_cast<PicoENC28J60lwIP *>(arg);
        async_context_set_work_pending(__getEthernetContext(),
                                       &self->_irqWorker);
    }

    // lwIP's handlePackets() hands over this many frames at most per call
    static constexpr uint32_t RX_BURST_LIMIT = 9;

    // The polling period right after traffic; it doubles from there while the
    // chip has nothing for us, up to the period that was set
    static constexpr uint32_t ACTIVE_POLL_MS = 1;

    async_at_time_worker_t _pollWorker;
    async_when_pending_worker_t _irqWorker;
    bool _pollQueued = false; // _pollWorker is in the queue of the context
#else
    static void wake(void *)
    {
    }
    static void sent(void *)
    {
    }
#endif
    bool _irq = false;
    uint32_t _pollMs = 5;
    uint32_t _waitMs = 5; // the current polling period, see nextPollMs()
    uint32_t _seenTraffic = 0;
};

#endif // PICO_ENC28J60_LWIP_H
