// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#include "PicoENC28J60.h"

#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <string.h>

extern "C" {
#include "pico/enc28j60-driver.h"
}

// The byte count in the receive status vector includes the Ethernet FCS.
static const uint16_t ETH_HEADER_LEN = 14;
static const uint16_t ETH_FCS_LEN = 4;

// A full frame needs 1.2 ms on the wire; leave room for deferral and retries.
static const uint32_t TX_TIMEOUT_US = 20000;

// Both RAM areas must hold a full frame plus the chip's per-frame overhead.
static const uint16_t RX_SIZE_MIN = 0x0600;
static const uint16_t RX_SIZE_MAX = 0x1A00;

PicoENC28J60::PicoENC28J60(const ENC28J60Config &config)
    : _config(config), _eth(nullptr), _initialized(false),
      _irq_callback(nullptr), _irq_user_data(nullptr)
{
}

PicoENC28J60::~PicoENC28J60()
{
    // enc28j60_deinit() cannot stop the PIO state machine, so the hardware
    // keeps running; this only drops the handle.
    if (_eth) {
        if (_irq_attached) {
            // The handler holds a pointer to this object; silence the pin first
            detachInterrupt(_config.irq_pin);
            enc28j60_irq_enable_blk(static_cast<enc28j60 *>(_eth), 0);
            _irq_attached = false;
        }
        enc28j60_deinit(static_cast<enc28j60 *>(_eth));
        _eth = nullptr;
    }
    _initialized = false;
}

void PicoENC28J60::_hardwareReset()
{
    if (_config.rst_pin >= 0) {
        gpio_init(_config.rst_pin);
        gpio_set_dir(_config.rst_pin, GPIO_OUT);
        gpio_put(_config.rst_pin, 0);
        sleep_ms(10);
        gpio_put(_config.rst_pin, 1);
        sleep_ms(10);
    }
}

bool PicoENC28J60::begin()
{
    if (_initialized) {
        return true;
    }

    // The receive buffer needs an even size (silicon errata: ERXRDPT must be
    // odd) and both RAM areas must hold a full frame.
    uint16_t rx = _config.rx_buffer_size & ~1u;
    if (rx < RX_SIZE_MIN)
        rx = RX_SIZE_MIN;
    if (rx > RX_SIZE_MAX)
        rx = RX_SIZE_MAX;
    _config.rx_buffer_size = rx;

    // The INT pin shares nothing with the other lines
    if (_config.irq_pin >= 0 && (_config.irq_pin == _config.mosi_pin ||
                                 _config.irq_pin == _config.miso_pin ||
                                 _config.irq_pin == _config.sclk_pin ||
                                 _config.irq_pin == _config.cs_pin ||
                                 _config.irq_pin == _config.rst_pin)) {
        return false;
    }

    _hardwareReset();

    // Get default configuration from original library
    enc28j60_config *cfg = enc28j60_get_default_config();
    if (!cfg) {
        return false;
    }

    // Override with user configuration
    cfg->pio = (_config.pio_instance == 1) ? pio1 : pio0;
    cfg->sm = _config.pio_sm;
    cfg->miso_pin = _config.miso_pin;
    cfg->mosi_pin = _config.mosi_pin;
    cfg->sclk_pin = _config.sclk_pin;
    cfg->cs_pin = _config.cs_pin;
    cfg->dma_irq = 0;
    cfg->rx_size = _config.rx_buffer_size;
    memcpy(cfg->mac_addr, _config.mac_addr, 6);

    // The C driver does not get the interrupt: it would claim the one GPIO
    // callback that the SDK has per core, which the core of Arduino-Pico needs
    // for attachInterrupt(). The pin is wired up below through the core
    // instead.
    cfg->irq_cb = nullptr;
    cfg->irq_data = nullptr;

    // No notification callbacks for Arduino (blocking mode)
    cfg->notify.notify = nullptr;
    cfg->notify.wait = nullptr;
    cfg->notify.data = nullptr;

    // Initialize the driver; NULL means that no ENC28J60 is answering
    enc28j60 *eth = enc28j60_init(cfg);
    if (!eth) {
        free(cfg);
        return false;
    }

    _eth = eth;
    _initialized = true;

    if (_config.irq_pin >= 0) {
        // INT is active low. The first interruptPending() reports true in any
        // case, in case the line was already low before the handler existed.
        _irq_pending = true;
        pinMode(_config.irq_pin, INPUT_PULLUP);
        attachInterruptParam(_config.irq_pin, _staticIRQHandler, FALLING, this);
        _irq_attached = true;

        // sendFrame() waits for the transmission itself, so TXIE stays off
        enc28j60_irq_enable_blk(eth,
                                INT_PACKET_RX | INT_LINK_CHANGE | INT_RX_ERROR);
    }

    return true;
}

bool PicoENC28J60::begin(const ENC28J60Config &config)
{
    if (_initialized) {
        return true;
    }
    _config = config;
    return begin();
}

bool PicoENC28J60::linkStatus()
{
    if (!_initialized)
        return false;
    return enc28j60_link_status_blk(static_cast<enc28j60 *>(_eth));
}

uint8_t PicoENC28J60::getRevision()
{
    if (!_initialized)
        return 0xFF;
    return enc28j60_revision_blk(static_cast<enc28j60 *>(_eth));
}

void PicoENC28J60::getMACAddress(uint8_t *mac) const
{
    if (!mac)
        return;
    memcpy(mac, _config.mac_addr, 6);
}

void PicoENC28J60::setMACAddress(const uint8_t *mac)
{
    if (!mac)
        return;
    memcpy(_config.mac_addr, mac, 6);
    if (_initialized) {
        enc28j60_set_mac_blk(static_cast<enc28j60 *>(_eth), mac);
    }
}

bool PicoENC28J60::sendFrame(const uint8_t *frame, uint16_t len)
{
    if (!_initialized || !frame || len == 0 || len > MAX_FRAME_SIZE) {
        return false;
    }

    enc28j60 *eth = static_cast<enc28j60 *>(_eth);

    // Upload the frame to the transmit area; this fails if there is no room
    uint32_t frame_id = enc28j60_frame_upload_blk(eth, len, frame);
    if (frame_id == ENC28J60_INVALID_FRAME_ID) {
        _stats.tx_failures++;
        return false;
    }

    // Send it and wait until it is out, which also releases the buffer space
    enc28j60_frame_tx_blk(eth, frame_id);
    if (!enc28j60_frame_tx_wait_blk(eth, frame_id, TX_TIMEOUT_US)) {
        _stats.tx_failures++;
        return false;
    }
    _stats.tx_frames++;
    return true;
}

uint8_t PicoENC28J60::available()
{
    if (!_initialized)
        return 0;
    return enc28j60_rx_count_blk(static_cast<enc28j60 *>(_eth));
}

// Reads the header of the next frame in the receive buffer. A header that makes
// no sense means that the buffer and the read pointer got out of step; start
// over instead of parsing garbage. Returns false if there is no valid frame.
bool PicoENC28J60::_readRxInfo(uint32_t *rx_info)
{
    enc28j60 *eth = static_cast<enc28j60 *>(_eth);
    if (enc28j60_rx_count_blk(eth) == 0) {
        return false;
    }

    uint32_t info = enc28j60_frame_rx_info_blk(eth);
    uint16_t size = ENC28J60_RX_SIZE(info);
    uint16_t next = ENC28J60_RX_NEXT(info);
    bool valid = size >= ETH_HEADER_LEN + ETH_FCS_LEN &&
                 size <= MAX_FRAME_SIZE + ETH_FCS_LEN &&
                 next < _config.rx_buffer_size && (next & 1) == 0;
    if (!valid) {
        enc28j60_rx_reset_blk(eth);
        _stats.rx_resets++;
        return false;
    }

    *rx_info = info;
    return true;
}

uint16_t PicoENC28J60::receiveFrame(uint8_t *buffer, uint16_t max_len)
{
    if (!_initialized || !buffer || max_len == 0) {
        return 0;
    }

    uint32_t rx_info;
    if (!_readRxInfo(&rx_info)) {
        return 0;
    }

    enc28j60 *eth = static_cast<enc28j60 *>(_eth);
    uint16_t frame_len = ENC28J60_RX_SIZE(rx_info) - ETH_FCS_LEN;
    if (frame_len > max_len) {
        // Does not fit the caller's buffer: drop it
        enc28j60_frame_discard_blk(eth, rx_info);
        _stats.rx_dropped++;
        return 0;
    }

    // Read the frame without its FCS; the pointers still move to the next frame
    rx_info = (rx_info & 0xFFFF0000u) | frame_len;
    enc28j60_frame_rx_blk(eth, rx_info, buffer);
    _stats.rx_frames++;
    return frame_len;
}

void PicoENC28J60::discardFrame()
{
    if (!_initialized)
        return;

    uint32_t rx_info;
    if (_readRxInfo(&rx_info)) {
        enc28j60_frame_discard_blk(static_cast<enc28j60 *>(_eth), rx_info);
    }
}

uint8_t PicoENC28J60::getInterruptFlags()
{
    if (!_initialized)
        return 0;
    uint8_t flags = enc28j60_irq_flags_blk(static_cast<enc28j60 *>(_eth));

    // The pin is masked now, so no new edge can arrive before the acknowledge,
    // and one that came earlier is covered by the flags just read. Without this
    // an edge whose handler ran late would make the next interruptPending()
    // report an event that has already been served.
    _irq_pending = false;
    return flags;
}

uint8_t PicoENC28J60::peekInterruptFlags()
{
    if (!_initialized)
        return 0;
    return enc28j60_irq_status_blk(static_cast<enc28j60 *>(_eth));
}

bool PicoENC28J60::hasPacket()
{
    return (peekInterruptFlags() & INT_PACKET_RX) != 0;
}

bool PicoENC28J60::hasTXComplete()
{
    return (peekInterruptFlags() & INT_TX_COMPLETE) != 0;
}

bool PicoENC28J60::hasLinkChange()
{
    return (peekInterruptFlags() & INT_LINK_CHANGE) != 0;
}

bool PicoENC28J60::hasRXError()
{
    return (peekInterruptFlags() & INT_RX_ERROR) != 0;
}

bool PicoENC28J60::interruptPending()
{
    if (!_initialized)
        return false;

    if (_config.irq_pin < 0) {
        // No wire: look at what the chip would have signalled
        return (peekInterruptFlags() &
                (INT_PACKET_RX | INT_LINK_CHANGE | INT_RX_ERROR)) != 0;
    }

    // The chip holds the line low until it is served, which makes the level a
    // reliable second source if an edge was missed
    bool low = digitalRead(_config.irq_pin) == LOW;

    // Test and clear. An edge that comes in after this is kept for the next
    // call.
    bool pending = _irq_pending;
    _irq_pending = false;
    return pending || low;
}

void PicoENC28J60::acknowledgeInterrupts(uint8_t flags)
{
    if (!_initialized)
        return;
    enc28j60_irq_ack_blk(static_cast<enc28j60 *>(_eth), flags);
}

void PicoENC28J60::setInterruptCallback(void (*callback)(void *),
                                        void *user_data)
{
    _irq_callback = callback;
    _irq_user_data = user_data;
}

void PicoENC28J60::poll()
{
    if (!_initialized || !_irq_callback || _config.irq_pin >= 0)
        return;

    if (interruptPending()) {
        _irq_callback(_irq_user_data);
    }
}

// Runs in interrupt context: touches no hardware besides this object
void PicoENC28J60::_staticIRQHandler(void *data)
{
    PicoENC28J60 *self = static_cast<PicoENC28J60 *>(data);
    self->_irq_count = self->_irq_count + 1;
    self->_irq_pending = true;
    if (self->_irq_callback) {
        self->_irq_callback(self->_irq_user_data);
    }
}
