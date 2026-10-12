// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#include "PicoENC28J60.h"

#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <string.h>

extern "C" {
#include "pico/enc28j60-driver.h"
}

// The byte count in the receive status vector includes the Ethernet FCS.
static const uint16_t ETH_HEADER_LEN = 14;
static const uint16_t ETH_FCS_LEN = 4;

// A full frame needs 1.2 ms on the wire; leave room for deferral and retries.
static const uint32_t TX_TIMEOUT_US = 20000;

// After a late collision the frame is sent again this many times at most, as
// the silicon errata (issue 13) asks.
static const uint8_t TX_MAX_RETRIES = 15;

// How long to leave the chip alone after an attempt to bring it back failed, so
// that a chip that is gone does not slow every call down.
static const uint32_t RECOVER_RETRY_MS = 1000;

// Both RAM areas must hold a full frame plus the chip's per-frame overhead.
static const uint16_t RX_SIZE_MIN = 0x0600;
static const uint16_t RX_SIZE_MAX = 0x1A00;

static inline enc28j60 *driver(void *handle)
{
    return static_cast<enc28j60 *>(handle);
}

static PicoENC28J60::Error toError(enc28j60_status status)
{
    switch (status) {
    case ENC28J60_OK:
        return PicoENC28J60::Error::None;
    case ENC28J60_ERR_CONFIG:
        return PicoENC28J60::Error::Config;
    case ENC28J60_ERR_PIO_BUSY:
        return PicoENC28J60::Error::PioBusy;
    case ENC28J60_ERR_PIO_SPACE:
        return PicoENC28J60::Error::PioSpace;
    case ENC28J60_ERR_NO_CHIP:
        return PicoENC28J60::Error::NoChip;
    case ENC28J60_ERR_NOMEM:
        return PicoENC28J60::Error::NoMemory;
    case ENC28J60_ERR_IO:
        break;
    }
    return PicoENC28J60::Error::Io;
}

const char *PicoENC28J60::errorString(Error error)
{
    switch (error) {
    case Error::None:
        return enc28j60_status_string(ENC28J60_OK);
    case Error::Config:
        return enc28j60_status_string(ENC28J60_ERR_CONFIG);
    case Error::PioBusy:
        return enc28j60_status_string(ENC28J60_ERR_PIO_BUSY);
    case Error::PioSpace:
        return enc28j60_status_string(ENC28J60_ERR_PIO_SPACE);
    case Error::NoChip:
        return enc28j60_status_string(ENC28J60_ERR_NO_CHIP);
    case Error::NoMemory:
        return enc28j60_status_string(ENC28J60_ERR_NOMEM);
    case Error::Io:
        break;
    }
    return enc28j60_status_string(ENC28J60_ERR_IO);
}

// Are the pins and the PIO choice something the driver can work with? The lines
// that are used must be on different GPIOs.
static bool validConfig(const ENC28J60Config &c)
{
    // The PIO drives these four, which must be GPIOs below 32
    const int gpios = NUM_BANK0_GPIOS;
    const int8_t spi[] = { c.mosi_pin, c.miso_pin, c.sclk_pin, c.cs_pin };
    const int count = sizeof(spi) / sizeof(spi[0]);
    for (int i = 0; i < count; i++) {
        if (spi[i] < 0 || spi[i] >= 32 || spi[i] >= gpios) {
            return false;
        }
        for (int j = 0; j < i; j++) {
            if (spi[i] == spi[j]) {
                return false;
            }
        }
    }

    // The reset and interrupt lines are optional
    const int8_t optional[] = { c.rst_pin, c.irq_pin };
    for (int i = 0; i < 2; i++) {
        if (optional[i] < -1 || optional[i] >= gpios) {
            return false;
        }
        for (int j = 0; j < count && optional[i] >= 0; j++) {
            if (optional[i] == spi[j]) {
                return false;
            }
        }
    }
    if (c.rst_pin >= 0 && c.rst_pin == c.irq_pin) {
        return false;
    }

    return c.pio_instance >= -1 && c.pio_instance < (int)NUM_PIOS &&
           c.pio_sm >= -1 && c.pio_sm < (int)NUM_PIO_STATE_MACHINES;
}

// A MAC address for the board: locally administered and unicast (0x02), the
// rest from the unique id of the board, which is that of its flash chip
static void boardMacAddress(uint8_t mac[6])
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    mac[0] = 0x02;
    mac[1] = id.id[3] ^ id.id[0];
    mac[2] = id.id[4] ^ id.id[1];
    mac[3] = id.id[5] ^ id.id[2];
    mac[4] = id.id[6];
    mac[5] = id.id[7];
}

static bool isZeroMac(const uint8_t mac[6])
{
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) {
            return false;
        }
    }
    return true;
}

PicoENC28J60::PicoENC28J60(const ENC28J60Config &config)
    : _config(config), _eth(nullptr), _cfg(nullptr), _initialized(false),
      _irq_callback(nullptr), _irq_user_data(nullptr)
{
}

PicoENC28J60::~PicoENC28J60()
{
    end();
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

    // Check before anything is touched
    if (!validConfig(_config)) {
        _error = Error::Config;
        return false;
    }

    if (isZeroMac(_config.mac_addr)) {
        boardMacAddress(_config.mac_addr);
    }

    _hardwareReset();

    // Get default configuration from original library
    enc28j60_config *cfg = enc28j60_get_default_config();
    if (!cfg) {
        _error = Error::NoMemory;
        return false;
    }

    // Override with user configuration. A PIO or a state machine that was not
    // chosen is left to the driver, which claims it.
    cfg->pio = _config.pio_instance < 0 ?
                   nullptr :
                   pio_get_instance(_config.pio_instance);
    cfg->sm = _config.pio_sm < 0 ? ENC28J60_SM_ANY : _config.pio_sm;
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

    // Initialize the driver; NULL means that it could not be done, and the
    // status says why
    enc28j60_status status;
    enc28j60 *eth = enc28j60_init_status(cfg, &status);
    if (!eth) {
        free(cfg);
        _error = toError(status);
        return false;
    }

    _eth = eth;
    _cfg = cfg;
    _initialized = true;
    _error = Error::None;
    _ioCounted = false;
    _recoverWait = false;

    if (_config.irq_pin >= 0) {
        // INT is active low. The first interruptPending() reports true in any
        // case, in case the line was already low before the handler existed.
        _irq_pending = true;
        pinMode(_config.irq_pin, INPUT_PULLUP);
        attachInterruptParam(_config.irq_pin, _staticIRQHandler, FALLING, this);
        _irq_attached = true;
        _enableChipInterrupts();
    }

    return true;
}

void PicoENC28J60::end()
{
    if (!_eth) {
        return;
    }

    if (_irq_attached) {
        // The handler holds a pointer to this object: remove it before the
        // pin can be taken away from the chip
        detachInterrupt(_config.irq_pin);
        _irq_attached = false;
    }

    // Stops the chip and gives back the state machine and the memory
    enc28j60_deinit(driver(_eth));
    free(_cfg);
    _eth = nullptr;
    _cfg = nullptr;
    _initialized = false;
    _irq_pending = false;
}

// sendFrame() waits for the transmission itself, so TXIE stays off
void PicoENC28J60::_enableChipInterrupts()
{
    enc28j60_irq_enable_blk(driver(_eth),
                            INT_PACKET_RX | INT_LINK_CHANGE | INT_RX_ERROR);
}

// The driver gave up on the state machine, see enc28j60_io_failed(): count it,
// and try to bring everything back, but not more often than every
// RECOVER_RETRY_MS. Everything returns the same failures meanwhile.
bool PicoENC28J60::_usable()
{
    if (!_initialized) {
        return false;
    }
    if (!enc28j60_io_failed(driver(_eth))) {
        return true;
    }

    if (!_ioCounted) {
        _ioCounted = true;
        _stats.io_errors++;
    }
    // _recoverAt is compared as a signed difference, which only holds for a
    // time that is at most 24 days away: it is looked at only while it is fresh
    if (_recoverWait && (int32_t)(millis() - _recoverAt) < 0) {
        return false;
    }

    // The chip may have been affected as well, so it starts from its reset
    _hardwareReset();
    bool recovered = enc28j60_recover(driver(_eth));
    if (recovered && _config.irq_pin >= 0) {
        _enableChipInterrupts();
        _irq_pending = true;
    }
    if (!recovered) {
        _recoverAt = millis() + RECOVER_RETRY_MS;
        _recoverWait = true;
        return false;
    }
    _recoverWait = false;
    _ioCounted = false;
    _stats.recoveries++;
    return true;
}

bool PicoENC28J60::_failed() const
{
    return enc28j60_io_failed(driver(_eth));
}

bool PicoENC28J60::begin(const ENC28J60Config &config)
{
    if (_initialized) {
        return true;
    }
    _config = config;
    return begin();
}

bool PicoENC28J60::checkChip()
{
    if (!_usable()) {
        return false;
    }
    enc28j60 *eth = driver(_eth);
    bool alive = enc28j60_alive_blk(eth);
    if (_failed()) {
        return false;
    }
    if (alive) {
        return true;
    }

    // Configured again from the start, as after a failed transfer. If that does
    // not work the driver stays failed, and _usable() tries again.
    _stats.chip_resets++;
    if (!enc28j60_recover(eth)) {
        return false;
    }
    if (_config.irq_pin >= 0) {
        _enableChipInterrupts();
        _irq_pending = true;
    }
    _stats.recoveries++;
    return true;
}

bool PicoENC28J60::linkStatus()
{
    if (!_usable())
        return false;
    bool up = enc28j60_link_status_blk(driver(_eth));
    return up && !_failed();
}

uint8_t PicoENC28J60::getRevision()
{
    if (!_usable())
        return 0xFF;
    uint8_t revision = enc28j60_revision_blk(driver(_eth));
    return _failed() ? 0xFF : revision;
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
        // Also remembered by the driver, which programs it again if it has to
        // start the chip over
        enc28j60_set_mac_blk(driver(_eth), mac);
    }
}

bool PicoENC28J60::sendFrame(const uint8_t *frame, uint16_t len)
{
    if (!_usable() || !frame || len == 0 || len > MAX_FRAME_SIZE) {
        return false;
    }

    enc28j60 *eth = driver(_eth);

    // Upload the frame to the transmit area; this fails if there is no room
    uint32_t frame_id = enc28j60_frame_upload_blk(eth, len, frame);
    if (_failed() || frame_id == ENC28J60_INVALID_FRAME_ID) {
        _stats.tx_failures++;
        return false;
    }

    // Send it and wait until it is out, which also releases the buffer space.
    // A frame that the MAC gave up on after a late collision is sent again.
    uint8_t retries = 0;
    enc28j60_tx_status result = enc28j60_frame_send_blk(
        eth, frame_id, TX_TIMEOUT_US, TX_MAX_RETRIES, &retries);
    _stats.tx_retries += retries;
    if (result != ENC28J60_TX_OK) {
        _stats.tx_failures++;
        if (result == ENC28J60_TX_ABORTED) {
            _stats.tx_aborts++;
        }
        return false;
    }
    _stats.tx_frames++;
    return true;
}

uint8_t PicoENC28J60::available()
{
    if (!_usable())
        return 0;
    uint8_t count = enc28j60_rx_count_blk(driver(_eth));
    return _failed() ? 0 : count;
}

// Reads the header of the next frame in the receive buffer. A header that makes
// no sense means that the buffer and the read pointer got out of step; start
// over instead of parsing garbage. Returns false if there is no valid frame.
bool PicoENC28J60::_readRxInfo(uint32_t *rx_info)
{
    enc28j60 *eth = driver(_eth);
    if (enc28j60_rx_count_blk(eth) == 0) {
        return false;
    }

    uint32_t info = enc28j60_frame_rx_info_blk(eth);
    if (_failed()) {
        return false;
    }
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
    if (!_usable() || !buffer || max_len == 0) {
        return 0;
    }

    uint32_t rx_info;
    if (!_readRxInfo(&rx_info)) {
        return 0;
    }

    enc28j60 *eth = driver(_eth);
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
    if (_failed()) {
        return 0;
    }
    _stats.rx_frames++;
    return frame_len;
}

void PicoENC28J60::discardFrame()
{
    if (!_usable())
        return;

    uint32_t rx_info;
    if (_readRxInfo(&rx_info)) {
        enc28j60_frame_discard_blk(driver(_eth), rx_info);
    }
}

uint8_t PicoENC28J60::getInterruptFlags()
{
    if (!_usable())
        return 0;
    uint8_t flags = enc28j60_irq_flags_blk(driver(_eth));

    // The pin is masked now, so no new edge can arrive before the acknowledge,
    // and one that came earlier is covered by the flags just read. Without this
    // an edge whose handler ran late would make the next interruptPending()
    // report an event that has already been served.
    _irq_pending = false;
    return flags;
}

uint8_t PicoENC28J60::peekInterruptFlags()
{
    if (!_usable())
        return 0;
    uint8_t flags = enc28j60_irq_status_blk(driver(_eth));
    return _failed() ? 0 : flags;
}

bool PicoENC28J60::hasPacket()
{
    return available() > 0;
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
    if (!_usable())
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
    if (!_usable())
        return;
    enc28j60_irq_ack_blk(driver(_eth), flags);
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
