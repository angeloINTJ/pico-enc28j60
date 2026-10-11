#include "PicoENC28J60.h"

// Include the original C driver headers
extern "C" {
#include "pico/enc28j60-driver.h"
#include <hardware/gpio.h>
#include <pico/stdlib.h>
#include <string.h>
}

PicoENC28J60::PicoENC28J60(const ENC28J60Config& config)
    : _config(config), _eth(nullptr), _initialized(false), _irq_callback(nullptr), _irq_user_data(nullptr) {
    // Copy MAC address
    memcpy(_config.mac_addr, config.mac_addr, 6);
}

PicoENC28J60::~PicoENC28J60() {
    if (_eth) {
        enc28j60_deinit(static_cast<enc28j60*>(_eth));
        _eth = nullptr;
    }
    _initialized = false;
}

void PicoENC28J60::_hardwareReset() {
    if (_config.rst_pin >= 0) {
        gpio_init(_config.rst_pin);
        gpio_set_dir(_config.rst_pin, GPIO_OUT);
        gpio_put(_config.rst_pin, 0);
        sleep_ms(10);
        gpio_put(_config.rst_pin, 1);
        sleep_ms(10);
    }
}

bool PicoENC28J60::begin() {
    if (_initialized) {
        return true;
    }
    
    // Hardware reset
    _hardwareReset();
    
    // Get default configuration from original library
    enc28j60_config* cfg = enc28j60_get_default_config();
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
    cfg->irq_pin = (_config.irq_pin >= 0) ? _config.irq_pin : _config.rst_pin;
    cfg->dma_irq = 0;
    cfg->rx_size = _config.rx_buffer_size;
    memcpy(cfg->mac_addr, _config.mac_addr, 6);
    
    // Set up interrupt callback if provided
    if (_irq_callback) {
        cfg->irq_cb = _staticIRQHandler;
        cfg->irq_data = this;
    } else {
        cfg->irq_cb = nullptr;
        cfg->irq_data = nullptr;
    }
    
    // No notification callbacks for Arduino (polling mode)
    cfg->notify.notify = nullptr;
    cfg->notify.wait = nullptr;
    cfg->notify.data = nullptr;
    
    // Initialize the driver
    enc28j60* eth = enc28j60_init(cfg);
    if (!eth) {
        free(cfg);
        return false;
    }
    
    _eth = eth;
    _initialized = true;
    
    return true;
}

bool PicoENC28J60::linkStatus() {
    if (!_initialized) return false;
    return enc28j60_link_status_blk(static_cast<enc28j60*>(_eth));
}

uint8_t PicoENC28J60::getRevision() {
    if (!_initialized) return 0xFF;
    return enc28j60_revision_blk(static_cast<enc28j60*>(_eth));
}

void PicoENC28J60::getMACAddress(uint8_t* mac) {
    if (!_initialized || !mac) return;
    memcpy(mac, _config.mac_addr, 6);
}

void PicoENC28J60::setMACAddress(const uint8_t* mac) {
    if (!mac) return;
    memcpy(_config.mac_addr, mac, 6);
    if (_initialized) {
        // Update the hardware MAC address using public API
        enc28j60* eth = static_cast<enc28j60*>(_eth);
        // We need to use the internal registers - use the blocking functions
        // The public API doesn't expose MAC writing directly, so we use the 
        // frame upload/tx functions which set up MAC internally
        // For now, just update our copy - full MAC change requires re-init
    }
}

bool PicoENC28J60::sendFrame(const uint8_t* frame, uint16_t len) {
    if (!_initialized || !frame || len == 0 || len > 1518) {
        return false;
    }
    
    enc28j60* eth = static_cast<enc28j60*>(_eth);
    
    // Upload frame to ENC28J60 buffer
    uint32_t frame_id = enc28j60_frame_upload_blk(eth, len, frame);
    if (frame_id == ENC28J60_INVALID_FRAME_ID) {
        return false;
    }
    
    // Transmit the frame
    enc28j60_frame_tx_blk(eth, frame_id);
    
    return true;
}

uint8_t PicoENC28J60::available() {
    if (!_initialized) return 0;
    return enc28j60_rx_count_blk(static_cast<enc28j60*>(_eth));
}

uint16_t PicoENC28J60::receiveFrame(uint8_t* buffer, uint16_t max_len) {
    if (!_initialized || !buffer || max_len == 0) {
        return 0;
    }
    
    enc28j60* eth = static_cast<enc28j60*>(_eth);
    
    // Check if there's a packet
    uint8_t count = enc28j60_rx_count_blk(eth);
    if (count == 0) {
        return 0;
    }
    
    // Get packet info
    uint32_t rx_info = enc28j60_frame_rx_info_blk(eth);
    uint16_t size = rx_info & 0xFFFF;
    
    if (size == 0 || size > max_len) {
        // Discard invalid packet
        enc28j60_frame_discard_blk(eth, rx_info);
        return 0;
    }
    
    // Read the packet
    enc28j60_frame_rx_blk(eth, rx_info, buffer);
    
    return size;
}

void PicoENC28J60::discardFrame() {
    if (!_initialized) return;
    
    enc28j60* eth = static_cast<enc28j60*>(_eth);
    uint8_t count = enc28j60_rx_count_blk(eth);
    if (count > 0) {
        uint32_t rx_info = enc28j60_frame_rx_info_blk(eth);
        enc28j60_frame_discard_blk(eth, rx_info);
    }
}

uint8_t PicoENC28J60::getInterruptFlags() {
    if (!_initialized) return 0;
    return enc28j60_irq_flags_blk(static_cast<enc28j60*>(_eth));
}

bool PicoENC28J60::hasPacket() {
    uint8_t flags = getInterruptFlags();
    return (flags & INT_PACKET_RX) != 0;
}

bool PicoENC28J60::hasTXComplete() {
    uint8_t flags = getInterruptFlags();
    return (flags & INT_TX_COMPLETE) != 0;
}

bool PicoENC28J60::hasLinkChange() {
    uint8_t flags = getInterruptFlags();
    return (flags & INT_LINK_CHANGE) != 0;
}

bool PicoENC28J60::hasRXError() {
    uint8_t flags = getInterruptFlags();
    return (flags & INT_RX_ERROR) != 0;
}

void PicoENC28J60::acknowledgeInterrupts(uint8_t flags) {
    if (!_initialized) return;
    enc28j60_irq_ack_blk(static_cast<enc28j60*>(_eth), flags);
}

void PicoENC28J60::setInterruptCallback(void (*callback)(void*), void* user_data) {
    _irq_callback = callback;
    _irq_user_data = user_data;
    
    // Note: Changing callback at runtime requires re-initialization
    // This is a limitation of the original driver design
}

void PicoENC28J60::poll() {
    if (!_initialized) return;
    
    uint8_t flags = getInterruptFlags();
    if (flags != 0) {
        // Acknowledge interrupts
        acknowledgeInterrupts(flags);
        
        // Call user callback if set
        if (_irq_callback) {
            _irq_callback(_irq_user_data);
        }
    }
}

void PicoENC28J60::_staticIRQHandler(void* data) {
    PicoENC28J60* self = static_cast<PicoENC28J60*>(data);
    if (self && self->_irq_callback) {
        self->_irq_callback(self->_irq_user_data);
    }
}
