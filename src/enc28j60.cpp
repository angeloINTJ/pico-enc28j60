#include "enc28j60.h"

extern "C" {
#include "enc28j60-cmd.h"
#include <stdlib.h>
#include <string.h>
}

// ============================================================================
// Constructor / Destructor / Move semantics
// ============================================================================

ENC28J60::ENC28J60() {
    // Initialize notification callbacks for DMA operations
    _notification.notify = _notify_func;
    _notification.wait = _wait_func;
    _notification.data = this;
}

ENC28J60::~ENC28J60() {
    end();
}

ENC28J60::ENC28J60(ENC28J60&& other) noexcept
    : _cfg(other._cfg), _eth(other._eth), _initialized(other._initialized),
      _link_up(other._link_up), _rx_size(other._rx_size),
      _irq_enabled(other._irq_enabled), _irq_callback(other._irq_callback),
      _irq_user_data(other._irq_user_data), _irq_flags(other._irq_flags),
      _notification(other._notification) {
    
    memcpy(_mac_addr, other._mac_addr, 6);
    
    // Null out source
    other._cfg = nullptr;
    other._eth = nullptr;
    other._initialized = false;
    other._irq_enabled = false;
    other._irq_callback = nullptr;
    other._irq_user_data = nullptr;
}

ENC28J60& ENC28J60::operator=(ENC28J60&& other) noexcept {
    if (this != &other) {
        end();
        
        _cfg = other._cfg;
        _eth = other._eth;
        _initialized = other._initialized;
        _link_up = other._link_up;
        _rx_size = other._rx_size;
        _irq_enabled = other._irq_enabled;
        _irq_callback = other._irq_callback;
        _irq_user_data = other._irq_user_data;
        _irq_flags = other._irq_flags;
        _notification = other._notification;
        memcpy(_mac_addr, other._mac_addr, 6);
        
        other._cfg = nullptr;
        other._eth = nullptr;
        other._initialized = false;
        other._irq_enabled = false;
        other._irq_callback = nullptr;
        other._irq_user_data = nullptr;
    }
    return *this;
}

// ============================================================================
// Initialization
// ============================================================================

bool ENC28J60::begin(const uint8_t* mac) {
    return begin(mac, DEFAULT_MISO_PIN, DEFAULT_MOSI_PIN, 
                 DEFAULT_SCLK_PIN, DEFAULT_IRQ_PIN, pio0, 0);
}

bool ENC28J60::begin(const uint8_t* mac, 
                     int miso_pin, int mosi_pin, int sclk_pin, int irq_pin,
                     PIO pio_instance, uint sm_index) {
    if (_initialized) {
        return true; // Already initialized
    }
    
    if (!mac) {
        return false;
    }
    
    memcpy(_mac_addr, mac, 6);
    
    // Get default config and customize pins
    _cfg = enc28j60_get_default_config();
    if (!_cfg) {
        return false;
    }
    
    _cfg->pio = pio_instance;
    _cfg->sm = sm_index;
    _cfg->miso_pin = miso_pin;
    _cfg->mosi_pin = mosi_pin;
    _cfg->sclk_pin = sclk_pin;
    _cfg->irq_pin = irq_pin;
    _cfg->rx_size = _rx_size;
    memcpy(_cfg->mac_addr, _mac_addr, 6);
    
    // Set up notification for non-blocking operations
    _cfg->notify = _notification;
    
    return _initDriver();
}

bool ENC28J60::begin(enc28j60_config* config) {
    if (_initialized || !config) {
        return false;
    }
    
    _cfg = config;
    memcpy(_mac_addr, _cfg->mac_addr, 6);
    _rx_size = _cfg->rx_size;
    
    // Ensure notification is set
    _cfg->notify = _notification;
    
    return _initDriver();
}

bool ENC28J60::_initDriver() {
    _eth = enc28j60_init(_cfg);
    if (!_eth) {
        _cleanup();
        return false;
    }
    
    _initialized = true;
    _link_up = linkStatus(); // Initial link check
    
    return true;
}

void ENC28J60::end() {
    if (!_initialized) {
        return;
    }
    
    // Disable interrupts if enabled
    if (_irq_enabled) {
        enableInterrupts(false);
    }
    
    if (_eth) {
        enc28j60_deinit(_eth);
        _eth = nullptr;
    }
    
    if (_cfg) {
        // Note: enc28j60_get_default_config() allocates with calloc
        // The config is owned by the driver after init, so we don't free it here
        // unless we created it ourselves. For safety, we don't free.
        _cfg = nullptr;
    }
    
    _initialized = false;
    _link_up = false;
}

void ENC28J60::_cleanup() {
    if (_eth) {
        enc28j60_deinit(_eth);
        _eth = nullptr;
    }
    _initialized = false;
}

// ============================================================================
// Status and Info
// ============================================================================

uint8_t ENC28J60::getRevision() const {
    if (!_initialized || !_eth) {
        return 0xFF;
    }
    return enc28j60_revision_blk(_eth);
}

bool ENC28J60::linkStatus() const {
    if (!_initialized || !_eth) {
        return false;
    }
    return enc28j60_link_status_blk(_eth);
}

void ENC28J60::getMacAddress(uint8_t* mac) const {
    if (mac) {
        memcpy(mac, _mac_addr, 6);
    }
}

void ENC28J60::setMacAddress(const uint8_t* mac) {
    if (!mac) {
        return;
    }
    memcpy(_mac_addr, mac, 6);
    if (_cfg) {
        memcpy(_cfg->mac_addr, mac, 6);
    }
}

// ============================================================================
// Packet Transmission
// ============================================================================

bool ENC28J60::sendPacket(const void* data, size_t size) {
    if (!_initialized || !_eth || !data || size == 0 || size > MAX_FRAME_SIZE) {
        return false;
    }
    
    uint32_t frame_id = enc28j60_frame_upload_blk(_eth, size, data);
    if (frame_id == ENC28J60_INVALID_FRAME_ID) {
        return false; // TX buffer full
    }
    
    enc28j60_frame_tx_blk(_eth, frame_id);
    
    // Wait for transmission to complete (blocking)
    // In a real implementation, you might want to poll or use interrupts
    // For now, we just confirm the frame was sent
    enc28j60_frame_tx_confirm(_eth, frame_id);
    
    return true;
}

bool ENC28J60::sendPacket(const void* data, size_t size, 
                          void (*callback)(void*), void* user_data) {
    if (!_initialized || !_eth || !data || size == 0 || size > MAX_FRAME_SIZE) {
        return false;
    }
    
    // Store callback for later use (when TX interrupt fires)
    _irq_callback = callback;
    _irq_user_data = user_data;
    _irq_enabled = true;
    
    // Enable TX interrupt
    // Note: The C driver enables interrupts during init if irq_cb is set
    // We need to ensure interrupts are configured
    
    uint32_t frame_id = enc28j60_frame_upload(_eth, size, data);
    if (frame_id == ENC28J60_INVALID_FRAME_ID) {
        return false;
    }
    
    enc28j60_frame_tx(_eth, frame_id);
    return true;
}

// ============================================================================
// Packet Reception
// ============================================================================

uint8_t ENC28J60::packetAvailable() const {
    if (!_initialized || !_eth) {
        return 0;
    }
    return enc28j60_rx_count_blk(_eth);
}

uint16_t ENC28J60::receivePacket(void* buffer, uint16_t max_size) {
    if (!_initialized || !_eth || !buffer || max_size == 0) {
        return 0;
    }
    
    uint8_t count = enc28j60_rx_count_blk(_eth);
    if (count == 0) {
        return 0;
    }
    
    uint32_t rx_info = enc28j60_frame_rx_info_blk(_eth);
    uint16_t size = ENC28J60_RX_SIZE(rx_info);
    
    if (size > max_size) {
        // Buffer too small, discard packet
        enc28j60_frame_discard_blk(_eth, rx_info);
        return 0;
    }
    
    enc28j60_frame_rx_blk(_eth, rx_info, buffer);
    return size;
}

uint16_t ENC28J60::receivePacketNonBlocking(void* buffer, uint16_t max_size) {
    if (!_initialized || !_eth || !buffer || max_size == 0) {
        return 0;
    }
    
    uint8_t count = enc28j60_rx_count(_eth);
    if (count == 0) {
        return 0;
    }
    
    uint32_t rx_info = enc28j60_frame_rx_info(_eth);
    uint16_t size = ENC28J60_RX_SIZE(rx_info);
    
    if (size > max_size) {
        enc28j60_frame_discard(_eth, rx_info);
        return 0;
    }
    
    enc28j60_frame_rx(_eth, rx_info, buffer);
    return size;
}

bool ENC28J60::discardPacket() {
    if (!_initialized || !_eth) {
        return false;
    }
    
    uint8_t count = enc28j60_rx_count_blk(_eth);
    if (count == 0) {
        return false;
    }
    
    uint32_t rx_info = enc28j60_frame_rx_info_blk(_eth);
    enc28j60_frame_discard_blk(_eth, rx_info);
    return true;
}

// ============================================================================
// Interrupt Handling
// ============================================================================

void ENC28J60::enableInterrupts(bool enable, 
                                void (*callback)(void*), 
                                void* user_data) {
    if (!_initialized || !_eth || !_cfg) {
        return;
    }
    
    if (enable) {
        _irq_callback = callback;
        _irq_user_data = user_data;
        _cfg->irq_cb = _irq_handler;
        _cfg->irq_data = this;
        _irq_enabled = true;
        
        // Re-initialize to set up GPIO interrupt
        // Note: This is a simplification - in practice you'd want to 
        // set up the interrupt without full re-init
    } else {
        _irq_enabled = false;
        _irq_callback = nullptr;
        _irq_user_data = nullptr;
        if (_cfg) {
            _cfg->irq_cb = nullptr;
            _cfg->irq_data = nullptr;
        }
    }
}

void ENC28J60::handleInterrupt() {
    if (!_initialized || !_eth || !_irq_enabled) {
        return;
    }
    
    // Read and clear interrupt flags
    uint8_t flags = enc28j60_irq_flags_blk(_eth);
    if (flags == 0) {
        return;
    }
    
    _processInterrupt(flags);
    enc28j60_irq_ack_blk(_eth, flags);
}

void ENC28J60::_irq_handler(void* data) {
    ENC28J60* self = static_cast<ENC28J60*>(data);
    if (self) {
        // Read flags in ISR context (non-blocking version)
        uint8_t flags = enc28j60_irq_flags(self->_eth);
        self->_irq_flags = flags;
        
        // Call user callback if set
        if (self->_irq_callback) {
            self->_irq_callback(self->_irq_user_data);
        }
    }
}

void ENC28J60::_processInterrupt(uint8_t flags) {
    _link_up = linkStatus(); // Update cached link status
    
    if (flags & PKTIF) {
        // Packet received - user can call receivePacket()
    }
    
    if (flags & TXIF) {
        // Transmission complete
        if (_irq_callback) {
            _irq_callback(_irq_user_data);
        }
    }
    
    if (flags & LINKIF) {
        _link_up = linkStatus();
    }
    
    if (flags & RXERIE) {
        // Receive error - drain buffer
        uint8_t count = enc28j60_rx_count_blk(_eth);
        for (int i = 0; i < count; ++i) {
            uint32_t rx_info = enc28j60_frame_rx_info_blk(_eth);
            enc28j60_frame_discard_blk(_eth, rx_info);
        }
    }
}

// ============================================================================
// DMA Notification Callbacks
// ============================================================================

void ENC28J60::_notify_func(void* data) {
    // Called from DMA ISR to wake up waiting task
    ENC28J60* self = static_cast<ENC28J60*>(data);
    // In Arduino context, we can't do much from ISR
    // The wait function will handle blocking
}

void ENC28J60::_wait_func(void* data) {
    // Block until DMA completes
    // In Arduino, this would typically be a busy wait or yield
    ENC28J60* self = static_cast<ENC28J60*>(data);
    while (self->_eth && self->_eth->cmd_buf) {
        // Simple spin wait - in practice you'd use proper synchronization
        // This is a placeholder for the actual DMA wait logic
        yield();
    }
}

// ============================================================================
// FreeRTOS Integration
// ============================================================================

#ifdef FREERTOS_H

bool ENC28J60::beginFreeRTOS(const char* interface_name,
                             uint32_t ip, uint32_t netmask, uint32_t gateway) {
    if (!_initialized) {
        return false;
    }
    
    // Initialize FreeRTOS interface structure
    memset(&_freertos_iface, 0, sizeof(_freertos_iface));
    _freertos_iface.iface.pcName = interface_name ? interface_name : "e0";
    _freertos_iface.iface.pvArgument = _cfg;
    _freertos_iface.iface.pfInitialise = enc28j60_freertos_initialize;
    _freertos_iface.iface.pfOutput = enc28j60_freertos_output_frame;
    _freertos_iface.iface.pfGetPhyLinkStatus = enc28j60_freertos_get_phy_link_status;
    
    // Configure IP endpoints if provided
    if (ip != 0 || netmask != 0 || gateway != 0) {
        // Static IP configuration would go here
        // For now, rely on DHCP
    }
    
    // Add interface to FreeRTOS+TCP
    if (FreeRTOS_AddNetworkInterface(&_freertos_iface.iface) != pdPASS) {
        return false;
    }
    
    // Start FreeRTOS IP stack
    FreeRTOS_IPInit_Multi();
    
    return true;
}

NetworkInterface_t* ENC28J60::getFreeRTOSInterface() const {
    return const_cast<NetworkInterface_t*>(&_freertos_iface.iface);
}

#endif // FREERTOS_H
