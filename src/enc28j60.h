#ifndef ENC28J60_H
#define ENC28J60_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// Include the C driver headers
extern "C" {
#include "include/pico/enc28j60-driver.h"
}

#ifdef FREERTOS_H
extern "C" {
#include "include/pico/enc28j60-freertos-driver.h"
}
#endif

/**
 * @brief Arduino-friendly wrapper for ENC28J60 Ethernet controller
 * 
 * This class provides a high-level C++ interface over the low-level C driver.
 * It handles initialization, configuration, and packet transmission/reception
 * using Arduino-style APIs.
 */
class ENC28J60 {
public:
    /**
     * @brief Default SPI pins for Raspberry Pi Pico
     */
    static constexpr int DEFAULT_MISO_PIN = 16;
    static constexpr int DEFAULT_MOSI_PIN = 17;
    static constexpr int DEFAULT_SCLK_PIN = 18;
    static constexpr int DEFAULT_CS_PIN = 19;     // SCLK + 1
    static constexpr int DEFAULT_IRQ_PIN = 20;
    
    /**
     * @brief Default RX buffer size (half of 8KB device RAM)
     */
    static constexpr uint16_t DEFAULT_RX_SIZE = 0x1000;
    
    /**
     * @brief Maximum Ethernet frame size
     */
    static constexpr uint16_t MAX_FRAME_SIZE = 1518;

    /**
     * @brief Constructor
     */
    ENC28J60();
    
    /**
     * @brief Destructor - cleans up resources
     */
    ~ENC28J60();
    
    // Delete copy constructor and assignment (not copyable)
    ENC28J60(const ENC28J60&) = delete;
    ENC28J60& operator=(const ENC28J60&) = delete;
    
    // Allow move semantics
    ENC28J60(ENC28J60&& other) noexcept;
    ENC28J60& operator=(ENC28J60&& other) noexcept;

    /**
     * @brief Initialize the ENC28J60 with default pins
     * @param mac 6-byte MAC address array
     * @return true if initialization successful
     */
    bool begin(const uint8_t* mac);
    
    /**
     * @brief Initialize with custom pins
     * @param mac 6-byte MAC address array
     * @param miso_pin MISO GPIO pin
     * @param mosi_pin MOSI GPIO pin
     * @param sclk_pin SCLK GPIO pin (CS will be sclk_pin + 1)
     * @param irq_pin Interrupt GPIO pin
     * @param pio_instance PIO instance (pio0 or pio1)
     * @param sm_index State machine index (0-3)
     * @return true if initialization successful
     */
    bool begin(const uint8_t* mac, 
               int miso_pin, int mosi_pin, int sclk_pin, int irq_pin,
               PIO pio_instance = pio0, uint sm_index = 0);
    
    /**
     * @brief Initialize with full configuration
     * @param config Pointer to configuration structure
     * @return true if initialization successful
     */
    bool begin(enc28j60_config* config);
    
    /**
     * @brief Deinitialize and free resources
     */
    void end();
    
    /**
     * @brief Check if the device is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return _initialized; }
    
    /**
     * @brief Get the hardware revision ID
     * @return Revision ID (0-7), or 0xFF on error
     */
    uint8_t getRevision() const;
    
    /**
     * @brief Check link status (blocking)
     * @return true if link is up
     */
    bool linkStatus() const;
    
    /**
     * @brief Check link status (non-blocking, uses cached value)
     * @return true if link was up at last check
     */
    bool linkStatusCached() const { return _link_up; }
    
    /**
     * @brief Send a packet (blocking)
     * @param data Pointer to packet data
     * @param size Packet size in bytes
     * @return true if packet queued for transmission
     */
    bool sendPacket(const void* data, size_t size);
    
    /**
     * @brief Send a packet (non-blocking with callback)
     * @param data Pointer to packet data
     * @param size Packet size in bytes
     * @param callback Function called when transmission completes
     * @param user_data User data passed to callback
     * @return true if packet queued for transmission
     */
    bool sendPacket(const void* data, size_t size, 
                    void (*callback)(void*), void* user_data);
    
    /**
     * @brief Check if a packet is available for reading
     * @return Number of packets in RX buffer
     */
    uint8_t packetAvailable() const;
    
    /**
     * @brief Receive a packet (blocking)
     * @param buffer Buffer to store received data
     * @param max_size Maximum size of buffer
     * @return Number of bytes received, 0 if no packet or error
     */
    uint16_t receivePacket(void* buffer, uint16_t max_size);
    
    /**
     * @brief Receive a packet (non-blocking)
     * @param buffer Buffer to store received data
     * @param max_size Maximum size of buffer
     * @return Number of bytes received, 0 if no packet available
     */
    uint16_t receivePacketNonBlocking(void* buffer, uint16_t max_size);
    
    /**
     * @brief Discard the next packet in RX buffer
     * @return true if packet was discarded
     */
    bool discardPacket();
    
    /**
     * @brief Get the MAC address
     * @param mac Output buffer (6 bytes)
     */
    void getMacAddress(uint8_t* mac) const;
    
    /**
     * @brief Set the MAC address (requires re-initialization)
     * @param mac 6-byte MAC address array
     */
    void setMacAddress(const uint8_t* mac);
    
    /**
     * @brief Get the RX buffer size
     * @return RX buffer size in bytes
     */
    uint16_t getRxBufferSize() const { return _rx_size; }
    
    /**
     * @brief Set the RX buffer size (must be called before begin())
     * @param size RX buffer size in bytes (0-0x2000)
     */
    void setRxBufferSize(uint16_t size) { _rx_size = size; }
    
    /**
     * @brief Enable/disable interrupt-driven operation
     * @param enable true to enable interrupts
     * @param callback Interrupt callback function
     * @param user_data User data for callback
     */
    void enableInterrupts(bool enable, 
                          void (*callback)(void*) = nullptr, 
                          void* user_data = nullptr);
    
    /**
     * @brief Handle interrupt (call from ISR or loop)
     * Must be called regularly when interrupts are enabled
     */
    void handleInterrupt();
    
    /**
     * @brief Get the underlying C driver instance (advanced use)
     * @return Pointer to enc28j60 struct
     */
    enc28j60* getDriver() const { return _eth; }
    
    /**
     * @brief Get the configuration structure (advanced use)
     * @return Pointer to enc28j60_config struct
     */
    enc28j60_config* getConfig() const { return _cfg; }

    // FreeRTOS integration
    #ifdef FREERTOS_H
    /**
     * @brief Initialize FreeRTOS+TCP network interface
     * @param interface_name Interface name (e.g., "e0")
     * @param ip Static IP (optional, use DHCP if 0)
     * @param netmask Netmask (optional)
     * @param gateway Gateway (optional)
     * @return true if successful
     */
    bool beginFreeRTOS(const char* interface_name,
                       uint32_t ip = 0, uint32_t netmask = 0, uint32_t gateway = 0);
    
    /**
     * @brief Get FreeRTOS network interface
     * @return Pointer to NetworkInterface_t
     */
    NetworkInterface_t* getFreeRTOSInterface() const;
    #endif

private:
    // Internal state
    enc28j60_config* _cfg = nullptr;
    enc28j60* _eth = nullptr;
    bool _initialized = false;
    bool _link_up = false;
    uint16_t _rx_size = DEFAULT_RX_SIZE;
    uint8_t _mac_addr[6] = {0xA2, 0x32, 0x15, 0xCC, 0x72, 0x0A};
    
    // Interrupt handling
    bool _irq_enabled = false;
    void (*_irq_callback)(void*) = nullptr;
    void* _irq_user_data = nullptr;
    volatile uint8_t _irq_flags = 0;
    
    // DMA notification for non-blocking mode
    static void _notify_func(void* data);
    static void _wait_func(void* data);
    enc28j60_notification _notification;
    
    // FreeRTOS
    #ifdef FREERTOS_H
    enc28j60_freertos_interface _freertos_iface;
    #endif
    
    // Helper methods
    bool _initDriver();
    void _cleanup();
    static void _irq_handler(void* data);
    void _processInterrupt(uint8_t flags);
};

#endif // ENC28J60_H
