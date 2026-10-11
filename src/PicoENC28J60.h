#ifndef PICO_ENC28J60_H
#define PICO_ENC28J60_H

/**
 * @file PicoENC28J60.h
 * @brief Arduino library for ENC28J60 Ethernet controller on Raspberry Pi Pico
 * 
 * This library is based on the excellent work by Lukasz Janyst
 * https://github.com/ljanyst/pico-enc28j60
 * 
 * The original implementation uses a PIO program for SPI communication,
 * allowing flexible pin assignments. This Arduino port maintains all
 * the original functionality while providing an Arduino-friendly API.
 */

#include <Arduino.h>

// Forward declaration
class PicoENC28J60;

/**
 * @brief Configuration structure for ENC28J60
 * 
 * Allows custom pin assignments for flexible hardware connections.
 * Default pins match the original library defaults.
 */
struct ENC28J60Config {
    // SPI pins (using PIO-based SPI internally)
    int8_t mosi_pin = 3;   // GP3 - SI (MOSI)
    int8_t miso_pin = 4;   // GP4 - SO (MISO)
    int8_t sclk_pin = 2;   // GP2 - SCK
    int8_t cs_pin = 5;     // GP5 - CS (separate from SCK)
    int8_t rst_pin = 7;    // GP7 - RST
    int8_t irq_pin = -1;   // Optional interrupt pin (-1 = polling mode)
    
    // MAC address (must be locally administered)
    uint8_t mac_addr[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x01};
    
    // Buffer configuration
    uint16_t rx_buffer_size = 0x1000;  // 4KB RX buffer (half of 8KB total)
    
    // PIO configuration
    uint8_t pio_instance = 0;  // 0 = pio0, 1 = pio1
    uint8_t pio_sm = 0;        // State machine index (0-3)
    
    // Default constructor with standard pinout
    ENC28J60Config() {}
};

/**
 * @brief Main ENC28J60 driver class
 * 
 * Provides Arduino-friendly interface to the ENC28J60 Ethernet controller.
 * Uses PIO-based SPI for communication, supporting custom pin assignments.
 */
class PicoENC28J60 {
public:
    /**
     * @brief Constructor
     * @param config Configuration structure (optional, uses defaults if not provided)
     */
    PicoENC28J60(const ENC28J60Config& config = ENC28J60Config());
    
    /**
     * @brief Destructor
     */
    ~PicoENC28J60();
    
    /**
     * @brief Initialize the ENC28J60
     * @return true if initialization successful, false otherwise
     */
    bool begin();
    
    /**
     * @brief Check if link is up
     * @return true if link is up, false otherwise
     */
    bool linkStatus();
    
    /**
     * @brief Get hardware revision ID
     * @return Revision ID (0x06 for ENC28J60)
     */
    uint8_t getRevision();
    
    /**
     * @brief Get MAC address
     * @param mac Pointer to 6-byte array to store MAC address
     */
    void getMACAddress(uint8_t* mac);
    
    /**
     * @brief Set MAC address (requires re-initialization to take effect in hardware)
     * @param mac Pointer to 6-byte array with new MAC address
     */
    void setMACAddress(const uint8_t* mac);
    
    /**
     * @brief Send an Ethernet frame
     * @param frame Pointer to frame data
     * @param len Frame length in bytes
     * @return true if frame queued for transmission, false on error
     */
    bool sendFrame(const uint8_t* frame, uint16_t len);
    
    /**
     * @brief Check for received frames
     * @return Number of frames in RX buffer
     */
    uint8_t available();
    
    /**
     * @brief Receive an Ethernet frame
     * @param buffer Buffer to store received frame
     * @param max_len Maximum buffer size
     * @return Number of bytes received, 0 if no frame available
     */
    uint16_t receiveFrame(uint8_t* buffer, uint16_t max_len);
    
    /**
     * @brief Discard the next received frame
     */
    void discardFrame();
    
    /**
     * @brief Get interrupt flags
     * @return Interrupt flags byte
     */
    uint8_t getInterruptFlags();
    
    /**
     * @brief Check if a packet was received
     * @return true if packet received interrupt is set
     */
    bool hasPacket();
    
    /**
     * @brief Check if transmission completed
     * @return true if TX interrupt is set
     */
    bool hasTXComplete();
    
    /**
     * @brief Check if link status changed
     * @return true if LINK interrupt is set
     */
    bool hasLinkChange();
    
    /**
     * @brief Check if receive error occurred
     * @return true if RX error interrupt is set
     */
    bool hasRXError();
    
    /**
     * @brief Acknowledge and clear interrupt flags
     * @param flags Flags to acknowledge
     */
    void acknowledgeInterrupts(uint8_t flags);
    
    /**
     * @brief Enable/disable interrupt callback
     * @param callback Function to call on interrupt (NULL to disable)
     * @param user_data User data passed to callback
     */
    void setInterruptCallback(void (*callback)(void*), void* user_data = nullptr);
    
    /**
     * @brief Poll for interrupts (call periodically in polling mode)
     * 
     * Must be called regularly when not using interrupt callback.
     * Checks for received packets and calls callback if set.
     */
    void poll();
    
    /**
     * @brief Check if driver is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return _initialized; }

    // Interrupt flag constants
    static constexpr uint8_t INT_LINK_CHANGE = 0x10;
    static constexpr uint8_t INT_TX_COMPLETE = 0x08;
    static constexpr uint8_t INT_PACKET_RX = 0x40;
    static constexpr uint8_t INT_RX_ERROR = 0x01;
    static constexpr uint8_t INT_GLOBAL = 0x80;

private:
    ENC28J60Config _config;
    void* _eth;           // Internal driver handle
    bool _initialized;
    void (*_irq_callback)(void*);
    void* _irq_user_data;
    
    // Internal helper functions
    void _hardwareReset();
    static void _staticIRQHandler(void* data);
    
    // Disable copy constructor and assignment
    PicoENC28J60(const PicoENC28J60&) = delete;
    PicoENC28J60& operator=(const PicoENC28J60&) = delete;
};

#endif // PICO_ENC28J60_H
