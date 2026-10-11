#ifndef PICO_ENC28J60_H
#define PICO_ENC28J60_H

/**
 * @file PicoENC28J60.h
 * @brief Arduino library for the ENC28J60 Ethernet controller on the Raspberry Pi Pico
 *
 * Based on pico-enc28j60 by Lukasz Janyst (https://github.com/ljanyst/pico-enc28j60).
 * The SPI link to the chip is a PIO program, so any free GPIOs can be used.
 *
 * This class is a raw Ethernet frame driver: it sends and receives whole
 * frames and leaves the protocols to you. For TCP/IP (DHCP, sockets, a web
 * server) use PicoENC28J60lwIP.h, which plugs this driver into the lwIP stack of
 * the arduino-pico core.
 */

#include <Arduino.h>

/**
 * @brief Pin, MAC address and buffer configuration
 *
 * The default pins are this library's own (they are not the GP16-GP20 defaults
 * of the original C driver): SCK=GP2, MOSI=GP3, MISO=GP4, CS=GP5, RST=GP7.
 */
struct ENC28J60Config {
    // SPI pins (PIO-based SPI)
    int8_t mosi_pin = 3;  // GP3 - SI (MOSI)
    int8_t miso_pin = 4;  // GP4 - SO (MISO)
    int8_t sclk_pin = 2;  // GP2 - SCK
    int8_t cs_pin = 5;    // GP5 - CS (separate from SCK)
    int8_t rst_pin = 7;   // GP7 - RST
    int8_t irq_pin = -1;  // Optional interrupt pin (-1 = polling mode)

    // MAC address: locally administered unicast. Give every device on the
    // network its own.
    uint8_t mac_addr[6] = {0x02, 0x42, 0xAC, 0x11, 0x00, 0x01};

    // Bytes of the chip's 8 KB RAM used for reception; the rest is the transmit
    // area. Rounded down to an even number and kept between 0x0600 and 0x1A00
    // so that both areas hold a full frame.
    uint16_t rx_buffer_size = 0x1000;

    // PIO configuration
    uint8_t pio_instance = 0;  // 0 = pio0, 1 = pio1
    uint8_t pio_sm = 0;        // State machine index (0-3)

    ENC28J60Config() {}
};

/**
 * @brief ENC28J60 driver
 *
 * Frames are handed over without the 4-byte Ethernet FCS in both directions:
 * the chip appends it when sending and checks and strips it when receiving.
 */
class PicoENC28J60 {
public:
    //! Largest frame sendFrame() accepts, without the FCS (14 byte header + 1500 bytes)
    static constexpr uint16_t MAX_FRAME_SIZE = 1514;

    /**
     * @brief Constructor
     * @param config Configuration (the defaults match the pins in ENC28J60Config)
     */
    PicoENC28J60(const ENC28J60Config& config = ENC28J60Config());

    ~PicoENC28J60();

    /**
     * @brief Initialize the ENC28J60 with the configuration given to the constructor
     * @return true on success; false if no ENC28J60 answers (check the wiring and
     *         the 3.3 V supply). It is safe to call begin() again after a failure.
     */
    bool begin();

    /**
     * @brief Initialize the ENC28J60 with a new configuration
     *
     * Convenient for global objects: construct with the defaults and configure
     * in setup(). Ignored if the driver is already initialized.
     */
    bool begin(const ENC28J60Config& config);

    /**
     * @brief Check if the Ethernet link is up
     */
    bool linkStatus();

    /**
     * @brief Hardware revision ID of the chip: 0x02 (B1), 0x04 (B4), 0x05 (B5) or 0x06 (B7)
     * @return The revision, or 0xFF if the driver is not initialized
     */
    uint8_t getRevision();

    /**
     * @brief Get the MAC address
     * @param mac Pointer to a 6-byte array
     */
    void getMACAddress(uint8_t* mac) const;

    /**
     * @brief Set the MAC address
     *
     * Takes effect in the hardware immediately when the driver is initialized.
     * Call it before traffic starts: frames arriving while the address changes
     * may be filtered wrongly.
     */
    void setMACAddress(const uint8_t* mac);

    /**
     * @brief Send an Ethernet frame (blocking)
     *
     * Returns once the frame has left the wire, which takes up to 1.2 ms for a
     * full-size frame. Short frames are padded to the 60 byte minimum and the
     * FCS is appended by the hardware.
     *
     * @param frame Destination MAC, source MAC, EtherType and payload
     * @param len Length in bytes, at most MAX_FRAME_SIZE
     * @return true if the frame was sent; false if it is invalid, the transmit
     *         buffer is full or the medium stayed busy (about 20 ms)
     */
    bool sendFrame(const uint8_t* frame, uint16_t len);

    /**
     * @brief Number of frames waiting in the receive buffer
     */
    uint8_t available();

    /**
     * @brief Receive the next frame
     *
     * The frame is copied without its FCS. A frame that does not fit in the
     * buffer is dropped.
     *
     * @param buffer Where to store the frame; MAX_FRAME_SIZE bytes always suffice
     * @param max_len Size of the buffer
     * @return Length of the frame in bytes, 0 if there is none
     */
    uint16_t receiveFrame(uint8_t* buffer, uint16_t max_len);

    /**
     * @brief Drop the next received frame
     */
    void discardFrame();

    /**
     * @brief Read the interrupt flags (EIR)
     *
     * Meant for an interrupt callback, which must follow up with
     * acknowledgeInterrupts(). In polling mode use hasPacket() and friends.
     */
    uint8_t getInterruptFlags();

    //! A frame is waiting in the receive buffer
    bool hasPacket();

    //! A transmission completed (not acknowledged automatically by sendFrame())
    bool hasTXComplete();

    //! The link went up or down since the last acknowledgeInterrupts(INT_LINK_CHANGE)
    bool hasLinkChange();

    //! The receive buffer overflowed or a frame was dropped
    bool hasRXError();

    /**
     * @brief Acknowledge and clear interrupt flags
     * @param flags INT_* flags to clear
     */
    void acknowledgeInterrupts(uint8_t flags);

    /**
     * @brief Register a function to call on a chip interrupt
     *
     * Must be called before begin(), and needs irq_pin to be set when using the
     * hardware interrupt. The callback runs in interrupt context there; it should
     * read the flags with getInterruptFlags() and clear them with
     * acknowledgeInterrupts().
     *
     * @param callback Function to call (nullptr to disable)
     * @param user_data Passed to the callback
     */
    void setInterruptCallback(void (*callback)(void*), void* user_data = nullptr);

    /**
     * @brief Poll for interrupt conditions
     *
     * Only useful together with setInterruptCallback() when the INT pin is not
     * wired: it calls the callback, exactly as the interrupt would, when any flag is
     * set. Without a callback it does nothing; use hasPacket() and friends.
     */
    void poll();

    //! The driver has been initialized successfully
    bool isInitialized() const { return _initialized; }

    //! Counters for diagnostics; they only ever grow
    struct Stats {
        uint32_t rx_frames;    //!< frames returned by receiveFrame()
        uint32_t rx_dropped;   //!< frames dropped because they did not fit the caller's buffer
        uint32_t rx_resets;    //!< times the receiver was reset because of a bad frame header
        uint32_t tx_frames;    //!< frames sent
        uint32_t tx_failures;  //!< sendFrame() calls that failed
    };
    const Stats& stats() const { return _stats; }

    /**
     * @brief The handle of the underlying C driver (struct enc28j60 *)
     *
     * For code that wants to use the C API of pico/enc28j60-driver.h directly.
     * nullptr until begin() succeeds.
     */
    void* getDriverHandle() const { return _eth; }

    // Interrupt flag constants (EIR bits)
    static constexpr uint8_t INT_LINK_CHANGE = 0x10;
    static constexpr uint8_t INT_TX_COMPLETE = 0x08;
    static constexpr uint8_t INT_PACKET_RX = 0x40;
    static constexpr uint8_t INT_RX_ERROR = 0x01;
    static constexpr uint8_t INT_GLOBAL = 0x80;

private:
    ENC28J60Config _config;
    void* _eth;  // internal driver handle
    bool _initialized;
    void (*_irq_callback)(void*);
    void* _irq_user_data;
    Stats _stats = {};

    void _hardwareReset();
    bool _readRxInfo(uint32_t* rx_info);
    static void _staticIRQHandler(void* data);

    // Not copyable
    PicoENC28J60(const PicoENC28J60&) = delete;
    PicoENC28J60& operator=(const PicoENC28J60&) = delete;
};

#endif  // PICO_ENC28J60_H
