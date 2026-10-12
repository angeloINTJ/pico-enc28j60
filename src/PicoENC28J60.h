// Copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#ifndef PICO_ENC28J60_H
#define PICO_ENC28J60_H

/**
 * @file PicoENC28J60.h
 * @brief Arduino library for the ENC28J60 Ethernet controller on the Raspberry
 * Pi Pico
 *
 * Based on pico-enc28j60 by Lukasz Janyst
 * (https://github.com/ljanyst/pico-enc28j60). The SPI link to the chip is a PIO
 * program, so any free GPIOs can be used.
 *
 * This class is a raw Ethernet frame driver: it sends and receives whole
 * frames and leaves the protocols to you. For TCP/IP (DHCP, sockets, a web
 * server) use PicoENC28J60lwIP.h, which plugs this driver into the lwIP stack
 * of the arduino-pico core.
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
    int8_t mosi_pin = 3; // GP3 - SI (MOSI)
    int8_t miso_pin = 4; // GP4 - SO (MISO)
    int8_t sclk_pin = 2; // GP2 - SCK
    int8_t cs_pin = 5; // GP5 - CS (separate from SCK)
    int8_t rst_pin = 7; // GP7 - RST
    int8_t irq_pin = -1; // GPIO wired to the chip's INT pin (-1 = not wired,
                         // the sketch polls)

    // MAC address: locally administered unicast. Give every device on the
    // network its own.
    uint8_t mac_addr[6] = { 0x02, 0x42, 0xAC, 0x11, 0x00, 0x01 };

    // Bytes of the chip's 8 KB RAM used for reception; the rest is the transmit
    // area. Rounded down to an even number and kept between 0x0600 and 0x1A00
    // so that both areas hold a full frame.
    uint16_t rx_buffer_size = 0x1000;

    // PIO configuration. The state machine is claimed, so the Arduino core and
    // other libraries that use the PIO keep away from it. Leave both at -1 and
    // the driver takes any state machine that is free.
    int8_t pio_instance = -1; // 0 = pio0, 1 = pio1, ... -1 = any
    int8_t pio_sm = -1; // State machine index (0-3), -1 = any

    ENC28J60Config()
    {
    }
};

/**
 * @brief ENC28J60 driver
 *
 * Frames are handed over without the 4-byte Ethernet FCS in both directions:
 * the chip appends it when sending and checks and strips it when receiving.
 */
class PicoENC28J60 {
  public:
    //! Largest frame sendFrame() accepts, without the FCS (14 byte header +
    //! 1500 bytes)
    static constexpr uint16_t MAX_FRAME_SIZE = 1514;

    /**
     * @brief Constructor
     * @param config Configuration (the defaults match the pins in
     * ENC28J60Config)
     */
    PicoENC28J60(const ENC28J60Config &config = ENC28J60Config());

    ~PicoENC28J60();

    //! Why begin() failed
    enum class Error : uint8_t {
        None = 0, //!< begin() worked
        Config, //!< a pin is out of range or used twice, or the PIO choice is
                //!< not valid
        PioBusy, //!< the state machine chosen in the configuration is in use
        PioSpace, //!< no PIO has the room for the SPI program and a free state
                  //!< machine
        NoChip, //!< no ENC28J60 answers: check the wiring and the 3.3 V supply
        NoMemory, //!< out of memory
        Io //!< the PIO state machine did not run
    };

    /**
     * @brief Initialize the ENC28J60 with the configuration given to the
     * constructor
     * @return true on success; false if it could not be done, and lastError()
     *         says why: the configuration, the PIO, or no chip that answers
     *         (check the wiring and the 3.3 V supply). It is safe to call
     *         begin() again after a failure. The Ethernet link comes up a
     *         second or two later, see linkStatus().
     */
    bool begin();

    //! Why the last begin() failed; Error::None after one that worked
    Error lastError() const
    {
        return _error;
    }

    //! A sentence that tells what an Error means
    static const char *errorString(Error error);

    /**
     * @brief Stop the chip and give back the PIO state machine and the pins
     *
     * The destructor does it too. Afterwards begin() starts it again, with a
     * new configuration if you pass one.
     */
    void end();

    /**
     * @brief Initialize the ENC28J60 with a new configuration
     *
     * Convenient for global objects: construct with the defaults and configure
     * in setup(). Ignored if the driver is already initialized; call end()
     * first to change the configuration.
     */
    bool begin(const ENC28J60Config &config);

    /**
     * @brief Check if the Ethernet link is up
     */
    bool linkStatus();

    /**
     * @brief Hardware revision ID of the chip: 0x02 (B1), 0x04 (B4), 0x05 (B5)
     * or 0x06 (B7)
     * @return The revision, or 0xFF if the driver is not initialized
     */
    uint8_t getRevision();

    /**
     * @brief Get the MAC address
     * @param mac Pointer to a 6-byte array
     */
    void getMACAddress(uint8_t *mac) const;

    /**
     * @brief Set the MAC address
     *
     * Takes effect in the hardware immediately when the driver is initialized.
     * Call it before traffic starts: frames arriving while the address changes
     * may be filtered wrongly.
     */
    void setMACAddress(const uint8_t *mac);

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
    bool sendFrame(const uint8_t *frame, uint16_t len);

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
     * @param buffer Where to store the frame; MAX_FRAME_SIZE bytes always
     * suffice
     * @param max_len Size of the buffer
     * @return Length of the frame in bytes, 0 if there is none
     */
    uint16_t receiveFrame(uint8_t *buffer, uint16_t max_len);

    /**
     * @brief Drop the next received frame
     */
    void discardFrame();

    /**
     * @brief Has the chip asked for attention since the last call?
     *
     * Interrupt-driven operation in three steps. Put the INT pin in
     * ENC28J60Config::irq_pin, then in loop():
     *
     *     if (eth.interruptPending()) {
     *         uint8_t flags = eth.getInterruptFlags();
     *         while (eth.available()) {
     *             len = eth.receiveFrame(buf, sizeof(buf));
     *             ...
     *         }
     *         if (flags & PicoENC28J60::INT_LINK_CHANGE) {
     *             ...  // eth.linkStatus() says up or down
     *         }
     *         eth.acknowledgeInterrupts(flags);
     *     }
     *
     * With the pin wired this costs no SPI traffic: it looks at the INT line,
     * so a loop that finds nothing to do leaves the chip alone. It stays true
     * while the line is low, so an event cannot be missed even if its edge was.
     * Without irq_pin it looks at the chip's flags instead (one SPI read), so
     * the same loop works on a board that has no INT wire.
     *
     * The chip raises the line for a received frame, a link change and a
     * receive overflow. It does not for a finished transmission, because
     * sendFrame() waits for that itself.
     */
    bool interruptPending();

    /**
     * @brief Read the interrupt flags (EIR) and mask the INT pin
     *
     * The sequence the datasheet prescribes, which cannot lose an event that
     * arrives while you work: read the flags here, handle them, then call
     * acknowledgeInterrupts() with the same flags. Until then the pin stays
     * quiet.
     */
    uint8_t getInterruptFlags();

    //! Read the interrupt flags without masking the INT pin, for code that only
    //! looks
    uint8_t peekInterruptFlags();

    //! A frame is waiting in the receive buffer (only looks: the INT pin is not
    //! masked). It asks the packet counter of the chip, because the flag of
    //! the chip is not reliable (silicon errata, issue 6): INT_PACKET_RX in
    //! the flags of getInterruptFlags() and peekInterruptFlags() is the same
    //! answer.
    bool hasPacket();

    //! A transmission completed (only looks; sendFrame() clears the flag when
    //! it starts)
    bool hasTXComplete();

    //! The link went up or down since the last
    //! acknowledgeInterrupts(INT_LINK_CHANGE) (only looks)
    bool hasLinkChange();

    //! The receive buffer overflowed or a frame was dropped (only looks)
    bool hasRXError();

    /**
     * @brief Clear interrupt flags and unmask the INT pin
     * @param flags INT_* flags to clear, normally what getInterruptFlags()
     * returned. A flag that is left set raises the line again at once.
     */
    void acknowledgeInterrupts(uint8_t flags);

    /**
     * @brief Register a function to call when the INT line falls
     *
     * Call it before begin(); it needs irq_pin. The function runs in interrupt
     * context, so keep it to a few instructions and do not call into this class
     * from it: the chip is reached over a single PIO state machine, and an
     * interrupt in the middle of a transfer of the main program would corrupt
     * both. Set a flag, or wake something up, and do the talking to the chip in
     * loop() (interruptPending() is made for that).
     *
     * Without irq_pin, poll() calls it from loop() instead.
     *
     * @param callback Function to call (nullptr to disable)
     * @param user_data Passed to the callback
     */
    void setInterruptCallback(void (*callback)(void *),
                              void *user_data = nullptr);

    /**
     * @brief Run the interrupt callback from loop() when there is no INT wire
     *
     * Calls the function given to setInterruptCallback() when
     * interruptPending() says so, which costs one SPI read. It does nothing
     * when irq_pin is set, because then the pin calls the function itself.
     */
    void poll();

    //! Number of falling edges seen on the INT pin (0 without irq_pin)
    uint32_t interruptCount() const
    {
        return _irq_count;
    }

    //! The driver has been initialized successfully
    bool isInitialized() const
    {
        return _initialized;
    }

    //! Counters for diagnostics; they only ever grow
    struct Stats {
        uint32_t rx_frames; //!< frames returned by receiveFrame()
        uint32_t rx_dropped; //!< frames dropped because they did not fit the
                             //!< caller's buffer
        uint32_t rx_resets; //!< times the receiver was reset because of a bad
                            //!< frame header
        uint32_t tx_frames; //!< frames sent
        uint32_t tx_failures; //!< sendFrame() calls that failed
        uint32_t tx_aborts; //!< frames the MAC gave up on, after the retries
        uint32_t tx_retries; //!< frames sent again after a late collision
        uint32_t io_errors; //!< times the PIO state machine did not run
        uint32_t recoveries; //!< times the chip was started over after one
    };
    const Stats &stats() const
    {
        return _stats;
    }

    /**
     * @brief The handle of the underlying C driver (struct enc28j60 *)
     *
     * For code that wants to use the C API of pico/enc28j60-driver.h directly.
     * nullptr until begin() succeeds.
     */
    void *getDriverHandle() const
    {
        return _eth;
    }

    // Interrupt flag constants (EIR bits)
    static constexpr uint8_t INT_LINK_CHANGE = 0x10;
    static constexpr uint8_t INT_TX_COMPLETE = 0x08;
    static constexpr uint8_t INT_PACKET_RX = 0x40;
    static constexpr uint8_t INT_RX_ERROR = 0x01;
    static constexpr uint8_t INT_GLOBAL = 0x80;

  private:
    ENC28J60Config _config;
    void *_eth; // internal driver handle
    void *_cfg; // configuration of the driver, owned by this object
    bool _initialized;
    Error _error = Error::None;
    bool _ioCounted = false; // the failure that is being dealt with is counted
    bool _recoverWait = false; // a recovery failed: wait until _recoverAt
    uint32_t _recoverAt = 0; // millis(): do not try to recover before this
    bool _irq_attached = false;
    volatile bool _irq_pending = false;
    volatile uint32_t _irq_count = 0;
    void (*_irq_callback)(void *);
    void *_irq_user_data;
    Stats _stats = {};

    void _hardwareReset();
    void _enableChipInterrupts();
    bool _usable();
    bool _failed() const;
    bool _readRxInfo(uint32_t *rx_info);
    static void _staticIRQHandler(void *data);

    // Not copyable
    PicoENC28J60(const PicoENC28J60 &) = delete;
    PicoENC28J60 &operator=(const PicoENC28J60 &) = delete;
};

#endif // PICO_ENC28J60_H
