// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Modifications copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#pragma once

#include <hardware/pio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//! Why the initialization of the driver failed, or what is wrong with it
typedef enum {
    ENC28J60_OK = 0,
    ENC28J60_ERR_CONFIG, //!< the configuration is not valid (pins, buffer)
    ENC28J60_ERR_PIO_BUSY, //!< the state machine asked for is already in use
    ENC28J60_ERR_PIO_SPACE, //!< no PIO has the room and a free state machine
    ENC28J60_ERR_NO_CHIP, //!< no ENC28J60 answers on the SPI pins
    ENC28J60_ERR_NOMEM, //!< out of memory
    ENC28J60_ERR_IO, //!< an SPI transfer stalled: the PIO does not run
    ENC28J60_ERR_DMA //!< no free DMA channel for the asynchronous functions
} enc28j60_status;

//! The outcome of the transmission of a frame
typedef enum {
    ENC28J60_TX_OK, //!< the frame left the wire
    ENC28J60_TX_ABORTED, //!< the MAC gave up (collisions, deferral, underrun)
    ENC28J60_TX_TIMEOUT, //!< still pending after the timeout; the transmitter
                         //!< was reset
    ENC28J60_TX_IO //!< an SPI transfer stalled
} enc28j60_tx_status;

typedef void (*enc28j60_irq_hanler)(void *);

typedef struct {
    void (*notify)(void *);
    void (*wait)(void *);
    void *data;
} enc28j60_notification;

//! Let the driver pick any free state machine of the PIO in enc28j60_config.sm
#define ENC28J60_SM_ANY ((uint) - 1)

//! The SPI clock when enc28j60_config.spi_hz is 0. The errata (issue 1) ask for
//! 8 MHz or more with the silicon revisions B1 and B4. The PIO divides the
//! system clock by a fraction, and a period of the SPI clock is a system clock
//! cycle longer or shorter than the nominal one from time to time (8 ns of 100
//! at 125 MHz); with 10 MHz every period is still shorter than the 125 ns of 8
//! MHz, with 8 MHz most of them would not be
#define ENC28J60_SPI_HZ_DEFAULT 10000000u

//! The fastest SPI clock the chip allows (data sheet, table 16-6)
#define ENC28J60_SPI_HZ_MAX 20000000u

//! The slowest one that the driver accepts
#define ENC28J60_SPI_HZ_MIN 100000u

typedef struct {
    //! The PIO block that runs the SPI program, or NULL to let the driver look
    //! for any block that has the room and a free state machine (sm is then
    //! ignored and set to the state machine that was chosen)
    PIO pio;
    //! The state machine of that block, or ENC28J60_SM_ANY. It is claimed, so
    //! the other users of the PIO, like the Arduino core, keep away from it.
    uint sm;
    uint miso_pin;
    uint mosi_pin;
    uint sclk_pin;
    uint cs_pin; //!< Slave select pin (separate from sclk_pin)
    //! The SPI clock in Hz, from ENC28J60_SPI_HZ_MIN to ENC28J60_SPI_HZ_MAX;
    //! 0 stands for ENC28J60_SPI_HZ_DEFAULT. The PIO runs at four times this,
    //! so the system clock has to allow it. The silicon revisions B1 and B4
    //! read and write the MAC registers unreliably below 8 MHz (errata, issue
    //! 1), see ENC28J60_SPI_HZ_DEFAULT. The clock is never faster than this,
    //! and is lowered when the fractional divider of the PIO would make a
    //! period shorter than 50 ns (20 MHz becomes 18.75 MHz with a system clock
    //! of 150 MHz, the default of the Pico 2).
    uint32_t spi_hz;
    uint irq_pin;
    //! The DMA interrupt of the asynchronous functions: 0 for DMA_IRQ_0, 1 for
    //! DMA_IRQ_1
    uint dma_irq;

    //! Size of the  recevie buffer in bytes; the device has 8kB RAM for
    //! buffers; what is not a receive buffer will be used as a transmit
    //! buffer; the relative sizes depend on the anticipated application
    //! traffic.
    uint16_t rx_size;

    uint8_t mac_addr[6];
    void *irq_data;
    enc28j60_irq_hanler irq_cb;
    enc28j60_notification notify;
} enc28j60_config;

struct enc28j60_cmd_buf;
typedef struct enc28j60_cmd_buf enc28j60_cmd_buf;

typedef struct {
    enc28j60_config *cfg;
    enc28j60_cmd_buf *cmd_buf;
    uint16_t dev_rptr;
    uint16_t wptr;
    bool io_error; //!< an SPI transfer stalled; see enc28j60_io_failed()
} enc28j60;

//! Get a default configuration object; it lets the driver choose the PIO and
//! the state machine
enc28j60_config *enc28j60_get_default_config();

//! Configure an ENC28J60; returns NULL if it cannot be done, and
//! enc28j60_init_status() says why: a configuration that makes no sense, a
//! state machine that is taken, no room in the PIO, or no ENC28J60 answering on
//! the SPI pins (check the wiring and the power supply). The configuration
//! stays owned by the caller in that case, as it does after a success: it must
//! outlive the driver.
enc28j60 *enc28j60_init(enc28j60_config *cfg);

//! Like enc28j60_init(), and tells why it failed
enc28j60 *enc28j60_init_status(enc28j60_config *cfg, enc28j60_status *status);

//! A short description of a status
const char *enc28j60_status_string(enc28j60_status status);

//! Stop the chip and give back what the driver took: the state machine, the
//! SPI program when it is the last user, the memory of the driver and, when
//! notify callbacks were given, the DMA channels and the DMA interrupt handler
//! of the asynchronous functions (call it on the core that called the init
//! then). The configuration stays with the caller.
void enc28j60_deinit(enc28j60 *eth);

//! An SPI transfer made no progress for a long time, which means that the PIO
//! state machine does not run any more (another piece of code took it, or it
//! was stopped). The driver stops using it, so that nothing hangs, and every
//! read returns zeros until enc28j60_recover() succeeds.
bool enc28j60_io_failed(const enc28j60 *eth);

//! Start the state machine again and configure the chip from scratch, which
//! empties both buffers. Returns false if the chip does not answer.
bool enc28j60_recover(enc28j60 *eth);

//! Is the chip still configured? A reset of the chip, say by a glitch of its
//! supply, puts the registers back to their defaults, which leaves the receiver
//! off, and nothing else turns it off once the driver has started it. One SPI
//! transaction.
bool enc28j60_alive_blk(enc28j60 *eth);

//! Get the revision id of the hardware
uint8_t enc28j60_revision_blk(enc28j60 *eth);

//! Get link status
bool enc28j60_link_status_blk(enc28j60 *eth);

//! Disable the interrupt and get the interrupt flags. PKTIF is set exactly when
//! the chip holds a packet: the flag of the chip is not reliable (silicon
//! errata, issue 6), the packet counter is.
uint8_t enc28j60_irq_flags_blk(enc28j60 *eth);

//! Acknowledge a phy interrupt
void enc28j60_irq_phy_ack_blk(enc28j60 *eth);

//! Acknowledge and enable the interrupt*
void enc28j60_irq_ack_blk(enc28j60 *eth, uint8_t flags);

//! Read the interrupt flags without touching the interrupt output; the
//! counterpart of enc28j60_irq_flags_blk() for code that only wants to look,
//! with the same correction of PKTIF
uint8_t enc28j60_irq_status_blk(enc28j60 *eth);

//! Program which events drive the INT pin, for code that routes the GPIO
//! interrupt itself instead of passing irq_cb to enc28j60_init(). `sources` is
//! a mix of the EIE bits PKTIE (0x40), LINKIE (0x10), TXIE (0x08), TXERIE
//! (0x02) and RXERIE (0x01); pending flags are cleared first, and 0 turns the
//! pin off. The function sets INTIE itself.
void enc28j60_irq_enable_blk(enc28j60 *eth, uint8_t sources);

//! Check if the interrupt is the link interrput
bool enc28j60_irq_is_link(uint8_t flags);

//! Has the transmission of the frame ended, sent (TXIF) or given up on
//! (TXERIF)? After a late collision the chip may raise TXERIF alone (silicon
//! errata, issue 13), so both flags count.
bool enc28j60_irq_is_tx(uint8_t flags);

//! Was the transmission of the frame aborted (TXERIF)? The MAC does not retry
//! a frame after a late collision; enc28j60_tx_status_blk() tells whether that
//! is what happened.
bool enc28j60_irq_is_tx_err(uint8_t flags);

//! Check if the interrupt is the receive interrupt
bool enc28j60_irq_is_rx(uint8_t flags);

//! Check if the interrupt is the receive error interrupt
bool enc28j60_irq_is_rx_err(uint8_t flags);

//! Disable the interrupt and get the interrupt flags; PKTIF comes from the
//! packet counter, as in enc28j60_irq_flags_blk()
uint8_t enc28j60_irq_flags(enc28j60 *eth);

//! Acknowledge and enable the interrupt
void enc28j60_irq_ack(enc28j60 *eth, uint8_t flags);

#define ENC28J60_INVALID_FRAME_ID 0xffffffff

//! Upload a frame
//! Returns a frame id needed for transmission
uint32_t enc28j60_frame_upload(enc28j60 *eth, size_t size, const void *data);
uint32_t enc28j60_frame_upload_blk(enc28j60 *eth, size_t size,
                                   const void *data);

//! Transmit a frame. The asynchronous version resets the transmit logic first
//! (silicon errata, issue 12), which also stops a transmission still going on,
//! so call it when the previous one has ended (enc28j60_irq_is_tx()). The
//! blocking version is used with enc28j60_frame_tx_wait_blk() or through
//! enc28j60_frame_send_blk().
void enc28j60_frame_tx(enc28j60 *eth, uint32_t frame_id);
void enc28j60_frame_tx_blk(enc28j60 *eth, uint32_t frame_id);

//! Wait for a frame started with enc28j60_frame_tx_blk() to leave the wire
//! and release its buffer space; this takes the place of
//! enc28j60_frame_tx_confirm() for the blocking functions. Returns false if the
//! frame was aborted by the MAC, or was still being transmitted after
//! timeout_us, in which case the transmitter is reset.
bool enc28j60_frame_tx_wait_blk(enc28j60 *eth, uint32_t frame_id,
                                uint32_t timeout_us);

//! Transmit a frame that was uploaded, wait for the outcome and release its
//! buffer space. In half duplex a late collision, which some link partners
//! provoke with their link pulses, aborts the frame without a retry by the MAC
//! (silicon errata, issue 13): the frame is sent again after one, up to
//! max_retries times. `retries`, if not NULL, gets the number of retries.
enc28j60_tx_status enc28j60_frame_send_blk(enc28j60 *eth, uint32_t frame_id,
                                           uint32_t timeout_us,
                                           uint8_t max_retries,
                                           uint8_t *retries);

//! The second half of enc28j60_frame_send_blk(), for a frame that
//! enc28j60_frame_tx_blk() started: wait for its outcome, send it again after a
//! late collision, and release its buffer space. Other frames may be uploaded
//! while it is on the wire, which is the point of starting a frame and waiting
//! for it apart; no other transmission may start before this returns.
enc28j60_tx_status enc28j60_frame_send_finish_blk(enc28j60 *eth,
                                                  uint32_t frame_id,
                                                  uint32_t timeout_us,
                                                  uint8_t max_retries,
                                                  uint8_t *retries);

//! Has the transmission started last ended, sent or given up on? One read of
//! EIR (TXIF or TXERIF), no wait. A caller that acknowledges TXIF or TXERIF
//! itself cannot use it.
bool enc28j60_frame_tx_ended_blk(enc28j60 *eth);

//! Read the transmit status vector of the frame that was sent last, seven
//! bytes laid out as in the data sheet (table 7-3), least significant byte
//! first. Valid until the next frame is uploaded over it.
void enc28j60_tx_status_blk(enc28j60 *eth, uint32_t frame_id, uint8_t tsv[7]);

//! Confirm frame transmission
void enc28j60_frame_tx_confirm(enc28j60 *eth, uint32_t frame_id);

//! Get the size of an incoxing frame
uint32_t enc28j60_frame_rx_info(enc28j60 *eth);
uint32_t enc28j60_frame_rx_info_blk(enc28j60 *eth);

#define ENC28J60_RX_SIZE(X) ((uint16_t)((X) & 0xffff))
#define ENC28J60_RX_NEXT(X) ((uint16_t)((X) >> 16))

//! Receive a frame
void enc28j60_frame_rx(enc28j60 *eth, uint32_t rx_info, void *buffer);
void enc28j60_frame_rx_blk(enc28j60 *eth, uint32_t rx_info, void *buffer);

//! Discard the incoming frame
void enc28j60_frame_discard(enc28j60 *eth, uint32_t rx_info);
void enc28j60_frame_discard_blk(enc28j60 *eth, uint32_t rx_info);

//! Get number of packets in the receive buffer
uint8_t enc28j60_rx_count(enc28j60 *eth);
uint8_t enc28j60_rx_count_blk(enc28j60 *eth);

//! Drop everything in the receive buffer and restart the receiver; use it to
//! recover when a frame header read from the buffer makes no sense
void enc28j60_rx_reset_blk(enc28j60 *eth);

//! Program a new MAC address; call it before traffic starts, frames being
//! received while the address changes may be filtered wrongly
void enc28j60_set_mac_blk(enc28j60 *eth, const uint8_t *mac);
