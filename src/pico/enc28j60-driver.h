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
    ENC28J60_ERR_IO //!< an SPI transfer stalled: the PIO does not run
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
    uint irq_pin;
    uint dma_irq; //!< IRQ channel used to handle DMAs

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
//! SPI program when it is the last user, and the memory of the driver. The
//! configuration stays with the caller. The DMA channels and the interrupt
//! handler of the command buffer, which only exist when notify callbacks were
//! given, are not released.
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
//! a mix of the EIE bits PKTIE (0x40), LINKIE (0x10), TXIE (0x08) and RXERIE
//! (0x01); pending flags are cleared first, and 0 turns the pin off. The
//! function sets INTIE itself.
void enc28j60_irq_enable_blk(enc28j60 *eth, uint8_t sources);

//! Check if the interrupt is the link interrput
bool enc28j60_irq_is_link(uint8_t flags);

//! Check if the interrupt is the transmit interrupt
bool enc28j60_irq_is_tx(uint8_t flags);

//! Check if the interrupt is the receive interrupt
bool enc28j60_irq_is_rx(uint8_t flags);

//! Check if the interrupt is the receive error interrupt
bool enc28j60_irq_is_rx_err(uint8_t flags);

//! Disable the interrupt and get the interrupt flags
uint8_t enc28j60_irq_flags(enc28j60 *eth);

//! Acknowledge and enable the interrupt
void enc28j60_irq_ack(enc28j60 *eth, uint8_t flags);

#define ENC28J60_INVALID_FRAME_ID 0xffffffff

//! Upload a frame
//! Returns a frame id needed for transmission
uint32_t enc28j60_frame_upload(enc28j60 *eth, size_t size, const void *data);
uint32_t enc28j60_frame_upload_blk(enc28j60 *eth, size_t size,
                                   const void *data);

//! Transmit a frame
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

//! Read the transmit status vector of the frame that was sent last, seven
//! bytes laid out as in the data sheet (table 7-3), least significant byte
//! first. Valid until the next frame is uploaded over it.
void enc28j60_tx_status_blk(enc28j60 *eth, uint32_t frame_id, uint8_t tsv[7]);

//! Confirm frame transmission
void enc28j60_frame_tx_confirm(enc28j60 *eth, uint32_t frame_id);

//! Get the size of an incoxing frame
uint32_t enc28j60_frame_rx_info(enc28j60 *eth);
uint32_t enc28j60_frame_rx_info_blk(enc28j60 *eth);

#define ENC28J60_RX_SIZE(X) (X & 0xffff)
#define ENC28J60_RX_NEXT(X) (X >> 16)

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
