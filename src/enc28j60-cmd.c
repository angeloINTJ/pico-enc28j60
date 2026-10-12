// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Modifications copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#include "enc28j60-cmd.h"
#include "pico/enc28j60-driver.h"

#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio.h>

#include <stdlib.h>
#include <string.h>

// The state machine moves a byte through the FIFOs in 8 clocks of the SPI: 0.8
// us at 10 MHz, 80 us at 100 kHz, the slowest clock. A turn of the loop below
// takes about 0.15 us. If neither FIFO moves for this many turns, which are
// some 7 ms and 90 times the longest byte, the state machine does not run:
// another piece of code took it, or it was stopped. The turns are counted
// rather than timed, so that an interrupt, or the other core locking this one
// out while it writes to the flash, cannot be taken for a stall.
#define STALL_SPINS 50000

#ifdef ENC28J60_PROFILE
enc28j60_profile enc28j60_profile_counters;

void enc28j60_profile_reset(void)
{
    memset(&enc28j60_profile_counters, 0, sizeof(enc28j60_profile_counters));
}

void enc28j60_profile_read(enc28j60_profile *out)
{
    *out = enc28j60_profile_counters;
}

//! Add a command that started at the given time to the counters, whichever way
//! it ended
static __force_inline void profile_command_end(uint32_t started)
{
    ++enc28j60_profile_counters.commands;
    enc28j60_profile_counters.command_us += time_us_32() - started;
}
#endif

// Don't keep this function in flash
void __time_critical_func(enc28j60_execute_blocking)(enc28j60 *eth,
                                                     uint32_t cmd, void *dst,
                                                     const void *src)
{
#ifdef ENC28J60_PROFILE
    uint32_t started = time_us_32();
#endif
    io_rw_8 *txfifo = (io_rw_8 *)&eth->cfg->pio->txf[eth->cfg->sm];
    io_rw_8 *rxfifo = (io_rw_8 *)&eth->cfg->pio->rxf[eth->cfg->sm];

    uint8_t c = CMD_OP(cmd);
    size_t rx_sz = CMD_RXSZ(cmd);
    size_t tx_sz = CMD_TXSZ(cmd);

    size_t tx_remain = 2 + tx_sz;
    size_t rx_remain = rx_sz;
    uint8_t data[4] = { (uint8_t)(cmd >> 24), (uint8_t)(cmd >> 16),
                        (uint8_t)(cmd >> 8), (uint8_t)cmd };
    const uint8_t *snd = data;
    uint8_t *rcv = dst;
    uint32_t spins = 0;

    // After a stall the state machine is left alone until enc28j60_recover()
    // starts it again; reads give zeros rather than whatever the buffer held
    if (eth->io_error) {
        goto stalled;
    }

    // For WBM, we first transmit the command header and then the data payload
    // from the source buffer skipping the data payload byte from the command
    // itself
    if (c == WBM_OP) {
        for (int i = 0; i < 3;) {
            if (!pio_sm_is_tx_fifo_full(eth->cfg->pio, eth->cfg->sm)) {
                *txfifo = *snd++;
                ++i;
                spins = 0;
            } else if (++spins > STALL_SPINS) {
                goto stall;
            }
        }
        tx_remain = tx_sz - 1;
        snd = src;
    }

    // Push the data to the TX fifo and read from the RX fifo at the same time
    // as necessary for SPI, although we don't have commands where reading
    // separately would actually block the PIO program
    while (tx_remain || rx_remain) {
        bool progress = false;
        if (tx_remain && !pio_sm_is_tx_fifo_full(eth->cfg->pio, eth->cfg->sm)) {
            *txfifo = *snd++;
            --tx_remain;
            progress = true;
        }
        if (rx_remain &&
            !pio_sm_is_rx_fifo_empty(eth->cfg->pio, eth->cfg->sm)) {
            *rcv++ = *rxfifo;
            --rx_remain;
            progress = true;
        }
        if (progress) {
            spins = 0;
        } else if (++spins > STALL_SPINS) {
            goto stall;
        }
    }
#ifdef ENC28J60_PROFILE
    profile_command_end(started);
#endif
    return;

stall:
    eth->io_error = true;
stalled:
    if (rx_remain) {
        memset(rcv, 0, rx_remain);
    }
#ifdef ENC28J60_PROFILE
    profile_command_end(started);
#endif
}

typedef struct irq_arg_chain {
    uint irq;
    uint channel;
    void *data;
    void (*cb)(void *);
    struct irq_arg_chain *next;
} irq_arg_chain;

static irq_arg_chain irq_args;

static void __time_critical_func(dma_irq_handler)()
{
    irq_arg_chain *cur = irq_args.next;
    while (cur) {
        if (dma_irqn_get_channel_status(cur->irq, cur->channel)) {
            cur->cb(cur->data);
            dma_irqn_acknowledge_channel(cur->irq, cur->channel);
            break;
        }
        cur = cur->next;
    }
}

//! Put arg, filled in here, at the head of the chain that dma_irq_handler()
//! walks; it is linked last, so that the handler never sees half of it
static void add_channel_callback(irq_arg_chain *arg, uint irq, uint channel,
                                 void (*cb)(void *), void *data)
{
    arg->irq = irq;
    arg->channel = channel;
    arg->data = data;
    arg->cb = cb;
    arg->next = irq_args.next;
    irq_args.next = arg;
}

//! Take every entry of the chain that belongs to a driver out, and free it.
//! The channels of those entries no longer raise interrupts, and the handler,
//! which runs on this core, sees the chain whole at every step: each unlink is
//! one store.
static void remove_channel_callbacks(const void *data)
{
    irq_arg_chain **link = &irq_args.next;
    while (*link != NULL) {
        irq_arg_chain *entry = *link;
        if (entry->data == data) {
            *link = entry->next;
            free(entry);
        } else {
            link = &entry->next;
        }
    }
}

// How many drivers use each of the two DMA interrupts; dma_irq_handler() is
// installed on one when its first user comes and removed when the last leaves.
// The SDK has a handful of shared handler slots for the whole system, and
// adding the same handler again for each driver used one up every time.
static uint8_t irq_users[2];

static uint dma_irq_number(uint dma_irq)
{
    return dma_irq ? DMA_IRQ_1 : DMA_IRQ_0;
}

typedef struct {
    uint32_t len;
    const void *data;
} tx_ctrl_t;

#define TX_SIZE 512
#define RX_SIZE 64
#define CTRL_NUM 128

struct enc28j60_cmd_buf {
    uint8_t tx_buf[TX_SIZE];
    uint8_t *tx_buf_ptr;

    uint8_t rx_buf_int[RX_SIZE];
    uint8_t *rx_buf_ptr;
    uint16_t rx_size;
    uint8_t *rx_buf;

    tx_ctrl_t tx_ctrl[CTRL_NUM];
    uint16_t tx_ctrl_idx;

    uint16_t tx_bytes_total;
    uint16_t tx_bytes_current;
    uint16_t rx_bytes;

    uint tx_channel;
    uint rx_channel;
    uint tx_ctrl_channel;

    //! enc28j60_config.dma_irq at the init: what the deinit gives back must not
    //! depend on the configuration staying as it was
    uint dma_irq;

    //! How many of the two DMA transfers of the execution in progress have not
    //! finished yet: the chain that sends the commands, and the reception when
    //! the commands read something
    volatile uint8_t pending;
};

// An execution is over when both of its transfers are. The reception alone is
// not enough: the commands that come after the last read (enc28j60_frame_rx()
// moves the read pointers and decrements the packet counter after reading the
// frame) are still being sent when the last byte comes in, and a new execution
// started then would restart the chain under them. The end of the chain alone
// is not enough either, since the bytes being read come in after the last
// command went out. Both handlers run in the same interrupt, one at a time.
static void __time_critical_func(transfer_done)(enc28j60 *eth)
{
    if (eth->cmd_buf->pending > 0 && --eth->cmd_buf->pending == 0) {
        eth->cfg->notify.notify(eth->cfg->notify.data);
    }
}

static void __time_critical_func(tx_irq_handler)(void *data)
{
    transfer_done(data);
}

static void __time_critical_func(rx_irq_handler)(void *data)
{
    transfer_done(data);
}

enc28j60_status enc28j60_cmd_buf_init(enc28j60 *eth)
{
    enc28j60_config *cfg = eth->cfg;

    // Everything that can be missing is taken first, so that a failure leaves
    // nothing behind
    enc28j60_cmd_buf *b = calloc(1, sizeof(enc28j60_cmd_buf));
    irq_arg_chain *tx_arg = calloc(1, sizeof(irq_arg_chain));
    irq_arg_chain *rx_arg = calloc(1, sizeof(irq_arg_chain));
    if (b == NULL || tx_arg == NULL || rx_arg == NULL) {
        free(b);
        free(tx_arg);
        free(rx_arg);
        return ENC28J60_ERR_NOMEM;
    }
    int channels[3];
    for (int i = 0; i < 3; ++i) {
        channels[i] = dma_claim_unused_channel(false);
        if (channels[i] < 0) {
            while (--i >= 0) {
                dma_channel_unclaim((uint)channels[i]);
            }
            free(b);
            free(tx_arg);
            free(rx_arg);
            return ENC28J60_ERR_DMA;
        }
    }

    eth->cmd_buf = b;
    b->dma_irq = cfg->dma_irq;
    b->tx_buf_ptr = b->tx_buf;
    b->rx_buf_ptr = b->rx_buf;

    b->tx_channel = (uint)channels[0];
    b->tx_ctrl_channel = (uint)channels[1];
    b->rx_channel = (uint)channels[2];

    dma_channel_config c = dma_channel_get_default_config(b->tx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(cfg->pio, cfg->sm, true));

    // Trigger trigger the control channel when tx channel completes
    channel_config_set_chain_to(&c, b->tx_ctrl_channel);

    // Raise the IRQ when 0 is written to a trigger register (end of chain)
    channel_config_set_irq_quiet(&c, true);

    dma_channel_configure(b->tx_channel, &c,
                          (void *)&cfg->pio->txf[cfg->sm], // write address
                          NULL, // read address
                          0, // transfer count
                          false // trigger
    );
    add_channel_callback(tx_arg, cfg->dma_irq, b->tx_channel, tx_irq_handler,
                         eth);
    dma_irqn_set_channel_enabled(cfg->dma_irq, b->tx_channel, true);

    c = dma_channel_get_default_config(b->rx_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(cfg->pio, cfg->sm, false));
    dma_channel_configure(b->rx_channel, &c,
                          NULL, // write address
                          (void *)&cfg->pio->rxf[cfg->sm], // read address
                          0, // transfer count
                          false // trigger
    );
    add_channel_callback(rx_arg, cfg->dma_irq, b->rx_channel, rx_irq_handler,
                         eth);
    dma_irqn_set_channel_enabled(cfg->dma_irq, b->rx_channel, true);

    c = dma_channel_get_default_config(b->tx_ctrl_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);

    // Only 3 lower bits of the write address are allowed to change, so we loop
    // every 8 bytes written
    channel_config_set_ring(&c, true, 3);

    // The DMA's four CSRs are aliased multiple times in memory. Each alias —
    // of which there are four — exposes the same four physical registers, but
    // in a different order.
    // We'll use alias 3. See section 2.5.2.1. of the RP2040 datasheet.
    dma_channel_configure(b->tx_ctrl_channel, &c,
                          &dma_hw->ch[b->tx_channel].al3_transfer_count, NULL,
                          2, false);

    if (irq_users[b->dma_irq]++ == 0) {
        uint irq = dma_irq_number(b->dma_irq);
        irq_add_shared_handler(irq, dma_irq_handler,
                               PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(irq, true);
    }
    return ENC28J60_OK;
}

void enc28j60_cmd_buf_deinit(enc28j60 *eth)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    if (b == NULL) {
        return;
    }
    const uint channels[] = { b->tx_ctrl_channel, b->tx_channel,
                              b->rx_channel };

    // Normally nothing runs here, but a wait that gave up may have left a list
    // going. The channels are disabled first, since a disabled channel ignores
    // triggers: otherwise the transmit channel, ending a block while the others
    // are stopped, would start the control channel again, on the control blocks
    // freed below.
    for (size_t i = 0; i < 3; ++i) {
        hw_clear_bits(&dma_hw->ch[channels[i]].al1_ctrl,
                      DMA_CH0_CTRL_TRIG_EN_BITS);
    }

    // The interrupts go off before the channels are stopped: a transfer in
    // flight when a channel is aborted can still complete and signal it
    // (erratum RP2040-E13, see dma_channel_abort() in the SDK)
    for (size_t i = 0; i < 3; ++i) {
        dma_irqn_set_channel_enabled(b->dma_irq, channels[i], false);
    }
    for (size_t i = 0; i < 3; ++i) {
        dma_channel_abort(channels[i]);
        dma_irqn_acknowledge_channel(b->dma_irq, channels[i]);
        dma_channel_unclaim(channels[i]);
    }
    remove_channel_callbacks(eth);

    if (irq_users[b->dma_irq] > 0 && --irq_users[b->dma_irq] == 0) {
        uint irq = dma_irq_number(b->dma_irq);
        irq_remove_handler(irq, dma_irq_handler);
        // Nobody else shares the interrupt: it goes back to how it was found
        if (!irq_has_shared_handler(irq)) {
            irq_set_enabled(irq, false);
        }
    }

    free(b);
    eth->cmd_buf = NULL;
}

bool enc28j60_cmd_buf_encode_cmd(enc28j60 *eth, uint32_t cmd, const void *src)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;

    uint8_t c = CMD_OP(cmd);
    size_t rx_sz = CMD_RXSZ(cmd);
    size_t tx_sz = 2 + CMD_TXSZ(cmd);
    size_t payload_sz = 0;
    uint8_t data[4] = { (uint8_t)(cmd >> 24), (uint8_t)(cmd >> 16),
                        (uint8_t)(cmd >> 8), (uint8_t)cmd };

    if (c == WBM_OP) {
        payload_sz = tx_sz - 3;
        tx_sz = 3;
    }

    if (b->tx_bytes_total + tx_sz > TX_SIZE) {
        return false;
    }

    if (b->rx_bytes + rx_sz > b->rx_size) {
        return false;
    }

    // We're starting a new buffer, so put its address in the control buffer.
    // We need enough space to place this buffer, and a marker for the end
    // of transmission.
    if (b->tx_bytes_current == 0) {
        if (c == WBM_OP && b->tx_ctrl_idx + 2 >= CTRL_NUM) {
            return false;
        }

        if (b->tx_ctrl_idx + 2 >= CTRL_NUM) {
            return false;
        }

        b->tx_ctrl[b->tx_ctrl_idx].data = b->tx_buf_ptr;
    }

    for (size_t i = 0; i < tx_sz; ++i) {
        *b->tx_buf_ptr++ = data[i];
    }

    b->tx_bytes_total = (uint16_t)(b->tx_bytes_total + tx_sz);
    b->tx_bytes_current = (uint16_t)(b->tx_bytes_current + tx_sz);
    b->rx_bytes = (uint16_t)(b->rx_bytes + rx_sz);

    if (c == WBM_OP) {
        b->tx_ctrl[b->tx_ctrl_idx].len = b->tx_bytes_current;
        ++b->tx_ctrl_idx;
        b->tx_ctrl[b->tx_ctrl_idx].data = src;
        b->tx_ctrl[b->tx_ctrl_idx].len = payload_sz;
        ++b->tx_ctrl_idx;
        b->tx_bytes_current = 0;
    }

    return true;
}

bool enc28j60_cmd_buf_decode_rcr(enc28j60 *eth, bool is_eth, uint8_t *val)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    size_t rx_sz = is_eth ? 1 : 2;
    if ((uint32_t)b->rx_buf_ptr - (uint32_t)b->rx_buf + rx_sz >= RX_SIZE) {
        return false;
    }

    if (is_eth) {
        *val = *b->rx_buf_ptr++;
    } else {
        ++b->rx_buf_ptr;
        *val = *b->rx_buf_ptr++;
    }

    return true;
}

void enc28j60_cmd_buf_reset(enc28j60 *eth)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;

    b->rx_buf = b->rx_buf_int;
    b->rx_size = RX_SIZE;
    b->rx_buf_ptr = b->rx_buf;
    b->rx_bytes = 0;

    b->tx_buf_ptr = b->tx_buf;
    b->tx_bytes_total = 0;
    b->tx_bytes_current = 0;
    b->tx_ctrl_idx = 0;
}

void enc28j60_cmd_buf_execute(enc28j60 *eth)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    enc28j60_config *cfg = eth->cfg;

    // Set before any transfer starts, so that no interrupt can see the count
    // of the previous execution
    b->pending = b->rx_bytes ? 2 : 1;

    if (b->rx_bytes) {
        dma_channel_set_write_addr(b->rx_channel, b->rx_buf, false);
        dma_channel_set_trans_count(b->rx_channel, b->rx_bytes, true);
    }

    if (b->tx_bytes_current != 0) {
        b->tx_ctrl[b->tx_ctrl_idx].len = b->tx_bytes_current;
        b->tx_bytes_current = 0;
        ++b->tx_ctrl_idx;
    }
    b->tx_ctrl[b->tx_ctrl_idx].data = NULL;
    b->tx_ctrl[b->tx_ctrl_idx].len = 0;

    dma_channel_set_read_addr(b->tx_ctrl_channel, b->tx_ctrl, true);

    cfg->notify.wait(cfg->notify.data);
}

void enc28j60_cmd_buf_set_rx_ptr(enc28j60 *eth, size_t size, void *rbuf)
{
    enc28j60_cmd_buf *b = eth->cmd_buf;
    b->rx_buf = rbuf;
    b->rx_size = (uint16_t)size;
    b->rx_buf_ptr = b->rx_buf;
    b->rx_bytes = 0;
}
