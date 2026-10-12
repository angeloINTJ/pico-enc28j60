// Copyright 2024 Lukasz Janyst <lukasz@jany.st>
// Modifications copyright 2026 Ângelo Moisés Alves
// Licensed under the MIT license, see the LICENSE file for details.

#include "pico/enc28j60-driver.h"
#include "enc28j60-cmd.h"
#include "enc28j60-spi.pio.h"

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <pico/stdlib.h>

#include <stdint.h>
#include <stdlib.h>

#define RAM_SIZE 0x2000
#define TX_EPILOG_SIZE 7
#define TX_PROLOG_SIZE 1
#define TX_OVERHEAD_SIZE (TX_PROLOG_SIZE + TX_EPILOG_SIZE)

typedef struct irq_arg_chain {
    uint gpio;
    void *data;
    enc28j60_irq_hanler cb;
    struct irq_arg_chain *next;
} irq_arg_chain;

static irq_arg_chain irq_args;

static void __time_critical_func(irq_handler)(uint gpio, uint32_t events)
{
    (void)events;
    irq_arg_chain *cur = irq_args.next;
    while (cur) {
        if (gpio == cur->gpio) {
            cur->cb(cur->data);
            break;
        }
        cur = cur->next;
    }
}

enc28j60_config *enc28j60_get_default_config()
{
    enc28j60_config *cfg = calloc(1, sizeof(enc28j60_config));
    if (cfg == NULL) {
        panic("Failed to allocate memory for ENC28J60 config");
    }

    // Let the driver find a free state machine, see enc28j60_config
    cfg->pio = NULL;
    cfg->sm = ENC28J60_SM_ANY;
    cfg->miso_pin = 16;
    cfg->mosi_pin = 17;
    cfg->sclk_pin = 18;
    cfg->cs_pin = 19; // Separate CS pin
    cfg->spi_hz = ENC28J60_SPI_HZ_DEFAULT;
    cfg->irq_pin = 20;

    // Use half of the buffer as a receive FIFO
    cfg->rx_size = 0x1000;

    // Locally Administered Addresses are defined by the
    // second-least-significant bit of the first octet. You may choose any MAC
    // addresses you want, as long as it matches one of the following:
    //
    // x2:xx:xx:xx:xx:xx
    // x6:xx:xx:xx:xx:xx
    // xA:xx:xx:xx:xx:xx
    // xE:xx:xx:xx:xx:xx
    cfg->mac_addr[0] = 0xa2;
    cfg->mac_addr[1] = 0x32;
    cfg->mac_addr[2] = 0x15;
    cfg->mac_addr[3] = 0xcc;
    cfg->mac_addr[4] = 0x72;
    cfg->mac_addr[5] = 0x0a;

    cfg->irq_cb = NULL;
    return cfg;
}

// The SPI program is loaded once per PIO and shared by the instances of the
// driver that run on it; the last one to leave removes it. That way the driver
// can be started again after a failure, or after enc28j60_deinit(), without
// running out of instruction memory.
static struct {
    uint users;
    uint offset;
} spi_program[NUM_PIOS];

// The SPI program takes this many PIO clocks for each clock of the SPI, see
// enc28j60-spi.pio
#define SPI_PIO_CLOCKS_PER_BIT 4

//! The SPI clock of a configuration, in Hz
static uint32_t spi_hz(const enc28j60_config *cfg)
{
    return cfg->spi_hz ? cfg->spi_hz : ENC28J60_SPI_HZ_DEFAULT;
}

//! Do the pins, the buffer, the SPI clock and the choice of the state machine
//! make sense?
static enc28j60_status validate_config(const enc28j60_config *cfg)
{
    // The PIO drives the four SPI lines, which are different GPIOs below 32
    const uint spi_pins[] = { cfg->mosi_pin, cfg->miso_pin, cfg->sclk_pin,
                              cfg->cs_pin };
    const size_t count = sizeof(spi_pins) / sizeof(spi_pins[0]);
    for (size_t i = 0; i < count; ++i) {
        if (spi_pins[i] >= 32 || spi_pins[i] >= NUM_BANK0_GPIOS) {
            return ENC28J60_ERR_CONFIG;
        }
        for (size_t j = 0; j < i; ++j) {
            if (spi_pins[i] == spi_pins[j]) {
                return ENC28J60_ERR_CONFIG;
            }
        }
    }
    if (cfg->irq_cb != NULL &&
        (cfg->irq_pin >= NUM_BANK0_GPIOS || cfg->irq_pin == cfg->mosi_pin ||
         cfg->irq_pin == cfg->miso_pin || cfg->irq_pin == cfg->sclk_pin ||
         cfg->irq_pin == cfg->cs_pin)) {
        return ENC28J60_ERR_CONFIG;
    }

    // The receive buffer ends on an odd address (silicon errata, issue 14), so
    // its size is even; the rest of the 8 KB is the transmit buffer
    if (cfg->rx_size == 0 || cfg->rx_size % 2 != 0 ||
        cfg->rx_size >= RAM_SIZE) {
        return ENC28J60_ERR_CONFIG;
    }

    // The PIO runs at 4 times the SPI clock, which the system clock has to
    // allow
    const uint32_t hz = spi_hz(cfg);
    if (hz < ENC28J60_SPI_HZ_MIN || hz > ENC28J60_SPI_HZ_MAX ||
        (uint64_t)hz * SPI_PIO_CLOCKS_PER_BIT > clock_get_hz(clk_sys)) {
        return ENC28J60_ERR_CONFIG;
    }

    if (cfg->pio != NULL) {
        bool known = false;
        for (uint i = 0; i < NUM_PIOS; ++i) {
            known = known || cfg->pio == pio_get_instance(i);
        }
        if (!known ||
            (cfg->sm != ENC28J60_SM_ANY && cfg->sm >= NUM_PIO_STATE_MACHINES)) {
            return ENC28J60_ERR_CONFIG;
        }
    }
    return ENC28J60_OK;
}

//! Claim a state machine, so that nobody else takes it, and make sure that its
//! PIO has the SPI program. Fills in the PIO and the state machine of the
//! configuration if they were left to the driver.
static enc28j60_status acquire_pio(enc28j60_config *cfg)
{
    if (cfg->pio == NULL) {
        // Prefer a PIO that has the program already, then any that has room
        bool found = false;
        for (int pass = 0; pass < 2 && !found; ++pass) {
            for (uint i = 0; i < NUM_PIOS && !found; ++i) {
                PIO pio = pio_get_instance(i);
                bool loaded = spi_program[i].users > 0;
                if (loaded != (pass == 0)) {
                    continue;
                }
                if (!loaded &&
                    !pio_can_add_program(pio, &enc28j60_spi_program)) {
                    continue;
                }
                int sm = pio_claim_unused_sm(pio, false);
                if (sm >= 0) {
                    cfg->pio = pio;
                    cfg->sm = (uint)sm;
                    found = true;
                }
            }
        }
        if (!found) {
            return ENC28J60_ERR_PIO_SPACE;
        }
    } else if (cfg->sm == ENC28J60_SM_ANY) {
        int sm = pio_claim_unused_sm(cfg->pio, false);
        if (sm < 0) {
            return ENC28J60_ERR_PIO_BUSY;
        }
        cfg->sm = (uint)sm;
    } else {
        if (pio_sm_is_claimed(cfg->pio, cfg->sm)) {
            return ENC28J60_ERR_PIO_BUSY;
        }
        pio_sm_claim(cfg->pio, cfg->sm);
    }

    uint pio_idx = pio_get_index(cfg->pio);
    if (spi_program[pio_idx].users == 0) {
        // A negative result is an error: the instruction memory is full
        int offset = pio_add_program(cfg->pio, &enc28j60_spi_program);
        if (offset < 0) {
            pio_sm_unclaim(cfg->pio, cfg->sm);
            return ENC28J60_ERR_PIO_SPACE;
        }
        spi_program[pio_idx].offset = (uint)offset;
    }
    ++spi_program[pio_idx].users;
    return ENC28J60_OK;
}

//! Give back what acquire_pio() took, and the pins
static void release_pio(const enc28j60_config *cfg)
{
    uint pio_idx = pio_get_index(cfg->pio);
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);
    pio_sm_unclaim(cfg->pio, cfg->sm);
    if (--spi_program[pio_idx].users == 0) {
        pio_remove_program(cfg->pio, &enc28j60_spi_program,
                           spi_program[pio_idx].offset);
    }
    hw_clear_bits(&cfg->pio->input_sync_bypass, 1u << cfg->miso_pin);
    gpio_deinit(cfg->mosi_pin);
    gpio_deinit(cfg->miso_pin);
    gpio_deinit(cfg->sclk_pin);
    gpio_deinit(cfg->cs_pin);
}

//! Run the SPI program on the state machine, from the start. It also puts the
//! lines in their idle state, which ends a transfer that was cut short, so it
//! is how a stopped or stuck state machine is brought back.
static void start_sm(const enc28j60_config *cfg)
{
    uint offset = spi_program[pio_get_index(cfg->pio)].offset;
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);
    pio_sm_config c = enc28j60_spi_program_get_default_config(offset);

    // out pins: MOSI (1 pin)
    sm_config_set_out_pins(&c, cfg->mosi_pin, 1);
    // in pins: MISO
    sm_config_set_in_pins(&c, cfg->miso_pin);
    // set pins: CS (1 pin) - controlled via set pins instruction
    sm_config_set_set_pins(&c, cfg->cs_pin, 1);
    // sideset pins: SCK (1 pin) - controlled via side_set
    sm_config_set_sideset_pins(&c, cfg->sclk_pin);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    // The PIO program takes SPI_PIO_CLOCKS_PER_BIT PIO clocks for each SPI
    // clock, so the PIO runs at that many times the SPI clock. The chip select
    // timing of the program is that of the highest clock of the chip, see
    // enc28j60-spi.pio. The original driver used 3.5 MHz because MAC, MII and
    // PHY accesses were unreliable above it. Its program let go of the chip
    // select 143 ns after a read, where the data sheet asks for 210 ns (TCSH)
    // for the MAC and MII registers, and less at a faster clock.
    float div = (float)clock_get_hz(clk_sys) /
                (float)((uint64_t)spi_hz(cfg) * SPI_PIO_CLOCKS_PER_BIT);
    if (div < 1.0f) {
        div = 1.0f; // the system clock was lowered after the check: as fast as
                    // it goes
    }
    sm_config_set_clkdiv(&c, div);

    // Initialize pins to default state: CS high, SCK low, MOSI low
    pio_sm_set_pins_with_mask(cfg->pio, cfg->sm,
                              (1u << cfg->cs_pin), // CS high
                              (1u << cfg->cs_pin) | (1u << cfg->sclk_pin) |
                                  (1u << cfg->mosi_pin));
    // Set pin directions: CS, SCK, MOSI as output; MISO as input
    pio_sm_set_pindirs_with_mask(
        cfg->pio, cfg->sm,
        (1u << cfg->cs_pin) | (1u << cfg->sclk_pin) | (1u << cfg->mosi_pin),
        (1u << cfg->cs_pin) | (1u << cfg->sclk_pin) | (1u << cfg->mosi_pin) |
            (1u << cfg->miso_pin));
    pio_gpio_init(cfg->pio, cfg->mosi_pin);
    pio_gpio_init(cfg->pio, cfg->miso_pin);
    pio_gpio_init(cfg->pio, cfg->sclk_pin);
    pio_gpio_init(cfg->pio, cfg->cs_pin);
    hw_set_bits(&cfg->pio->input_sync_bypass, 1u << cfg->miso_pin);

    // Run the program
    pio_sm_init(cfg->pio, cfg->sm, offset, &c);
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

//! Call cfg->irq_cb when the INT pin falls. Returns false if out of memory.
static bool add_gpio_irq(const enc28j60_config *cfg)
{
    irq_arg_chain *arg = calloc(1, sizeof(irq_arg_chain));
    if (arg == NULL) {
        return false;
    }
    arg->gpio = cfg->irq_pin;
    arg->data = cfg->irq_data;
    arg->cb = cfg->irq_cb;
    arg->next = irq_args.next;
    irq_args.next = arg;
    gpio_set_irq_enabled_with_callback(cfg->irq_pin, GPIO_IRQ_EDGE_FALL, true,
                                       irq_handler);
    return true;
}

//! The opposite of add_gpio_irq()
static void remove_gpio_irq(const enc28j60_config *cfg)
{
    // Off first, so that the handler is not walking the list while we change it
    gpio_set_irq_enabled(cfg->irq_pin, GPIO_IRQ_EDGE_FALL, false);
    for (irq_arg_chain **link = &irq_args.next; *link != NULL;
         link = &(*link)->next) {
        irq_arg_chain *entry = *link;
        if (entry->gpio == cfg->irq_pin && entry->data == cfg->irq_data) {
            *link = entry->next;
            free(entry);
            break;
        }
    }
}

//! Returns false if there is no ENC28J60 answering on the SPI pins
static bool initialize_enc28j60(enc28j60 *eth, enc28j60_config *cfg)
{
    // Reset, just in case we power cycled the controller, but not the device.
    // We need to wait 1ms to make sure the PHY clock is ready after a soft
    // reset, as per errata.
    system_reset(eth);
    sleep_ms(1);

    // Wait for the oscillator to stabilize before accessing MAC or PHY. A
    // missing device or wrong wiring never reports ready, so give up instead
    // of hanging the caller.
    absolute_time_t deadline = make_timeout_time_ms(100);
    while (!(reg_read_e_blk(eth, ESTAT) & CLKRDY)) {
        if (eth->io_error || time_reached(deadline)) {
            return false;
        }
    }

    // EREVID is a 5 bit field; all zeros, or anything above that, means that
    // we are not talking to an ENC28J60 (floating or stuck MISO).
    bank_set_blk(eth, 3);
    uint8_t rev = reg_read_e_blk(eth, EREVID);
    if (rev == 0 || rev > 0x1f) {
        return false;
    }

    // Set up the receving FIFO, section 6.1 of the manual
    bank_set_blk(eth, 0);

    // The receive buffer starts at 0
    reg_write16_blk(eth, ERXSTL, 0);

    // The receive buffer ends at the place accommodating the configured size;
    // the byte pointed to by the reg is included in the rx buffer
    uint16_t rx_sz = cfg->rx_size - 1;
    reg_write16_blk(eth, ERXNDL, rx_sz);

    // The receive read pointer marks the end of the free space. Per the silicon
    // errata it must always be odd, so it starts at the end of the buffer
    // (the byte "before" ERXST in the circular buffer).
    reg_write16_blk(eth, ERXRDPTL, rx_sz);

    // Read pointer to the beginning of the receive buffer
    reg_write16_blk(eth, ERDPTL, 0);

    // Write pointer to the beginning of the transmit buffer
    reg_write16_blk(eth, EWRPTL, cfg->rx_size);

    // Accept only valid unicast, multicast, and broadcast packets;
    // sections 6.3 and 8.0
    bank_set_blk(eth, 1);
    reg_write_blk(eth, ERXFCON, UCEN | CRCEN | MCEN | BCEN);

    // The device does not support automatic duplex negotiation, if plugged to a
    // switch with automatic duplex negotiation it will appear a as 10M
    // half-duplex. For full duplex, the switch needs to be reconfigured
    // manually. We want this to be plug-and-play, so we won't bother with full
    // duplex.

    // MAC initialization; chapter 6.5
    bank_set_blk(eth, 2);

    // Enable the MAC to receive packets
    reg_write_blk(eth, MACON1, MARXEN);

    // Enable automatic padding of short frames and CRC generation
    reg_write_blk(eth, MACON3, PADCRC);

    // When the medium is occupied, the MAC will wait indefinitely for it to
    // become free when attempting to transmit; IEEE 802.3 compliance
    reg_write_blk(eth, MACON4, DEFER);

    // Ethernet normally supports frames of 1518 bytes or less
    uint16_t max_frame_length = 1518;
    reg_write16_blk(eth, MAMXFLL, max_frame_length);

    // Delay between the end of one transmission and the beginning of the next
    // in a back-to-back sequence. 0x12 represents the minimum IEEE specified
    // Inter-Packet Gap (IPG) of 9.6 us.
    reg_write_blk(eth, MABBIPG, 0x12);

    // Non-Back-to-Back Inter-Packet Gap
    reg_write_blk(eth, MAIPGL, 0x12);
    reg_write_blk(eth, MAIPGH, 0x0c);

    // Program the MAC address
    bank_set_blk(eth, 3);
    reg_write_blk(eth, MAADR1, cfg->mac_addr[0]);
    reg_write_blk(eth, MAADR2, cfg->mac_addr[1]);
    reg_write_blk(eth, MAADR3, cfg->mac_addr[2]);
    reg_write_blk(eth, MAADR4, cfg->mac_addr[3]);
    reg_write_blk(eth, MAADR5, cfg->mac_addr[4]);
    reg_write_blk(eth, MAADR6, cfg->mac_addr[5]);

    // Disable the half-duplex loopback
    reg_write_p_blk(eth, PHCON2, HDLDIS);

    // Set up the interrupt: zero all flags, set enables. TXERIE as well as
    // TXIE: the end of a frame that was aborted may only raise TXERIF (silicon
    // errata, issue 13).
    if (cfg->irq_cb) {
        reg_write_p_blk(eth, PHIE, PGEIE | PLNKIE);
        reg_write_blk(eth, EIR, 0);
        reg_write_blk(eth, EIE,
                      INTIE | PKTIE | LINKIE | TXIE | TXERIE | RXERIE);
    }

    // Enable the receiver
    reg_bits_set_blk(eth, ECON1, RXEN);
    return true;
}

enc28j60 *enc28j60_init(enc28j60_config *cfg)
{
    return enc28j60_init_status(cfg, NULL);
}

enc28j60 *enc28j60_init_status(enc28j60_config *cfg, enc28j60_status *status)
{
    if (cfg == NULL) {
        panic("Missing ENC28J60 config");
    }

    // What the caller asked for, to put back if this fails, so that the same
    // configuration can be tried again
    PIO requested_pio = cfg->pio;
    uint requested_sm = cfg->sm;
    bool irq_added = false;
    enc28j60 *eth = NULL;

    enc28j60_status result = validate_config(cfg);
    if (result != ENC28J60_OK) {
        goto restore;
    }

    result = acquire_pio(cfg);
    if (result != ENC28J60_OK) {
        goto restore;
    }

    eth = calloc(1, sizeof(enc28j60));
    if (eth == NULL) {
        result = ENC28J60_ERR_NOMEM;
        goto release;
    }
    eth->cfg = cfg;
    eth->wptr = cfg->rx_size;
    eth->dev_rptr = eth->wptr;

    start_sm(cfg);
    if (cfg->irq_cb) {
        irq_added = add_gpio_irq(cfg);
        if (!irq_added) {
            result = ENC28J60_ERR_NOMEM;
            goto unwind;
        }
    }
    if (!initialize_enc28j60(eth, cfg)) {
        result = eth->io_error ? ENC28J60_ERR_IO : ENC28J60_ERR_NO_CHIP;
        goto unwind;
    }

    if (cfg->notify.notify != NULL && cfg->notify.wait != NULL) {
        enc28j60_cmd_buf_init(eth);
    }

    if (status != NULL) {
        *status = ENC28J60_OK;
    }
    return eth;

unwind:
    if (irq_added) {
        remove_gpio_irq(cfg);
    }
    free(eth);
release:
    release_pio(cfg);
restore:
    cfg->pio = requested_pio;
    cfg->sm = requested_sm;
    if (status != NULL) {
        *status = result;
    }
    return NULL;
}

const char *enc28j60_status_string(enc28j60_status status)
{
    switch (status) {
    case ENC28J60_OK:
        return "ok";
    case ENC28J60_ERR_CONFIG:
        return "invalid configuration: check the pins, the buffer size and the "
               "SPI clock";
    case ENC28J60_ERR_PIO_BUSY:
        return "the PIO state machine is already in use";
    case ENC28J60_ERR_PIO_SPACE:
        return "no PIO has room for the SPI program and a free state machine";
    case ENC28J60_ERR_NO_CHIP:
        return "no ENC28J60 answers: check the wiring and the 3.3 V supply";
    case ENC28J60_ERR_NOMEM:
        return "out of memory";
    case ENC28J60_ERR_IO:
        return "the SPI transfer stalled: the PIO state machine does not run";
    }
    return "unknown error";
}

bool enc28j60_alive_blk(enc28j60 *eth)
{
    return (reg_read_e_blk(eth, ECON1) & RXEN) != 0;
}

uint8_t enc28j60_revision_blk(enc28j60 *eth)
{
    bank_set_blk(eth, 3);
    return reg_read_e_blk(eth, EREVID);
}

bool enc28j60_link_status_blk(enc28j60 *eth)
{
    uint16_t phstat2 = reg_read_p_blk(eth, PHSTAT2);
    return (phstat2 & LSTAT) ? 1 : 0;
}

void enc28j60_deinit(enc28j60 *eth)
{
    if (eth == NULL) {
        return;
    }
    enc28j60_config *cfg = eth->cfg;

    // Quiet the chip, if it can be reached: no more frames, no more interrupts
    if (!eth->io_error) {
        reg_bits_clear_blk(eth, ECON1, RXEN);
        reg_write_blk(eth, EIE, 0);
    }
    if (cfg->irq_cb) {
        remove_gpio_irq(cfg);
    }
    release_pio(cfg);
    free(eth);
}

bool enc28j60_io_failed(const enc28j60 *eth)
{
    return eth->io_error;
}

bool enc28j60_recover(enc28j60 *eth)
{
    enc28j60_config *cfg = eth->cfg;

    // Whatever the state machine was doing, it begins again; the chip is then
    // configured as at the start, which leaves both of its buffers empty
    start_sm(cfg);
    eth->io_error = false;
    eth->wptr = cfg->rx_size;
    eth->dev_rptr = eth->wptr;
    if (!initialize_enc28j60(eth, cfg)) {
        // Nothing answers; keep the transfers off until the next attempt
        eth->io_error = true;
        return false;
    }
    return true;
}

//! Silicon errata, issue 6: EIR.PKTIF does not reliably tell if a packet is
//! pending, while the packet counter does. The INT pin is not affected: it
//! does go low when a packet arrives and the interrupt is enabled.
static uint8_t eir_with_packet_flag(enc28j60 *eth, uint8_t eir)
{
    return enc28j60_rx_count_blk(eth) ? (uint8_t)(eir | PKTIF) :
                                        (uint8_t)(eir & (uint8_t)~PKTIF);
}

uint8_t enc28j60_irq_flags_blk(enc28j60 *eth)
{
    // According to the section 12 of the manual
    reg_bits_clear_blk(eth, EIE, INTIE);
    return eir_with_packet_flag(eth, reg_read_e_blk(eth, EIR));
}

void enc28j60_irq_phy_ack_blk(enc28j60 *eth)
{
    // Clear the PGIF flag by readin PHIR; section 12.1.5 of the manual
    reg_read_p_blk(eth, PHIR);
}

void enc28j60_irq_ack_blk(enc28j60 *eth, uint8_t flags)
{
    // If the interrupt was a link interrupt, ack it directly in the PHY
    if (flags & LINKIF) {
        enc28j60_irq_phy_ack_blk(eth);
    }

    // According to the section 12 of the manual
    reg_bits_clear_blk(eth, EIR, flags);
    reg_bits_set_blk(eth, EIE, INTIE);
}

uint8_t enc28j60_irq_status_blk(enc28j60 *eth)
{
    // A plain read: unlike enc28j60_irq_flags_blk() this leaves the interrupt
    // output enabled, so it is safe to use for looking without acting
    return eir_with_packet_flag(eth, reg_read_e_blk(eth, EIR));
}

void enc28j60_irq_enable_blk(enc28j60 *eth, uint8_t sources)
{
    // Keep the interrupt output quiet while the flags are cleared
    reg_write_blk(eth, EIE, 0);

    // A link change reaches EIR.LINKIF through the PHY interrupt; reading PHIR
    // clears whatever it had pending
    reg_write_p_blk(eth, PHIE, (sources & LINKIE) ? (PGEIE | PLNKIE) : 0);
    enc28j60_irq_phy_ack_blk(eth);
    reg_write_blk(eth, EIR, 0);

    if (sources) {
        reg_write_blk(eth, EIE, INTIE | sources);
    }
}

bool enc28j60_irq_is_link(uint8_t flags)
{
    return flags & LINKIF;
}

bool enc28j60_irq_is_tx(uint8_t flags)
{
    // A frame the MAC gave up on after a late collision may raise TXERIF
    // without TXIF (silicon errata, issue 13): either one ends the request
    return flags & (TXIF | TXERIF);
}

bool enc28j60_irq_is_tx_err(uint8_t flags)
{
    return flags & TXERIF;
}

bool enc28j60_irq_is_rx(uint8_t flags)
{
    return flags & PKTIF;
}

bool enc28j60_irq_is_rx_err(uint8_t flags)
{
    return flags & RXERIF;
}

uint8_t enc28j60_irq_flags(enc28j60 *eth)
{
    // According to the section 12 of the manual
    uint8_t flags = 0;
    enc28j60_cmd_buf_reset(eth);
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(EIE, INTIE), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, RCR_E_CMD(EIR), NULL);
    enc28j60_cmd_buf_execute(eth);
    enc28j60_cmd_buf_decode_rcr(eth, true, &flags);
    return flags;
}

void enc28j60_irq_ack(enc28j60 *eth, uint8_t flags)
{
    // If the interrupt was a link interrupt, ack it directly in the PHY
    if (flags & LINKIF) {
        enc28j60_irq_phy_ack_blk(eth);
    }

    // According to the section 12 of the manual
    enc28j60_cmd_buf_reset(eth);
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(EIR, flags), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, BFS_CMD(EIE, INTIE), NULL);
    enc28j60_cmd_buf_execute(eth);
}

static bool tx_buffer_fit(enc28j60 *eth, size_t size)
{
    // The read pointer is not in front of us and we cannot fit the frame in the
    // remaining RAM; so we
    if (eth->wptr >= eth->dev_rptr &&
        eth->wptr + size + TX_OVERHEAD_SIZE > RAM_SIZE) {
        if (eth->dev_rptr == eth->cfg->rx_size) {
            return false;
        }
        eth->wptr = eth->cfg->rx_size;
    }

    // The read pointer is if front of us, so we need to check if we
    // can fit the frame
    if (eth->wptr < eth->dev_rptr &&
        eth->wptr + size + TX_OVERHEAD_SIZE >= eth->dev_rptr) {
        return false;
    }
    return true;
}

static uint32_t tx_compute_ptrs(enc28j60 *eth, size_t size)
{
    uint32_t id = ((uint32_t)eth->wptr) << 16;
    eth->wptr = (uint16_t)(eth->wptr + size);
    id |= eth->wptr;
    eth->wptr += TX_OVERHEAD_SIZE;
    if (eth->wptr % 2) {
        ++eth->wptr;
    }
    return id;
}

uint32_t enc28j60_frame_upload(enc28j60 *eth, size_t size, const void *data)
{
    // Section 7.1 of the datasheet
    if (!tx_buffer_fit(eth, size)) {
        return ENC28J60_INVALID_FRAME_ID;
    }

    enc28j60_cmd_buf_reset(eth);

    // Register bank 0
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(ECON1, 0x3), NULL);

    // Set the write pointer
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(EWRPTL, eth->wptr), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(EWRPTH, eth->wptr >> 8), NULL);

    // Set the frame header - transmit according to MACON3
    static const uint8_t zero = 0;
    enc28j60_cmd_buf_encode_cmd(eth, WBM_CMD(1), &zero);

    // Encode the frame
    size_t remaining = size;
    const uint8_t *ptr = data;
    while (remaining) {
        size_t to_write = remaining;
        if (to_write > WBM_MAX) {
            to_write = WBM_MAX;
        }
        enc28j60_cmd_buf_encode_cmd(eth, WBM_CMD(to_write), ptr);
        ptr += to_write;
        remaining -= to_write;
    }

    // Exec the command buffer
    enc28j60_cmd_buf_execute(eth);

    return tx_compute_ptrs(eth, size);
}

uint32_t enc28j60_frame_upload_blk(enc28j60 *eth, size_t size, const void *data)
{
    // Section 7.1 of the datasheet
    if (!tx_buffer_fit(eth, size)) {
        return ENC28J60_INVALID_FRAME_ID;
    }

    // Register bank 0
    bank_set_blk(eth, 0);

    // Set the write pointer
    reg_write16_blk(eth, EWRPTL, eth->wptr);

    // Set the frame header - transmit according to MACON3
    static const uint8_t zero = 0;
    mem_write_blk(eth, 1, &zero);

    // Encode the frame
    size_t remaining = size;
    const uint8_t *ptr = data;
    while (remaining) {
        size_t to_write = remaining;
        if (to_write > WBM_MAX) {
            to_write = WBM_MAX;
        }
        mem_write_blk(eth, (uint8_t)to_write, ptr);
        ptr += to_write;
        remaining -= to_write;
    }

    return tx_compute_ptrs(eth, size);
}

#define TX_START(X) ((uint16_t)((X) >> 16))
#define TX_END(X) ((uint16_t)((X) & 0xffff))

void enc28j60_frame_tx(enc28j60 *eth, uint32_t frame_id)
{
    // Section 7.1 of the datasheet
    uint16_t start = TX_START(frame_id);
    uint16_t end = TX_END(frame_id);

    enc28j60_cmd_buf_reset(eth);

    // Register bank 0, and the request of the frame before withdrawn: after a
    // late collision the transmit logic may keep TXRTS set without raising TXIF
    // (silicon errata, issue 13), and setting it again would then start nothing
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(ECON1, TXRTS | 0x3), NULL);

    // Silicon errata, issue 12: an abort can leave the transmit logic stuck, so
    // it is reset before every frame; the reset may raise TXERIF, cleared after
    enc28j60_cmd_buf_encode_cmd(eth, BFS_CMD(ECON1, TXRST), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(ECON1, TXRST), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(EIR, TXERIF | TXIF), NULL);

    // Set the pointers
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ETXSTL, start), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ETXSTH, start >> 8), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ETXNDL, end), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ETXNDH, end >> 8), NULL);

    // Fire things up
    enc28j60_cmd_buf_encode_cmd(eth, BFS_CMD(ECON1, TXRTS), NULL);
    enc28j60_cmd_buf_execute(eth);
}

void enc28j60_frame_tx_blk(enc28j60 *eth, uint32_t frame_id)
{
    // Section 7.1 of the datasheet
    uint16_t start = TX_START(frame_id);
    uint16_t end = TX_END(frame_id);

    bank_set_blk(eth, 0);

    // Silicon errata (rev B1-B7): in half duplex, a transmit abort (excessive
    // collisions, late collision) can leave the transmit logic stuck with
    // TXRTS set forever. Reset it before every transmission, then clear the
    // flags the reset may raise.
    reg_bits_set_blk(eth, ECON1, TXRST);
    reg_bits_clear_blk(eth, ECON1, TXRST);
    reg_bits_clear_blk(eth, EIR, TXERIF | TXIF);

    // Start of the packet
    reg_write16_blk(eth, ETXSTL, start);

    // Last byte of the data payload
    reg_write16_blk(eth, ETXNDL, end);

    // Fire things up
    reg_bits_set_blk(eth, ECON1, TXRTS);
}

// The transmit status vector follows the last byte of a frame in the buffer
#define TSV_SIZE 7

// Its bit 29, the late collision, is bit 5 of the fourth byte
#define TSV_LATE_COLLISION 0x20

//! Wait until the transmission that was started ends, one way or another
static enc28j60_tx_status tx_wait(enc28j60 *eth, uint32_t timeout_us)
{
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    enc28j60_tx_status result;

    while (true) {
        uint8_t eir = reg_read_e_blk(eth, EIR);
        if (eth->io_error) {
            return ENC28J60_TX_IO;
        }
        // An abort is looked for first: the MAC raises TXIF as well when it
        // gives up on a frame
        if (eir & TXERIF) {
            result = ENC28J60_TX_ABORTED;
            break;
        }
        if (eir & TXIF) {
            result = ENC28J60_TX_OK;
            break;
        }
        if (time_reached(deadline)) {
            result = ENC28J60_TX_TIMEOUT;
            break;
        }
    }

    // The request is over whatever happened. After an abort the transmit logic
    // may not know it: this tells it (silicon errata, issue 13).
    reg_bits_clear_blk(eth, ECON1, TXRTS);
    if (result == ENC28J60_TX_TIMEOUT) {
        // Still transmitting after the timeout (medium busy or stuck
        // transmitter): stop it so the buffer space can be reused.
        reg_bits_set_blk(eth, ECON1, TXRST);
        reg_bits_clear_blk(eth, ECON1, TXRST);
        reg_bits_clear_blk(eth, EIR, TXERIF | TXIF);
    }
    return result;
}

bool enc28j60_frame_tx_wait_blk(enc28j60 *eth, uint32_t frame_id,
                                uint32_t timeout_us)
{
    bool sent = tx_wait(eth, timeout_us) == ENC28J60_TX_OK;

    // Release the buffer space either way
    enc28j60_frame_tx_confirm(eth, frame_id);
    return sent;
}

void enc28j60_tx_status_blk(enc28j60 *eth, uint32_t frame_id, uint8_t tsv[7])
{
    // Reading the vector moves the read pointer of the buffer, which the code
    // that receives frames expects at the start of the next one: put it back
    bank_set_blk(eth, 0);
    uint16_t read_ptr = reg_read_e_blk(eth, ERDPTL);
    read_ptr |= (uint16_t)reg_read_e_blk(eth, ERDPTH) << 8;
    mem_read_at_blk(eth, TX_END(frame_id) + 1, TSV_SIZE, tsv);
    reg_write16_blk(eth, ERDPTL, read_ptr);
}

enc28j60_tx_status enc28j60_frame_send_blk(enc28j60 *eth, uint32_t frame_id,
                                           uint32_t timeout_us,
                                           uint8_t max_retries,
                                           uint8_t *retries)
{
    uint8_t attempts = 0;
    enc28j60_tx_status result;

    while (true) {
        enc28j60_frame_tx_blk(eth, frame_id);
        result = tx_wait(eth, timeout_us);
        if (result != ENC28J60_TX_ABORTED || attempts >= max_retries) {
            break;
        }

        // The MAC retries a frame after an ordinary collision by itself, but
        // not after a late one: that is for us to do (silicon errata, issue
        // 13). An abort for any other reason is final.
        uint8_t tsv[TSV_SIZE];
        enc28j60_tx_status_blk(eth, frame_id, tsv);
        if (!(tsv[3] & TSV_LATE_COLLISION)) {
            break;
        }
        ++attempts;
    }

    enc28j60_frame_tx_confirm(eth, frame_id);
    if (retries != NULL) {
        *retries = attempts;
    }
    return result;
}

void enc28j60_set_mac_blk(enc28j60 *eth, const uint8_t *mac)
{
    bank_set_blk(eth, 3);
    reg_write_blk(eth, MAADR1, mac[0]);
    reg_write_blk(eth, MAADR2, mac[1]);
    reg_write_blk(eth, MAADR3, mac[2]);
    reg_write_blk(eth, MAADR4, mac[3]);
    reg_write_blk(eth, MAADR5, mac[4]);
    reg_write_blk(eth, MAADR6, mac[5]);
    for (int i = 0; i < 6; ++i) {
        eth->cfg->mac_addr[i] = mac[i];
    }
}

void enc28j60_rx_reset_blk(enc28j60 *eth)
{
    uint16_t rx_end = eth->cfg->rx_size - 1;

    // Stop receiving and reset the receive logic
    reg_bits_clear_blk(eth, ECON1, RXEN);
    reg_bits_set_blk(eth, ECON1, RXRST);
    reg_bits_clear_blk(eth, ECON1, RXRST);

    // Empty the buffer: both pointers back to the start state
    bank_set_blk(eth, 0);
    reg_write16_blk(eth, ERXRDPTL, rx_end);
    reg_write16_blk(eth, ERDPTL, 0);

    // The packet counter can only be decremented, one frame at a time
    bank_set_blk(eth, 1);
    for (int i = 0; i < 255 && reg_read_e_blk(eth, EPKTCNT) > 0; ++i) {
        reg_bits_set_blk(eth, ECON2, PKTDEC);
    }

    bank_set_blk(eth, 0);
    reg_bits_set_blk(eth, ECON1, RXEN);
}

void enc28j60_frame_tx_confirm(enc28j60 *eth, uint32_t frame_id)
{
    eth->dev_rptr = (frame_id & 0xffff) + TX_OVERHEAD_SIZE;
    if (eth->dev_rptr % 2) {
        ++eth->dev_rptr;
    }

    // Nothing is waiting any more: start over from the beginning of the
    // transmit area. A frame that does not fit before the end of the RAM goes
    // there, and it has to find the whole area free; with the read pointer left
    // where the last frame ended, a transmit area shorter than two full frames
    // refused every long frame, for ever.
    if (eth->dev_rptr == eth->wptr) {
        eth->dev_rptr = eth->wptr = eth->cfg->rx_size;
    }
}

//! The value for ERXRDPT after the frame ending right before next_packet has
//! been consumed. Silicon errata (rev B1-B7): the receive hardware may corrupt
//! the receive buffer if ERXRDPT is even, so it is set to next_packet - 1,
//! wrapping to the end of the buffer (the receive buffer starts at 0).
static uint16_t rx_read_pointer(const enc28j60 *eth, uint16_t next_packet)
{
    return next_packet == 0 ? eth->cfg->rx_size - 1 : next_packet - 1;
}

//! Queue the commands that hand the frame ending right before next_packet back
//! to the chip, for the asynchronous functions: the same steps as at the end of
//! enc28j60_frame_rx_blk(). The hardware pads every frame to an even length, so
//! next_packet is always even, and writing it to ERXRDPT as it is would break
//! the rule above on every frame.
static void rx_release_encode(enc28j60 *eth, uint16_t next_packet)
{
    // Register bank 0
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(ECON1, 0x3), NULL);

    // RX read pointer; it frees the space up to the end of this frame
    uint16_t rdpt = rx_read_pointer(eth, next_packet);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ERXRDPTL, rdpt), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ERXRDPTH, rdpt >> 8), NULL);

    // RBM read pointer
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ERDPTL, next_packet), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, WCR_CMD(ERDPTH, next_packet >> 8), NULL);

    // Decrement the packet counter
    enc28j60_cmd_buf_encode_cmd(eth, BFS_CMD(ECON2, PKTDEC), NULL);
}

uint32_t enc28j60_frame_rx_info(enc28j60 *eth)
{
    // Read the header - Section 7.2.2 of the datasheet
    uint32_t ret = 0;
    uint8_t val = 0;

    enc28j60_cmd_buf_reset(eth);
    enc28j60_cmd_buf_encode_cmd(eth, RBM_CMD(6), NULL);
    enc28j60_cmd_buf_execute(eth);

    enc28j60_cmd_buf_decode_rcr(eth, true, &val);
    ret |= ((uint32_t)val) << 16;
    enc28j60_cmd_buf_decode_rcr(eth, true, &val);
    ret |= ((uint32_t)val) << 24;
    enc28j60_cmd_buf_decode_rcr(eth, true, &val);
    ret |= ((uint32_t)val);
    enc28j60_cmd_buf_decode_rcr(eth, true, &val);
    ret |= ((uint32_t)val) << 8;
    return ret;
}

uint32_t enc28j60_frame_rx_info_blk(enc28j60 *eth)
{
    // Read the header - Section 7.2.2 of the datasheet
    uint32_t ret = 0;
    uint8_t header[6] = { 0 };
    mem_read_blk(eth, 6, header);
    ret = (((uint32_t)header[1]) << 24) | (((uint32_t)header[0]) << 16);
    ret |= (((uint32_t)header[3]) << 8) | header[2];
    return ret;
}

void enc28j60_frame_rx(enc28j60 *eth, uint32_t rx_info, void *buffer)
{
    uint16_t size = ENC28J60_RX_SIZE(rx_info);
    uint16_t next_packet = ENC28J60_RX_NEXT(rx_info);

    enc28j60_cmd_buf_reset(eth);
    enc28j60_cmd_buf_set_rx_ptr(eth, size, buffer);

    size_t remaining = size;
    while (remaining) {
        size_t to_read = remaining;
        if (to_read > RBM_MAX) {
            to_read = RBM_MAX;
        }
        enc28j60_cmd_buf_encode_cmd(eth, RBM_CMD(to_read), NULL);
        remaining -= to_read;
    }

    rx_release_encode(eth, next_packet);
    enc28j60_cmd_buf_execute(eth);
}

void enc28j60_frame_rx_blk(enc28j60 *eth, uint32_t rx_info, void *buffer)
{
    uint16_t size = ENC28J60_RX_SIZE(rx_info);
    uint16_t next_packet = ENC28J60_RX_NEXT(rx_info);

    uint8_t *ptr = buffer;
    size_t remaining = size;
    while (remaining) {
        size_t to_read = remaining;
        if (to_read > RBM_MAX) {
            to_read = RBM_MAX;
        }
        mem_read_blk(eth, (uint8_t)to_read, ptr);
        ptr += to_read;
        remaining -= to_read;
    }

    // Register bank 0
    bank_set_blk(eth, 0);

    // RX read pointer; it frees the space up to the end of this frame
    uint16_t rdpt = rx_read_pointer(eth, next_packet);
    reg_write16_blk(eth, ERXRDPTL, rdpt);

    // RBM read pointer
    reg_write16_blk(eth, ERDPTL, next_packet);

    // Decrement the packet counter
    reg_bits_set_blk(eth, ECON2, PKTDEC);
}

void enc28j60_frame_discard(enc28j60 *eth, uint32_t rx_info)
{
    uint16_t next_packet = ENC28J60_RX_NEXT(rx_info);

    enc28j60_cmd_buf_reset(eth);
    rx_release_encode(eth, next_packet);
    enc28j60_cmd_buf_execute(eth);
}

void enc28j60_frame_discard_blk(enc28j60 *eth, uint32_t rx_info)
{
    uint16_t next_packet = ENC28J60_RX_NEXT(rx_info);

    // Register bank 0
    bank_set_blk(eth, 0);

    // RX read pointer; it frees the space up to the end of this frame
    uint16_t rdpt = rx_read_pointer(eth, next_packet);
    reg_write16_blk(eth, ERXRDPTL, rdpt);

    // RBM read pointer
    reg_write16_blk(eth, ERDPTL, next_packet);

    // Decrement the packet counter
    reg_bits_set_blk(eth, ECON2, PKTDEC);
}

uint8_t enc28j60_rx_count(enc28j60 *eth)
{
    uint8_t count = 0;
    enc28j60_cmd_buf_reset(eth);
    // Register bank 1
    enc28j60_cmd_buf_encode_cmd(eth, BFC_CMD(ECON1, 0x3), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, BFS_CMD(ECON1, 0x1), NULL);
    enc28j60_cmd_buf_encode_cmd(eth, RCR_E_CMD(EPKTCNT), NULL);
    enc28j60_cmd_buf_execute(eth);
    enc28j60_cmd_buf_decode_rcr(eth, true, &count);
    return count;
}

uint8_t enc28j60_rx_count_blk(enc28j60 *eth)
{
    // Register bank 1
    bank_set_blk(eth, 1);
    return reg_read_e_blk(eth, EPKTCNT);
}
