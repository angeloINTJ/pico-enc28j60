# The C driver, outside the Arduino IDE

The Arduino classes of this library are built on a driver in C (`src/enc28j60-*.c`,
`src/pico/enc28j60-driver.h`) that also works on its own in a project of the Raspberry
Pi Pico SDK. This page is its documentation, written by **Lukasz Janyst** for
[ljanyst/pico-enc28j60](https://github.com/ljanyst/pico-enc28j60) and kept with the
updates that the changes of this fork called for. For the Arduino IDE, see the
[README](../README.md).

The implementation is split into two libraries. A base library (`src/`) that provides
the infrastructure necessary for communication with the device, and a driver for the
FreeRTOS TCP stack (`freertos/`). If you happen to use a different stack, you can
easily write your own driver using the base library and taking the FreeRTOS driver as
an example.

## RP2040 resources used

The base driver uses:

 * 4 GPIOs for the SPI link to the device, and one more for the interrupt signal when
   an interrupt callback (`irq_cb`) is given; without one, `irq_pin` is not touched.
 * The SPI GPIOs are driven by a PIO program that is 17 instructions long and uses 1
   state machine, which the driver claims so that other users of the PIO (SerialPIO,
   tone, Servo, ...) are given another one. The hardware SPI is not used because the
   device commands are variable length.
 * In non-blocking mode, the driver uses 3 DMA channels: 1 for data reception and two
   for scatter-gather transmission.

The FreeRTOS driver spawns a task and requires two task notifications to function: one
for device interrupt and one for DMA completion interrupt.

## Using in a CMake project

Simply add this repository as a submodule and include it in your master CMakeLists.txt
as follows:

```
set(ENABLE_ENC28J60_FREERTOS_DRIVER FALSE)
add_subdirectory(lib/pico-enc28j60/src)
```

The FreeRTOS driver may be switched on or off.

## Base driver setup

Driver can be configured through an `enc28j60_config` instance. It can be created and
initialized to defaults using `enc28j60_get_default_config()`. The struct has the
following fields:

 * pio - a PIO instance to be used for SPI: pio0 or pio1, or NULL (the default) to let
         the driver look for any one with room and a free state machine
 * sm - index to the state machine to run the PIO program, or ENC28J60_SM_ANY (the
        default); the state machine is claimed
 * miso_pin - defaults to 16
 * mosi_pin - defaults to 17
 * sclk_pin - defaults to 18
 * cs_pin - slave select, defaults to 19
 * spi_hz - the SPI clock in Hz, from ENC28J60_SPI_HZ_MIN (100 kHz) to ENC28J60_SPI_HZ_MAX
           (20 MHz, the limit of the chip); 0 or the default is ENC28J60_SPI_HZ_DEFAULT
           (10 MHz). The PIO runs at four times this, so the system clock has to allow it.
           The clock is never faster than asked for, and is lowered when the fractional
           divider of the PIO would make a period shorter than 50 ns: with a system clock
           of 150 MHz, the default of the Pico 2, 20 MHz becomes 18.75 MHz
 * irq_pin - defaults to 20
 * dma_irq - interrupt channel to handle DMA IRQs, defaults to 0
 * rx_size - the device has 8kB big buffer that is used for RX and TX queues the range
            from 0 to rx_size - 1 will be used for RX, the reminder for TX; defaults to
            0x1000
 * mac_addr - MAC address of the device; the default is the same fixed address for every
   device, so choose one for each
 * irq_cb - callback function for the device interrupts
 * irq_data - data for the callback
 * notify - notification helper described below

The DMA notification struct looks as follows:

```c
typedef struct {
    void (*notify)(void *);
    void (*wait)(void *);
    void *data;
} enc28j60_notification;
```

The notify function is called in an interrupt context to wake a task that is going to
service the interrupt, while wait blocks that task until an interrupt has been raised.
data is passed to both functions when they are called.

### Added in this fork

 * `enc28j60_init_status()` is `enc28j60_init()` that also says why it failed (a status
   from `enc28j60_status`; `enc28j60_status_string()` has a sentence for each): a
   configuration that makes no sense, a state machine that is taken, no room in the PIO,
   no ENC28J60 answering, no memory, a PIO that does not run.
 * `enc28j60_deinit()` stops the chip and gives the state machine, the PIO program (when it
   is the last user) and the memory back, and in the asynchronous mode the DMA channels and
   the DMA interrupt handler (on the core that called the init).
 * Blocking transfers (the functions that end in `_blk`) give up when the state machine
   makes no progress for some 8 ms, instead of waiting for ever: `enc28j60_io_failed()`
   says that this happened, and `enc28j60_recover()` starts the state machine and the chip
   again.
 * `enc28j60_alive_blk()` tells whether the chip still holds its configuration; a chip that
   was reset by a glitch of its supply has the receiver off.
 * `enc28j60_frame_send_blk()` sends a frame that was uploaded, waits for it, and sends it
   again after a late collision (silicon errata, issue 13); `enc28j60_tx_status_blk()` reads
   the transmit status vector of the last frame. It is `enc28j60_frame_tx_blk()` followed by
   `enc28j60_frame_send_finish_blk()`: in between, the next frame can be uploaded while this
   one is on the wire, which is how the lwIP interface sends. No other frame may start
   before the finish returns, because the start of a frame resets the transmit logic.
   `enc28j60_frame_tx_ended_blk()` tells, with one read, whether the frame on the wire has
   ended.
 * `enc28j60_set_mac_blk()`, `enc28j60_rx_reset_blk()`, `enc28j60_irq_status_blk()` and
   `enc28j60_irq_enable_blk()`.
 * `enc28j60_irq_is_tx_err()` and the status `ENC28J60_ERR_DMA`, for the asynchronous
   functions below.

### The asynchronous functions

The functions without `_blk`, which the FreeRTOS driver uses, run their commands through
DMA and wait for them with the `notify` callbacks. They follow the same silicon errata as
the blocking ones:

 * `ERXRDPT` is always odd after a frame was read or dropped (issue 14). They used to write
   the next packet pointer there as it is, which the hardware always makes even.
 * `enc28j60_frame_tx()` resets the transmit logic before every frame (issue 12) and
   withdraws a request that an abort left pending. It also stops a transmission still
   going on, so call it once the previous one has ended.
 * `enc28j60_irq_is_tx()` counts a frame as ended when the MAC sent it (TXIF) or gave up
   on it (TXERIF): after a late collision the chip may raise TXERIF alone (issue 13), and a
   driver that waits for TXIF waits for ever. When an interrupt callback is given, TXERIF
   drives the INT pin as well. `enc28j60_irq_is_tx_err()` tells an abort apart. A frame
   lost to a late collision is not sent again in this mode, as `enc28j60_frame_send_blk()`
   does; `enc28j60_tx_status_blk()` says whether that was the cause.
 * `enc28j60_irq_flags()` takes PKTIF from the packet counter (issue 6).

A call returns when the last of its commands has gone out. `enc28j60_frame_rx()` used to
return as soon as the frame was read, while the commands that move the read pointers and
decrement the packet counter were still being sent, and a call made right then restarted
the DMA under them. `enc28j60_deinit()` gives back the DMA channels, the interrupt handler
and the memory of this mode, and `enc28j60_init_status()` returns `ENC28J60_ERR_DMA`
instead of stopping the program when fewer than three DMA channels are free. `dma_irq` is
0 or 1.

What was run: the bench sketch (`extras/bench/BenchTest`) has an asynchronous mode that
does what the interrupt branch of the FreeRTOS driver does, with callbacks that spin
instead of a FreeRTOS task, and `bench.py` checks it with traffic (see the
[bench README](../extras/bench/README.md)). The FreeRTOS+TCP driver itself is still not
built or run here. Unlike the blocking functions, the asynchronous ones have no time
limit: if the state machine stops, `notify` never comes and `wait` never returns.

## FreeRTOS setup

When using the FreeRTOS driver, especially in a multicore setup, it is important to
ensure the consistency of heap operations. Consider adding PICO_USE_MALLOC_MUTEX to your
global defines:

```cmake
add_definitions(
  -DPICO_USE_MALLOC_MUTEX
)
```

and configuring the FreeRTOS-Pico sync interop in your FreeRTOSConfig.h:

```c
#define configSUPPORT_PICO_SYNC_INTEROP 1
```

The driver releases every network buffer it is given once the frame is in the chip, so
it needs `ipconfigZERO_COPY_TX_DRIVER` set to 1 in FreeRTOSIPConfig.h, where the stack
always hands the buffer over.

The FreeRTOS driver is normally initialized the following way assuming that most of the
driver configuration options are left at their defaults:

```c
#include <pico/enc28j60-driver.h>
#include <pico/enc28j60-freertos-driver.h>

enc28j60_freertos_interface iface;
static NetworkEndPoint_t endpoint_v4;
static NetworkEndPoint_t endpoint_v6_link;

void setup_network()
{
    memset(&iface, 0, sizeof(enc28j60_freertos_interface));
    enc28j60_config *drv_cfg = enc28j60_get_default_config();

    // Choose a MAC address
    memcpy(drv_cfg->mac_addr, mac_addr, 6);

    // Interface setup
    iface.iface.pcName = "e0";
    iface.iface.pvArgument = drv_cfg;
    iface.iface.pfInitialise = enc28j60_freertos_initialize;
    iface.iface.pfOutput = enc28j60_freertos_output_frame;
    iface.iface.pfGetPhyLinkStatus = enc28j60_freertos_get_phy_link_status;
    FreeRTOS_AddNetworkInterface(&iface.iface);

    // IPv4
    memset(&endpoint_v4, 0, sizeof(endpoint_v4));
    // The ip, net_mask, and gateway below may just be zeros if DHCP is used
    FreeRTOS_FillEndPoint(&iface.iface, &endpoint_v4, (uint8_t *)&ip,
                          (uint8_t *)&net_mask, (uint8_t *)&gateway,
                          (uint8_t *)&any_ip, cfg->mac_addr);

    endpoint_v4.bits.bWantDHCP = pdTRUE;

    // IPv6 Link Scope endpoint. A global unicast address requires another
    // endpoint.
    memset(&endpoint_v6_link, 0, sizeof(endpoint_v6_link));
    IPv6_Address_t prefix;
    IPv6_Address_t address;
    FreeRTOS_inet_pton6("fe80::", &prefix);
    memcpy(&address, cfg->ipv6_addr_link, 16);

    FreeRTOS_FillEndPoint_IPv6(&iface.iface, &endpoint_v6_link, &address,
                               &prefix, 64, NULL, NULL, cfg->mac_addr);

    FreeRTOS_IPInit_Multi();
}
```
