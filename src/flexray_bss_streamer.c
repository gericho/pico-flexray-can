#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"

#include "flexray_bss_streamer.pio.h"
#include "flexray_bss_streamer.h"
#include "flexray_frame.h"

uint dma_data_from_fr1_chan;
uint dma_data_from_fr2_chan;
uint dma_data_from_fr3_chan;
uint dma_data_from_fr4_chan;
static uint dma_rearm_fr1_chan;
static uint dma_rearm_fr2_chan;
static uint dma_rearm_fr3_chan;
static uint dma_rearm_fr4_chan;

static PIO streamer_pio_fr12;
static PIO streamer_pio_fr34;

volatile uint8_t fr1_ring_buffer[FR_RING_SIZE_BYTES] __attribute__((aligned(FR_RING_SIZE_BYTES)));
volatile uint8_t fr2_ring_buffer[FR_RING_SIZE_BYTES] __attribute__((aligned(FR_RING_SIZE_BYTES)));
volatile uint8_t fr3_ring_buffer[FR_RING_SIZE_BYTES] __attribute__((aligned(FR_RING_SIZE_BYTES)));
volatile uint8_t fr4_ring_buffer[FR_RING_SIZE_BYTES] __attribute__((aligned(FR_RING_SIZE_BYTES)));

volatile void *buffer_addresses[4] = {
    (void *)fr1_ring_buffer,
    (void *)fr2_ring_buffer,
    (void *)fr3_ring_buffer,
    (void *)fr4_ring_buffer,
};

volatile uint32_t irq_counter = 0;
volatile uint32_t irq_handler_call_count = 0;
volatile uint32_t current_buffer_index = 0;

#define DMA_BLOCK_COUNT_BYTES (4096u | 0x10000000)

static volatile uint32_t fr1_prev_write_idx = 0;
static volatile uint32_t fr2_prev_write_idx = 0;
static volatile uint32_t fr3_prev_write_idx = 0;
static volatile uint32_t fr4_prev_write_idx = 0;

static inline uint32_t dma_ring_write_idx(uint dma_chan, volatile uint8_t *ring_base)
{
    uint32_t wa = dma_channel_hw_addr(dma_chan)->write_addr;
    return (wa - (uint32_t)(uintptr_t)ring_base) & FR_RING_MASK;
}

#define NOTIFY_RING_SIZE 1024u
static volatile uint32_t notify_ring[NOTIFY_RING_SIZE];
static volatile uint16_t notify_head = 0;
static volatile uint16_t notify_tail = 0;
static volatile uint32_t notify_dropped = 0;

void notify_queue_init(void)
{
    notify_head = 0;
    notify_tail = 0;
    notify_dropped = 0;
}

static inline bool notify_queue_push(uint32_t value)
{
    uint16_t head = notify_head;
    uint16_t next = (uint16_t)((head + 1u) & (NOTIFY_RING_SIZE - 1u));
    if (next == notify_tail)
    {
        notify_dropped++;
        return false;
    }
    notify_ring[head] = value;
    notify_head = next;
    __sev();
    return true;
}

bool notify_queue_pop(uint32_t *encoded)
{
    uint16_t tail = notify_tail;
    if (tail == notify_head)
    {
        return false;
    }
    *encoded = notify_ring[tail];
    notify_tail = (uint16_t)((tail + 1u) & (NOTIFY_RING_SIZE - 1u));
    return true;
}

uint32_t notify_queue_dropped(void)
{
    return notify_dropped;
}

static inline void handle_pair_irq(PIO pio,
                                   uint dma_a, volatile uint8_t *ring_a, volatile uint32_t *prev_a, uint8_t source_a,
                                   uint dma_b, volatile uint8_t *ring_b, volatile uint32_t *prev_b, uint8_t source_b)
{
    uint32_t end_idx;
    uint8_t source;
    uint32_t idx_a = dma_ring_write_idx(dma_a, ring_a);
    uint32_t idx_b = dma_ring_write_idx(dma_b, ring_b);
    bool a_advanced = idx_a != *prev_a;
    bool b_advanced = idx_b != *prev_b;

    if (a_advanced && !b_advanced)
    {
        *prev_a = idx_a;
        end_idx = idx_a;
        source = source_a;
    }
    else if (!a_advanced && b_advanced)
    {
        *prev_b = idx_b;
        end_idx = idx_b;
        source = source_b;
    }
    else
    {
        uint32_t delta_a = (idx_a - *prev_a) & FR_RING_MASK;
        uint32_t delta_b = (idx_b - *prev_b) & FR_RING_MASK;
        if (delta_b > delta_a)
        {
            *prev_b = idx_b;
            end_idx = idx_b;
            source = source_b;
        }
        else
        {
            *prev_a = idx_a;
            end_idx = idx_a;
            source = source_a;
        }
    }

    uint32_t encoded = notify_encode(source, ((irq_counter++) & 0x1FFFFu), (uint16_t)end_idx);
    (void)notify_queue_push(encoded);
    pio_interrupt_clear(pio, 3);
}

void __time_critical_func(streamer_irq0_handler_fr12)(void)
{
    sio_hw->gpio_set = (1u << 7);
    irq_handler_call_count++;
    handle_pair_irq(streamer_pio_fr12,
                    dma_data_from_fr1_chan, fr1_ring_buffer, &fr1_prev_write_idx, FROM_FR1,
                    dma_data_from_fr2_chan, fr2_ring_buffer, &fr2_prev_write_idx, FROM_FR2);
    sio_hw->gpio_clr = (1u << 7);
}

void __time_critical_func(streamer_irq0_handler_fr34)(void)
{
    sio_hw->gpio_set = (1u << 7);
    irq_handler_call_count++;
    handle_pair_irq(streamer_pio_fr34,
                    dma_data_from_fr3_chan, fr3_ring_buffer, &fr3_prev_write_idx, FROM_FR3,
                    dma_data_from_fr4_chan, fr4_ring_buffer, &fr4_prev_write_idx, FROM_FR4);
    sio_hw->gpio_clr = (1u << 7);
}

void streamer_irq0_handler(void)
{
    streamer_irq0_handler_fr12();
}

static void setup_stream_pair(PIO pio,
                              uint rx_pin_a, uint tx_en_pin_b,
                              uint rx_pin_b, uint tx_en_pin_a,
                              volatile uint8_t *ring_a, uint *dma_a,
                              volatile uint8_t *ring_b, uint *dma_b,
                              uint *dma_rearm_a, uint *dma_rearm_b,
                              irq_handler_t handler)
{
    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    uint sm_a = pio_claim_unused_sm(pio, true);
    uint sm_b = pio_claim_unused_sm(pio, true);

    flexray_bss_streamer_program_init(pio, sm_a, offset, rx_pin_a, tx_en_pin_b);
    flexray_bss_streamer_program_init(pio, sm_b, offset, rx_pin_b, tx_en_pin_a);

    *dma_a = dma_claim_unused_channel(true);
    *dma_b = dma_claim_unused_channel(true);
    dma_channel_config dma_c_a = dma_channel_get_default_config(*dma_a);
    dma_channel_config dma_c_b = dma_channel_get_default_config(*dma_b);
    channel_config_set_transfer_data_size(&dma_c_a, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_b, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_a, false);
    channel_config_set_read_increment(&dma_c_b, false);
    channel_config_set_write_increment(&dma_c_a, true);
    channel_config_set_write_increment(&dma_c_b, true);
    channel_config_set_dreq(&dma_c_a, pio_get_dreq(pio, sm_a, false));
    channel_config_set_dreq(&dma_c_b, pio_get_dreq(pio, sm_b, false));
    channel_config_set_ring(&dma_c_a, true, 12);
    channel_config_set_ring(&dma_c_b, true, 12);
    *dma_rearm_a = dma_claim_unused_channel(true);
    *dma_rearm_b = dma_claim_unused_channel(true);
    channel_config_set_chain_to(&dma_c_a, *dma_rearm_a);
    channel_config_set_chain_to(&dma_c_b, *dma_rearm_b);

    dma_channel_configure(*dma_a, &dma_c_a, (void *)ring_a, &pio->rxf[sm_a], DMA_BLOCK_COUNT_BYTES, true);
    dma_channel_configure(*dma_b, &dma_c_b, (void *)ring_b, &pio->rxf[sm_b], DMA_BLOCK_COUNT_BYTES, true);

    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 7);
    pio_sm_set_enabled(pio, sm_a, true);
    pio_sm_set_enabled(pio, sm_b, true);
}

void setup_stream_fr12(PIO pio,
                       uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                       uint rx_pin_from_fr2, uint tx_en_pin_to_fr1)
{
    streamer_pio_fr12 = pio;
    setup_stream_pair(pio,
                      rx_pin_from_fr1, tx_en_pin_to_fr2,
                      rx_pin_from_fr2, tx_en_pin_to_fr1,
                      fr1_ring_buffer, &dma_data_from_fr1_chan,
                      fr2_ring_buffer, &dma_data_from_fr2_chan,
                      &dma_rearm_fr1_chan, &dma_rearm_fr2_chan,
                      streamer_irq0_handler_fr12);
}

void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3)
{
    streamer_pio_fr34 = pio;
    setup_stream_pair(pio,
                      rx_pin_from_fr3, tx_en_pin_to_fr4,
                      rx_pin_from_fr4, tx_en_pin_to_fr3,
                      fr3_ring_buffer, &dma_data_from_fr3_chan,
                      fr4_ring_buffer, &dma_data_from_fr4_chan,
                      &dma_rearm_fr3_chan, &dma_rearm_fr4_chan,
                      streamer_irq0_handler_fr34);
}
