#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"

#include <string.h>

#include "flexray_bss_streamer.pio.h"
#include "flexray_bss_streamer.h"
#include "flexray_forwarder_with_injector.h"
#include "flexray_frame.h"
#include "panda_usb.h"

// ===================== FR1/FR2 (primary) stream state =====================
uint dma_data_from_fr1_chan;
uint dma_data_from_fr2_chan;

static PIO streamer_pio;
static uint streamer_sm_fr1;
static uint streamer_sm_fr2;
static uint streamer_program_offset_fr12;
static uint streamer_txen_fr1_pin;
static uint streamer_txen_fr2_pin;
static bool streamer_fr12_active = false;

volatile uint8_t fr1_ring_buffer[FR1_RING_SIZE_BYTES] __attribute__((aligned(FR1_RING_SIZE_BYTES)));
volatile uint8_t fr2_ring_buffer[FR2_RING_SIZE_BYTES] __attribute__((aligned(FR2_RING_SIZE_BYTES)));

static volatile uint32_t fr1_prev_write_idx = 0;
static volatile uint32_t fr2_prev_write_idx = 0;

// ===================== FR3/FR4 (secondary) stream state =====================
static uint dma_data_from_fr3_chan;
static uint dma_data_from_fr4_chan;

static PIO streamer_pio_fr34;
static uint streamer_sm_fr3;
static uint streamer_sm_fr4;
static uint streamer_program_offset_fr34;
static uint streamer_txen_fr3_pin;
static uint streamer_txen_fr4_pin;
static bool streamer_fr34_active = false;

volatile uint8_t fr3_ring_buffer[FR3_RING_SIZE_BYTES] __attribute__((aligned(FR3_RING_SIZE_BYTES)));
volatile uint8_t fr4_ring_buffer[FR4_RING_SIZE_BYTES] __attribute__((aligned(FR4_RING_SIZE_BYTES)));

static volatile uint32_t fr3_prev_write_idx = 0;
static volatile uint32_t fr4_prev_write_idx = 0;

// ===================== Shared state =====================
#define DMA_BLOCK_COUNT_BYTES  (4096u | 0x10000000) // self trigger

volatile uint32_t irq_counter = 0;
volatile uint32_t irq_handler_call_count = 0;

volatile void *buffer_addresses[2] = {
    (void *)fr1_ring_buffer,
    (void *)fr2_ring_buffer};

volatile int dma_inject_chan_to_fr1 = -1;
volatile int dma_inject_chan_to_fr2 = -1;
volatile int dma_inject_chan_to_fr3 = -1;
volatile int dma_inject_chan_to_fr4 = -1;

volatile uint32_t current_buffer_index = 0;

static inline uint32_t dma_ring_write_idx(uint dma_chan, volatile uint8_t *ring_base, uint32_t ring_mask)
{
    uint32_t wa = dma_channel_hw_addr(dma_chan)->write_addr;
    return (wa - (uint32_t)(uintptr_t)ring_base) & ring_mask;
}

static inline uint8_t pio_relative_pc(PIO pio, uint sm, uint program_offset)
{
    return (uint8_t)((pio->sm[sm].addr - program_offset) & 0x1Fu);
}

// ===================== Lightweight PIO frame-end timing diagnostics =====================
static volatile streamer_edge_diag_t streamer_edge_diag = {0};

static inline void edge_diag_update_bus(uint8_t bus)
{
    uint32_t now = time_us_32();
    uint32_t prev = streamer_edge_diag.last_event_us[bus];

    streamer_edge_diag.event_count[bus]++;
    streamer_edge_diag.last_event_us[bus] = now;
    if (prev != 0) {
        uint32_t dt = now - prev;
        streamer_edge_diag.last_dt_us[bus] = dt;
        if (streamer_edge_diag.min_dt_us[bus] == 0 || dt < streamer_edge_diag.min_dt_us[bus]) {
            streamer_edge_diag.min_dt_us[bus] = dt;
        }
        if (dt > streamer_edge_diag.max_dt_us[bus]) {
            streamer_edge_diag.max_dt_us[bus] = dt;
        }
    }

    if (bus == 0 && streamer_fr12_active) {
        uint16_t fr1_idx = (uint16_t)dma_ring_write_idx(dma_data_from_fr1_chan, fr1_ring_buffer, FR1_RING_MASK);
        uint16_t fr2_idx = (uint16_t)dma_ring_write_idx(dma_data_from_fr2_chan, fr2_ring_buffer, FR2_RING_MASK);
        streamer_edge_diag.last_rx_delta[0] = (uint16_t)((fr1_idx - streamer_edge_diag.last_rx_idx[0]) & FR1_RING_MASK);
        streamer_edge_diag.last_rx_delta[1] = (uint16_t)((fr2_idx - streamer_edge_diag.last_rx_idx[1]) & FR2_RING_MASK);
        streamer_edge_diag.last_rx_idx[0] = fr1_idx;
        streamer_edge_diag.last_rx_idx[1] = fr2_idx;
    } else if (bus == 1 && streamer_fr34_active) {
        uint16_t fr3_idx = (uint16_t)dma_ring_write_idx(dma_data_from_fr3_chan, fr3_ring_buffer, FR3_RING_MASK);
        uint16_t fr4_idx = (uint16_t)dma_ring_write_idx(dma_data_from_fr4_chan, fr4_ring_buffer, FR4_RING_MASK);
        streamer_edge_diag.last_rx_delta[2] = (uint16_t)((fr3_idx - streamer_edge_diag.last_rx_idx[2]) & FR3_RING_MASK);
        streamer_edge_diag.last_rx_delta[3] = (uint16_t)((fr4_idx - streamer_edge_diag.last_rx_idx[3]) & FR4_RING_MASK);
        streamer_edge_diag.last_rx_idx[2] = fr3_idx;
        streamer_edge_diag.last_rx_idx[3] = fr4_idx;
    }
}

static void __time_critical_func(streamer_timing_irq0_handler)(void)
{
    pio_interrupt_clear(streamer_pio, 3);
    edge_diag_update_bus(0);
}

#if !FLEXRAY_ENABLE_INJECTOR
static void __time_critical_func(streamer_timing_fr34_irq0_handler)(void)
{
    pio_interrupt_clear(streamer_pio_fr34, 3);
    edge_diag_update_bus(1);
}
#endif

void streamer_get_edge_diag(streamer_edge_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    uint32_t irq_state = save_and_disable_interrupts();
    *out = streamer_edge_diag;
    restore_interrupts(irq_state);
    out->pio_irq[0] = (uint8_t)(pio0->irq & 0xFFu);
    out->pio_irq[1] = (uint8_t)(pio1->irq & 0xFFu);
    out->active_mask = (streamer_fr12_active ? 0x03u : 0u) | (streamer_fr34_active ? 0x0Cu : 0u);
}

void streamer_reset_edge_diag(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    memset((void *)&streamer_edge_diag, 0, sizeof(streamer_edge_diag));
    if (streamer_fr12_active) {
        streamer_edge_diag.last_rx_idx[0] = (uint16_t)dma_ring_write_idx(dma_data_from_fr1_chan, fr1_ring_buffer, FR1_RING_MASK);
        streamer_edge_diag.last_rx_idx[1] = (uint16_t)dma_ring_write_idx(dma_data_from_fr2_chan, fr2_ring_buffer, FR2_RING_MASK);
    }
    if (streamer_fr34_active) {
        streamer_edge_diag.last_rx_idx[2] = (uint16_t)dma_ring_write_idx(dma_data_from_fr3_chan, fr3_ring_buffer, FR3_RING_MASK);
        streamer_edge_diag.last_rx_idx[3] = (uint16_t)dma_ring_write_idx(dma_data_from_fr4_chan, fr4_ring_buffer, FR4_RING_MASK);
    }
    restore_interrupts(irq_state);
}

// ===================== Cross-core notification ring =====================
#define NOTIFY_RING_SIZE 1024u
static volatile uint32_t notify_ring[NOTIFY_RING_SIZE];
static volatile uint16_t notify_head = 0;
static volatile uint16_t notify_tail = 0;
static volatile uint32_t notify_dropped = 0;

static volatile uint16_t current_frame_id = 0;
static volatile uint8_t current_cycle_count = 0;

// Saturating counters per frame ID for FR3/FR4 source identification.
// A frame must be seen SOURCE_CONFIRM_THRESHOLD times before it is
// considered confirmed on that channel.  Counters are periodically
// decremented (decay) so stale/noisy entries time out naturally.
#define SOURCE_CONFIRM_THRESHOLD 3
#define SOURCE_COUNTER_MAX       6

static volatile uint8_t fr3_source_counts[2048];
static volatile uint8_t fr4_source_counts[2048];

static inline void record_frame_id(volatile uint8_t *counts, uint16_t frame_id)
{
    if (frame_id >= 2048) return;
    uint8_t c = counts[frame_id];
    if (c < SOURCE_COUNTER_MAX) counts[frame_id] = c + 1;
}

static inline void process_injector_stream_frame(volatile uint8_t *ring_base, uint32_t ring_mask, uint32_t start_idx)
{
    uint8_t h0 = ring_base[(start_idx + 0) & ring_mask];
    uint8_t h1 = ring_base[(start_idx + 1) & ring_mask];
    uint8_t h2 = ring_base[(start_idx + 2) & ring_mask];
    uint8_t h4 = ring_base[(start_idx + 4) & ring_mask];
    uint16_t frame_id = (uint16_t)(((uint16_t)(h0 & 0x07) << 8) | h1);
    uint8_t cycle_count = (uint8_t)(h4 & 0x3F);
    uint16_t payload_len = (uint16_t)(((h2 >> 1) & 0x7F) * 2u);
    uint16_t frame_len = (uint16_t)(5u + payload_len + 3u);

    if (frame_len > MAX_FRAME_BUF_SIZE_BYTES) {
        return;
    }

    uint8_t frame_buf[MAX_FRAME_BUF_SIZE_BYTES];
    for (uint16_t i = 0; i < frame_len; i++) {
        frame_buf[i] = ring_base[(start_idx + i) & ring_mask];
    }

    try_cache_last_target_frame(frame_id, cycle_count, frame_len, frame_buf);
    try_inject_frame(frame_id, cycle_count);
}

uint8_t lookup_frame_source(uint16_t frame_id)
{
    if (frame_id >= 2048) return FROM_UNKNOWN;
    bool fr3 = fr3_source_counts[frame_id] >= SOURCE_CONFIRM_THRESHOLD;
    bool fr4 = fr4_source_counts[frame_id] >= SOURCE_CONFIRM_THRESHOLD;
    if (fr3 == fr4) return FROM_UNKNOWN;
    return fr3 ? FROM_FR3 : FROM_FR4;
}

void clear_frame_source_bitmaps(void)
{
    memset((void *)fr3_source_counts, 0, sizeof(fr3_source_counts));
    memset((void *)fr4_source_counts, 0, sizeof(fr4_source_counts));
}

void decay_frame_source_counts(void)
{
    for (int i = 0; i < 2048; i++) {
        if (fr3_source_counts[i] > 0) fr3_source_counts[i]--;
        if (fr4_source_counts[i] > 0) fr4_source_counts[i]--;
    }
}

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

void streamer_process_notify(uint32_t encoded)
{
    notify_info_t info;
    notify_decode(encoded, &info);

    volatile uint8_t *ring_base;
    uint32_t ring_mask;
    uint8_t source;

    if (info.bus == 0) {
        ring_base = info.is_fr2 ? fr2_ring_buffer : fr1_ring_buffer;
        ring_mask = info.is_fr2 ? FR2_RING_MASK : FR1_RING_MASK;
        source = info.is_fr2 ? FROM_FR2 : FROM_FR1;
    } else {
        ring_base = info.is_fr2 ? fr4_ring_buffer : fr3_ring_buffer;
        ring_mask = info.is_fr2 ? FR4_RING_MASK : FR3_RING_MASK;
        source = info.is_fr2 ? FROM_FR4 : FROM_FR3;
    }

    uint32_t start_idx = info.end_idx & ring_mask;
    uint8_t h2 = ring_base[(start_idx + 2) & ring_mask];
    uint16_t payload_len = (uint16_t)(((h2 >> 1) & 0x7F) * 2u);
    uint16_t frame_len = (uint16_t)(5u + payload_len + 3u);
    if (frame_len > MAX_FRAME_BUF_SIZE_BYTES) {
        return;
    }

    uint8_t frame_buf[MAX_FRAME_BUF_SIZE_BYTES];
    for (uint16_t i = 0; i < frame_len; i++) {
        frame_buf[i] = ring_base[(start_idx + i) & ring_mask];
    }

    flexray_frame_t frame;
    if (parse_frame_from_slice(frame_buf, frame_len, source, &frame)) {
        (void)panda_flexray_fifo_push(&frame);
    }
}

void streamer_get_timing_diag(streamer_timing_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->irq_counter = irq_counter;
    out->irq_handler_call_count = irq_handler_call_count;
    out->notify_dropped = notify_dropped;
    out->active_mask = (streamer_fr12_active ? 0x03u : 0u) | (streamer_fr34_active ? 0x0Cu : 0u);

    if (streamer_fr12_active) {
        out->rx_write_idx[0] = (uint16_t)dma_ring_write_idx(dma_data_from_fr1_chan, fr1_ring_buffer, FR1_RING_MASK);
        out->rx_write_idx[1] = (uint16_t)dma_ring_write_idx(dma_data_from_fr2_chan, fr2_ring_buffer, FR2_RING_MASK);
        out->dma_trans_count[0] = (uint16_t)(dma_channel_hw_addr(dma_data_from_fr1_chan)->transfer_count & 0xFFFFu);
        out->dma_trans_count[1] = (uint16_t)(dma_channel_hw_addr(dma_data_from_fr2_chan)->transfer_count & 0xFFFFu);
        out->streamer_pc[0] = pio_relative_pc(streamer_pio, streamer_sm_fr1, streamer_program_offset_fr12);
        out->streamer_pc[1] = pio_relative_pc(streamer_pio, streamer_sm_fr2, streamer_program_offset_fr12);
        out->txen_level[0] = (uint8_t)gpio_get(streamer_txen_fr1_pin);
        out->txen_level[1] = (uint8_t)gpio_get(streamer_txen_fr2_pin);
    }
    if (streamer_fr34_active) {
        out->rx_write_idx[2] = (uint16_t)dma_ring_write_idx(dma_data_from_fr3_chan, fr3_ring_buffer, FR3_RING_MASK);
        out->rx_write_idx[3] = (uint16_t)dma_ring_write_idx(dma_data_from_fr4_chan, fr4_ring_buffer, FR4_RING_MASK);
        out->dma_trans_count[2] = (uint16_t)(dma_channel_hw_addr(dma_data_from_fr3_chan)->transfer_count & 0xFFFFu);
        out->dma_trans_count[3] = (uint16_t)(dma_channel_hw_addr(dma_data_from_fr4_chan)->transfer_count & 0xFFFFu);
        out->streamer_pc[2] = pio_relative_pc(streamer_pio_fr34, streamer_sm_fr3, streamer_program_offset_fr34);
        out->streamer_pc[3] = pio_relative_pc(streamer_pio_fr34, streamer_sm_fr4, streamer_program_offset_fr34);
        out->txen_level[2] = (uint8_t)gpio_get(streamer_txen_fr3_pin);
        out->txen_level[3] = (uint8_t)gpio_get(streamer_txen_fr4_pin);
    }
    out->pio0_irq = (uint8_t)(pio0->irq & 0xFFu);
    out->pio1_irq = (uint8_t)(pio1->irq & 0xFFu);
}

// ===================== FR1/FR2 IRQ handler =====================
void __time_critical_func(streamer_irq0_handler)(void)
{
    sio_hw->gpio_set = (1u << 7);
    uint32_t start_idx = 0;

    irq_handler_call_count++;
    pio_interrupt_clear(streamer_pio, 3);

    uint32_t fr1_idx_now = dma_ring_write_idx(dma_data_from_fr1_chan, fr1_ring_buffer, FR1_RING_MASK);
    uint32_t fr2_idx_now = dma_ring_write_idx(dma_data_from_fr2_chan, fr2_ring_buffer, FR2_RING_MASK);

    bool fr1_advanced = (fr1_idx_now != fr1_prev_write_idx);
    bool fr2_advanced = (fr2_idx_now != fr2_prev_write_idx);

    bool is_fr2 = false;

    if (fr1_advanced && !fr2_advanced)
    {
        start_idx = fr1_prev_write_idx;
        fr1_prev_write_idx = fr1_idx_now;
    }
    else if (!fr1_advanced && fr2_advanced)
    {
        start_idx = fr2_prev_write_idx;
        is_fr2 = true;
        fr2_prev_write_idx = fr2_idx_now;
    }
    else
    {
        uint32_t fr1_delta = (fr1_idx_now - fr1_prev_write_idx) & FR1_RING_MASK;
        uint32_t fr2_delta = (fr2_idx_now - fr2_prev_write_idx) & FR2_RING_MASK;
        if (fr2_delta > fr1_delta)
        {
            start_idx = fr2_prev_write_idx;
            is_fr2 = true;
            fr2_prev_write_idx = fr2_idx_now;
        }
        else
        {
            start_idx = fr1_prev_write_idx;
            fr1_prev_write_idx = fr1_idx_now;
        }
    }

    {
        volatile uint8_t *ring_base = is_fr2 ? fr2_ring_buffer : fr1_ring_buffer;
        uint32_t ring_mask = is_fr2 ? FR2_RING_MASK : FR1_RING_MASK;

        uint8_t h0 = ring_base[(start_idx + 0) & ring_mask];
        uint8_t h1 = ring_base[(start_idx + 1) & ring_mask];
        uint8_t h4 = ring_base[(start_idx + 4) & ring_mask];
        current_frame_id = (uint16_t)(((uint16_t)(h0 & 0x07) << 8) | h1);
        current_cycle_count = (uint8_t)(h4 & 0x3F);

        process_injector_stream_frame(ring_base, ring_mask, start_idx);
    }

    uint32_t encoded = notify_encode(is_fr2, 0, ((irq_counter++) & 0x3FFFF), (uint16_t)start_idx);
    (void)notify_queue_push(encoded);
    sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR3/FR4 IRQ handler =====================
// Optional diagnostic handler. The normal forward-only EPS path does not enable
// this IRQ, so FR3/FR4 forwarding is handled by PIO/DMA without CPU work.
void __time_critical_func(streamer_fr34_irq0_handler)(void)
{
    sio_hw->gpio_set = (1u << 7);
    pio_interrupt_clear(streamer_pio_fr34, 3);

    uint32_t fr3_idx_now = dma_ring_write_idx(dma_data_from_fr3_chan, fr3_ring_buffer, FR3_RING_MASK);
    uint32_t fr4_idx_now = dma_ring_write_idx(dma_data_from_fr4_chan, fr4_ring_buffer, FR4_RING_MASK);

    if (fr3_idx_now != fr3_prev_write_idx) {
        uint32_t start = fr3_prev_write_idx;
        uint8_t h0 = fr3_ring_buffer[(start + 0) & FR3_RING_MASK];
        uint8_t h1 = fr3_ring_buffer[(start + 1) & FR3_RING_MASK];
        uint8_t h4 = fr3_ring_buffer[(start + 4) & FR3_RING_MASK];
        uint16_t fid = (uint16_t)(((uint16_t)(h0 & 0x07) << 8) | h1);
        record_frame_id(fr3_source_counts, fid);
        current_frame_id = fid;
        current_cycle_count = (uint8_t)(h4 & 0x3F);
        process_injector_stream_frame(fr3_ring_buffer, FR3_RING_MASK, start);
        uint32_t encoded = notify_encode(false, 1, ((irq_counter++) & 0x3FFFF), (uint16_t)start);
        (void)notify_queue_push(encoded);
        fr3_prev_write_idx = fr3_idx_now;
    }

    if (fr4_idx_now != fr4_prev_write_idx) {
        uint32_t start = fr4_prev_write_idx;
        uint8_t h0 = fr4_ring_buffer[(start + 0) & FR4_RING_MASK];
        uint8_t h1 = fr4_ring_buffer[(start + 1) & FR4_RING_MASK];
        uint8_t h4 = fr4_ring_buffer[(start + 4) & FR4_RING_MASK];
        uint16_t fid = (uint16_t)(((uint16_t)(h0 & 0x07) << 8) | h1);
        record_frame_id(fr4_source_counts, fid);
        current_frame_id = fid;
        current_cycle_count = (uint8_t)(h4 & 0x3F);
        process_injector_stream_frame(fr4_ring_buffer, FR4_RING_MASK, start);
        uint32_t encoded = notify_encode(true, 1, ((irq_counter++) & 0x3FFFF), (uint16_t)start);
        (void)notify_queue_push(encoded);
        fr4_prev_write_idx = fr4_idx_now;
    }

    sio_hw->gpio_clr = (1u << 7);
}

// ===================== FR1/FR2 setup =====================
void setup_stream(PIO pio,
                  uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                  uint rx_pin_from_fr2, uint tx_en_pin_to_fr1)
{
    streamer_pio = pio;

    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    streamer_program_offset_fr12 = offset;
    uint sm_fr1 = pio_claim_unused_sm(pio, true);
    uint sm_fr2 = pio_claim_unused_sm(pio, true);

    streamer_sm_fr1 = sm_fr1;
    streamer_sm_fr2 = sm_fr2;
    streamer_txen_fr2_pin = tx_en_pin_to_fr2;
    streamer_txen_fr1_pin = tx_en_pin_to_fr1;

    flexray_bss_streamer_program_init(pio, sm_fr1, offset, rx_pin_from_fr1, tx_en_pin_to_fr2);
    flexray_bss_streamer_program_init(pio, sm_fr2, offset, rx_pin_from_fr2, tx_en_pin_to_fr1);
    dma_data_from_fr1_chan = dma_claim_unused_channel(true);
    dma_data_from_fr2_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c_fr1 = dma_channel_get_default_config(dma_data_from_fr1_chan);
    dma_channel_config dma_c_fr2 = dma_channel_get_default_config(dma_data_from_fr2_chan);
    channel_config_set_transfer_data_size(&dma_c_fr1, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_fr2, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_fr1, false);
    channel_config_set_read_increment(&dma_c_fr2, false);
    channel_config_set_write_increment(&dma_c_fr1, true);
    channel_config_set_write_increment(&dma_c_fr2, true);
    channel_config_set_dreq(&dma_c_fr1, pio_get_dreq(pio, sm_fr1, false));
    channel_config_set_dreq(&dma_c_fr2, pio_get_dreq(pio, sm_fr2, false));

    uint8_t fr1_ring_bits = 0;
    if (FR1_RING_SIZE_BYTES > 1)
    {
        fr1_ring_bits = 32 - __builtin_clz(FR1_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr1, true, fr1_ring_bits);

    uint8_t fr2_ring_bits = 0;
    if (FR2_RING_SIZE_BYTES > 1)
    {
        fr2_ring_bits = 32 - __builtin_clz(FR2_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr2, true, fr2_ring_bits);
    dma_channel_configure(dma_data_from_fr1_chan, &dma_c_fr1,
                          (void *)fr1_ring_buffer,
                          &pio->rxf[sm_fr1],
                          dma_encode_endless_transfer_count(),
                          true);
    dma_channel_configure(dma_data_from_fr2_chan, &dma_c_fr2,
                          (void *)fr2_ring_buffer,
                          &pio->rxf[sm_fr2],
                          dma_encode_endless_transfer_count(),
                          true);

    #if FLEXRAY_CPU_STREAMING
    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);
    #elif FLEXRAY_TIMING_IRQ_DIAG
    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_timing_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);
    #endif

    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 7);
    pio_sm_set_enabled(pio, sm_fr1, true);
    pio_sm_set_enabled(pio, sm_fr2, true);
    streamer_fr12_active = true;
}

// ===================== FR3/FR4 setup =====================
void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3)
{
    streamer_pio_fr34 = pio;

    uint offset = pio_add_program(pio, &flexray_bss_streamer_program);
    streamer_program_offset_fr34 = offset;
    uint sm_fr3 = pio_claim_unused_sm(pio, true);
    uint sm_fr4 = pio_claim_unused_sm(pio, true);

    streamer_sm_fr3 = sm_fr3;
    streamer_sm_fr4 = sm_fr4;
    streamer_txen_fr4_pin = tx_en_pin_to_fr4;
    streamer_txen_fr3_pin = tx_en_pin_to_fr3;

    flexray_bss_streamer_program_init(pio, sm_fr3, offset, rx_pin_from_fr3, tx_en_pin_to_fr4);
    flexray_bss_streamer_program_init(pio, sm_fr4, offset, rx_pin_from_fr4, tx_en_pin_to_fr3);

    dma_data_from_fr3_chan = dma_claim_unused_channel(true);
    dma_data_from_fr4_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_c_fr3 = dma_channel_get_default_config(dma_data_from_fr3_chan);
    dma_channel_config dma_c_fr4 = dma_channel_get_default_config(dma_data_from_fr4_chan);
    channel_config_set_transfer_data_size(&dma_c_fr3, DMA_SIZE_8);
    channel_config_set_transfer_data_size(&dma_c_fr4, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_c_fr3, false);
    channel_config_set_read_increment(&dma_c_fr4, false);
    channel_config_set_write_increment(&dma_c_fr3, true);
    channel_config_set_write_increment(&dma_c_fr4, true);
    channel_config_set_dreq(&dma_c_fr3, pio_get_dreq(pio, sm_fr3, false));
    channel_config_set_dreq(&dma_c_fr4, pio_get_dreq(pio, sm_fr4, false));

    uint8_t fr3_ring_bits = 0;
    if (FR3_RING_SIZE_BYTES > 1)
    {
        fr3_ring_bits = 32 - __builtin_clz(FR3_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr3, true, fr3_ring_bits);

    uint8_t fr4_ring_bits = 0;
    if (FR4_RING_SIZE_BYTES > 1)
    {
        fr4_ring_bits = 32 - __builtin_clz(FR4_RING_SIZE_BYTES - 1);
    }
    channel_config_set_ring(&dma_c_fr4, true, fr4_ring_bits);

    dma_channel_configure(dma_data_from_fr3_chan, &dma_c_fr3,
                          (void *)fr3_ring_buffer,
                          &pio->rxf[sm_fr3],
                          dma_encode_endless_transfer_count(),
                          true);
    dma_channel_configure(dma_data_from_fr4_chan, &dma_c_fr4,
                          (void *)fr4_ring_buffer,
                          &pio->rxf[sm_fr4],
                          dma_encode_endless_transfer_count(),
                          true);

    #if FLEXRAY_ENABLE_INJECTOR
    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_fr34_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);
    #elif FLEXRAY_TIMING_IRQ_DIAG
    pio_set_irq0_source_enabled(pio, pis_interrupt3, true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), streamer_timing_fr34_irq0_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);
    #endif

    pio_interrupt_clear(pio, 3);
    pio_interrupt_clear(pio, 7);
    pio_sm_set_enabled(pio, sm_fr3, true);
    pio_sm_set_enabled(pio, sm_fr4, true);
    streamer_fr34_active = true;
}
