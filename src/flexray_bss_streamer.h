#ifndef FLEXRAY_BSS_STREAMER_H
#define FLEXRAY_BSS_STREAMER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "flexray_frame.h"

// CPU-side frame streaming/parsing caused long-run forwarding instability on
// FR3/FR4. Keep it disabled by default; PIO/DMA forwarding still runs.
#ifndef FLEXRAY_CPU_STREAMING
#define FLEXRAY_CPU_STREAMING 0
#endif

#ifndef FLEXRAY_TIMING_IRQ_DIAG
#define FLEXRAY_TIMING_IRQ_DIAG 1
#endif

// --- Global State ---
extern uint dma_data_from_fr1_chan;
extern uint dma_data_from_fr2_chan;

// Ring-ready buffers for FR1/FR2 stream (defined in flexray_bss_streamer.c)
extern volatile uint8_t fr1_ring_buffer[];
extern volatile uint8_t fr2_ring_buffer[];

// Ring-ready buffers for FR3/FR4 stream (defined in flexray_bss_streamer.c)
extern volatile uint8_t fr3_ring_buffer[];
extern volatile uint8_t fr4_ring_buffer[];

// Ring sizes for FR1/FR2
#define FR1_RING_SIZE_BYTES   (1u << 12)
#define FR2_RING_SIZE_BYTES   (1u << 12)
#define FR1_RING_MASK         (FR1_RING_SIZE_BYTES - 1)
#define FR2_RING_MASK         (FR2_RING_SIZE_BYTES - 1)

// Ring sizes for FR3/FR4
#define FR3_RING_SIZE_BYTES   (1u << 12)
#define FR4_RING_SIZE_BYTES   (1u << 12)
#define FR3_RING_MASK         (FR3_RING_SIZE_BYTES - 1)
#define FR4_RING_MASK         (FR4_RING_SIZE_BYTES - 1)

// Address table for automatic buffer switching
extern volatile void *buffer_addresses[2];

// --- Function Prototypes ---
void streamer_irq0_handler(void);

// Setup primary stream (FR1/FR2)
void setup_stream(PIO pio,
                  uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                  uint rx_pin_from_fr2, uint tx_en_pin_to_fr1);

// Setup secondary stream (FR3/FR4) for source identification
void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3);

// Look up which channel (FR3 or FR4) a frame ID belongs to, based on
// bitmaps populated by the FR3/FR4 ISR. Returns FROM_FR3, FROM_FR4, or FROM_UNKNOWN.
uint8_t lookup_frame_source(uint16_t frame_id);

// Clear FR3/FR4 source counters (hard reset).
void clear_frame_source_bitmaps(void);

// Decrement all FR3/FR4 source counters by 1 (gradual timeout).
void decay_frame_source_counts(void);

// --- Cross-core notification ring (single producer on core1 ISR, single consumer on core0) ---
// Encoded format: [31]=is_fr2_or_fr4, [30]=bus(0=FR12,1=FR34), [29:12]=seq(18 bits), [11:0]=ring index
bool notify_queue_pop(uint32_t *encoded);
void notify_queue_init(void);
uint32_t notify_queue_dropped(void);
void streamer_process_notify(uint32_t encoded);

// Decoded notification info
typedef struct {
    bool is_fr2;        // bus 0: FR2 if true; bus 1: FR4 if true
    uint8_t bus;        // 0 = FR1/FR2, 1 = FR3/FR4
    uint32_t seq;       // 18-bit sequence
    uint16_t end_idx;   // 12-bit ring index (end position)
} notify_info_t;

typedef struct __attribute__((packed)) {
    uint32_t irq_counter;
    uint32_t irq_handler_call_count;
    uint32_t notify_dropped;
    uint16_t rx_write_idx[4];
    uint16_t dma_trans_count[4];
    uint8_t streamer_pc[4];
    uint8_t txen_level[4];
    uint8_t pio0_irq;
    uint8_t pio1_irq;
    uint8_t active_mask;
    uint8_t reserved;
} streamer_timing_diag_t;

void streamer_get_timing_diag(streamer_timing_diag_t *out);

typedef struct __attribute__((packed)) {
    uint32_t event_count[2];      // 0=FR1/FR2 streamer PIO, 1=FR3/FR4 streamer PIO
    uint32_t last_event_us[2];
    uint32_t last_dt_us[2];
    uint32_t min_dt_us[2];
    uint32_t max_dt_us[2];
    uint16_t last_rx_idx[4];
    uint16_t last_rx_delta[4];
    uint8_t pio_irq[2];
    uint8_t active_mask;
    uint8_t reserved;
} streamer_edge_diag_t;

void streamer_get_edge_diag(streamer_edge_diag_t *out);
void streamer_reset_edge_diag(void);

// Decode encoded notification into structured fields
static inline void notify_decode(uint32_t encoded, notify_info_t *out)
{
    out->is_fr2 = (encoded >> 31) & 0x1;
    out->bus = (uint8_t)((encoded >> 30) & 0x1);
    out->seq = (encoded >> 12) & 0x3FFFF;
    out->end_idx = (uint16_t)(encoded & 0x0FFF);
}

static inline uint32_t notify_encode(bool is_fr2, uint8_t bus, uint32_t seq, uint16_t end_idx)
{
    return ((uint32_t)is_fr2 << 31) | ((uint32_t)(bus & 1u) << 30) | ((seq & 0x3FFFFu) << 12) | (end_idx & 0x0FFF);
}

#endif // FLEXRAY_BSS_STREAMER_H
