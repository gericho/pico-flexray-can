#ifndef FLEXRAY_BSS_STREAMER_H
#define FLEXRAY_BSS_STREAMER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "flexray_frame.h"

// --- Global State ---
extern uint dma_data_from_fr1_chan;
extern uint dma_data_from_fr2_chan;
extern uint dma_data_from_fr3_chan;
extern uint dma_data_from_fr4_chan;

// Ring-ready buffers (defined in flexray_bss_streamer.c)
extern volatile uint8_t fr1_ring_buffer[];
extern volatile uint8_t fr2_ring_buffer[];
extern volatile uint8_t fr3_ring_buffer[];
extern volatile uint8_t fr4_ring_buffer[];

// Ring sizes for consumers (must match definitions in .c)
#define FR_RING_SIZE_BYTES   (1u << 12)
#define FR_RING_MASK         (FR_RING_SIZE_BYTES - 1)

// Address table for automatic buffer switching
extern volatile void *buffer_addresses[4];

// Debug counter removed; using multicore FIFO notifications

// --- Function Prototypes ---
void streamer_irq0_handler(void);
void setup_stream_fr12(PIO pio,
                       uint rx_pin_from_fr1, uint tx_en_pin_to_fr2,
                       uint rx_pin_from_fr2, uint tx_en_pin_to_fr1);
void setup_stream_fr34(PIO pio,
                       uint rx_pin_from_fr3, uint tx_en_pin_to_fr4,
                       uint rx_pin_from_fr4, uint tx_en_pin_to_fr3);

// --- Cross-core notification ring (single producer on core1 ISR, single consumer on core0) ---
// Encoded format: [31:29]=source(FR1..FR4), [28:12]=seq(17 bits), [11:0]=ring index
bool notify_queue_pop(uint32_t *encoded);
void notify_queue_init(void);
uint32_t notify_queue_dropped(void);

// Decoded notification info
typedef struct {
    uint8_t source;     // FROM_FR1..FROM_FR4
    uint32_t seq;       // 17-bit sequence
    uint16_t end_idx;   // 12-bit ring index (end position)
} notify_info_t;

// Decode encoded notification into structured fields
static inline void notify_decode(uint32_t encoded, notify_info_t *out)
{
    out->source = (uint8_t)((encoded >> 29) & 0x7);
    out->seq = (encoded >> 12) & 0x1FFFF;
    out->end_idx = (uint16_t)(encoded & 0x0FFF);
}

static inline uint32_t notify_encode(uint8_t source, uint32_t seq, uint16_t end_idx)
{
    return ((uint32_t)(source & 0x7) << 29) | ((seq & 0x1FFFFu) << 12) | (end_idx & 0x0FFF);
}

#endif // FLEXRAY_BSS_STREAMER_H
