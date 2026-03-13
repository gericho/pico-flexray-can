#include "can_bus.h"

#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

#define CAN_DEFAULT_BITRATE 500000u
#define CAN3_TX_PIN 47u
#define CAN3_RX_PIN 46u
#define CAN3_STB_PIN 45u
#define CAN_RELAY_1_2_PIN 41u
#define CAN_RX_RING_CAP 32u
#define CAN_IDLE_BITS 11u

#define DEMCR_REG (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL_REG (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT_REG (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_TRCENA_BIT (1u << 24)
#define DWT_CYCCNTENA_BIT (1u << 0)

typedef struct {
    uint32_t next_sample_cycle;
    uint32_t bit_cycles;
    uint16_t crc;
    uint8_t prev_bit;
    uint8_t same_count;
    bool failed;
    bool stuff_error;
} can_decode_state_t;

static volatile can_bus_frame_t can_frames[CAN_RX_RING_CAP];
static volatile uint32_t can_head;
static volatile uint32_t can_tail;

static volatile uint32_t can_rx_total;
static volatile uint32_t can_parse_error;
static volatile uint32_t can_crc_error;
static volatile uint32_t can_stuff_error;
static volatile uint32_t can_overflow_count;

static bool can_started;
static uint32_t current_bitrate = CAN_DEFAULT_BITRATE;
static uint32_t can_bit_cycles = 200u;

static inline uint32_t cycle_now(void)
{
    return DWT_CYCCNT_REG;
}

static inline void cycle_wait_until(uint32_t target)
{
    while ((int32_t)(cycle_now() - target) < 0) {
        tight_loop_contents();
    }
}

static inline bool can_rx_level(void)
{
    return (sio_hw->gpio_hi_in & (1u << (CAN3_RX_PIN - 32u))) != 0u;
}

static inline void can_crc_feed(can_decode_state_t *state, uint8_t bit)
{
    uint8_t top = (uint8_t)((state->crc >> 14) & 0x1u);
    state->crc = (uint16_t)((state->crc << 1) & 0x7fffu);
    if ((top ^ bit) != 0u) {
        state->crc ^= 0x4599u;
    }
}

static uint8_t can_sample_raw_bit(can_decode_state_t *state)
{
    cycle_wait_until(state->next_sample_cycle);
    state->next_sample_cycle += state->bit_cycles;
    return can_rx_level() ? 1u : 0u;
}

static uint8_t can_read_data_bit(can_decode_state_t *state)
{
    if (state->same_count == 5u) {
        uint8_t stuffed = can_sample_raw_bit(state);
        if (stuffed == state->prev_bit) {
            state->failed = true;
            state->stuff_error = true;
            return 1u;
        }
    }

    uint8_t bit = can_sample_raw_bit(state);
    if (bit == state->prev_bit) {
        state->same_count++;
    } else {
        state->prev_bit = bit;
        state->same_count = 1u;
    }
    return bit;
}

static uint32_t can_read_bits(can_decode_state_t *state, uint8_t count, bool feed_crc)
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t bit = can_read_data_bit(state);
        if (state->failed) {
            return 0;
        }
        value = (value << 1) | bit;
        if (feed_crc) {
            can_crc_feed(state, bit);
        }
    }
    return value;
}

static bool can_push_frame(const can_bus_frame_t *frame)
{
    uint32_t head = can_head;
    uint32_t next = (head + 1u) % CAN_RX_RING_CAP;
    if (next == can_tail) {
        can_overflow_count++;
        return false;
    }
    can_frames[head] = *frame;
    can_head = next;
    return true;
}

static void can_decode_attempt(uint32_t start_cycle)
{
    can_decode_state_t state = {0};
    can_bus_frame_t frame = {0};

    state.bit_cycles = can_bit_cycles;
    state.next_sample_cycle = start_cycle + (state.bit_cycles / 2u);
    if (can_sample_raw_bit(&state) != 0u) {
        return;
    }

    state.crc = 0u;
    state.prev_bit = 0u;
    state.same_count = 1u;
    can_crc_feed(&state, 0u);

    state.next_sample_cycle = start_cycle + state.bit_cycles + (state.bit_cycles / 2u);

    uint32_t base_id = can_read_bits(&state, 11u, true);
    if (state.failed) {
        goto fail;
    }

    uint8_t srr_rtr = (uint8_t)can_read_bits(&state, 1u, true);
    uint8_t ide = (uint8_t)can_read_bits(&state, 1u, true);
    if (state.failed) {
        goto fail;
    }

    if (ide == 0u) {
        frame.id = base_id;
        frame.extended = false;
        frame.rtr = srr_rtr != 0u;

        (void)can_read_bits(&state, 1u, true);
        if (state.failed) {
            goto fail;
        }
    } else {
        uint32_t ext_id = can_read_bits(&state, 18u, true);
        uint8_t rtr = (uint8_t)can_read_bits(&state, 1u, true);
        (void)can_read_bits(&state, 1u, true);
        (void)can_read_bits(&state, 1u, true);
        if (state.failed) {
            goto fail;
        }
        frame.id = (base_id << 18) | ext_id;
        frame.extended = true;
        frame.rtr = rtr != 0u;
    }

    frame.dlc = (uint8_t)can_read_bits(&state, 4u, true);
    if (state.failed || frame.dlc > 8u) {
        goto fail;
    }

    if (!frame.rtr) {
        for (uint8_t i = 0; i < frame.dlc; ++i) {
            frame.data[i] = (uint8_t)can_read_bits(&state, 8u, true);
            if (state.failed) {
                goto fail;
            }
        }
    }

    uint16_t rx_crc = (uint16_t)can_read_bits(&state, 15u, false);
    if (state.failed) {
        goto fail;
    }

    if (can_sample_raw_bit(&state) != 1u) {
        goto fail;
    }
    (void)can_sample_raw_bit(&state);
    if (can_sample_raw_bit(&state) != 1u) {
        goto fail;
    }
    for (uint8_t i = 0; i < 7u; ++i) {
        if (can_sample_raw_bit(&state) != 1u) {
            goto fail;
        }
    }

    if (rx_crc != state.crc) {
        can_crc_error++;
        return;
    }

    frame.timestamp_us = time_us_32();
    can_rx_total++;
    (void)can_push_frame(&frame);
    return;

fail:
    if (state.stuff_error) {
        can_stuff_error++;
    }
    can_parse_error++;
}

void can_bus_init(void)
{
    gpio_init(CAN_RELAY_1_2_PIN);
    gpio_set_dir(CAN_RELAY_1_2_PIN, GPIO_OUT);
    gpio_put(CAN_RELAY_1_2_PIN, 1);

    gpio_init(CAN3_STB_PIN);
    gpio_set_dir(CAN3_STB_PIN, GPIO_OUT);
    gpio_put(CAN3_STB_PIN, 0);

    gpio_init(CAN3_RX_PIN);
    gpio_set_dir(CAN3_RX_PIN, GPIO_IN);
    gpio_pull_up(CAN3_RX_PIN);

    can_head = 0u;
    can_tail = 0u;
    can_rx_total = 0u;
    can_parse_error = 0u;
    can_crc_error = 0u;
    can_stuff_error = 0u;
    can_overflow_count = 0u;
    can_started = true;

    printf("CAN RX-only sniffer armed: RX=GPIO%u STB=GPIO%u RELAY=GPIO%u bitrate=%lu\n",
           CAN3_RX_PIN, CAN3_STB_PIN, CAN_RELAY_1_2_PIN, (unsigned long)current_bitrate);
}

void can_bus_worker_init(void)
{
    DEMCR_REG |= DEMCR_TRCENA_BIT;
    DWT_CYCCNT_REG = 0u;
    DWT_CTRL_REG |= DWT_CYCCNTENA_BIT;
    can_bit_cycles = clock_get_hz(clk_sys) / current_bitrate;
}

void can_bus_worker_poll(void)
{
    static bool last_level = true;
    static uint32_t last_transition_cycle = 0u;

    bool level = can_rx_level();
    uint32_t now = cycle_now();

    if (level != last_level) {
        if (last_level && !level) {
            uint32_t high_cycles = now - last_transition_cycle;
            if (high_cycles >= (CAN_IDLE_BITS * can_bit_cycles)) {
                can_decode_attempt(now);
                now = cycle_now();
            }
        }
        last_level = level;
        last_transition_cycle = now;
    }
}

void can_bus_reset(void)
{
    can_head = 0u;
    can_tail = 0u;
    can_rx_total = 0u;
    can_parse_error = 0u;
    can_crc_error = 0u;
    can_stuff_error = 0u;
    can_overflow_count = 0u;
}

bool can_bus_set_bitrate(uint32_t bitrate)
{
    if (bitrate != 500000u) {
        return false;
    }
    current_bitrate = bitrate;
    can_bit_cycles = clock_get_hz(clk_sys) / current_bitrate;
    return true;
}

void can_bus_get_status(can_bus_status_t *status)
{
    if (!status) {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->started = can_started;
    status->relay_enabled = gpio_get(CAN_RELAY_1_2_PIN) != 0;
    status->transceiver_enabled = gpio_get(CAN3_STB_PIN) == 0;
    status->bitrate = current_bitrate;
    status->rx_total = can_rx_total;
    status->parse_error = can_parse_error;
    status->crc_error = can_crc_error;
    status->stuff_error = can_stuff_error;
    status->overflow_count = can_overflow_count;
    status->error_count = can_crc_error + can_stuff_error;
}

uint32_t can_bus_get_bitrate(void)
{
    return current_bitrate;
}

bool can_bus_is_started(void)
{
    return can_started;
}

bool can_bus_pop_frame(can_bus_frame_t *frame)
{
    uint32_t tail = can_tail;
    if (tail == can_head) {
        return false;
    }
    if (frame) {
        *frame = can_frames[tail];
    }
    can_tail = (tail + 1u) % CAN_RX_RING_CAP;
    return true;
}
