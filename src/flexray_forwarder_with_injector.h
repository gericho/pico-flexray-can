#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#ifndef FLEXRAY_ENABLE_INJECTOR
#define FLEXRAY_ENABLE_INJECTOR 1
#endif

typedef struct __attribute__((packed)) {
    uint32_t override_submit_count;
    uint32_t override_submit_accept_count;
    uint32_t target_cache_count;
    uint32_t trigger_cycle_match_count;
    uint32_t override_pop_hit_count;
    uint32_t inject_fire_count;
    uint16_t last_target_id;
    uint8_t last_cycle_count;
    uint8_t last_direction;
    uint8_t last_replace_len;
    uint8_t injector_enabled;
} injector_diag_t;

typedef struct __attribute__((packed)) {
    uint8_t forwarder_pc[4];
    uint8_t txd_level[4];
    uint8_t pio2_irq;
    uint8_t injector_enabled;
    uint8_t active_mask;
    uint8_t reserved;
} forwarder_timing_diag_t;

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving a frame, check triggers; if matched, mutate template and request injection
void try_inject_frame(uint16_t frame_id, uint8_t cycle_count);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3);

void setup_forwarder_sas_only(PIO pio,
    uint rx_pin_from_fr1, uint tx_pin_to_fr2,
    uint rx_pin_from_fr2, uint tx_pin_to_fr1);

void setup_forwarder_eps_only(PIO pio,
    uint rx_pin_from_fr3, uint tx_pin_to_fr4,
    uint rx_pin_from_fr4, uint tx_pin_to_fr3);

// Submit a host-provided replacement slice to be used on next matching injection
// bytes must contain only the replacement payload slice; length must equal rule->replace_len
// The override applies when id matches a rule's target_id and (cycle_count & rule->cycle_mask) == rule->cycle_base
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);
void injector_get_diag(injector_diag_t *out);
void forwarder_get_timing_diag(forwarder_timing_diag_t *out);


#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H
