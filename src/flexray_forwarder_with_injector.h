#ifndef FLEXRAY_FORWARDER_WITH_INJECTOR_H
#define FLEXRAY_FORWARDER_WITH_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

typedef struct {
    uint32_t override_submit_count;
    uint32_t override_submit_accept_count;
    uint32_t target96_cache_count;
    uint32_t trigger60_cycle_match_count;
    uint32_t override96_pop_hit_count;
    uint32_t inject_fire_count;
    uint16_t last_target_id;
    uint8_t last_cycle_count;
    uint8_t last_direction;
    uint8_t last_replace_len;
    uint8_t injector_enabled;
} injector_diag_t;

// Cache a frame's raw bytes (header+payload+CRC) when rules match
void try_cache_last_target_frame(uint16_t frame_id, uint8_t cycle_count, uint16_t frame_length, uint8_t *captured_bytes);

// On receiving a frame, check triggers; if matched, mutate template and request injection
void try_inject_frame(uint16_t frame_id, uint8_t cycle_count);

void setup_forwarder_with_injector(PIO pio,
    uint rx_pin_from_ecu, uint tx_pin_to_vehicle,
    uint rx_pin_from_vehicle, uint tx_pin_to_ecu);

// Submit a host-provided replacement slice to be used on next matching injection
// bytes must contain only the replacement payload slice; length must equal rule->replace_len
// The override applies when id matches a rule's target_id and (cycle_count & rule->cycle_mask) == rule->cycle_base
bool injector_submit_override(uint16_t id, uint8_t base, uint16_t len, const uint8_t *bytes);

// Enable/disable injection at runtime
void injector_set_enabled(bool enabled);
bool injector_is_enabled(void);
void injector_get_diag(injector_diag_t *out);


#endif // FLEXRAY_FORWARDER_WITH_INJECTOR_H
