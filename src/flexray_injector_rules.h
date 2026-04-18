#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

#define INJECT_DIRECTION_TO_FR1 0
#define INJECT_DIRECTION_TO_FR2 1
#define INJECT_DIRECTION_TO_FR3 2
#define INJECT_DIRECTION_TO_FR4 3
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;     // trigger/host override cycle mask
	uint8_t cycle_base;     // trigger/host override cycle base
	uint8_t cache_cycle_mask;
	uint8_t cache_cycle_base;
	uint8_t inject_cycle_offset;
	uint8_t e2e_offset;
	uint8_t e2e_len;
	uint8_t e2e_init_value;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t direction;
	uint8_t raw_override;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// Current 2026-04-18 wiring: FR3 is the EPS ECU side. EPS-directed
	// candidate injections must go to FR3.
	{
		.trigger_id = 0x47,
		.target_id = 0x48,
		.cycle_mask = 0b11,
		.cycle_base = 1,
		.cache_cycle_mask = 0b11,
		.cache_cycle_base = 1,
		.inject_cycle_offset = 0,
		.e2e_offset = 0,
		.e2e_len = 15,
		.e2e_init_value = 0xd6,
		.replace_offset = 2,
		.replace_len = 14,
		.direction = INJECT_DIRECTION_TO_FR3,
		.raw_override = 0,
	},
	{
		// Current SAS/EPS capture: 0x40/src13 immediately precedes the real
		// 0x44/src13 branch. Odd 0x44 branches are zero templates.
		.trigger_id = 0x40,
		.target_id = 0x44,
		.cycle_mask = 0b01,
		.cycle_base = 0,
		.cache_cycle_mask = 0b01,
		.cache_cycle_base = 0,
		.inject_cycle_offset = 0,
		.e2e_offset = 0,
		.e2e_len = 0,
		.e2e_init_value = 0,
		.replace_offset = 0,
		.replace_len = 16,
		.direction = INJECT_DIRECTION_TO_FR3,
		.raw_override = 1,
	},
	{
		// Local 0x15 angle-request candidate.
		// Route 58/59 timing: same-src predecessor is usually 0x83 at cycle N,
		// target 0x15 real branch follows at cycle N+1. Cache only real/even
		// 0x15 templates, but pop host overrides on odd 0x83 trigger cycles.
		.trigger_id = 0x83,
		.target_id = 0x15,
		.cycle_mask = 0b01,
		.cycle_base = 1,
		.cache_cycle_mask = 0b01,
		.cache_cycle_base = 0,
		.inject_cycle_offset = 1,
		.e2e_offset = 0,
		.e2e_len = 0,
		.e2e_init_value = 0,
		.replace_offset = 0,
		.replace_len = 16,
		.direction = INJECT_DIRECTION_TO_FR3,
		.raw_override = 1,
	},
	{
		// Local 0x38 angle-like support/copy candidate.
		// Route 58/59 timing: 0x37 immediately precedes 0x38 on the same cycle.
		.trigger_id = 0x37,
		.target_id = 0x38,
		.cycle_mask = 0b00,
		.cycle_base = 0,
		.cache_cycle_mask = 0b00,
		.cache_cycle_base = 0,
		.inject_cycle_offset = 0,
		.e2e_offset = 0,
		.e2e_len = 0,
		.e2e_init_value = 0,
		.replace_offset = 0,
		.replace_len = 16,
		.direction = INJECT_DIRECTION_TO_FR3,
		.raw_override = 1,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
