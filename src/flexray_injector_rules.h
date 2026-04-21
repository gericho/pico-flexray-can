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
		// I-CAN-hack style STEER_REQUEST injection, adapted to our stream:
		// this EPS side has no 0x47 slot; 0x46 is the nearest live predecessor
		// observed immediately before 0x48 on odd cycles.
		.trigger_id = 0x46,
		.target_id = 0x48,
		.cycle_mask = 0b01,
		.cycle_base = 1,
		.cache_cycle_mask = 0b01,
		.cache_cycle_base = 1,
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
