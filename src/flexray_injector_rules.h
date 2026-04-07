#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

#define INJECT_DIRECTION_TO_VEHICLE 1
#define INJECT_DIRECTION_TO_ECU 0
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t e2e_offset;
	uint8_t e2e_len;
	uint8_t e2e_init_value;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// BMW i3 SAS-side lateral path uses 0x3c as the burst trigger.
	// The active TX draft now patches frame 0x60 (96 dec) directly.
	{
		.trigger_id = 0x3c,
		.target_id = 0x60,
		.cycle_mask = 0b11,
		.cycle_base = 1,
		.e2e_offset = 0,
		.e2e_len = 0,
		.e2e_init_value = 0xd6,
		.replace_offset = 0,
		.replace_len = 9,
		.direction = INJECT_DIRECTION_TO_VEHICLE,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
