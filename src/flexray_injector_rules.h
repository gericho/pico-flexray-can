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
	// Use 0x6A as the immediate same-src predecessor trigger for 0x83.
	// Cache live 0x83, keep bytes 0/1/2/7/8 live from the OEM template, replace
	// payload bytes 3..6 from the host, then fix the FlexRay cycle count +
	// frame CRC before injection.
	//
	// Current host-side reconstruction for frame 0x83:
	//   byte0 : live OEM cycle/phase
	//   byte1 : live OEM unresolved coded byte
	//   byte2 : live OEM rolling cadence (F0..FE)
	//   byte3 : low byte of u
	//   byte4 : high byte / wrap-counter byte of u
	//   byte5 : active gate 0x80
	//   byte6 : active gate 0x02
	//   byte7 : live OEM
	//   byte8 : live OEM
	// with u = byte3 + 256*byte4 carrying the current best stock-compatible
	// longitudinal target-speed-like value.
	{
		.trigger_id = 0x6A,
		.target_id = 0x83,
		.cycle_mask = 0b11,
		.cycle_base = 0,
		.e2e_offset = 0,
		.e2e_len = 0,
		.e2e_init_value = 0x00,
		// Preserve payload bytes 0, 1, 2, 7, and 8 from the live template.
		.replace_offset = 3,
		.replace_len = 4,
		.direction = INJECT_DIRECTION_TO_VEHICLE,
	},
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
