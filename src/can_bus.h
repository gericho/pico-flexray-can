#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
    bool rtr;
    uint32_t timestamp_us;
} can_bus_frame_t;

typedef struct {
    bool started;
    bool relay_enabled;
    bool transceiver_enabled;
    uint32_t bitrate;
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_attempt;
    uint32_t parse_error;
    uint32_t error_count;
    uint32_t crc_error;
    uint32_t stuff_error;
    uint32_t overflow_count;
} can_bus_status_t;

void can_bus_init(void);
void can_bus_worker_init(void);
void can_bus_worker_poll(void);
void can_bus_reset(void);
bool can_bus_set_bitrate(uint32_t bitrate);
void can_bus_get_status(can_bus_status_t *status);
uint32_t can_bus_get_bitrate(void);
bool can_bus_is_started(void);
bool can_bus_pop_frame(can_bus_frame_t *frame);

#endif
