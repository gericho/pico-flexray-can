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
    uint32_t bitrate;
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_attempt;
    uint32_t overflow_count;
    uint32_t error_count;
    uint32_t fwd_count;
    uint32_t checksum_error_count;
    uint32_t debug_c1con;
    uint32_t debug_c1txqcon;
    uint32_t debug_c1txqsta;
    uint32_t debug_c1txqua;
    uint8_t send_fail_reason;
    uint8_t rec;
    uint8_t tec;
    bool bus_off;
} can_bus_status_t;

void can_bus_init(void);
void can_bus_poll(void);
void can_bus_reset(void);
bool can_bus_set_bitrate(uint32_t bitrate);
uint32_t can_bus_get_bitrate(void);
bool can_bus_is_started(void);
bool can_bus_pop_frame(can_bus_frame_t *frame);
bool can_bus_send_frame(const can_bus_frame_t *frame);
void can_bus_get_status(can_bus_status_t *status);

#endif
