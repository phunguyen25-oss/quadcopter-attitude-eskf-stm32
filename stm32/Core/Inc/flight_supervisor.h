#ifndef FLIGHT_SUPERVISOR_H
#define FLIGHT_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FLIGHT_CALIBRATING = 0,
    FLIGHT_DISARMED,
    FLIGHT_READY,
    FLIGHT_ARMED,
    FLIGHT_FAULT
} FlightState_t;

typedef struct {
    FlightState_t state;
    bool seen_arm_switch_low;
    uint32_t fault_count;
} FlightSupervisor_t;

void flight_supervisor_init(FlightSupervisor_t *s);
FlightState_t flight_supervisor_update(FlightSupervisor_t *s,
        bool calibration_done, bool estimator_healthy, bool imu_healthy,
        bool radio_fresh, bool deadline_ok, bool arm_switch_high,
        bool throttle_low);

#endif

