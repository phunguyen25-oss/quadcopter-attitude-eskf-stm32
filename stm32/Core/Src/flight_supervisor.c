#include "flight_supervisor.h"

void flight_supervisor_init(FlightSupervisor_t *s)
{
    s->state = FLIGHT_CALIBRATING;
    s->seen_arm_switch_low = false;
    s->fault_count = 0U;
}

FlightState_t flight_supervisor_update(FlightSupervisor_t *s,
        bool calibrated, bool estimator_ok, bool imu_ok, bool radio_ok,
        bool deadline_ok, bool arm_high, bool throttle_low)
{
    bool system_ok = calibrated && estimator_ok && imu_ok && radio_ok && deadline_ok;
    if (!arm_high) s->seen_arm_switch_low = true;

    switch (s->state) {
    case FLIGHT_CALIBRATING:
        if (calibrated) s->state = system_ok ? FLIGHT_DISARMED : FLIGHT_FAULT;
        break;
    case FLIGHT_DISARMED:
        if (!system_ok) s->state = FLIGHT_FAULT;
        else if (s->seen_arm_switch_low) s->state = FLIGHT_READY;
        break;
    case FLIGHT_READY:
        if (!system_ok) s->state = FLIGHT_FAULT;
        else if (arm_high && throttle_low && s->seen_arm_switch_low) s->state = FLIGHT_ARMED;
        break;
    case FLIGHT_ARMED:
        if (!system_ok) {
            s->state = FLIGHT_FAULT;
            s->fault_count++;
        } else if (!arm_high) {
            s->state = FLIGHT_DISARMED;
        }
        break;
    case FLIGHT_FAULT:
        /* Fault chỉ được clear khi pilot hạ arm switch và toàn hệ khỏe lại. */
        if (!arm_high && system_ok) {
            s->state = FLIGHT_DISARMED;
            s->seen_arm_switch_low = true;
        }
        break;
    default:
        s->state = FLIGHT_FAULT;
        break;
    }
    return s->state;
}
