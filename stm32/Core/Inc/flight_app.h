#ifndef FLIGHT_APP_H
#define FLIGHT_APP_H

#include "imu_calibration.h"
#include "sensor_fusion.h"
#include "attitude_controller.h"
#include "control_allocation.h"
#include "flight_supervisor.h"

typedef struct {
    bool arm_switch_high;
    bool radio_fresh;
    float throttle_01;
    float roll_setpoint_rad;
    float pitch_setpoint_rad;
    float yaw_rate_setpoint_rps;
} RcCommand_t;

typedef struct {
    FlightState_t state;
    FusionOutput_t fusion;
    MotorThrusts_t motors;
    bool calibration_done;
    bool deadline_ok;
} FlightAppOutput_t;

typedef struct {
    ImuCalibration_t imu_calibration;
    GyroCalibrator_t gyro_calibrator;
    Vec3f accel_sum;
    uint32_t accel_count;
    SensorFusion_t fusion;
    AttitudeController_t controller;
    QuadXAllocatorConfig_t allocator_cfg;
    FlightSupervisor_t supervisor;
    uint32_t last_control_timestamp_us;
    bool fusion_ready;
    FlightAppOutput_t output;
} FlightApp_t;

void flight_app_init(FlightApp_t *app, ImuCalibration_t imu_calibration,
                     uint32_t gyro_calibration_samples);
const FlightAppOutput_t *flight_app_step(FlightApp_t *app, ImuSample_t sample,
        RcCommand_t rc, bool deadline_ok);

#endif

