#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "fusion_types.h"

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rate_rps;
} AttitudeSetpoint_t;

typedef struct {
    float angle_kp;
    float rate_kp[3];
    float rate_ki[3];
    float rate_kd[3];
    float max_rate_rps;
    float max_integral;
    float max_torque_nm[3];
} AttitudeControllerConfig_t;

typedef struct {
    AttitudeControllerConfig_t cfg;
    float integral[3];
    float previous_rate[3];
    bool initialized;
} AttitudeController_t;

AttitudeControllerConfig_t attitude_controller_default_config(void);
void attitude_controller_init(AttitudeController_t *c, AttitudeControllerConfig_t cfg);
void attitude_controller_reset(AttitudeController_t *c, Vec3f gyro_rps);
Vec3f attitude_controller_step(AttitudeController_t *c, AttitudeSetpoint_t sp,
                               const FusionOutput_t *attitude, Vec3f gyro_rps,
                               float dt_s);

#endif

