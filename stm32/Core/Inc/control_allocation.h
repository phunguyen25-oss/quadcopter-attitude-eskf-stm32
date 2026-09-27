#ifndef CONTROL_ALLOCATION_H
#define CONTROL_ALLOCATION_H

#include "fusion_types.h"

typedef struct {
    float arm_length_m;
    float yaw_moment_per_thrust_m; /* kappa = K_M/K_F [m] */
    float min_motor_thrust_n;
    float max_motor_thrust_n;
} QuadXAllocatorConfig_t;

typedef struct {
    float thrust_n[4];
    float torque_scale;
    bool saturated;
} MotorThrusts_t;

QuadXAllocatorConfig_t control_allocation_default_config(void);
MotorThrusts_t control_allocate_quad_x(QuadXAllocatorConfig_t cfg,
                                       float total_thrust_n, Vec3f torque_nm);

#endif

