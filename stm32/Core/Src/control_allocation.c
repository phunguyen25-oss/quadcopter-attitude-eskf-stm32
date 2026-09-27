#include "control_allocation.h"
#include <math.h>

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

QuadXAllocatorConfig_t control_allocation_default_config(void)
{
    QuadXAllocatorConfig_t c = {
        .arm_length_m = 0.165f,
        .yaw_moment_per_thrust_m = 0.015f,
        .min_motor_thrust_n = 0.0f,
        .max_motor_thrust_n = 4.0f
    };
    return c;
}

MotorThrusts_t control_allocate_quad_x(QuadXAllocatorConfig_t c,
                                       float total, Vec3f tau)
{
    MotorThrusts_t out = {{0}, 1.0f, false};
    float l = c.arm_length_m*0.70710678118f; /* chiếu tay đòn quad-X */
    float k = c.yaw_moment_per_thrust_m;
    if (l <= 1.0e-5f || k <= 1.0e-5f || c.max_motor_thrust_n <= c.min_motor_thrust_n) {
        out.saturated = true;
        out.torque_scale = 0.0f;
        return out;
    }
    float base = clampf_local(0.25f*total, c.min_motor_thrust_n, c.max_motor_thrust_n);
    if (base != 0.25f*total) out.saturated = true;

    /* FLU, thrust +Z. M1 FL(CCW), M2 FR(CW), M3 RR(CCW), M4 RL(CW). */
    float d[4] = {
        +tau.x/(4.0f*l) - tau.y/(4.0f*l) - tau.z/(4.0f*k),
        -tau.x/(4.0f*l) - tau.y/(4.0f*l) + tau.z/(4.0f*k),
        -tau.x/(4.0f*l) + tau.y/(4.0f*l) - tau.z/(4.0f*k),
        +tau.x/(4.0f*l) + tau.y/(4.0f*l) + tau.z/(4.0f*k)
    };
    float alpha = 1.0f;
    for (int i=0; i<4; ++i) {
        if (d[i] > 0.0f) alpha = fminf(alpha, (c.max_motor_thrust_n-base)/d[i]);
        if (d[i] < 0.0f) alpha = fminf(alpha, (base-c.min_motor_thrust_n)/(-d[i]));
    }
    alpha = clampf_local(alpha, 0.0f, 1.0f);
    if (alpha < 0.9999f) out.saturated = true;
    out.torque_scale = alpha;
    for (int i=0; i<4; ++i)
        out.thrust_n[i] = clampf_local(base+alpha*d[i],
                                      c.min_motor_thrust_n, c.max_motor_thrust_n);
    return out;
}

