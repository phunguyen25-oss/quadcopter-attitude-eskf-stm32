#include "attitude_controller.h"
#include <math.h>
#include <string.h>

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

AttitudeControllerConfig_t attitude_controller_default_config(void)
{
    AttitudeControllerConfig_t c = {
        /* Giá trị chỉ để software bring-up; phải tune với mô hình và test có dây. */
        .angle_kp = 4.0f,
        .rate_kp = {0.05f, 0.05f, 0.03f},
        .rate_ki = {0.02f, 0.02f, 0.01f},
        .rate_kd = {0.001f, 0.001f, 0.0f},
        .max_rate_rps = 3.5f,
        .max_integral = 0.3f,
        .max_torque_nm = {0.5f, 0.5f, 0.2f}
    };
    return c;
}

void attitude_controller_init(AttitudeController_t *c, AttitudeControllerConfig_t cfg)
{
    memset(c, 0, sizeof(*c));
    c->cfg = cfg;
}

void attitude_controller_reset(AttitudeController_t *c, Vec3f gyro)
{
    memset(c->integral, 0, sizeof(c->integral));
    c->previous_rate[0] = gyro.x;
    c->previous_rate[1] = gyro.y;
    c->previous_rate[2] = gyro.z;
    c->initialized = true;
}

Vec3f attitude_controller_step(AttitudeController_t *c, AttitudeSetpoint_t sp,
                               const FusionOutput_t *a, Vec3f gyro, float dt)
{
    if (!c->initialized) attitude_controller_reset(c, gyro);
    float desired_rate[3] = {
        c->cfg.angle_kp*(sp.roll_rad-a->roll_rad),
        c->cfg.angle_kp*(sp.pitch_rad-a->pitch_rad),
        sp.yaw_rate_rps /* MPU6050 không giữ yaw tuyệt đối. */
    };
    float measured[3] = {gyro.x, gyro.y, gyro.z};
    float torque[3];
    for (int i=0; i<3; ++i) {
        desired_rate[i] = clampf_local(desired_rate[i], -c->cfg.max_rate_rps, c->cfg.max_rate_rps);
        float error = desired_rate[i]-measured[i];
        c->integral[i] = clampf_local(c->integral[i]+error*dt,
                                      -c->cfg.max_integral, c->cfg.max_integral);
        /* Derivative on measurement tránh derivative kick khi đổi setpoint. */
        float rate_dot = (measured[i]-c->previous_rate[i])/dt;
        torque[i] = c->cfg.rate_kp[i]*error + c->cfg.rate_ki[i]*c->integral[i]
                    - c->cfg.rate_kd[i]*rate_dot;
        torque[i] = clampf_local(torque[i], -c->cfg.max_torque_nm[i], c->cfg.max_torque_nm[i]);
        c->previous_rate[i] = measured[i];
    }
    return (Vec3f){torque[0], torque[1], torque[2]};
}

