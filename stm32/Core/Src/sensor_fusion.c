#include "sensor_fusion.h"
#include "quaternion.h"
#include <string.h>

static void publish_output(SensorFusion_t *sf)
{
    sf->output.q_wb = sf->eskf.q_wb;
    sf->output.gyro_bias_rps = sf->eskf.gyro_bias_rps;
    sf->output.nis_acc = sf->eskf.nis_acc;
    sf->output.accel_reject_count = sf->eskf.accel_reject_count;
    sf->output.healthy = sf->eskf.healthy && sf->invalid_imu_streak < 5U;
    quat_to_euler_zyx(sf->eskf.q_wb, &sf->output.roll_rad,
                      &sf->output.pitch_rad, &sf->output.yaw_rad);
}

bool sensor_fusion_init(SensorFusion_t *sf, Vec3f accel, Vec3f gyro_bias,
                        AttitudeEskfConfig_t cfg)
{
    memset(sf, 0, sizeof(*sf));
    if (!attitude_eskf_init(&sf->eskf, accel, gyro_bias, cfg)) return false;
    publish_output(sf);
    return true;
}

bool sensor_fusion_step(SensorFusion_t *sf, const ImuSample_t *s)
{
    if (!s->valid) {
        sf->invalid_imu_count++;
        sf->invalid_imu_streak++;
        publish_output(sf);
        return false; /* Không dùng lại dữ liệu cũ. */
    }
    sf->invalid_imu_streak = 0U;
    if (sf->last_timestamp_us == 0U) {
        sf->last_timestamp_us = s->timestamp_us;
        attitude_eskf_update_accel(&sf->eskf, s->accel_mps2);
        publish_output(sf);
        return sf->output.healthy;
    }
    uint32_t delta_us = s->timestamp_us - sf->last_timestamp_us; /* wrap-safe unsigned */
    sf->last_timestamp_us = s->timestamp_us;
    float dt = 1.0e-6f*(float)delta_us;
    if (dt < 0.0002f || dt > 0.02f) {
        sf->timing_fault_count++;
        sf->eskf.healthy = false; /* Không clamp dt rồi âm thầm under-integrate. */
        publish_output(sf);
        return false;
    }
    if (!attitude_eskf_predict(&sf->eskf, s->gyro_rps, dt)) {
        publish_output(sf);
        return false;
    }
    (void)attitude_eskf_update_accel(&sf->eskf, s->accel_mps2);
    publish_output(sf);
    return sf->output.healthy;
}

const FusionOutput_t *sensor_fusion_output(const SensorFusion_t *sf)
{
    return &sf->output;
}

