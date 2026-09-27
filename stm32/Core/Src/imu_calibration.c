#include "imu_calibration.h"
#include "generated_imu_calibration.h"
#include <math.h>
#include <string.h>

static Vec3f axis_map(Vec3f v, ImuAxisMap_t m)
{
    float in[3] = {v.x, v.y, v.z};
    Vec3f out = {
        m.sign[0] * in[m.index[0]],
        m.sign[1] * in[m.index[1]],
        m.sign[2] * in[m.index[2]]
    };
    return out;
}

static float norm3(Vec3f v)
{
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

ImuCalibration_t imu_calibration_default(void)
{
    ImuCalibration_t c = {
        .accel_bias_mps2 = {IMU_ACC_BIAS_X, IMU_ACC_BIAS_Y, IMU_ACC_BIAS_Z},
        .accel_scale = {IMU_ACC_SCALE_X, IMU_ACC_SCALE_Y, IMU_ACC_SCALE_Z},
        .sensor_to_body = {{0, 1, 2}, {+1, +1, +1}}
    };
    return c;
}

void imu_apply_calibration(const ImuCalibration_t *c, ImuSample_t *s)
{
    Vec3f a = {
        (s->accel_mps2.x - c->accel_bias_mps2.x) * c->accel_scale.x,
        (s->accel_mps2.y - c->accel_bias_mps2.y) * c->accel_scale.y,
        (s->accel_mps2.z - c->accel_bias_mps2.z) * c->accel_scale.z
    };
    s->accel_mps2 = axis_map(a, c->sensor_to_body);
    /* Không trừ gyro bias ở đây: bias boot là giá trị khởi tạo state b_g của ESKF. */
    s->gyro_rps = axis_map(s->gyro_rps, c->sensor_to_body);
}

void gyro_calibrator_start(GyroCalibrator_t *c, uint32_t target_samples)
{
    memset(c, 0, sizeof(*c));
    c->target_samples = (target_samples < 100U) ? 100U : target_samples;
}

bool gyro_calibrator_push(GyroCalibrator_t *c, const ImuSample_t *s)
{
    if (!s->valid || c->complete) return c->complete;
    float a_norm = norm3(s->accel_mps2);
    float w_norm = norm3(s->gyro_rps);
    /* Giảm nguy cơ hiệu chuẩn khi đang chuyển động bằng gyro norm và |a|.
       Lưu ý vật lý: quay đều rất chậm quanh phương trọng lực vẫn có thể giống bias;
       người vận hành vẫn phải đặt drone đứng yên trong lúc boot. */
    if (fabsf(a_norm - FUSION_G_MSS) > 0.15f*FUSION_G_MSS || w_norm > 0.15f) {
        c->failed_motion = true;
        c->count = 0U;
        c->mean = (Vec3f){0};
        c->m2 = (Vec3f){0};
        return false;
    }

    c->failed_motion = false;
    c->count++;
    float n = (float)c->count;
    float values[3] = {s->gyro_rps.x, s->gyro_rps.y, s->gyro_rps.z};
    float *mean[3] = {&c->mean.x, &c->mean.y, &c->mean.z};
    float *m2[3] = {&c->m2.x, &c->m2.y, &c->m2.z};
    for (int i = 0; i < 3; ++i) {
        float d = values[i] - *mean[i];
        *mean[i] += d/n;
        *m2[i] += d*(values[i] - *mean[i]);
    }
    c->complete = (c->count >= c->target_samples);
    return c->complete;
}

bool gyro_calibrator_result(const GyroCalibrator_t *c, Vec3f *bias, Vec3f *std)
{
    if (!c->complete || c->count < 2U) return false;
    *bias = c->mean;
    std->x = sqrtf(c->m2.x/(float)(c->count - 1U));
    std->y = sqrtf(c->m2.y/(float)(c->count - 1U));
    std->z = sqrtf(c->m2.z/(float)(c->count - 1U));
    /* 0.02 rad/s ~ 1.15 deg/s: ngưỡng bring-up, nên siết sau khi có log thật. */
    return std->x < 0.02f && std->y < 0.02f && std->z < 0.02f;
}
