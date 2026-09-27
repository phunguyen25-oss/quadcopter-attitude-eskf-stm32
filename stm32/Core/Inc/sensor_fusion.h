#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "attitude_eskf.h"

typedef struct {
    AttitudeEskf_t eskf;
    uint32_t last_timestamp_us;
    uint32_t invalid_imu_count;
    uint16_t invalid_imu_streak;
    uint32_t timing_fault_count;
    FusionOutput_t output;
} SensorFusion_t;

bool sensor_fusion_init(SensorFusion_t *sf, Vec3f stationary_accel,
                        Vec3f gyro_bias_rps, AttitudeEskfConfig_t cfg);
bool sensor_fusion_step(SensorFusion_t *sf, const ImuSample_t *sample);
const FusionOutput_t *sensor_fusion_output(const SensorFusion_t *sf);

#endif

