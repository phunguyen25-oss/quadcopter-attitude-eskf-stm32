#ifndef ATTITUDE_ESKF_H
#define ATTITUDE_ESKF_H

#include "fusion_types.h"

#define ESKF_N 6

typedef struct {
    float gyro_noise_rps_sqrt_hz;
    float gyro_bias_rw_rps2_sqrt_hz;
    float accel_dir_variance;
    float accel_adaptive_lambda;
    float accel_norm_gate_mps2;
    float accel_nis_gate;
    uint16_t max_consecutive_rejects;
} AttitudeEskfConfig_t;

typedef struct {
    Quatf q_wb;
    Vec3f gyro_bias_rps;
    float P[ESKF_N][ESKF_N];
    AttitudeEskfConfig_t cfg;
    float nis_acc;
    uint32_t accel_reject_count;
    uint16_t consecutive_rejects;
    bool initialized;
    bool healthy;
} AttitudeEskf_t;

AttitudeEskfConfig_t attitude_eskf_default_config(void);
bool attitude_eskf_init(AttitudeEskf_t *f, Vec3f stationary_accel,
                        Vec3f initial_gyro_bias, AttitudeEskfConfig_t cfg);
bool attitude_eskf_predict(AttitudeEskf_t *f, Vec3f gyro_rps, float dt_s);
bool attitude_eskf_update_accel(AttitudeEskf_t *f, Vec3f accel_mps2);
bool attitude_eskf_check_health(AttitudeEskf_t *f);

#endif

