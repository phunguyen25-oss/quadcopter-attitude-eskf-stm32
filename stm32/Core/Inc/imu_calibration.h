#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "fusion_types.h"

typedef struct {
    /* body[i] = sign[i] * sensor[index[i]], signed permutation only. */
    uint8_t index[3];
    int8_t sign[3];
} ImuAxisMap_t;

typedef struct {
    Vec3f accel_bias_mps2;
    Vec3f accel_scale;
    ImuAxisMap_t sensor_to_body;
} ImuCalibration_t;

typedef struct {
    uint32_t target_samples;
    uint32_t count;
    Vec3f mean;
    Vec3f m2;
    bool failed_motion;
    bool complete;
} GyroCalibrator_t;

ImuCalibration_t imu_calibration_default(void);
void imu_apply_calibration(const ImuCalibration_t *cal, ImuSample_t *sample);
void gyro_calibrator_start(GyroCalibrator_t *c, uint32_t target_samples);
bool gyro_calibrator_push(GyroCalibrator_t *c, const ImuSample_t *sample);
bool gyro_calibrator_result(const GyroCalibrator_t *c, Vec3f *bias_rps, Vec3f *std_rps);

#endif
