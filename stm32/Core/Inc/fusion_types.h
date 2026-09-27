#ifndef FUSION_TYPES_H
#define FUSION_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define FUSION_G_MSS 9.80665f
#define FUSION_PI    3.14159265358979323846f

typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quatf;

typedef struct {
    uint32_t timestamp_us;
    Vec3f accel_mps2;
    Vec3f gyro_rps;
    float temperature_c;
    bool valid;
} ImuSample_t;

typedef struct {
    Quatf q_wb;
    Vec3f gyro_bias_rps;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float nis_acc;
    uint32_t accel_reject_count;
    bool healthy;
} FusionOutput_t;

#endif

