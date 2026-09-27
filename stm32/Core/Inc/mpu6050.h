#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"       /* I2C_HandleTypeDef, HAL_StatusTypeDef */
#include "fusion_types.h"

typedef uint32_t (*MpuTimeUsFn)(void);

typedef enum {
    MPU6050_DMA_IDLE = 0,
    MPU6050_DMA_BUSY,
    MPU6050_DMA_SAMPLE_READY
} Mpu6050DmaState_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    MpuTimeUsFn time_us;
    uint16_t address_8bit;
    uint8_t dma_rx[14];
    volatile Mpu6050DmaState_t dma_state;
    volatile uint32_t sample_timestamp_us;
    volatile bool transfer_failed;
    uint32_t read_fail_count;
    uint16_t read_fail_streak;
    uint32_t dma_overrun_count;
    bool initialized;
} Mpu6050_t;

HAL_StatusTypeDef mpu6050_init(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c,
                               MpuTimeUsFn time_us);
HAL_StatusTypeDef mpu6050_start_read_dma(Mpu6050_t *imu);
void mpu6050_on_rx_complete_isr(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c);
void mpu6050_on_error_isr(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c);
bool mpu6050_sample_ready(const Mpu6050_t *imu);
bool mpu6050_dma_busy(const Mpu6050_t *imu);
bool mpu6050_take_sample(Mpu6050_t *imu, ImuSample_t *sample);

#endif
