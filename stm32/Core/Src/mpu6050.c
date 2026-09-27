#include "mpu6050.h"
#include <string.h>

#define MPU_ADDR_8BIT       (0x68U << 1)
#define REG_SMPLRT_DIV      0x19U
#define REG_CONFIG          0x1AU
#define REG_GYRO_CONFIG     0x1BU
#define REG_ACCEL_CONFIG    0x1CU
#define REG_ACCEL_XOUT_H    0x3BU
#define REG_PWR_MGMT_1      0x6BU
#define REG_WHO_AM_I        0x75U

static HAL_StatusTypeDef write_reg(Mpu6050_t *imu, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(imu->hi2c, imu->address_8bit, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U, 10U);
}

static int16_t be_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

HAL_StatusTypeDef mpu6050_init(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c,
                               MpuTimeUsFn time_us)
{
    memset(imu, 0, sizeof(*imu));
    imu->hi2c = hi2c;
    imu->time_us = time_us;
    imu->address_8bit = MPU_ADDR_8BIT;
    uint8_t who = 0U;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c, imu->address_8bit, REG_WHO_AM_I,
                                            I2C_MEMADD_SIZE_8BIT, &who, 1U, 10U);
    if (st != HAL_OK || who != 0x68U) return HAL_ERROR;

    /* 200 Hz sample, DLPF_CFG=3: gyro/accel ~44 Hz; gyro +-500 dps, accel +-4g. */
    if (write_reg(imu, REG_PWR_MGMT_1, 0x01U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50U);
    if (write_reg(imu, REG_CONFIG, 0x03U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, REG_SMPLRT_DIV, 0x04U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, REG_GYRO_CONFIG, 0x08U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, REG_ACCEL_CONFIG, 0x08U) != HAL_OK) return HAL_ERROR;
    imu->dma_state = MPU6050_DMA_IDLE;
    imu->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef mpu6050_start_read_dma(Mpu6050_t *imu)
{
    if (!imu->initialized || imu->time_us == NULL) return HAL_ERROR;
    if (imu->dma_state != MPU6050_DMA_IDLE) {
        imu->dma_overrun_count++;
        return HAL_BUSY;
    }

    /* Timestamp tại lúc yêu cầu đọc, không lấy ở callback hoàn tất DMA. */
    imu->sample_timestamp_us = imu->time_us();
    imu->transfer_failed = false;
    imu->dma_state = MPU6050_DMA_BUSY;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(imu->hi2c, imu->address_8bit,
                                                REG_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT,
                                                imu->dma_rx, sizeof(imu->dma_rx));
    if (st != HAL_OK) {
        imu->dma_state = MPU6050_DMA_IDLE;
        imu->transfer_failed = true;
        imu->read_fail_count++;
        imu->read_fail_streak++;
    }
    return st;
}

void mpu6050_on_rx_complete_isr(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c)
{
    if (imu->initialized && hi2c == imu->hi2c && imu->dma_state == MPU6050_DMA_BUSY) {
        /* ISR chỉ đổi state; không đổi đơn vị, không chạy filter tại đây. */
        imu->dma_state = MPU6050_DMA_SAMPLE_READY;
    }
}

void mpu6050_on_error_isr(Mpu6050_t *imu, I2C_HandleTypeDef *hi2c)
{
    if (imu->initialized && hi2c == imu->hi2c) {
        imu->dma_state = MPU6050_DMA_IDLE;
        imu->transfer_failed = true;
        imu->read_fail_count++;
        imu->read_fail_streak++;
    }
}

bool mpu6050_sample_ready(const Mpu6050_t *imu)
{
    return imu->dma_state == MPU6050_DMA_SAMPLE_READY;
}

bool mpu6050_dma_busy(const Mpu6050_t *imu)
{
    return imu->dma_state == MPU6050_DMA_BUSY;
}

bool mpu6050_take_sample(Mpu6050_t *imu, ImuSample_t *sample)
{
    if (imu->dma_state != MPU6050_DMA_SAMPLE_READY) return false;

    /* Không có DMA mới nào được khởi động khi SAMPLE_READY, nên buffer ổn định. */
    uint8_t b[14];
    memcpy(b, imu->dma_rx, sizeof(b));
    uint32_t timestamp_us = imu->sample_timestamp_us;
    imu->dma_state = MPU6050_DMA_IDLE;

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp_us;
    imu->read_fail_streak = 0U;
    const float acc_scale = FUSION_G_MSS/8192.0f;
    const float gyro_scale = (FUSION_PI/180.0f)/65.5f;
    sample->accel_mps2 = (Vec3f){be_i16(&b[0])*acc_scale,
                                 be_i16(&b[2])*acc_scale,
                                 be_i16(&b[4])*acc_scale};
    sample->temperature_c = be_i16(&b[6])/340.0f + 36.53f;
    sample->gyro_rps = (Vec3f){be_i16(&b[8])*gyro_scale,
                               be_i16(&b[10])*gyro_scale,
                               be_i16(&b[12])*gyro_scale};
    sample->valid = true;
    return true;
}
