#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "quaternion.h"
#include "attitude_eskf.h"
#include "control_allocation.h"
#include "flight_supervisor.h"

static uint32_t fake_time_us_value;
static HAL_StatusTypeDef fake_dma_start_status = HAL_OK;

static uint32_t fake_time_us(void)
{
    return fake_time_us_value;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t address,
                                    uint16_t reg, uint16_t reg_size,
                                    uint8_t *data, uint16_t size,
                                    uint32_t timeout)
{
    (void)hi2c; (void)address; (void)reg; (void)reg_size;
    (void)data; (void)size; (void)timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
                                   uint16_t reg, uint16_t reg_size,
                                   uint8_t *data, uint16_t size,
                                   uint32_t timeout)
{
    (void)hi2c; (void)address; (void)reg_size; (void)timeout;
    if (reg == 0x75U && size == 1U) {
        data[0] = 0x68U;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c,
                                       uint16_t address, uint16_t reg,
                                       uint16_t reg_size, uint8_t *data,
                                       uint16_t size)
{
    (void)hi2c; (void)address; (void)reg; (void)reg_size;
    (void)data; (void)size;
    return fake_dma_start_status;
}

void HAL_Delay(uint32_t ms)
{
    (void)ms;
}

static int near(float a, float b, float tol)
{
    return fabsf(a-b) <= tol;
}

static void put_be_i16(uint8_t *p, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    p[0] = (uint8_t)(bits >> 8);
    p[1] = (uint8_t)bits;
}

static void test_mpu6050_dma(void)
{
    I2C_HandleTypeDef i2c = {0};
    I2C_HandleTypeDef other_i2c = {0};
    Mpu6050_t imu;
    ImuSample_t sample;

    fake_dma_start_status = HAL_OK;
    fake_time_us_value = 100000U;
    assert(mpu6050_init(&imu, &i2c, fake_time_us) == HAL_OK);
    assert(imu.initialized);
    assert(!mpu6050_dma_busy(&imu));
    assert(!mpu6050_sample_ready(&imu));

    assert(mpu6050_start_read_dma(&imu) == HAL_OK);
    assert(mpu6050_dma_busy(&imu));
    assert(imu.sample_timestamp_us == 100000U);

    /* Không được ghi đè buffer khi lần DMA trước chưa được xử lý. */
    assert(mpu6050_start_read_dma(&imu) == HAL_BUSY);
    assert(imu.dma_overrun_count == 1U);

    memset(imu.dma_rx, 0, sizeof(imu.dma_rx));
    put_be_i16(&imu.dma_rx[0], 8192);    /* +1 g */
    put_be_i16(&imu.dma_rx[2], -8192);   /* -1 g */
    put_be_i16(&imu.dma_rx[4], 0);
    put_be_i16(&imu.dma_rx[6], 0);       /* 36.53 degC */
    put_be_i16(&imu.dma_rx[8], 655);     /* +10 deg/s */
    put_be_i16(&imu.dma_rx[10], -655);   /* -10 deg/s */
    put_be_i16(&imu.dma_rx[12], 0);

    /* Callback của I2C khác không được làm đổi trạng thái driver. */
    mpu6050_on_rx_complete_isr(&imu, &other_i2c);
    assert(mpu6050_dma_busy(&imu));
    mpu6050_on_rx_complete_isr(&imu, &i2c);
    assert(mpu6050_sample_ready(&imu));

    assert(mpu6050_take_sample(&imu, &sample));
    assert(sample.valid);
    assert(sample.timestamp_us == 100000U);
    assert(near(sample.accel_mps2.x, FUSION_G_MSS, 1.0e-5f));
    assert(near(sample.accel_mps2.y, -FUSION_G_MSS, 1.0e-5f));
    assert(near(sample.temperature_c, 36.53f, 1.0e-5f));
    assert(near(sample.gyro_rps.x, 10.0f*FUSION_PI/180.0f, 1.0e-5f));
    assert(near(sample.gyro_rps.y, -10.0f*FUSION_PI/180.0f, 1.0e-5f));
    assert(!mpu6050_take_sample(&imu, &sample));

    /* Lỗi async phải trả state về IDLE và tăng bộ đếm sức khỏe. */
    fake_time_us_value = 200000U;
    assert(mpu6050_start_read_dma(&imu) == HAL_OK);
    mpu6050_on_error_isr(&imu, &i2c);
    assert(!mpu6050_dma_busy(&imu));
    assert(imu.transfer_failed);
    assert(imu.read_fail_count == 1U);
    assert(imu.read_fail_streak == 1U);

    /* HAL từ chối khởi động DMA cũng phải được ghi nhận. */
    fake_dma_start_status = HAL_ERROR;
    assert(mpu6050_start_read_dma(&imu) == HAL_ERROR);
    assert(imu.read_fail_count == 2U);
    assert(imu.read_fail_streak == 2U);
}

static void test_quaternion(void)
{
    Quatf q = quat_from_rotation_vector((Vec3f){0,0,FUSION_PI/2.0f});
    Vec3f y = quat_rotate(q, (Vec3f){1,0,0});
    assert(near(y.x, 0.0f, 1.0e-5f));
    assert(near(y.y, 1.0f, 1.0e-5f));
    assert(near(y.z, 0.0f, 1.0e-5f));
}

static void test_eskf_stationary_bias(void)
{
    AttitudeEskf_t f;
    AttitudeEskfConfig_t cfg = attitude_eskf_default_config();
    cfg.accel_nis_gate = 100.0f; /* test hội tụ, không test tuning gate */
    assert(attitude_eskf_init(&f, (Vec3f){0,0,FUSION_G_MSS}, (Vec3f){0,0,0}, cfg));
    for (int k=0; k<4000; ++k) {
        assert(attitude_eskf_predict(&f, (Vec3f){0.02f, -0.01f, 0.0f}, 0.005f));
        (void)attitude_eskf_update_accel(&f, (Vec3f){0,0,FUSION_G_MSS});
        assert(f.healthy);
    }
    float roll, pitch, yaw;
    quat_to_euler_zyx(f.q_wb, &roll, &pitch, &yaw);
    assert(fabsf(roll) < 0.03f);
    assert(fabsf(pitch) < 0.03f);
    assert(fabsf(f.gyro_bias_rps.x-0.02f) < 0.01f);
    assert(fabsf(f.gyro_bias_rps.y+0.01f) < 0.01f);
}

static void test_allocator(void)
{
    QuadXAllocatorConfig_t c = control_allocation_default_config();
    Vec3f cmd = {0.1f, -0.08f, 0.02f};
    MotorThrusts_t m = control_allocate_quad_x(c, 8.0f, cmd);
    assert(!m.saturated);
    float l = c.arm_length_m*0.70710678118f;
    float k = c.yaw_moment_per_thrust_m;
    float total = m.thrust_n[0]+m.thrust_n[1]+m.thrust_n[2]+m.thrust_n[3];
    float tx = l*(m.thrust_n[0]-m.thrust_n[1]-m.thrust_n[2]+m.thrust_n[3]);
    float ty = l*(-m.thrust_n[0]-m.thrust_n[1]+m.thrust_n[2]+m.thrust_n[3]);
    float tz = k*(-m.thrust_n[0]+m.thrust_n[1]-m.thrust_n[2]+m.thrust_n[3]);
    assert(near(total, 8.0f, 1.0e-5f));
    assert(near(tx, cmd.x, 1.0e-5f));
    assert(near(ty, cmd.y, 1.0e-5f));
    assert(near(tz, cmd.z, 1.0e-5f));
}

static void test_supervisor(void)
{
    FlightSupervisor_t s;
    flight_supervisor_init(&s);
    assert(flight_supervisor_update(&s, false,false,true,true,true,false,true)==FLIGHT_CALIBRATING);
    assert(flight_supervisor_update(&s, true,true,true,true,true,false,true)==FLIGHT_DISARMED);
    assert(flight_supervisor_update(&s, true,true,true,true,true,false,true)==FLIGHT_READY);
    assert(flight_supervisor_update(&s, true,true,true,true,true,true,true)==FLIGHT_ARMED);
    assert(flight_supervisor_update(&s, true,false,true,true,true,true,false)==FLIGHT_FAULT);
}

int main(void)
{
    test_mpu6050_dma();
    test_quaternion();
    test_eskf_stationary_bias();
    test_allocator();
    test_supervisor();
    puts("All host tests passed.");
    return 0;
}
