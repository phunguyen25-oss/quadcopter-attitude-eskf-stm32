# Tích hợp STM32F411 + I2C DMA bằng CubeMX

Không thay toàn bộ `main.c`. Chỉ thêm vào các vùng `USER CODE`.

## 1. CubeMX

- I2C1 Fast Mode 400 kHz cho MPU6050.
- I2C1 → DMA Settings → thêm **I2C1_RX**, Mode Normal, Memory Increment Enable.
- Bật NVIC cho DMA stream mà CubeMX gán cho I2C1 RX.
- TIM5 free-running 32-bit, counter 1 MHz để tạo timestamp microsecond.
- Một timer/task 200 Hz chỉ để đánh thức vòng flight.
- Timer PWM hoặc DShot driver riêng cho bốn ESC.
- IWDG chỉ bật sau khi bring-up cơ bản ổn định.
- MPU6050 AD0 thấp, địa chỉ 0x68.

Ưu tiên ngắt DMA có thể cao hơn task/main nhưng không được vi phạm giới hạn FreeRTOS nếu callback gọi API RTOS. Driver này không gọi API RTOS trong callback.

## 2. Include — `USER CODE BEGIN Includes`

```c
#include "mpu6050.h"
#include "flight_app.h"
```

## 3. Biến — `USER CODE BEGIN PV`

```c
static Mpu6050_t g_mpu;
static FlightApp_t g_flight;
static volatile uint8_t g_flight_tick = 0U;
static volatile uint32_t g_deadline_miss = 0U;
static uint32_t g_last_good_sample_ms = 0U;
```

## 4. Timestamp wrap-safe

TIM5 chạy trực tiếp ở 1 MHz. Không dùng `DWT->CYCCNT / cycles_per_us` rồi mới trừ, vì phép chia trước làm mất tính wrap-safe.

```c
static uint32_t time_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}
```

Vì counter đã có đơn vị microsecond nên `uint32_t delta_us = now_us - previous_us;` vẫn đúng khi counter wrap.

## 5. Sau phần CubeMX init — `USER CODE BEGIN 2`

```c
HAL_TIM_Base_Start(&htim5);

if (mpu6050_init(&g_mpu, &hi2c1, time_us) != HAL_OK) {
    Error_Handler();
}

ImuCalibration_t cal = imu_calibration_default();

/* Ví dụ sensor X=body Y, sensor Y=-body X, sensor Z=body Z:
cal.sensor_to_body = (ImuAxisMap_t){{1, 0, 2}, {+1, -1, +1}};
*/

flight_app_init(&g_flight, cal, 400U); /* 400 mẫu = 2 s ở 200 Hz */
HAL_TIM_Base_Start_IT(&htimX);         /* timer flight 200 Hz */
```

## 6. Timer callback — chỉ đặt cờ

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIMX) {
        if (g_flight_tick || mpu6050_dma_busy(&g_mpu) ||
            mpu6050_sample_ready(&g_mpu)) {
            g_deadline_miss++;
        } else {
            g_flight_tick = 1U;
        }
    }

    /* Giữ HAL_IncTick() do CubeMX sinh nếu project dùng timer làm HAL timebase. */
}
```

Không gọi `HAL_I2C_Mem_Read_DMA()` trong timer ISR. HAL có thể thao tác state machine và lock nội bộ; khởi động DMA ở main/task dễ kiểm soát hơn.

## 7. Callback I2C DMA

```c
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    mpu6050_on_rx_complete_isr(&g_mpu, hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    mpu6050_on_error_isr(&g_mpu, hi2c);
}
```

Callback chỉ thay đổi state/counter. Không chạy calibration, quaternion, ESKF, PID hoặc `printf()` trong callback.

## 8. Vòng lặp chính hoặc FreeRTOS task

```c
/* Bước 1: timer đã đến hạn thì khởi động DMA ngoài ISR. */
if (g_flight_tick && !mpu6050_dma_busy(&g_mpu) &&
    !mpu6050_sample_ready(&g_mpu)) {
    g_flight_tick = 0U;
    if (mpu6050_start_read_dma(&g_mpu) != HAL_OK) {
        /* Driver tự tăng read_fail_count; không dùng lại mẫu cũ. */
    }
}

/* Bước 2: chỉ xử lý khi DMA đã hoàn tất một mẫu mới. */
ImuSample_t imu;
if (mpu6050_take_sample(&g_mpu, &imu)) {
    RcCommand_t rc = {
        .arm_switch_high = radio_arm_switch(),
        .radio_fresh = radio_age_ms() < 100U,
        .throttle_01 = radio_throttle_01(),
        .roll_setpoint_rad = radio_roll_rad(),
        .pitch_setpoint_rad = radio_pitch_rad(),
        .yaw_rate_setpoint_rps = radio_yaw_rate_rps()
    };

    bool deadline_ok = (g_deadline_miss == 0U);
    const FlightAppOutput_t *out = flight_app_step(&g_flight, imu, rc, deadline_ok);

    if (out->state == FLIGHT_ARMED) {
        motor_write_thrusts(out->motors.thrust_n); /* hàm riêng của project */
    } else {
        motor_stop_all();
    }

    g_last_good_sample_ms = HAL_GetTick();
    HAL_IWDG_Refresh(&hiwdg); /* Xóa nếu CubeMX chưa bật IWDG. */
}

/* Mất dữ liệu >100 ms: khóa motor; không tiếp tục giữ lệnh PWM cũ. */
if ((HAL_GetTick() - g_last_good_sample_ms) > 100U) {
    motor_stop_all();
}
```

Nếu dùng FreeRTOS, đặt hai bước trên trong Controller Task. Không cần tạo Sensor Task riêng cho một MPU6050; callback DMA đã cung cấp event hoàn tất.

## 9. State machine DMA

```text
IDLE
  | mpu6050_start_read_dma()
  v
BUSY
  | HAL_I2C_MemRxCpltCallback
  v
SAMPLE_READY
  | mpu6050_take_sample()
  v
IDLE

BUSY -- HAL_I2C_ErrorCallback --> IDLE + fail counter
```

Driver không cho khởi động DMA mới khi mẫu cũ chưa được lấy. Việc này ngăn DMA ghi đè buffer mà ESKF đang đọc.

## 10. Thrust sang PWM

Không map tuyến tính tùy ý rồi bay. Đo bảng `PWM -> thrust` bằng đúng motor, ESC, pin và cánh; nội suy ngược bảng để viết `motor_write_thrusts()`.

## 11. Telemetry nên log

```text
t_us, ax,ay,az, gx,gy,gz,
roll,pitch,yaw,bgx,bgy,bgz,
nis_acc,accel_reject_count,
imu_fail_count,dma_overrun_count,deadline_miss,
flight_state,allocator_saturated,torque_scale
```

## 12. Gate trước khi gắn cánh

- Axis/sign đúng cả ba trục.
- Accel 6 mặt đã hiệu chuẩn.
- Giữ yên 2 giây chỉ để khởi tạo gyro bias và attitude; không thay thế accel 6 mặt.
- Gyro calibration không hoàn tất khi cầm board rung.
- DMA chạy 10 phút không overrun và không lỗi I2C.
- Rút SDA/SCL phải làm motor bị khóa, không tiếp tục dùng mẫu cũ.
- 10 phút tĩnh không NaN/Inf và không vào `FLIGHT_FAULT`.
- Mixer test từng torque với motor bị vô hiệu hóa.
- Đo WCET; vòng phải xong trước 5 ms với margin ít nhất 50%.
