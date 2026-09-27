# Kiến trúc ba phần

## Phần A — Offline trên PC

| Script | Đầu vào | Kết quả |
|---|---|---|
| `accel_6point.py` | Log 6 tư thế ±X, ±Y, ±Z | Bias/scale và file header C |
| `allan_variance.py` | Log gyro tĩnh dài | Allan deviation và noise gợi ý |
| `validate_nis.py` | Log `nis_acc`/innovation | Tỷ lệ chi-square và whiteness |

Các script không chạy trên STM32. Chúng tạo tham số cố định cho firmware.

## Phần B — Online trên STM32

| Module | Trách nhiệm |
|---|---|
| `mpu6050` | Cấu hình, burst-read I2C DMA, timestamp, state machine và I2C health |
| `imu_calibration` | Axis map, accel bias/scale, gyro boot calibration |
| `quaternion` | Toán quaternion độc lập phần cứng |
| `attitude_eskf` | Predict, accel update, NIS gate, Joseph, health |
| `sensor_fusion` | Tính `dt`, gọi ESKF, xuất Euler và trạng thái sức khỏe |

## Phần C — Tích hợp hệ thống

| Module | Trách nhiệm |
|---|---|
| `attitude_controller` | Angle-P ngoài + rate-PID trong |
| `control_allocation` | Phân phối thrust/moment sang bốn motor quad-X |
| `flight_supervisor` | Chuỗi arm an toàn, mất sensor/radio/deadline |
| `flight_app` | Nối fusion, supervisor, controller và allocator |

## Ranh giới an toàn

- Driver chỉ báo dữ liệu hợp lệ; không tự che giấu lỗi bằng dữ liệu cũ.
- Timer ISR chỉ đặt cờ; callback DMA chỉ đổi state. Chuyển raw sang SI và chạy ESKF ở main/task.
- ESKF chỉ ước lượng và báo health; không tự reset khi đang bay.
- Supervisor là nơi duy nhất cho phép hoặc cấm motor.
- Allocator không tự viết PWM; nó trả thrust và cờ saturation.
- Tầng motor phải chuyển thrust sang PWM bằng dữ liệu thrust-stand thực nghiệm.

## Trình tự phát triển

1. Test driver và trục khi tháo cánh.
2. Hiệu chuẩn accel/gyro.
3. Test ESKF tĩnh và nghiêng tay.
4. Test controller với motor vô hiệu hóa.
5. Test chiều motor và mixer từng trục.
6. Test motor không cánh để đo rung/nhiễu bus.
7. Tethered hover.
8. Chỉ sau đó mới thêm barometer/GPS/magnetometer.
