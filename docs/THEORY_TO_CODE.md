# Ánh xạ lý thuyết sang code

| Nội dung lý thuyết | Code/project |
|---|---|
| Observability: accel không quan sát yaw | `README.md`; yaw controller dùng rate, không dùng heading |
| Hệ trục sensor/body/world | `ImuAxisMap_t`, `README.md` |
| Anti-aliasing, ODR, DLPF | `mpu6050.c`: 200 Hz, DLPF_CFG=3 |
| Burst acquisition và timestamp | `mpu6050_start_read_dma()` + callback + `mpu6050_take_sample()` |
| Accel 6 mặt | `offline/accel_6point.py` |
| Gyro bias khi boot | `GyroCalibrator_t` |
| Bias gyro trôi online | state `gyro_bias_rps` trong `AttitudeEskf_t` |
| Allan variance | `offline/allan_variance.py` |
| Quaternion danh nghĩa | `quaternion.c` |
| Error state 6 chiều | `attitude_eskf.h/.c` |
| Predict phi tuyến | `attitude_eskf_predict()` |
| Jacobian/F và covariance | `Phi` và `congruence6()` trong predict |
| Measurement gravity | `attitude_eskf_update_accel()` |
| Adaptive R | biến `R` trong accel update |
| NIS chi-square gate | `vector_nis()` và `accel_nis_gate` |
| Sequential scalar update | ba lần `scalar_update()` |
| Joseph form | bên trong `scalar_update()` |
| Inject/reset error state | cuối `attitude_eskf_update_accel()` |
| Ép đối xứng P | `enforce_symmetry()` |
| NaN/Inf/covariance health | `attitude_eskf_check_health()` |
| Innovation validation | `offline/validate_nis.py` |
| Multi-rate/latency GPS | Chưa thuộc project attitude-only |
| ESKF 15 trạng thái | Nâng cấp sau khi attitude bay ổn |
| Failsafe | `flight_supervisor.c` |
| Control allocation | `control_allocation.c` |
| WCET/deadline | `g_flight_tick`, `g_deadline_miss` trong hướng dẫn tích hợp |

## Công thức state convention

Nominal quaternion quay body sang world:

```text
v_world = R(q_wb) v_body
q_true = q_nominal ⊗ Exp(dtheta)
```

Error dynamics dùng trong predict:

```text
dtheta_dot = -skew(omega) dtheta - dbg - ng
dbg_dot    = nbg
```

Accel measurement sau chuẩn hóa:

```text
z = accel / |accel|
h(q) = R(q_wb)^T [0 0 1]^T
H = [[h]x, 0]
```

Nếu đổi sang quaternion JPL, left-error hoặc NED/FRD thì dấu của `F`, `H`, inject/reset phải được suy ra lại; không được chỉ đổi vài dấu ở output Euler.
