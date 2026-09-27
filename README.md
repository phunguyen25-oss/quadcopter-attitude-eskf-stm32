# Quadcopter Attitude ESKF on STM32F411

> **Status: Work in progress.** The algorithm core builds and passes host tests on a PC.
> It has **not** been flown, and the default noise and calibration values are placeholders.

Attitude estimation and inner-loop control for an X-configuration quadcopter.
Target hardware: STM32F411 with an MPU6050 IMU over I2C (DMA), running at 200 Hz.

## What it does

| Layer | Module | Algorithm |
|---|---|---|
| Sensor driver | `mpu6050.c` | 14-byte burst read over I2C DMA, timestamping, non-blocking state machine. 200 Hz ODR, DLPF_CFG=3, gyro ±500 dps, accel ±4 g |
| Calibration | `imu_calibration.c` | Signed axis permutation (sensor → body), accel bias/scale, gyro bias estimated at boot from stationary samples |
| Estimator | `attitude_eskf.c`, `quaternion.c` | **Error-state Kalman filter with 6 error states** (δθ, δb_g). Nominal state is a Hamilton quaternion plus gyro bias. Gyro-driven prediction; accelerometer gravity-direction update with adaptive R, accel-norm gate, χ² NIS gate (16.27, 3 DoF, 99.9%), sequential scalar updates in Joseph form, inject/reset, NaN/covariance health checks |
| Fusion wrapper | `sensor_fusion.c` | Computes dt from timestamps, runs the ESKF, outputs quaternion, roll/pitch/yaw and health flags |
| Controller | `attitude_controller.c` | Cascade: outer angle P loop (roll/pitch) and inner rate PID with integral clamp and torque limits; yaw is rate-controlled |
| Allocation | `control_allocation.c` | Quad-X mixer from total thrust + 3-axis torque to 4 motor thrusts. When a motor would saturate, the torque is scaled down and a `saturated` flag is set |
| Supervisor | `flight_supervisor.c` | Arming state machine (CALIBRATING → DISARMED → READY → ARMED / FAULT). Disarms on estimator, IMU, radio or deadline faults |
| App | `flight_app.c` | Wires fusion, supervisor, controller and allocator together |

Yaw is only relative: the MPU6050 has no magnetometer, so heading is not observable.

Conventions: world frame ENU, body frame FLU, Hamilton quaternion `q_wb`, right-multiplied error `q_true = q_nom ⊗ Exp(δθ)`.

## Repository layout

```
stm32/Core/Inc, stm32/Core/Src   firmware modules (HAL only in mpu6050.c)
host_test/                       PC test harness with a stubbed HAL (test_main.c, stubs/main.h, Makefile)
offline/                         Python tools: 6-position accel calibration, Allan deviation, NIS validation
docs/                            architecture, CubeMX integration guide, theory-to-code map (in Vietnamese)
```

## How to run

**Host tests** (GCC; no hardware needed):

```bash
cd host_test
make test
```

Without `make`:

```bash
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Istubs -I../stm32/Core/Inc test_main.c ../stm32/Core/Src/*.c -lm -o test_runner && ./test_runner
```

The tests cover the MPU6050 DMA state machine, the quaternion math, ESKF gyro-bias convergence while stationary, the allocator, and the supervisor.

**Offline tools** (Python, `pip install -r offline/requirements.txt`):

```bash
python offline/accel_6point.py accel_six_pose.csv --output generated_imu_calibration.h
python offline/allan_variance.py gyro_static.csv --plot allan.png
python offline/validate_nis.py flight_log.csv
```

**Firmware:** create an STM32F411 CubeMX/HAL project, copy `stm32/Core/Inc` and `stm32/Core/Src` into it, and follow `docs/INTEGRATION_STM32F411.md`.
`generated_imu_calibration.h` holds neutral values until you replace it with the output of `accel_6point.py`.

## Results

Re-run on 2026-09-26 with GCC 15.2 (MSYS2):

- Host tests: `All host tests passed.` (with `-Wall -Wextra -Werror -pedantic`)
- Python scripts: syntax check only. The tools were not run, because no sensor logs are included in this repository.

## Not done yet

- No flight or bench test on real hardware
- Noise parameters are defaults, not identified from real IMU data
- No thrust-to-PWM/DShot mapping (the allocator outputs thrust in newtons)
- No GPS/barometer/magnetometer (see the separate 15-state ESKF repository)

## About this code

This code was written with an AI assistant at my request. I defined the problem and the requirements.
The verification results above were re-run on my own machine.

## License

MIT. See [LICENSE](LICENSE). This is research and learning code with no warranty; it is not certified for safety-critical use.
