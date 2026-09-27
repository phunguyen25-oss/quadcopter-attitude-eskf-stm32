#include "flight_app.h"
#include <string.h>

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void flight_app_init(FlightApp_t *app, ImuCalibration_t imu_calibration,
                     uint32_t gyro_calibration_samples)
{
    memset(app, 0, sizeof(*app));
    app->imu_calibration = imu_calibration;
    gyro_calibrator_start(&app->gyro_calibrator, gyro_calibration_samples);
    attitude_controller_init(&app->controller, attitude_controller_default_config());
    app->allocator_cfg = control_allocation_default_config();
    flight_supervisor_init(&app->supervisor);
    app->output.state = FLIGHT_CALIBRATING;
}

const FlightAppOutput_t *flight_app_step(FlightApp_t *app, ImuSample_t sample,
        RcCommand_t rc, bool deadline_ok)
{
    imu_apply_calibration(&app->imu_calibration, &sample);

    if (!app->fusion_ready && sample.valid) {
        bool calibration_complete = gyro_calibrator_push(&app->gyro_calibrator, &sample);
        if (app->gyro_calibrator.count == 0U) {
            /* Calibrator vừa phát hiện chuyển động và reset cửa sổ. */
            app->accel_sum = (Vec3f){0};
            app->accel_count = 0U;
        } else {
            app->accel_sum.x += sample.accel_mps2.x;
            app->accel_sum.y += sample.accel_mps2.y;
            app->accel_sum.z += sample.accel_mps2.z;
            app->accel_count++;
        }
        if (calibration_complete) {
            Vec3f bias, std;
            bool good = gyro_calibrator_result(&app->gyro_calibrator, &bias, &std);
            if (good && app->accel_count > 0U) {
                float inv = 1.0f/(float)app->accel_count;
                Vec3f mean_acc = {app->accel_sum.x*inv, app->accel_sum.y*inv, app->accel_sum.z*inv};
                app->fusion_ready = sensor_fusion_init(&app->fusion, mean_acc, bias,
                                                       attitude_eskf_default_config());
                if (app->fusion_ready)
                    app->output.fusion = *sensor_fusion_output(&app->fusion);
            } else {
                /* Có chuyển động/rung: tự thu lại từ đầu, motor vẫn bị khóa. */
                gyro_calibrator_start(&app->gyro_calibrator,
                                      app->gyro_calibrator.target_samples);
                app->accel_sum = (Vec3f){0};
                app->accel_count = 0U;
            }
        }
    } else if (app->fusion_ready) {
        (void)sensor_fusion_step(&app->fusion, &sample);
        app->output.fusion = *sensor_fusion_output(&app->fusion);
    }

    bool estimator_ok = app->fusion_ready && app->output.fusion.healthy;
    bool imu_ok = sample.valid;
    bool throttle_low = clamp01(rc.throttle_01) < 0.05f;
    app->output.state = flight_supervisor_update(&app->supervisor,
        app->fusion_ready, estimator_ok, imu_ok, rc.radio_fresh, deadline_ok,
        rc.arm_switch_high, throttle_low);
    app->output.calibration_done = app->fusion_ready;
    app->output.deadline_ok = deadline_ok;

    Vec3f gyro_corrected = {
        sample.gyro_rps.x-app->output.fusion.gyro_bias_rps.x,
        sample.gyro_rps.y-app->output.fusion.gyro_bias_rps.y,
        sample.gyro_rps.z-app->output.fusion.gyro_bias_rps.z
    };

    if (app->output.state != FLIGHT_ARMED) {
        app->output.motors = (MotorThrusts_t){{0,0,0,0}, 1.0f, false};
        attitude_controller_reset(&app->controller, gyro_corrected);
        app->last_control_timestamp_us = sample.timestamp_us;
        return &app->output;
    }

    uint32_t du = sample.timestamp_us - app->last_control_timestamp_us;
    app->last_control_timestamp_us = sample.timestamp_us;
    float dt = 1.0e-6f*(float)du;
    if (dt < 0.0002f || dt > 0.02f) {
        app->output.state = FLIGHT_FAULT;
        app->supervisor.state = FLIGHT_FAULT;
        app->output.motors = (MotorThrusts_t){{0,0,0,0}, 0.0f, true};
        return &app->output;
    }

    AttitudeSetpoint_t sp = {rc.roll_setpoint_rad, rc.pitch_setpoint_rad,
                             rc.yaw_rate_setpoint_rps};
    Vec3f torque = attitude_controller_step(&app->controller, sp,
        &app->output.fusion, gyro_corrected, dt);
    float total_thrust = clamp01(rc.throttle_01) *
                         4.0f*app->allocator_cfg.max_motor_thrust_n;
    app->output.motors = control_allocate_quad_x(app->allocator_cfg,
                                                  total_thrust, torque);
    return &app->output;
}
