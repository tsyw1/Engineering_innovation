#include "JY901S_ACCEL_VELOCITY.h"

#include <math.h>
#include <string.h>

static float jy901s_accel_velocity_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t jy901s_accel_velocity_config_valid(
    const jy901s_accel_velocity_config_t *config)
{
    uint8_t row;
    uint8_t column;

    if ((config == NULL) || (config->gravity_mps2 <= 0.0f) ||
        (config->calibration_samples == 0U) ||
        (config->deadband_mps2 < 0.0f) ||
        (config->stationary_accel_mps2 <= 0.0f) ||
        (config->stationary_gyro_rad_s <= 0.0f) ||
        (config->acceleration_filter_alpha <= 0.0f) ||
        (config->acceleration_filter_alpha > 1.0f))
    {
        return 0U;
    }

    for (row = 0U; row < 2U; row++)
    {
        for (column = 0U; column < 3U; column++)
        {
            if (!isfinite(config->vehicle_from_sensor[row][column]))
            {
                return 0U;
            }
        }
    }

    return 1U;
}

static void jy901s_accel_velocity_default_config(
    jy901s_accel_velocity_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->gravity_mps2 = JY901S_ACCEL_VELOCITY_DEFAULT_GRAVITY_MPS2;
    config->calibration_samples =
        JY901S_ACCEL_VELOCITY_DEFAULT_CALIBRATION_SAMPLES;
    config->deadband_mps2 = JY901S_ACCEL_VELOCITY_DEFAULT_DEADBAND_MPS2;
    config->stationary_accel_mps2 =
        JY901S_ACCEL_VELOCITY_DEFAULT_STATIONARY_ACCEL_MPS2;
    config->stationary_gyro_rad_s =
        JY901S_ACCEL_VELOCITY_DEFAULT_STATIONARY_GYRO_RAD_S;
    config->acceleration_filter_alpha =
        JY901S_ACCEL_VELOCITY_DEFAULT_ACCEL_FILTER_ALPHA;
    config->vehicle_from_sensor[0][0] = 1.0f;
    config->vehicle_from_sensor[1][1] = 1.0f;
}

/* 根据 JY901S 的横滚/俯仰角，计算重力在传感器坐标系的分量。 */
static void jy901s_accel_velocity_gravity_sensor(
    const jy901s_accel_velocity_config_t *config,
    const jy901s_data_t *data,
    float gravity_sensor_mps2[3])
{
    float roll_rad = data->angle_deg[0] * JY901S_DEG_TO_RAD;
    float pitch_rad = data->angle_deg[1] * JY901S_DEG_TO_RAD;
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);

    /* 假定水平静止时 JY901S 的 Z 轴输出约为 +1g。 */
    gravity_sensor_mps2[0] = -config->gravity_mps2 * sin_pitch;
    gravity_sensor_mps2[1] = config->gravity_mps2 * sin_roll * cos_pitch;
    gravity_sensor_mps2[2] = config->gravity_mps2 * cos_roll * cos_pitch;
}

static void jy901s_accel_velocity_compensate_acceleration(
    jy901s_accel_velocity_t *solver,
    const jy901s_data_t *data,
    float linear_sensor_mps2[3])
{
    float gravity_sensor_mps2[3];
    uint8_t axis;

    jy901s_accel_velocity_gravity_sensor(&solver->config,
                                         data,
                                         gravity_sensor_mps2);
    for (axis = 0U; axis < 3U; axis++)
    {
        solver->gravity_sensor_mps2[axis] = gravity_sensor_mps2[axis];
        linear_sensor_mps2[axis] =
            data->acceleration_mps2[axis] -
            gravity_sensor_mps2[axis] -
            solver->acceleration_bias_sensor_mps2[axis];
        solver->acceleration_sensor_mps2[axis] = linear_sensor_mps2[axis];
    }
}

static void jy901s_accel_velocity_to_vehicle(
    const jy901s_accel_velocity_t *solver,
    const float linear_sensor_mps2[3],
    float linear_vehicle_mps2[2])
{
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < 2U; row++)
    {
        linear_vehicle_mps2[row] = 0.0f;
        for (column = 0U; column < 3U; column++)
        {
            linear_vehicle_mps2[row] +=
                solver->config.vehicle_from_sensor[row][column] *
                linear_sensor_mps2[column];
        }
    }
}

static void jy901s_accel_velocity_apply_deadband(
    const jy901s_accel_velocity_t *solver,
    float acceleration_vehicle_mps2[2])
{
    uint8_t axis;

    for (axis = 0U; axis < 2U; axis++)
    {
        if (jy901s_accel_velocity_abs(acceleration_vehicle_mps2[axis]) <
            solver->config.deadband_mps2)
        {
            acceleration_vehicle_mps2[axis] = 0.0f;
        }
    }
}

static uint8_t jy901s_accel_velocity_is_stationary(
    const jy901s_accel_velocity_t *solver,
    const jy901s_data_t *data,
    const float linear_sensor_mps2[3],
    const float acceleration_vehicle_mps2[2])
{
    float gyro_norm;
    float acceleration_norm;

    acceleration_norm = sqrtf(acceleration_vehicle_mps2[0] *
                              acceleration_vehicle_mps2[0] +
                              acceleration_vehicle_mps2[1] *
                              acceleration_vehicle_mps2[1]);
    gyro_norm = sqrtf(data->angular_velocity_rads[0] *
                      data->angular_velocity_rads[0] +
                      data->angular_velocity_rads[1] *
                      data->angular_velocity_rads[1] +
                      data->angular_velocity_rads[2] *
                      data->angular_velocity_rads[2]);

    return (acceleration_norm <= solver->config.stationary_accel_mps2) &&
           (jy901s_accel_velocity_abs(linear_sensor_mps2[2]) <=
            solver->config.stationary_accel_mps2) &&
           (gyro_norm <= solver->config.stationary_gyro_rad_s);
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_init(
    jy901s_accel_velocity_t *solver,
    const jy901s_t *sensor)
{
    if ((solver == NULL) || (sensor == NULL))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }

    memset(solver, 0, sizeof(*solver));
    solver->sensor = sensor;
    jy901s_accel_velocity_default_config(&solver->config);
    solver->valid = 1U;
    return JY901S_ACCEL_VELOCITY_OK;
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_set_config(
    jy901s_accel_velocity_t *solver,
    const jy901s_accel_velocity_config_t *config)
{
    if ((solver == NULL) || (solver->valid == 0U) ||
        (jy901s_accel_velocity_config_valid(config) == 0U))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }

    solver->config = *config;
    jy901s_accel_velocity_reset(solver);
    return JY901S_ACCEL_VELOCITY_OK;
}

void jy901s_accel_velocity_reset(jy901s_accel_velocity_t *solver)
{
    const jy901s_t *sensor;
    jy901s_accel_velocity_config_t config;

    if (solver == NULL)
    {
        return;
    }

    sensor = solver->sensor;
    config = solver->config;
    memset(solver, 0, sizeof(*solver));
    solver->sensor = sensor;
    solver->config = config;
    solver->valid = 1U;
}

void jy901s_accel_velocity_zero_velocity_update(
    jy901s_accel_velocity_t *solver)
{
    if ((solver == NULL) || (solver->valid == 0U))
    {
        return;
    }

    solver->velocity_vehicle_mps[0] = 0.0f;
    solver->velocity_vehicle_mps[1] = 0.0f;
    solver->filtered_acceleration_vehicle_mps2[0] = 0.0f;
    solver->filtered_acceleration_vehicle_mps2[1] = 0.0f;
    solver->acceleration_filter_initialized = 0U;
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_update(
    jy901s_accel_velocity_t *solver,
    const jy901s_data_t *data,
    uint32_t now_ms)
{
    float linear_sensor_mps2[3];
    float linear_vehicle_mps2[2];
    float dt_s;
    float raw_linear_sensor_mps2;
    uint8_t axis;

    if ((solver == NULL) || (data == NULL) || (solver->valid == 0U) ||
        (solver->sensor == NULL))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }

    jy901s_accel_velocity_compensate_acceleration(solver,
                                                  data,
                                                  linear_sensor_mps2);

    if (solver->calibrated == 0U)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            /* 当前 residual + 旧 bias = 未校准的线性加速度测量值。 */
            raw_linear_sensor_mps2 =
                linear_sensor_mps2[axis] +
                solver->acceleration_bias_sensor_mps2[axis];
            solver->acceleration_bias_sensor_mps2[axis] =
                (solver->acceleration_bias_sensor_mps2[axis] *
                 (float)solver->calibration_count +
                 raw_linear_sensor_mps2) /
                (float)(solver->calibration_count + 1U);
        }
        solver->calibration_count++;
        if (solver->calibration_count >= solver->config.calibration_samples)
        {
            solver->calibrated = 1U;
            solver->last_update_ms = now_ms;
        }
        return JY901S_ACCEL_VELOCITY_CALIBRATING;
    }

    if (now_ms == solver->last_update_ms)
    {
        return JY901S_ACCEL_VELOCITY_TIME_ERROR;
    }

    dt_s = (float)(now_ms - solver->last_update_ms) * 0.001f;
    solver->last_update_ms = now_ms;
    if ((dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        solver->velocity_vehicle_mps[0] = 0.0f;
        solver->velocity_vehicle_mps[1] = 0.0f;
        return JY901S_ACCEL_VELOCITY_TIME_ERROR;
    }

    /* 重新用已完成的零偏计算当前线性加速度。 */
    jy901s_accel_velocity_compensate_acceleration(solver,
                                                  data,
                                                  linear_sensor_mps2);
    jy901s_accel_velocity_to_vehicle(solver,
                                     linear_sensor_mps2,
                                     linear_vehicle_mps2);
    jy901s_accel_velocity_apply_deadband(solver, linear_vehicle_mps2);
    solver->acceleration_vehicle_mps2[0] = linear_vehicle_mps2[0];
    solver->acceleration_vehicle_mps2[1] = linear_vehicle_mps2[1];

    if (solver->acceleration_filter_initialized == 0U)
    {
        solver->filtered_acceleration_vehicle_mps2[0] =
            linear_vehicle_mps2[0];
        solver->filtered_acceleration_vehicle_mps2[1] =
            linear_vehicle_mps2[1];
        solver->acceleration_filter_initialized = 1U;
    }
    else
    {
        float alpha = solver->config.acceleration_filter_alpha;
        float one_minus_alpha = 1.0f - alpha;

        solver->filtered_acceleration_vehicle_mps2[0] =
            one_minus_alpha * solver->filtered_acceleration_vehicle_mps2[0] +
            alpha * linear_vehicle_mps2[0];
        solver->filtered_acceleration_vehicle_mps2[1] =
            one_minus_alpha * solver->filtered_acceleration_vehicle_mps2[1] +
            alpha * linear_vehicle_mps2[1];
    }

    solver->velocity_vehicle_mps[0] +=
        solver->filtered_acceleration_vehicle_mps2[0] * dt_s;
    solver->velocity_vehicle_mps[1] +=
        solver->filtered_acceleration_vehicle_mps2[1] * dt_s;

    solver->stationary = jy901s_accel_velocity_is_stationary(
        solver, data, linear_sensor_mps2, linear_vehicle_mps2);
    /*
     * 不能仅凭 IMU 的瞬时加速度和角速度判断小车静止：匀速运动时，
     * 这两个量同样接近零。零速修正必须由编码器或上层运动状态确认，
     * 否则会把匀速阶段的积分速度错误清零。
     */

    return JY901S_ACCEL_VELOCITY_OK;
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_get(
    const jy901s_accel_velocity_t *solver,
    float *vx_mps,
    float *vy_mps)
{
    if ((solver == NULL) || (vx_mps == NULL) || (vy_mps == NULL) ||
        (solver->valid == 0U))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }
    if (solver->calibrated == 0U)
    {
        return JY901S_ACCEL_VELOCITY_NOT_READY;
    }

    *vx_mps = solver->velocity_vehicle_mps[0];
    *vy_mps = solver->velocity_vehicle_mps[1];
    return JY901S_ACCEL_VELOCITY_OK;
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_get_acceleration(
    const jy901s_accel_velocity_t *solver,
    float *ax_mps2,
    float *ay_mps2)
{
    if ((solver == NULL) || (ax_mps2 == NULL) || (ay_mps2 == NULL) ||
        (solver->valid == 0U))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }
    if (solver->calibrated == 0U)
    {
        return JY901S_ACCEL_VELOCITY_NOT_READY;
    }

    *ax_mps2 = solver->acceleration_vehicle_mps2[0];
    *ay_mps2 = solver->acceleration_vehicle_mps2[1];
    return JY901S_ACCEL_VELOCITY_OK;
}

jy901s_accel_velocity_result_t jy901s_accel_velocity_get_filtered_acceleration(
    const jy901s_accel_velocity_t *solver,
    float *ax_mps2,
    float *ay_mps2)
{
    if ((solver == NULL) || (ax_mps2 == NULL) || (ay_mps2 == NULL) ||
        (solver->valid == 0U))
    {
        return JY901S_ACCEL_VELOCITY_INVALID_PARAM;
    }
    if (solver->calibrated == 0U)
    {
        return JY901S_ACCEL_VELOCITY_NOT_READY;
    }

    *ax_mps2 = solver->filtered_acceleration_vehicle_mps2[0];
    *ay_mps2 = solver->filtered_acceleration_vehicle_mps2[1];
    return JY901S_ACCEL_VELOCITY_OK;
}

uint8_t jy901s_accel_velocity_is_calibrated(
    const jy901s_accel_velocity_t *solver)
{
    if (solver == NULL)
    {
        return 0U;
    }

    return solver->calibrated;
}
