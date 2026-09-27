//
// 麦轮底盘控制实现，使用 motor_can 原生 CAN 电机接口。
//

#include "chassis_mecanum.h"

#include <math.h>
#include <string.h>

#define MECANUM_PI_F 3.14159265358979323846f

static float mecanum_abs_f(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t mecanum_round_rpm(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static bool mecanum_config_valid(const mecanum_config_t *config)
{
    uint8_t index;
    motor_bus_t *bus;

    if ((config == NULL) || (config->wheel_radius_m <= 0.0f) ||
        (config->half_length_m < 0.0f) || (config->half_width_m < 0.0f) ||
        (config->pulses_per_rev == 0U) || (config->max_wheel_rpm == 0U) ||
        (config->max_wheel_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return false;
    }

    bus = NULL;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if ((config->motor[index] == NULL) ||
            (config->motor[index]->bus == NULL) ||
            ((config->direction_sign[index] != 1) &&
             (config->direction_sign[index] != -1)))
        {
            return false;
        }

        if (bus == NULL)
        {
            bus = config->motor[index]->bus;
        }
        else if (config->motor[index]->bus != bus)
        {
            return false;
        }
    }

    return true;
}

static mecanum_result_t mecanum_send_wheel_rpm(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t motor_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;
    motor_result_t result;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_rpm[index] = (int16_t)((int32_t)wheel_rpm[index] *
                                     chassis->config.direction_sign[index]);
    }

    result = motor_set_speeds_sync(chassis->config.motor,
                                   motor_rpm,
                                   MECANUM_WHEEL_COUNT,
                                   chassis->config.acceleration);
    if (result != MOTOR_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->target_wheel_rpm[index] = wheel_rpm[index];
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_init(mecanum_chassis_t *chassis,
                                      const mecanum_config_t *config)
{
    if ((chassis == NULL) || !mecanum_config_valid(config))
    {
        return MECANUM_INVALID_PARAM;
    }

    memset(chassis, 0, sizeof(*chassis));
    memcpy(&chassis->config, config, sizeof(chassis->config));
    mecanum_chassis_reset_odometry(chassis);
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_enable(mecanum_chassis_t *chassis,
                                        bool enable)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if (motor_enable(chassis->config.motor[index],
                         enable,
                         MOTOR_EXECUTE_NOW) != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if (motor_stop(chassis->config.motor[index],
                       MOTOR_EXECUTE_NOW) != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        chassis->target_wheel_rpm[index] = 0;
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_wheel_speed(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t limited_rpm[MECANUM_WHEEL_COUNT];
    int32_t max_abs = 0;
    uint8_t index;
    float scale = 1.0f;

    if ((chassis == NULL) || (wheel_rpm == NULL) ||
        !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int32_t value = wheel_rpm[index];
        int32_t abs_value = (value < 0) ? -value : value;

        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > (int32_t)chassis->config.max_wheel_rpm)
    {
        scale = (float)chassis->config.max_wheel_rpm / (float)max_abs;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        limited_rpm[index] = mecanum_round_rpm((float)wheel_rpm[index] *
                                               scale);
    }

    if (mecanum_send_wheel_rpm(chassis, limited_rpm) != MECANUM_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    return (scale < 1.0f) ? MECANUM_CLAMPED : MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_velocity(mecanum_chassis_t *chassis,
                                              float vx_mps,
                                              float vy_mps,
                                              float wz_rad_s)
{
    float wheel_rad_s[MECANUM_WHEEL_COUNT];
    float max_abs = 0.0f;
    float geometry_factor;
    float scale = 1.0f;
    int16_t wheel_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    geometry_factor = chassis->config.half_length_m +
                      chassis->config.half_width_m;

    /* 标准 X 型麦轮运动学，轮速单位为 rad/s。 */
    wheel_rad_s[MECANUM_WHEEL_FL] = vx_mps - vy_mps -
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_FR] = vx_mps + vy_mps +
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RL] = vx_mps + vy_mps -
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RR] = vx_mps - vy_mps +
                                    geometry_factor * wz_rad_s;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float abs_value = mecanum_abs_f(wheel_rad_s[index]);
        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > 0.0f)
    {
        float max_rpm = (max_abs / chassis->config.wheel_radius_m) *
                        60.0f / (2.0f * MECANUM_PI_F);
        if (max_rpm > (float)chassis->config.max_wheel_rpm)
        {
            scale = (float)chassis->config.max_wheel_rpm / max_rpm;
        }
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float rpm = (wheel_rad_s[index] * scale /
                     chassis->config.wheel_radius_m) *
                    60.0f / (2.0f * MECANUM_PI_F);
        wheel_rpm[index] = mecanum_round_rpm(rpm);
    }

    if (mecanum_send_wheel_rpm(chassis, wheel_rpm) != MECANUM_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    return (scale < 1.0f) ? MECANUM_CLAMPED : MECANUM_OK;
}

mecanum_result_t mecanum_chassis_read_feedback(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    /* 请求是异步的，HAL CAN 中断会更新 motor_t 的缓存字段。 */
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_t *motor = chassis->config.motor[index];

        if ((motor_read_speed(motor) != MOTOR_OK) ||
            (motor_read_position(motor) != MOTOR_OK))
        {
            return MECANUM_MOTOR_ERROR;
        }

        chassis->actual_wheel_rpm[index] = (int16_t)(
            (int32_t)motor->actual_speed_rpm *
            chassis->config.direction_sign[index]);
    }

    chassis->last_feedback_tick = HAL_GetTick();
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_update_odometry(mecanum_chassis_t *chassis)
{
    float wheel_distance[MECANUM_WHEEL_COUNT];
    float geometry_factor;
    float dx;
    float dy;
    float dyaw;
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_t *motor = chassis->config.motor[index];

        /* last_function==0x36 表示最近一次收到的是位置反馈。 */
        if (motor->last_function != (uint8_t)MOTOR_INFO_POSITION)
        {
            return MECANUM_NOT_READY;
        }

        if (chassis->odom_initialized == 0U)
        {
            chassis->last_input_pulses[index] =
                motor->actual_position_pulse;
        }

        wheel_distance[index] = (float)(
            (motor->actual_position_pulse -
             chassis->last_input_pulses[index]) *
            chassis->config.direction_sign[index]);
        wheel_distance[index] *= (2.0f * MECANUM_PI_F *
                                  chassis->config.wheel_radius_m) /
                                 (float)chassis->config.pulses_per_rev;
        chassis->last_input_pulses[index] = motor->actual_position_pulse;
    }

    if (chassis->odom_initialized == 0U)
    {
        chassis->odom_initialized = 1U;
        return MECANUM_OK;
    }

    geometry_factor = chassis->config.half_length_m +
                      chassis->config.half_width_m;
    if (geometry_factor <= 0.0f)
    {
        return MECANUM_INVALID_PARAM;
    }

    dx = (wheel_distance[MECANUM_WHEEL_FL] +
          wheel_distance[MECANUM_WHEEL_FR] +
          wheel_distance[MECANUM_WHEEL_RL] +
          wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    dy = (-wheel_distance[MECANUM_WHEEL_FL] +
          wheel_distance[MECANUM_WHEEL_FR] +
          wheel_distance[MECANUM_WHEEL_RL] -
          wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    dyaw = (-wheel_distance[MECANUM_WHEEL_FL] +
            wheel_distance[MECANUM_WHEEL_FR] -
            wheel_distance[MECANUM_WHEEL_RL] +
            wheel_distance[MECANUM_WHEEL_RR]) /
           (4.0f * geometry_factor);

    /* 使用中点航向角将车体增量转换到世界坐标。 */
    {
        float mid_yaw = chassis->odom_yaw_rad + 0.5f * dyaw;
        float cos_yaw = cosf(mid_yaw);
        float sin_yaw = sinf(mid_yaw);

        chassis->odom_x_m += dx * cos_yaw - dy * sin_yaw;
        chassis->odom_y_m += dx * sin_yaw + dy * cos_yaw;
    }

    chassis->odom_yaw_rad += dyaw;
    chassis->last_delta_x_m = dx;
    chassis->last_delta_y_m = dy;
    chassis->last_delta_yaw_rad = dyaw;
    return MECANUM_OK;
}

void mecanum_chassis_reset_odometry(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if (chassis == NULL)
    {
        return;
    }

    chassis->odom_x_m = 0.0f;
    chassis->odom_y_m = 0.0f;
    chassis->odom_yaw_rad = 0.0f;
    chassis->last_delta_x_m = 0.0f;
    chassis->last_delta_y_m = 0.0f;
    chassis->last_delta_yaw_rad = 0.0f;
    chassis->odom_initialized = 0U;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->last_input_pulses[index] = 0;
    }
}

void mecanum_chassis_set_yaw(mecanum_chassis_t *chassis, float yaw_rad)
{
    if (chassis != NULL)
    {
        chassis->odom_yaw_rad = yaw_rad;
    }
}
