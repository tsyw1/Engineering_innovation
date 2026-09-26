//
// 麦轮底盘控制实现。
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

    if ((config == NULL) || (config->wheel_radius_m <= 0.0f) ||
        (config->half_length_m < 0.0f) || (config->half_width_m < 0.0f) ||
        (config->pulses_per_rev == 0U) || (config->max_wheel_rpm == 0U) ||
        (config->max_wheel_rpm > MOTOR_TTL_MAX_SPEED_RPM))
    {
        return false;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if ((config->motor[index] == NULL) ||
            ((config->direction_sign[index] != 1) &&
             (config->direction_sign[index] != -1)))
        {
            return false;
        }
    }

    return true;
}

static mecanum_result_t mecanum_send_wheel_rpm(mecanum_chassis_t *chassis,
                                               const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t motor_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;
    motor_ttl_result_t result;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_rpm[index] = (int16_t)(wheel_rpm[index] *
                                     chassis->config.direction_sign[index]);
    }

    result = motor_ttl_set_speeds_sync(chassis->config.motor,
                                       motor_rpm,
                                       MECANUM_WHEEL_COUNT,
                                       chassis->config.acceleration);
    if (result != MOTOR_TTL_OK)
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
    motor_ttl_result_t result;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        result = motor_ttl_enable(chassis->config.motor[index],
                                  enable,
                                  MOTOR_TTL_EXECUTE_NOW);
        if (result != MOTOR_TTL_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *chassis)
{
    uint8_t index;
    motor_ttl_result_t result;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    /* 不能使用 motor_ttl_stop_all，因为地址 5、6 可能属于机械臂。 */
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        result = motor_ttl_stop(chassis->config.motor[index],
                                MOTOR_TTL_EXECUTE_NOW);
        if (result != MOTOR_TTL_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        chassis->target_wheel_rpm[index] = 0;
        HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_wheel_speed(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t limited_rpm[MECANUM_WHEEL_COUNT];
    int16_t max_abs;
    uint8_t index;
    float scale = 1.0f;

    if ((chassis == NULL) || (wheel_rpm == NULL) ||
        !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    max_abs = 0;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int16_t value = wheel_rpm[index];
        int16_t abs_value = (value < 0) ? (int16_t)-value : value;

        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > (int16_t)chassis->config.max_wheel_rpm)
    {
        scale = (float)chassis->config.max_wheel_rpm / (float)max_abs;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        limited_rpm[index] = mecanum_round_rpm((float)wheel_rpm[index] * scale);
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
    float k;
    float wheel_rad_s[MECANUM_WHEEL_COUNT];
    float max_abs;
    float scale = 1.0f;
    int16_t wheel_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    k = chassis->config.half_length_m + chassis->config.half_width_m;

    /* 标准 X 型麦轮运动学，轮速单位为 rad/s。 */
    wheel_rad_s[MECANUM_WHEEL_FL] = vx_mps - vy_mps - k * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_FR] = vx_mps + vy_mps + k * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RL] = vx_mps + vy_mps - k * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RR] = vx_mps - vy_mps + k * wz_rad_s;

    max_abs = 0.0f;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float abs_value = mecanum_abs_f(wheel_rad_s[index]);
        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    k = (float)chassis->config.wheel_radius_m;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float rpm = (wheel_rad_s[index] / k) * 60.0f /
                    (2.0f * MECANUM_PI_F);
        wheel_rpm[index] = mecanum_round_rpm(rpm);
    }

    if (max_abs > 0.0f)
    {
        float max_rpm = (max_abs / chassis->config.wheel_radius_m) *
                        60.0f / (2.0f * MECANUM_PI_F);
        if (max_rpm > (float)chassis->config.max_wheel_rpm)
        {
            scale = (float)chassis->config.max_wheel_rpm / max_rpm;
            for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
            {
                wheel_rpm[index] = mecanum_round_rpm(
                    (float)wheel_rpm[index] * scale);
            }
        }
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
    motor_ttl_result_t result;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        result = motor_ttl_read_speed(chassis->config.motor[index]);
        if (result != MOTOR_TTL_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);

        result = motor_ttl_read_input_pulses(chassis->config.motor[index]);
        if (result != MOTOR_TTL_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);

        chassis->actual_wheel_rpm[index] = (int16_t)(
            chassis->config.motor[index]->actual_speed_rpm *
            chassis->config.direction_sign[index]);
    }

    chassis->last_feedback_tick = HAL_GetTick();
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_update_odometry(mecanum_chassis_t *chassis)
{
    float wheel_distance[MECANUM_WHEEL_COUNT];
    float k;
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
        motor_ttl_motor_t *motor = chassis->config.motor[index];

        if (motor->actual_input_pulses_valid == 0U)
        {
            return MECANUM_NOT_READY;
        }

        if (chassis->odom_initialized == 0U)
        {
            chassis->last_input_pulses[index] = motor->actual_input_pulses;
        }

        wheel_distance[index] = ((float)(motor->actual_input_pulses -
                                         chassis->last_input_pulses[index]) *
                                 (2.0f * MECANUM_PI_F) *
                                 chassis->config.wheel_radius_m) /
                                (float)chassis->config.pulses_per_rev;
        wheel_distance[index] *= (float)chassis->config.direction_sign[index];
        chassis->last_input_pulses[index] = motor->actual_input_pulses;
    }

    if (chassis->odom_initialized == 0U)
    {
        chassis->odom_initialized = 1U;
        return MECANUM_OK;
    }

    k = chassis->config.half_length_m + chassis->config.half_width_m;
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
            wheel_distance[MECANUM_WHEEL_RR]) / (4.0f * k);

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
