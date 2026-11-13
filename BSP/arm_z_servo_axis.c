// 机械臂 Z 轴串行总线舵机接口实现。

#include "arm_z_servo_axis.h"

#include "joho_bus_servo.h"
#include "usart.h"

#include <math.h>
#include <string.h>

static uint8_t arm_z_servo_axis_ready(
    const arm_z_servo_axis_t *axis)
{
    return (axis != NULL) && (axis->initialized != 0U) &&
           (axis->hardware_bound != 0U) && (axis->huart != NULL);
}

static arm_z_servo_axis_result_t arm_z_servo_axis_from_bus_result(
    joho_bus_servo_result_t result)
{
    if (result == JOHO_BUS_SERVO_OK)
    {
        return ARM_Z_SERVO_AXIS_OK;
    }
    if (result == JOHO_BUS_SERVO_INVALID_PARAM)
    {
        return ARM_Z_SERVO_AXIS_INVALID_PARAM;
    }
    return ARM_Z_SERVO_AXIS_SERVO_ERROR;
}

void arm_z_servo_axis_config_default(arm_z_servo_axis_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->min_angle_deg = ARM_Z_SERVO_AXIS_DEFAULT_MIN_ANGLE_DEG;
    config->max_angle_deg = ARM_Z_SERVO_AXIS_DEFAULT_MAX_ANGLE_DEG;
    config->zero_angle_deg = 0.0f;
    config->reverse = 0U;
    config->servo_id = ARM_Z_SERVO_AXIS_DEFAULT_SERVO_ID;
    config->run_time_ms = ARM_Z_SERVO_AXIS_DEFAULT_RUN_TIME_MS;
}

arm_z_servo_axis_result_t arm_z_servo_axis_bind(
    arm_z_servo_axis_t *axis,
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    const arm_z_servo_axis_config_t *config)
{
    if ((axis == NULL) || (config == NULL) ||
        !isfinite(config->min_angle_deg) ||
        !isfinite(config->max_angle_deg) ||
        (config->min_angle_deg >= config->max_angle_deg) ||
        !isfinite(config->zero_angle_deg) ||
        (config->zero_angle_deg < config->min_angle_deg) ||
        (config->zero_angle_deg > config->max_angle_deg) ||
        (servo_id == 0U) || (servo_id == 0xFFU))
    {
        return ARM_Z_SERVO_AXIS_INVALID_PARAM;
    }

    memset(axis, 0, sizeof(*axis));
    memcpy(&axis->config, config, sizeof(axis->config));
    axis->config.servo_id = servo_id;
    axis->target_angle_deg = config->zero_angle_deg;
    axis->initialized = 1U;

    /* 允许先完成软件绑定，硬件串口由上层稍后提供。 */
    if (huart == NULL)
    {
        return ARM_Z_SERVO_AXIS_OK;
    }

    axis->huart = huart;
    axis->hardware_bound = 1U;
    return ARM_Z_SERVO_AXIS_OK;
}

arm_z_servo_axis_result_t arm_z_servo_axis_bind_usart2(
    arm_z_servo_axis_t *axis,
    const arm_z_servo_axis_config_t *config)
{
    uint8_t servo_id;

    if (config == NULL)
    {
        return ARM_Z_SERVO_AXIS_INVALID_PARAM;
    }

    servo_id = (config->servo_id != 0U) ?
               config->servo_id : ARM_Z_SERVO_AXIS_DEFAULT_SERVO_ID;
    return arm_z_servo_axis_bind(axis, &huart2, servo_id, config);
}

arm_z_servo_axis_result_t arm_z_servo_axis_start(
    arm_z_servo_axis_t *axis)
{
    /* 总线舵机没有 PWM 启动动作；保持接口兼容但不发送上电位置。 */
    return arm_z_servo_axis_ready(axis) ?
           ARM_Z_SERVO_AXIS_OK : ARM_Z_SERVO_AXIS_NOT_READY;
}

arm_z_servo_axis_result_t arm_z_servo_axis_move_to(
    arm_z_servo_axis_t *axis,
    float angle_deg)
{
    float relative_angle;
    float output_angle;
    joho_bus_servo_result_t result;
    uint16_t max_angle;

    if (!arm_z_servo_axis_ready(axis))
    {
        return ARM_Z_SERVO_AXIS_NOT_READY;
    }
    if (!isfinite(angle_deg) ||
        (angle_deg < axis->config.min_angle_deg) ||
        (angle_deg > axis->config.max_angle_deg))
    {
        return ARM_Z_SERVO_AXIS_INVALID_PARAM;
    }

    max_angle = (uint16_t)(axis->config.max_angle_deg -
                           axis->config.min_angle_deg);
    relative_angle = angle_deg - axis->config.min_angle_deg;
    output_angle = axis->config.reverse ?
                   ((float)max_angle - relative_angle) :
                   relative_angle;
    result = joho_bus_servo_set_angle(axis->huart,
                                      axis->config.servo_id,
                                      (uint16_t)output_angle,
                                      max_angle,
                                      axis->config.run_time_ms);
    if (result == JOHO_BUS_SERVO_OK)
    {
        axis->target_angle_deg = angle_deg;
    }
    return arm_z_servo_axis_from_bus_result(result);
}

arm_z_servo_axis_result_t arm_z_servo_axis_move_to_position(
    arm_z_servo_axis_t *axis,
    uint16_t position)
{
    if (axis == NULL)
    {
        return ARM_Z_SERVO_AXIS_NOT_READY;
    }
    return arm_z_servo_axis_move_to_position_timed(
        axis,
        position,
        axis->config.run_time_ms);
}

arm_z_servo_axis_result_t arm_z_servo_axis_move_to_position_timed(
    arm_z_servo_axis_t *axis,
    uint16_t position,
    uint16_t run_time_ms)
{
    joho_bus_servo_result_t result;

    if (!arm_z_servo_axis_ready(axis))
    {
        return ARM_Z_SERVO_AXIS_NOT_READY;
    }
    if ((position > JOHO_BUS_SERVO_POSITION_MAX) || (run_time_ms == 0U))
    {
        return ARM_Z_SERVO_AXIS_INVALID_PARAM;
    }

    result = joho_bus_servo_set_position(axis->huart,
                                         axis->config.servo_id,
                                         position,
                                         run_time_ms);
    if (result == JOHO_BUS_SERVO_OK)
    {
        axis->target_angle_deg = ((float)position *
                                  (axis->config.max_angle_deg -
                                   axis->config.min_angle_deg)) /
                                 (float)JOHO_BUS_SERVO_POSITION_MAX +
                                 axis->config.min_angle_deg;
    }
    return arm_z_servo_axis_from_bus_result(result);
}

arm_z_servo_axis_result_t arm_z_servo_axis_stop(
    arm_z_servo_axis_t *axis)
{
    /* 当前总线位置控制接口没有停止帧，停止表示不再发送新目标。 */
    return arm_z_servo_axis_ready(axis) ?
           ARM_Z_SERVO_AXIS_OK : ARM_Z_SERVO_AXIS_NOT_READY;
}

arm_z_servo_axis_result_t arm_z_servo_axis_set_zero(
    arm_z_servo_axis_t *axis)
{
    if ((axis == NULL) || (axis->initialized == 0U))
    {
        return ARM_Z_SERVO_AXIS_NOT_READY;
    }

    axis->config.zero_angle_deg = axis->target_angle_deg;
    return ARM_Z_SERVO_AXIS_OK;
}

float arm_z_servo_axis_get_target_angle(
    const arm_z_servo_axis_t *axis)
{
    return (axis != NULL) ? axis->target_angle_deg : 0.0f;
}
