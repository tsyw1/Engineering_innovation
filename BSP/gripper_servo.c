//
// 机械臂夹爪舵机驱动实现。
//

#include "gripper_servo.h"

#include "tim.h"

#include <string.h>

static gripper_servo_result_t gripper_servo_from_servo_result(
    servo_result_t result)
{
    switch (result)
    {
        case SERVO_OK:
            return GRIPPER_SERVO_OK;
        case SERVO_INVALID_PARAM:
            return GRIPPER_SERVO_INVALID_PARAM;
        default:
            return GRIPPER_SERVO_ERROR;
    }
}

static uint8_t gripper_servo_config_valid(
    const gripper_servo_config_t *config)
{
    if ((config == NULL) ||
        (config->open_angle_deg > SERVO_MAX_ANGLE_DEG) ||
        (config->closed_angle_deg > SERVO_MAX_ANGLE_DEG) ||
        (config->open_angle_deg == config->closed_angle_deg) ||
        (config->min_pulse_us >= config->max_pulse_us) ||
        (config->max_pulse_us >= SERVO_PERIOD_US))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t gripper_servo_ready(const gripper_servo_t *gripper)
{
    return (gripper != NULL) && (gripper->initialized != 0U) &&
           (gripper->servo.htim != NULL);
}

void gripper_servo_config_default(gripper_servo_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->open_angle_deg = GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG;
    config->closed_angle_deg = GRIPPER_SERVO_DEFAULT_CLOSED_ANGLE_DEG;
    config->min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    config->max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
}

gripper_servo_result_t gripper_servo_init(
    gripper_servo_t *gripper,
    const gripper_servo_config_t *config)
{
    servo_result_t result;

    if ((gripper == NULL) ||
        (gripper_servo_config_valid(config) == 0U))
    {
        return GRIPPER_SERVO_INVALID_PARAM;
    }

    memset(gripper, 0, sizeof(*gripper));
    memcpy(&gripper->config, config, sizeof(gripper->config));

    result = servo_init(&gripper->servo, &htim1, TIM_CHANNEL_1);
    if (result != SERVO_OK)
    {
        return gripper_servo_from_servo_result(result);
    }
    result = servo_set_pulse_range(&gripper->servo,
                                   config->min_pulse_us,
                                   config->max_pulse_us);
    if (result != SERVO_OK)
    {
        return gripper_servo_from_servo_result(result);
    }

    /* 启动 PWM 前先写入闭合位置，上电后夹爪自动复位到闭合状态。 */
    result = servo_set_angle(&gripper->servo, config->closed_angle_deg);
    if (result != SERVO_OK)
    {
        return gripper_servo_from_servo_result(result);
    }
    result = servo_start(&gripper->servo);
    if (result != SERVO_OK)
    {
        return gripper_servo_from_servo_result(result);
    }

    gripper->state = GRIPPER_SERVO_STATE_CLOSED;
    gripper->initialized = 1U;
    return GRIPPER_SERVO_OK;
}

gripper_servo_result_t gripper_servo_open(gripper_servo_t *gripper)
{
    servo_result_t result;

    if (gripper_servo_ready(gripper) == 0U)
    {
        return GRIPPER_SERVO_NOT_READY;
    }

    result = servo_set_angle(&gripper->servo,
                             gripper->config.open_angle_deg);
    if (result == SERVO_OK)
    {
        gripper->state = GRIPPER_SERVO_STATE_OPEN;
    }
    return gripper_servo_from_servo_result(result);
}

gripper_servo_result_t gripper_servo_close(gripper_servo_t *gripper)
{
    servo_result_t result;

    if (gripper_servo_ready(gripper) == 0U)
    {
        return GRIPPER_SERVO_NOT_READY;
    }

    result = servo_set_angle(&gripper->servo,
                             gripper->config.closed_angle_deg);
    if (result == SERVO_OK)
    {
        gripper->state = GRIPPER_SERVO_STATE_CLOSED;
    }
    return gripper_servo_from_servo_result(result);
}

gripper_servo_result_t gripper_servo_stop(gripper_servo_t *gripper)
{
    if (gripper_servo_ready(gripper) == 0U)
    {
        return GRIPPER_SERVO_NOT_READY;
    }

    return gripper_servo_from_servo_result(servo_stop(&gripper->servo));
}

gripper_servo_state_t gripper_servo_get_state(
    const gripper_servo_t *gripper)
{
    if (gripper_servo_ready(gripper) == 0U)
    {
        return GRIPPER_SERVO_STATE_UNKNOWN;
    }

    return gripper->state;
}
