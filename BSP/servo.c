#include "servo.h"

static uint8_t servo_channel_is_valid(uint32_t channel)
{
    if ((channel == TIM_CHANNEL_1) ||
        (channel == TIM_CHANNEL_2) ||
        (channel == TIM_CHANNEL_3) ||
        (channel == TIM_CHANNEL_4))
    {
        return 1U;
    }

    return 0U;
}

servo_result_t servo_init(servo_t *servo,
                          TIM_HandleTypeDef *htim,
                          uint32_t channel)
{
    if ((servo == NULL) || (htim == NULL) ||
        (servo_channel_is_valid(channel) == 0U))
    {
        return SERVO_INVALID_PARAM;
    }

    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    servo->max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;

    // 初始化时保持低电平，设置好目标角度后再启动 PWM。
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, 0U);

    return SERVO_OK;
}

servo_result_t servo_start(servo_t *servo)
{
    HAL_StatusTypeDef status;

    if ((servo == NULL) || (servo->htim == NULL))
    {
        return SERVO_INVALID_PARAM;
    }

    status = HAL_TIM_PWM_Start(servo->htim, servo->channel);
    if (status != HAL_OK)
    {
        return SERVO_ERROR;
    }

    return SERVO_OK;
}

servo_result_t servo_set_angle_with_range(servo_t *servo,
                                          uint16_t angle_deg,
                                          uint16_t max_angle_deg)
{
    uint32_t pulse_range_us;
    uint32_t pulse_us;

    if ((servo == NULL) || (servo->htim == NULL) ||
        (max_angle_deg == 0U))
    {
        return SERVO_INVALID_PARAM;
    }

    if (angle_deg > max_angle_deg)
    {
        angle_deg = max_angle_deg;
    }

    pulse_range_us = servo->max_pulse_us - servo->min_pulse_us;
    pulse_us = servo->min_pulse_us +
               (pulse_range_us * angle_deg) / max_angle_deg;

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);

    return SERVO_OK;
}

servo_result_t servo_set_angle(servo_t *servo, uint16_t angle_deg)
{
    return servo_set_angle_with_range(servo,
                                      angle_deg,
                                      SERVO_MAX_ANGLE_DEG);
}

servo_result_t servo_set_pulse_us(servo_t *servo, uint16_t pulse_us)
{
    uint16_t compare_value;

    if ((servo == NULL) || (servo->htim == NULL))
    {
        return SERVO_INVALID_PARAM;
    }

    compare_value = pulse_us;

    if (compare_value < servo->min_pulse_us)
    {
        compare_value = servo->min_pulse_us;
    }
    else if (compare_value > servo->max_pulse_us)
    {
        compare_value = servo->max_pulse_us;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, compare_value);

    return SERVO_OK;
}

servo_result_t servo_set_pulse_range(servo_t *servo,
                                     uint16_t min_pulse_us,
                                     uint16_t max_pulse_us)
{
    if ((servo == NULL) || (servo->htim == NULL) ||
        (min_pulse_us >= max_pulse_us) ||
        (max_pulse_us >= SERVO_PERIOD_US))
    {
        return SERVO_INVALID_PARAM;
    }

    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;

    return SERVO_OK;
}

servo_result_t servo_stop(servo_t *servo)
{
    HAL_StatusTypeDef status;

    if ((servo == NULL) || (servo->htim == NULL))
    {
        return SERVO_INVALID_PARAM;
    }

    status = HAL_TIM_PWM_Stop(servo->htim, servo->channel);
    if (status != HAL_OK)
    {
        return SERVO_ERROR;
    }

    return SERVO_OK;
}
