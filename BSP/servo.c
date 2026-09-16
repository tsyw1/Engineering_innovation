#include "servo.h"

static bool servo_is_valid_channel(uint32_t channel)
{
    return (channel == TIM_CHANNEL_1) ||
           (channel == TIM_CHANNEL_2) ||
           (channel == TIM_CHANNEL_3) ||
           (channel == TIM_CHANNEL_4);
}

static servo_result_t servo_check(const servo_t *servo)
{
    if ((servo == NULL) || (servo->htim == NULL) ||
        !servo_is_valid_channel(servo->channel) ||
        (servo->min_pulse_us >= servo->max_pulse_us) ||
        (servo->max_pulse_us >= SERVO_PERIOD_US))
    {
        return SERVO_INVALID_PARAM;
    }

    return SERVO_OK;
}

servo_result_t servo_init(servo_t *servo,
                          TIM_HandleTypeDef *htim,
                          uint32_t channel)
{
    if ((servo == NULL) || (htim == NULL) || !servo_is_valid_channel(channel))
    {
        return SERVO_INVALID_PARAM;
    }

    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    servo->max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    servo->angle_deg = 90U;
    servo->started = false;

    // 上电先输出中位脉宽，启动 PWM 后舵机不会收到随机比较值。
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, 1500U);

    return SERVO_OK;
}

servo_result_t servo_set_pulse_range(servo_t *servo,
                                     uint16_t min_pulse_us,
                                     uint16_t max_pulse_us)
{
    if ((servo_check(servo) != SERVO_OK) ||
        (min_pulse_us >= max_pulse_us) ||
        (max_pulse_us >= SERVO_PERIOD_US))
    {
        return SERVO_INVALID_PARAM;
    }

    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;

    return servo_set_angle(servo, servo->angle_deg);
}

servo_result_t servo_start(servo_t *servo)
{
    HAL_StatusTypeDef status;

    if (servo_check(servo) != SERVO_OK)
    {
        return SERVO_INVALID_PARAM;
    }

    status = HAL_TIM_PWM_Start(servo->htim, servo->channel);
    if (status != HAL_OK)
    {
        return SERVO_HAL_ERROR;
    }

    servo->started = true;
    return SERVO_OK;
}

servo_result_t servo_stop(servo_t *servo)
{
    HAL_StatusTypeDef status;

    if (servo_check(servo) != SERVO_OK)
    {
        return SERVO_INVALID_PARAM;
    }

    status = HAL_TIM_PWM_Stop(servo->htim, servo->channel);
    if (status != HAL_OK)
    {
        return SERVO_HAL_ERROR;
    }

    servo->started = false;
    return SERVO_OK;
}

servo_result_t servo_set_pulse_us(servo_t *servo, uint16_t pulse_us)
{
    if (servo_check(servo) != SERVO_OK)
    {
        return SERVO_INVALID_PARAM;
    }

    if ((pulse_us < servo->min_pulse_us) ||
        (pulse_us > servo->max_pulse_us))
    {
        return SERVO_INVALID_PARAM;
    }

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
    return SERVO_OK;
}

servo_result_t servo_set_angle(servo_t *servo, uint16_t angle_deg)
{
    uint32_t pulse_us;

    if (servo_check(servo) != SERVO_OK ||
        (angle_deg > SERVO_MAX_ANGLE_DEG))
    {
        return SERVO_INVALID_PARAM;
    }

    // 整数线性映射：0 度对应最小脉宽，180 度对应最大脉宽。
    pulse_us = servo->min_pulse_us +
               ((uint32_t)(servo->max_pulse_us - servo->min_pulse_us) * angle_deg) /
               SERVO_MAX_ANGLE_DEG;

    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
    servo->angle_deg = angle_deg;

    return SERVO_OK;
}
