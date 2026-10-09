//
// 三工位物料盘舵机驱动实现。
//

#include "material_tray_servo.h"

#include "tim.h"

#include <string.h>

static material_tray_servo_result_t material_tray_from_servo_result(
    servo_result_t result)
{
    switch (result)
    {
        case SERVO_OK:
            return MATERIAL_TRAY_SERVO_OK;
        case SERVO_INVALID_PARAM:
            return MATERIAL_TRAY_SERVO_INVALID_PARAM;
        default:
            return MATERIAL_TRAY_SERVO_ERROR;
    }
}

static uint8_t material_tray_position_valid(material_tray_position_t position)
{
    switch (position)
    {
        case MATERIAL_TRAY_POSITION_1:
        case MATERIAL_TRAY_POSITION_2:
        case MATERIAL_TRAY_POSITION_3:
            return 1U;
        default:
            return 0U;
    }
}

static uint8_t material_tray_config_valid(
    const material_tray_servo_config_t *config)
{
    uint8_t index;

    if ((config == NULL) ||
        (material_tray_position_valid(config->initial_position) == 0U) ||
        (config->min_pulse_us >= config->max_pulse_us) ||
        (config->max_pulse_us >= SERVO_PERIOD_US))
    {
        return 0U;
    }

    for (index = 0U; index < MATERIAL_TRAY_POSITION_COUNT; index++)
    {
        if (config->position_angle_deg[index] >
            MATERIAL_TRAY_SERVO_MAX_ANGLE_DEG)
        {
            return 0U;
        }
    }
    if ((config->position_angle_deg[0] == config->position_angle_deg[1]) ||
        (config->position_angle_deg[0] == config->position_angle_deg[2]) ||
        (config->position_angle_deg[1] == config->position_angle_deg[2]))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t material_tray_ready(const material_tray_servo_t *tray)
{
    return (tray != NULL) && (tray->initialized != 0U) &&
           (tray->servo.htim != NULL);
}

void material_tray_servo_config_default(
    material_tray_servo_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->position_angle_deg[MATERIAL_TRAY_POSITION_1] =
        MATERIAL_TRAY_DEFAULT_POSITION_1_DEG;
    config->position_angle_deg[MATERIAL_TRAY_POSITION_2] =
        MATERIAL_TRAY_DEFAULT_POSITION_2_DEG;
    config->position_angle_deg[MATERIAL_TRAY_POSITION_3] =
        MATERIAL_TRAY_DEFAULT_POSITION_3_DEG;
    config->min_pulse_us = MATERIAL_TRAY_SERVO_MIN_PULSE_US;
    config->max_pulse_us = MATERIAL_TRAY_SERVO_MAX_PULSE_US;
    config->initial_position = MATERIAL_TRAY_POSITION_1;
}

material_tray_servo_result_t material_tray_servo_init(
    material_tray_servo_t *tray,
    const material_tray_servo_config_t *config)
{
    servo_result_t result;

    if ((tray == NULL) || (material_tray_config_valid(config) == 0U))
    {
        return MATERIAL_TRAY_SERVO_INVALID_PARAM;
    }

    memset(tray, 0, sizeof(*tray));
    memcpy(&tray->config, config, sizeof(tray->config));

    result = servo_init(&tray->servo, &htim1, TIM_CHANNEL_2);
    if (result != SERVO_OK)
    {
        return material_tray_from_servo_result(result);
    }
    result = servo_set_pulse_range(&tray->servo,
                                   config->min_pulse_us,
                                   config->max_pulse_us);
    if (result != SERVO_OK)
    {
        return material_tray_from_servo_result(result);
    }
    tray->current_position = MATERIAL_TRAY_POSITION_INVALID;
    tray->pwm_started = 0U;
    tray->initialized = 1U;
    return MATERIAL_TRAY_SERVO_OK;
}

material_tray_servo_result_t material_tray_servo_set_position(
    material_tray_servo_t *tray,
    material_tray_position_t position)
{
    servo_result_t result;

    if (material_tray_ready(tray) == 0U)
    {
        return MATERIAL_TRAY_SERVO_NOT_READY;
    }
    if (material_tray_position_valid(position) == 0U)
    {
        return MATERIAL_TRAY_SERVO_OUT_OF_RANGE;
    }

    result = servo_set_angle_with_range(
        &tray->servo,
        tray->config.position_angle_deg[position],
        MATERIAL_TRAY_SERVO_MAX_ANGLE_DEG);
    if (result != SERVO_OK)
    {
        return material_tray_from_servo_result(result);
    }

    if (tray->pwm_started == 0U)
    {
        result = servo_start(&tray->servo);
        if (result != SERVO_OK)
        {
            return material_tray_from_servo_result(result);
        }
        tray->pwm_started = 1U;
    }

    tray->current_position = position;
    return MATERIAL_TRAY_SERVO_OK;
}

material_tray_servo_result_t material_tray_servo_next(
    material_tray_servo_t *tray)
{
    if (material_tray_ready(tray) == 0U)
    {
        return MATERIAL_TRAY_SERVO_NOT_READY;
    }
    if (tray->current_position == MATERIAL_TRAY_POSITION_INVALID)
    {
        return MATERIAL_TRAY_SERVO_OUT_OF_RANGE;
    }
    if (tray->current_position >= MATERIAL_TRAY_POSITION_3)
    {
        return MATERIAL_TRAY_SERVO_OUT_OF_RANGE;
    }

    return material_tray_servo_set_position(
        tray,
        (material_tray_position_t)(tray->current_position + 1));
}

material_tray_servo_result_t material_tray_servo_previous(
    material_tray_servo_t *tray)
{
    if (material_tray_ready(tray) == 0U)
    {
        return MATERIAL_TRAY_SERVO_NOT_READY;
    }
    if (tray->current_position == MATERIAL_TRAY_POSITION_INVALID)
    {
        return MATERIAL_TRAY_SERVO_OUT_OF_RANGE;
    }
    if (tray->current_position <= MATERIAL_TRAY_POSITION_1)
    {
        return MATERIAL_TRAY_SERVO_OUT_OF_RANGE;
    }

    return material_tray_servo_set_position(
        tray,
        (material_tray_position_t)(tray->current_position - 1));
}

material_tray_servo_result_t material_tray_servo_stop(
    material_tray_servo_t *tray)
{
    if (material_tray_ready(tray) == 0U)
    {
        return MATERIAL_TRAY_SERVO_NOT_READY;
    }
    if (tray->pwm_started == 0U)
    {
        return MATERIAL_TRAY_SERVO_OK;
    }

    if (servo_stop(&tray->servo) != SERVO_OK)
    {
        return MATERIAL_TRAY_SERVO_ERROR;
    }

    tray->pwm_started = 0U;
    return MATERIAL_TRAY_SERVO_OK;
}

material_tray_position_t material_tray_servo_get_position(
    const material_tray_servo_t *tray)
{
    if (material_tray_ready(tray) == 0U)
    {
        return MATERIAL_TRAY_POSITION_INVALID;
    }

    return tray->current_position;
}
