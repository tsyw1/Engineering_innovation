//
// 机械臂升降轴驱动实现。
//

#include "arm_lift_axis.h"

#include <math.h>
#include <string.h>

#define ARM_LIFT_AXIS_INIT_RETRY_COUNT  3U
#define ARM_LIFT_AXIS_INIT_RETRY_MS     20U

static arm_lift_axis_result_t arm_lift_axis_from_motor_result(
    motor_result_t result)
{
    switch (result)
    {
        case MOTOR_OK:
            return ARM_LIFT_AXIS_OK;
        case MOTOR_BUSY:
            return ARM_LIFT_AXIS_BUSY;
        case MOTOR_INVALID_PARAM:
            return ARM_LIFT_AXIS_INVALID_PARAM;
        case MOTOR_NOT_READY:
            return ARM_LIFT_AXIS_NOT_READY;
        default:
            return ARM_LIFT_AXIS_MOTOR_ERROR;
    }
}

static bool arm_lift_axis_config_valid(const arm_lift_axis_config_t *config)
{
    if ((config == NULL) ||
        (config->command_pulses_per_motor_rev == 0U) ||
        !isfinite(config->travel_per_motor_rev_mm) ||
        (config->travel_per_motor_rev_mm <= 0.0f) ||
        ((config->direction_sign != 1) &&
         (config->direction_sign != -1)) ||
        (config->speed_rpm == 0U) ||
        (config->speed_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return false;
    }

    if (config->soft_limit_enabled &&
        (!isfinite(config->min_height_mm) ||
         !isfinite(config->max_height_mm) ||
         (config->min_height_mm >= config->max_height_mm)))
    {
        return false;
    }

    return true;
}

static bool arm_lift_axis_ready(const arm_lift_axis_t *axis)
{
    return (axis != NULL) && (axis->initialized != 0U) &&
           (axis->motor != NULL) && (axis->motor->bus != NULL);
}

static bool arm_lift_axis_height_to_pulse(const arm_lift_axis_t *axis,
                                          float height_mm,
                                          int64_t *pulse)
{
    double value;

    if ((axis == NULL) || (pulse == NULL) || !isfinite(height_mm))
    {
        return false;
    }

    value = (double)height_mm *
            (double)axis->config.command_pulses_per_motor_rev /
            (double)axis->config.travel_per_motor_rev_mm;
    value *= (double)axis->config.direction_sign;
    if ((value > (double)UINT32_MAX) ||
        (value < -(double)UINT32_MAX))
    {
        return false;
    }

    *pulse = (value >= 0.0) ?
             (int64_t)(value + 0.5) : (int64_t)(value - 0.5);
    return true;
}

arm_lift_axis_result_t arm_lift_axis_config_default(
    arm_lift_axis_config_t *config,
    float travel_per_motor_rev_mm)
{
    if ((config == NULL) || !isfinite(travel_per_motor_rev_mm) ||
        (travel_per_motor_rev_mm <= 0.0f))
    {
        return ARM_LIFT_AXIS_INVALID_PARAM;
    }

    memset(config, 0, sizeof(*config));
    config->command_pulses_per_motor_rev =
        ARM_LIFT_AXIS_DEFAULT_COMMAND_PULSES_PER_REV;
    config->travel_per_motor_rev_mm = travel_per_motor_rev_mm;
    /* 高度为正表示上升：正脉冲对应 CW，上升；负脉冲对应 CCW，下降。 */
    config->direction_sign = 1;
    config->speed_rpm = ARM_LIFT_AXIS_DEFAULT_SPEED_RPM;
    config->acceleration = ARM_LIFT_AXIS_DEFAULT_ACCELERATION;
    config->soft_limit_enabled = false;
    return ARM_LIFT_AXIS_OK;
}

arm_lift_axis_result_t arm_lift_axis_init(
    arm_lift_axis_t *axis,
    motor_bus_t *bus,
    motor_t *motor,
    const arm_lift_axis_config_t *config)
{
    motor_result_t result;
    uint8_t attempt;

    if ((axis == NULL) || (bus == NULL) || (motor == NULL) ||
        !arm_lift_axis_config_valid(config))
    {
        return ARM_LIFT_AXIS_INVALID_PARAM;
    }

    result = MOTOR_ERROR;
    for (attempt = 0U; attempt < ARM_LIFT_AXIS_INIT_RETRY_COUNT; attempt++)
    {
        result = motor_init(motor, bus, ARM_LIFT_AXIS_MOTOR_ID);
        if (result == MOTOR_OK)
        {
            break;
        }
        HAL_Delay(ARM_LIFT_AXIS_INIT_RETRY_MS);
    }
    if (result != MOTOR_OK)
    {
        return arm_lift_axis_from_motor_result(result);
    }

    memset(axis, 0, sizeof(*axis));
    axis->motor = motor;
    memcpy(&axis->config, config, sizeof(axis->config));
    axis->initialized = 1U;
    return ARM_LIFT_AXIS_OK;
}

arm_lift_axis_result_t arm_lift_axis_enable(arm_lift_axis_t *axis,
                                            bool enable)
{
    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    return arm_lift_axis_from_motor_result(
        motor_enable(axis->motor, enable, MOTOR_EXECUTE_NOW));
}

arm_lift_axis_result_t arm_lift_axis_move_to(arm_lift_axis_t *axis,
                                             float height_mm,
                                             motor_sync_t sync)
{
    int64_t pulse;
    motor_result_t result;

    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }
    if (sync > MOTOR_EXECUTE_SYNC)
    {
        return ARM_LIFT_AXIS_INVALID_PARAM;
    }
    if (axis->config.soft_limit_enabled &&
        ((height_mm < axis->config.min_height_mm) ||
         (height_mm > axis->config.max_height_mm)))
    {
        return ARM_LIFT_AXIS_OUT_OF_RANGE;
    }
    if (!arm_lift_axis_height_to_pulse(axis, height_mm, &pulse))
    {
        return ARM_LIFT_AXIS_OUT_OF_RANGE;
    }

    result = motor_set_position(axis->motor,
                                pulse,
                                axis->config.speed_rpm,
                                axis->config.acceleration,
                                MOTOR_POSITION_ABSOLUTE,
                                sync);
    if (result == MOTOR_OK)
    {
        axis->target_height_mm = height_mm;
        axis->target_command_pulse = pulse;
        axis->motor->state_flags &= (uint8_t)~MOTOR_STATE_POSITION_REACHED;
    }
    return arm_lift_axis_from_motor_result(result);
}

arm_lift_axis_result_t arm_lift_axis_stop(arm_lift_axis_t *axis)
{
    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    return arm_lift_axis_from_motor_result(
        motor_stop(axis->motor, MOTOR_EXECUTE_NOW));
}

arm_lift_axis_result_t arm_lift_axis_set_zero(arm_lift_axis_t *axis)
{
    motor_result_t result;

    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    result = motor_clear_position(axis->motor);
    if (result == MOTOR_OK)
    {
        axis->target_height_mm = 0.0f;
        axis->target_command_pulse = 0;
    }
    return arm_lift_axis_from_motor_result(result);
}

arm_lift_axis_result_t arm_lift_axis_home(arm_lift_axis_t *axis,
                                          uint8_t home_mode,
                                          motor_sync_t sync)
{
    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    return arm_lift_axis_from_motor_result(
        motor_home(axis->motor, home_mode, sync));
}

arm_lift_axis_result_t arm_lift_axis_home_abort(arm_lift_axis_t *axis)
{
    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    return arm_lift_axis_from_motor_result(motor_home_abort(axis->motor));
}

arm_lift_axis_result_t arm_lift_axis_read_feedback(arm_lift_axis_t *axis)
{
    motor_result_t result;

    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    result = motor_read_position(axis->motor);
    if (result != MOTOR_OK)
    {
        return arm_lift_axis_from_motor_result(result);
    }
    return arm_lift_axis_from_motor_result(motor_read_state(axis->motor));
}

arm_lift_axis_result_t arm_lift_axis_get_height(
    const arm_lift_axis_t *axis,
    float *height_mm)
{
    if (height_mm == NULL)
    {
        return ARM_LIFT_AXIS_INVALID_PARAM;
    }
    if (!arm_lift_axis_ready(axis))
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }
    if (axis->motor->position_valid == 0U)
    {
        return ARM_LIFT_AXIS_NOT_READY;
    }

    *height_mm = (float)axis->motor->actual_position_pulse *
                 axis->config.travel_per_motor_rev_mm /
                 ARM_LIFT_AXIS_FEEDBACK_COUNTS_PER_REV;
    *height_mm *= (float)axis->config.direction_sign;
    return ARM_LIFT_AXIS_OK;
}

bool arm_lift_axis_position_reached(const arm_lift_axis_t *axis)
{
    return arm_lift_axis_ready(axis) &&
           ((axis->motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U);
}
