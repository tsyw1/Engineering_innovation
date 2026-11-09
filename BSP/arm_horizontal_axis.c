// 机械臂水平伸缩步进轴实现。

#include "arm_horizontal_axis.h"

#include <math.h>
#include <string.h>

static arm_horizontal_axis_result_t arm_horizontal_axis_from_motor(
    motor_result_t result)
{
    switch (result)
    {
        case MOTOR_OK: return ARM_HORIZONTAL_AXIS_OK;
        case MOTOR_BUSY: return ARM_HORIZONTAL_AXIS_BUSY;
        case MOTOR_INVALID_PARAM: return ARM_HORIZONTAL_AXIS_INVALID_PARAM;
        case MOTOR_NOT_READY: return ARM_HORIZONTAL_AXIS_NOT_READY;
        default: return ARM_HORIZONTAL_AXIS_MOTOR_ERROR;
    }
}

static bool arm_horizontal_axis_ready(const arm_horizontal_axis_t *axis)
{
    return (axis != NULL) && (axis->initialized != 0U) &&
           (axis->motor != NULL) && (axis->motor->bus != NULL);
}

static bool arm_horizontal_axis_config_valid(
    const arm_horizontal_axis_config_t *config)
{
    return (config != NULL) &&
           (config->command_pulses_per_motor_rev != 0U) &&
           isfinite(config->travel_per_motor_rev_mm) &&
           (config->travel_per_motor_rev_mm > 0.0f) &&
           ((config->direction_sign == 1) || (config->direction_sign == -1)) &&
           (config->speed_rpm > 0U) &&
           (config->speed_rpm <= MOTOR_EMM_MAX_SPEED_RPM) &&
           (!config->soft_limit_enabled ||
            (isfinite(config->min_extension_mm) &&
             isfinite(config->max_extension_mm) &&
             (config->min_extension_mm < config->max_extension_mm)));
}

static bool arm_horizontal_axis_extension_to_pulse(
    const arm_horizontal_axis_t *axis,
    float extension_mm,
    int64_t *pulse)
{
    double value;

    if ((axis == NULL) || (pulse == NULL) || !isfinite(extension_mm))
    {
        return false;
    }
    value = (double)extension_mm *
            (double)axis->config.command_pulses_per_motor_rev /
            (double)axis->config.travel_per_motor_rev_mm;
    value *= (double)axis->config.direction_sign;
    if ((value > (double)UINT32_MAX) || (value < -(double)UINT32_MAX))
    {
        return false;
    }
    *pulse = (value >= 0.0) ?
             (int64_t)(value + 0.5) : (int64_t)(value - 0.5);
    return true;
}

arm_horizontal_axis_result_t arm_horizontal_axis_config_default(
    arm_horizontal_axis_config_t *config,
    float travel_per_motor_rev_mm)
{
    if ((config == NULL) || !isfinite(travel_per_motor_rev_mm) ||
        (travel_per_motor_rev_mm <= 0.0f))
    {
        return ARM_HORIZONTAL_AXIS_INVALID_PARAM;
    }
    memset(config, 0, sizeof(*config));
    config->command_pulses_per_motor_rev =
        ARM_HORIZONTAL_AXIS_DEFAULT_COMMAND_PULSES_PER_REV;
    config->travel_per_motor_rev_mm = travel_per_motor_rev_mm;
    config->direction_sign = 1;
    config->speed_rpm = ARM_HORIZONTAL_AXIS_DEFAULT_SPEED_RPM;
    config->acceleration = ARM_HORIZONTAL_AXIS_DEFAULT_ACCELERATION;
    return ARM_HORIZONTAL_AXIS_OK;
}

arm_horizontal_axis_result_t arm_horizontal_axis_init(
    arm_horizontal_axis_t *axis,
    motor_bus_t *bus,
    motor_t *motor,
    uint8_t motor_id,
    const arm_horizontal_axis_config_t *config)
{
    motor_result_t result;

    if ((axis == NULL) || (bus == NULL) || (motor == NULL) ||
        (motor_id == 0U) || !arm_horizontal_axis_config_valid(config))
    {
        return ARM_HORIZONTAL_AXIS_INVALID_PARAM;
    }
    result = motor_init(motor, bus, motor_id);
    if (result != MOTOR_OK)
    {
        return arm_horizontal_axis_from_motor(result);
    }
    memset(axis, 0, sizeof(*axis));
    axis->motor = motor;
    axis->motor_id = motor_id;
    memcpy(&axis->config, config, sizeof(axis->config));
    axis->initialized = 1U;
    return ARM_HORIZONTAL_AXIS_OK;
}

arm_horizontal_axis_result_t arm_horizontal_axis_enable(
    arm_horizontal_axis_t *axis,
    bool enable)
{
    if (!arm_horizontal_axis_ready(axis)) return ARM_HORIZONTAL_AXIS_NOT_READY;
    return arm_horizontal_axis_from_motor(
        motor_enable(axis->motor, enable, MOTOR_EXECUTE_NOW));
}

arm_horizontal_axis_result_t arm_horizontal_axis_move_to(
    arm_horizontal_axis_t *axis,
    float extension_mm,
    motor_sync_t sync)
{
    int64_t pulse;
    motor_result_t result;

    if (!arm_horizontal_axis_ready(axis)) return ARM_HORIZONTAL_AXIS_NOT_READY;
    if (sync > MOTOR_EXECUTE_SYNC) return ARM_HORIZONTAL_AXIS_INVALID_PARAM;
    if (axis->config.soft_limit_enabled &&
        ((extension_mm < axis->config.min_extension_mm) ||
         (extension_mm > axis->config.max_extension_mm)))
    {
        return ARM_HORIZONTAL_AXIS_OUT_OF_RANGE;
    }
    if (!arm_horizontal_axis_extension_to_pulse(axis, extension_mm, &pulse))
    {
        return ARM_HORIZONTAL_AXIS_OUT_OF_RANGE;
    }
    axis->motor->state_flags &= (uint8_t)~MOTOR_STATE_POSITION_REACHED;
    result = motor_set_position(axis->motor,
                                pulse,
                                axis->config.speed_rpm,
                                axis->config.acceleration,
                                MOTOR_POSITION_ABSOLUTE,
                                sync);
    if (result == MOTOR_OK)
    {
        axis->target_extension_mm = extension_mm;
        axis->target_command_pulse = pulse;
    }
    return arm_horizontal_axis_from_motor(result);
}

arm_horizontal_axis_result_t arm_horizontal_axis_stop(
    arm_horizontal_axis_t *axis)
{
    if (!arm_horizontal_axis_ready(axis)) return ARM_HORIZONTAL_AXIS_NOT_READY;
    return arm_horizontal_axis_from_motor(
        motor_stop(axis->motor, MOTOR_EXECUTE_NOW));
}

arm_horizontal_axis_result_t arm_horizontal_axis_set_zero(
    arm_horizontal_axis_t *axis)
{
    motor_result_t result;
    if (!arm_horizontal_axis_ready(axis)) return ARM_HORIZONTAL_AXIS_NOT_READY;
    result = motor_clear_position(axis->motor);
    if (result == MOTOR_OK)
    {
        axis->target_extension_mm = 0.0f;
        axis->target_command_pulse = 0;
    }
    return arm_horizontal_axis_from_motor(result);
}

arm_horizontal_axis_result_t arm_horizontal_axis_read_feedback(
    arm_horizontal_axis_t *axis)
{
    motor_result_t result;
    if (!arm_horizontal_axis_ready(axis)) return ARM_HORIZONTAL_AXIS_NOT_READY;
    result = motor_read_position(axis->motor);
    if (result != MOTOR_OK) return arm_horizontal_axis_from_motor(result);
    return arm_horizontal_axis_from_motor(motor_read_state(axis->motor));
}

arm_horizontal_axis_result_t arm_horizontal_axis_get_extension(
    const arm_horizontal_axis_t *axis,
    float *extension_mm)
{
    if (extension_mm == NULL) return ARM_HORIZONTAL_AXIS_INVALID_PARAM;
    if (!arm_horizontal_axis_ready(axis) ||
        (axis->motor->position_valid == 0U))
    {
        return ARM_HORIZONTAL_AXIS_NOT_READY;
    }
    *extension_mm = (float)axis->motor->actual_position_pulse *
                    axis->config.travel_per_motor_rev_mm /
                    (float)axis->config.command_pulses_per_motor_rev;
    *extension_mm *= (float)axis->config.direction_sign;
    return ARM_HORIZONTAL_AXIS_OK;
}

bool arm_horizontal_axis_position_reached(
    const arm_horizontal_axis_t *axis)
{
    return arm_horizontal_axis_ready(axis) &&
           ((axis->motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U);
}
