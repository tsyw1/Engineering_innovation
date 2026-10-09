//
// 机械臂 Z 轴旋转驱动实现。
//

#include "arm_z_axis.h"

#include <math.h>
#include <string.h>

#define ARM_Z_AXIS_INIT_RETRY_COUNT  3U
#define ARM_Z_AXIS_INIT_RETRY_MS     20U

static arm_z_axis_result_t arm_z_axis_from_motor_result(motor_result_t result)
{
    switch (result)
    {
        case MOTOR_OK:
            return ARM_Z_AXIS_OK;
        case MOTOR_BUSY:
            return ARM_Z_AXIS_BUSY;
        case MOTOR_INVALID_PARAM:
            return ARM_Z_AXIS_INVALID_PARAM;
        case MOTOR_NOT_READY:
            return ARM_Z_AXIS_NOT_READY;
        default:
            return ARM_Z_AXIS_MOTOR_ERROR;
    }
}

static bool arm_z_axis_config_valid(const arm_z_axis_config_t *config)
{
    if ((config == NULL) ||
        (config->command_pulses_per_motor_rev == 0U) ||
        !isfinite(config->reduction_ratio) ||
        (config->reduction_ratio <= 0.0f) ||
        ((config->direction_sign != 1) &&
         (config->direction_sign != -1)) ||
        (config->speed_rpm == 0U) ||
        (config->speed_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return false;
    }

    if (config->soft_limit_enabled &&
        (!isfinite(config->min_angle_deg) ||
         !isfinite(config->max_angle_deg) ||
         (config->min_angle_deg >= config->max_angle_deg)))
    {
        return false;
    }

    return true;
}

static bool arm_z_axis_ready(const arm_z_axis_t *axis)
{
    return (axis != NULL) && (axis->initialized != 0U) &&
           (axis->motor != NULL) && (axis->motor->bus != NULL);
}

static bool arm_z_axis_angle_to_pulse(const arm_z_axis_t *axis,
                                      float angle_deg,
                                      int64_t *pulse)
{
    double value;

    if ((axis == NULL) || (pulse == NULL) || !isfinite(angle_deg))
    {
        return false;
    }

    value = (double)angle_deg *
            (double)axis->config.command_pulses_per_motor_rev *
            (double)axis->config.reduction_ratio / 360.0;
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

arm_z_axis_result_t arm_z_axis_config_default(
    arm_z_axis_config_t *config,
    float reduction_ratio)
{
    if ((config == NULL) || !isfinite(reduction_ratio) ||
        (reduction_ratio <= 0.0f))
    {
        return ARM_Z_AXIS_INVALID_PARAM;
    }

    memset(config, 0, sizeof(*config));
    config->command_pulses_per_motor_rev =
        ARM_Z_AXIS_DEFAULT_COMMAND_PULSES_PER_REV;
    config->reduction_ratio = reduction_ratio;
    /* 正角度对应电机 CW，按机械约定表现为逆时针旋转。 */
    config->direction_sign = 1;
    config->speed_rpm = ARM_Z_AXIS_DEFAULT_SPEED_RPM;
    config->acceleration = ARM_Z_AXIS_DEFAULT_ACCELERATION;
    config->soft_limit_enabled = false;
    return ARM_Z_AXIS_OK;
}

arm_z_axis_result_t arm_z_axis_init(arm_z_axis_t *axis,
                                    motor_bus_t *bus,
                                    motor_t *motor,
                                    const arm_z_axis_config_t *config)
{
    motor_result_t result;
    uint8_t attempt;

    if ((axis == NULL) || (bus == NULL) || (motor == NULL) ||
        !arm_z_axis_config_valid(config))
    {
        return ARM_Z_AXIS_INVALID_PARAM;
    }

    result = MOTOR_ERROR;
    for (attempt = 0U; attempt < ARM_Z_AXIS_INIT_RETRY_COUNT; attempt++)
    {
        result = motor_init(motor, bus, ARM_Z_AXIS_MOTOR_ID);
        if (result == MOTOR_OK)
        {
            break;
        }
        HAL_Delay(ARM_Z_AXIS_INIT_RETRY_MS);
    }
    if (result != MOTOR_OK)
    {
        return arm_z_axis_from_motor_result(result);
    }

    memset(axis, 0, sizeof(*axis));
    axis->motor = motor;
    memcpy(&axis->config, config, sizeof(axis->config));
    axis->initialized = 1U;
    return ARM_Z_AXIS_OK;
}

arm_z_axis_result_t arm_z_axis_enable(arm_z_axis_t *axis, bool enable)
{
    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    return arm_z_axis_from_motor_result(
        motor_enable(axis->motor, enable, MOTOR_EXECUTE_NOW));
}

arm_z_axis_result_t arm_z_axis_move_to(arm_z_axis_t *axis,
                                       float angle_deg,
                                       motor_sync_t sync)
{
    int64_t pulse;
    motor_result_t result;

    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }
    if (sync > MOTOR_EXECUTE_SYNC)
    {
        return ARM_Z_AXIS_INVALID_PARAM;
    }
    if (axis->config.soft_limit_enabled &&
        ((angle_deg < axis->config.min_angle_deg) ||
         (angle_deg > axis->config.max_angle_deg)))
    {
        return ARM_Z_AXIS_OUT_OF_RANGE;
    }
    if (!arm_z_axis_angle_to_pulse(axis, angle_deg, &pulse))
    {
        return ARM_Z_AXIS_OUT_OF_RANGE;
    }

    result = motor_set_position(axis->motor,
                                pulse,
                                axis->config.speed_rpm,
                                axis->config.acceleration,
                                MOTOR_POSITION_ABSOLUTE,
                                sync);
    if (result == MOTOR_OK)
    {
        axis->target_angle_deg = angle_deg;
        axis->target_command_pulse = pulse;
        axis->motor->state_flags &= (uint8_t)~MOTOR_STATE_POSITION_REACHED;
    }
    return arm_z_axis_from_motor_result(result);
}

arm_z_axis_result_t arm_z_axis_stop(arm_z_axis_t *axis)
{
    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    return arm_z_axis_from_motor_result(
        motor_stop(axis->motor, MOTOR_EXECUTE_NOW));
}

arm_z_axis_result_t arm_z_axis_set_zero(arm_z_axis_t *axis)
{
    motor_result_t result;

    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    result = motor_clear_position(axis->motor);
    if (result == MOTOR_OK)
    {
        axis->target_angle_deg = 0.0f;
        axis->target_command_pulse = 0;
    }
    return arm_z_axis_from_motor_result(result);
}

arm_z_axis_result_t arm_z_axis_home(arm_z_axis_t *axis,
                                    uint8_t home_mode,
                                    motor_sync_t sync)
{
    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    return arm_z_axis_from_motor_result(
        motor_home(axis->motor, home_mode, sync));
}

arm_z_axis_result_t arm_z_axis_home_abort(arm_z_axis_t *axis)
{
    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    return arm_z_axis_from_motor_result(motor_home_abort(axis->motor));
}

arm_z_axis_result_t arm_z_axis_read_feedback(arm_z_axis_t *axis)
{
    motor_result_t result;

    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    result = motor_read_position(axis->motor);
    if (result != MOTOR_OK)
    {
        return arm_z_axis_from_motor_result(result);
    }
    return arm_z_axis_from_motor_result(motor_read_state(axis->motor));
}

arm_z_axis_result_t arm_z_axis_get_angle(const arm_z_axis_t *axis,
                                         float *angle_deg)
{
    if (angle_deg == NULL)
    {
        return ARM_Z_AXIS_INVALID_PARAM;
    }
    if (!arm_z_axis_ready(axis))
    {
        return ARM_Z_AXIS_NOT_READY;
    }
    if (axis->motor->position_valid == 0U)
    {
        return ARM_Z_AXIS_NOT_READY;
    }

    *angle_deg = (float)axis->motor->actual_position_pulse * 360.0f /
                 (ARM_Z_AXIS_FEEDBACK_COUNTS_PER_REV *
                  axis->config.reduction_ratio);
    *angle_deg *= (float)axis->config.direction_sign;
    return ARM_Z_AXIS_OK;
}

bool arm_z_axis_position_reached(const arm_z_axis_t *axis)
{
    return arm_z_axis_ready(axis) &&
           ((axis->motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U);
}
