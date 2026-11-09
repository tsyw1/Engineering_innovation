#include "HEADING_HOLD_CONTROLLER.h"

#include <math.h>
#include <string.h>

#define HEADING_HOLD_DEG_TO_RAD 0.01745329251994329577f

static float heading_hold_wrap_error_deg(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }
    while (error_deg <= -180.0f)
    {
        error_deg += 360.0f;
    }
    return error_deg;
}

static float heading_hold_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

heading_hold_controller_result_t heading_hold_controller_init(
    heading_hold_controller_t *controller)
{
    if (controller == NULL)
    {
        return HEADING_HOLD_CONTROLLER_INVALID_PARAM;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config.kp_rad_s_per_rad =
        HEADING_HOLD_DEFAULT_KP_RAD_S_PER_RAD;
    controller->config.max_correction_rad_s =
        HEADING_HOLD_DEFAULT_MAX_CORRECTION_RAD_S;
    controller->config.deadband_deg = HEADING_HOLD_DEFAULT_DEADBAND_DEG;
    controller->config.control_sign = HEADING_HOLD_DEFAULT_CONTROL_SIGN;
    controller->valid = 1U;
    return HEADING_HOLD_CONTROLLER_OK;
}

void heading_hold_controller_reset(
    heading_hold_controller_t *controller)
{
    heading_hold_controller_config_t config;

    if ((controller == NULL) || (controller->valid == 0U))
    {
        return;
    }

    config = controller->config;
    memset(controller, 0, sizeof(*controller));
    controller->config = config;
    controller->valid = 1U;
}

void heading_hold_controller_enable(
    heading_hold_controller_t *controller)
{
    if ((controller == NULL) || (controller->valid == 0U))
    {
        return;
    }

    controller->enabled = 1U;
    controller->target_valid = 0U;
    controller->error_deg = 0.0f;
    controller->correction_rad_s = 0.0f;
}

void heading_hold_controller_disable(
    heading_hold_controller_t *controller)
{
    if ((controller == NULL) || (controller->valid == 0U))
    {
        return;
    }

    controller->enabled = 0U;
    controller->target_valid = 0U;
    controller->error_deg = 0.0f;
    controller->correction_rad_s = 0.0f;
}

heading_hold_controller_result_t heading_hold_controller_set_target(
    heading_hold_controller_t *controller,
    float target_yaw_deg)
{
    if ((controller == NULL) || (controller->valid == 0U) ||
        !isfinite(target_yaw_deg))
    {
        return HEADING_HOLD_CONTROLLER_INVALID_PARAM;
    }
    if (controller->enabled == 0U)
    {
        return HEADING_HOLD_CONTROLLER_DISABLED;
    }

    controller->target_yaw_deg = heading_hold_wrap_error_deg(target_yaw_deg);
    controller->target_valid = 1U;
    controller->error_deg = 0.0f;
    controller->correction_rad_s = 0.0f;
    return HEADING_HOLD_CONTROLLER_OK;
}

heading_hold_controller_result_t heading_hold_controller_update(
    heading_hold_controller_t *controller,
    float current_yaw_deg,
    float *correction_rad_s)
{
    float correction;

    if ((controller == NULL) || (correction_rad_s == NULL) ||
        (controller->valid == 0U) || !isfinite(current_yaw_deg))
    {
        return HEADING_HOLD_CONTROLLER_INVALID_PARAM;
    }

    if (controller->enabled == 0U)
    {
        *correction_rad_s = 0.0f;
        return HEADING_HOLD_CONTROLLER_DISABLED;
    }

    if (controller->target_valid == 0U)
    {
        controller->target_yaw_deg = current_yaw_deg;
        controller->target_valid = 1U;
        controller->error_deg = 0.0f;
        controller->correction_rad_s = 0.0f;
        *correction_rad_s = 0.0f;
        return HEADING_HOLD_CONTROLLER_OK;
    }

    controller->error_deg = heading_hold_wrap_error_deg(
        controller->target_yaw_deg - current_yaw_deg);
    if (heading_hold_abs(controller->error_deg) <=
        controller->config.deadband_deg)
    {
        correction = 0.0f;
    }
    else
    {
        correction = controller->config.control_sign *
                     controller->config.kp_rad_s_per_rad *
                     controller->error_deg * HEADING_HOLD_DEG_TO_RAD;
        if (correction > controller->config.max_correction_rad_s)
        {
            correction = controller->config.max_correction_rad_s;
        }
        else if (correction < -controller->config.max_correction_rad_s)
        {
            correction = -controller->config.max_correction_rad_s;
        }
    }

    controller->correction_rad_s = correction;
    *correction_rad_s = correction;
    return HEADING_HOLD_CONTROLLER_OK;
}

heading_hold_controller_result_t heading_hold_controller_get_status(
    const heading_hold_controller_t *controller,
    float *target_yaw_deg,
    float *error_deg,
    float *correction_rad_s)
{
    if ((controller == NULL) || (target_yaw_deg == NULL) ||
        (error_deg == NULL) || (correction_rad_s == NULL) ||
        (controller->valid == 0U))
    {
        return HEADING_HOLD_CONTROLLER_INVALID_PARAM;
    }

    *target_yaw_deg = controller->target_yaw_deg;
    *error_deg = controller->error_deg;
    *correction_rad_s = controller->correction_rad_s;
    return HEADING_HOLD_CONTROLLER_OK;
}
