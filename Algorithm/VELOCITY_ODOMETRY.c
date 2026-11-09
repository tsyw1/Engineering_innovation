#include "VELOCITY_ODOMETRY.h"

#include <math.h>
#include <string.h>

#define VELOCITY_ODOMETRY_DEG_TO_RAD 0.01745329251994329577f

velocity_odometry_result_t velocity_odometry_init(
    velocity_odometry_t *odometry)
{
    if (odometry == NULL)
    {
        return VELOCITY_ODOMETRY_INVALID_PARAM;
    }

    memset(odometry, 0, sizeof(*odometry));
    odometry->config.max_dt_s = VELOCITY_ODOMETRY_DEFAULT_MAX_DT_S;
    odometry->valid = 1U;
    return VELOCITY_ODOMETRY_OK;
}

void velocity_odometry_reset(velocity_odometry_t *odometry)
{
    velocity_odometry_config_t config;

    if ((odometry == NULL) || (odometry->valid == 0U))
    {
        return;
    }

    config = odometry->config;
    memset(odometry, 0, sizeof(*odometry));
    odometry->config = config;
    odometry->valid = 1U;
}

velocity_odometry_result_t velocity_odometry_update(
    velocity_odometry_t *odometry,
    float vx_mps,
    float vy_mps,
    float yaw_deg,
    uint32_t now_ms)
{
    float dt_s;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float vx_world_mps;
    float vy_world_mps;

    if ((odometry == NULL) || (odometry->valid == 0U) ||
        !isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(yaw_deg))
    {
        return VELOCITY_ODOMETRY_INVALID_PARAM;
    }

    if (odometry->initialized == 0U)
    {
        odometry->last_update_ms = now_ms;
        odometry->initialized = 1U;
        return VELOCITY_ODOMETRY_OK;
    }

    if (now_ms == odometry->last_update_ms)
    {
        return VELOCITY_ODOMETRY_TIME_ERROR;
    }

    dt_s = (float)(now_ms - odometry->last_update_ms) * 0.001f;
    odometry->last_update_ms = now_ms;
    if ((dt_s <= 0.0f) || (dt_s > odometry->config.max_dt_s))
    {
        return VELOCITY_ODOMETRY_TIME_ERROR;
    }

    yaw_rad = yaw_deg * VELOCITY_ODOMETRY_DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    vx_world_mps = vx_mps * cos_yaw - vy_mps * sin_yaw;
    vy_world_mps = vx_mps * sin_yaw + vy_mps * cos_yaw;

    odometry->position_world_m[0] += vx_world_mps * dt_s;
    odometry->position_world_m[1] += vy_world_mps * dt_s;
    return VELOCITY_ODOMETRY_OK;
}

velocity_odometry_result_t velocity_odometry_get_position(
    const velocity_odometry_t *odometry,
    float *x_m,
    float *y_m)
{
    if ((odometry == NULL) || (x_m == NULL) || (y_m == NULL) ||
        (odometry->valid == 0U))
    {
        return VELOCITY_ODOMETRY_INVALID_PARAM;
    }

    if (odometry->initialized == 0U)
    {
        return VELOCITY_ODOMETRY_NOT_READY;
    }

    *x_m = odometry->position_world_m[0];
    *y_m = odometry->position_world_m[1];
    return VELOCITY_ODOMETRY_OK;
}
