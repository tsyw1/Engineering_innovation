#include "VELOCITY_COMPLEMENTARY_FILTER.h"

#include <math.h>
#include <string.h>

velocity_complementary_filter_result_t velocity_complementary_filter_init(
    velocity_complementary_filter_t *filter)
{
    if (filter == NULL)
    {
        return VELOCITY_COMPLEMENTARY_FILTER_INVALID_PARAM;
    }

    memset(filter, 0, sizeof(*filter));
    filter->config.time_constant_s =
        VELOCITY_COMPLEMENTARY_FILTER_DEFAULT_TIME_CONSTANT_S;
    filter->config.max_dt_s =
        VELOCITY_COMPLEMENTARY_FILTER_DEFAULT_MAX_DT_S;
    filter->valid = 1U;
    return VELOCITY_COMPLEMENTARY_FILTER_OK;
}

void velocity_complementary_filter_reset(
    velocity_complementary_filter_t *filter)
{
    velocity_complementary_filter_config_t config;

    if ((filter == NULL) || (filter->valid == 0U))
    {
        return;
    }

    config = filter->config;
    memset(filter, 0, sizeof(*filter));
    filter->config = config;
    filter->valid = 1U;
}

void velocity_complementary_filter_zero(
    velocity_complementary_filter_t *filter,
    uint32_t now_ms)
{
    if ((filter == NULL) || (filter->valid == 0U))
    {
        return;
    }

    filter->velocity_mps[0] = 0.0f;
    filter->velocity_mps[1] = 0.0f;
    filter->last_update_ms = now_ms;
    filter->initialized = 1U;
}

velocity_complementary_filter_result_t velocity_complementary_filter_update(
    velocity_complementary_filter_t *filter,
    float encoder_vx_mps,
    float encoder_vy_mps,
    float acceleration_x_mps2,
    float acceleration_y_mps2,
    uint32_t now_ms)
{
    float dt_s;
    float alpha;
    float predicted_vx_mps;
    float predicted_vy_mps;

    if ((filter == NULL) || (filter->valid == 0U) ||
        !isfinite(encoder_vx_mps) || !isfinite(encoder_vy_mps) ||
        !isfinite(acceleration_x_mps2) || !isfinite(acceleration_y_mps2))
    {
        return VELOCITY_COMPLEMENTARY_FILTER_INVALID_PARAM;
    }

    if (filter->initialized == 0U)
    {
        filter->velocity_mps[0] = encoder_vx_mps;
        filter->velocity_mps[1] = encoder_vy_mps;
        filter->last_update_ms = now_ms;
        filter->initialized = 1U;
        return VELOCITY_COMPLEMENTARY_FILTER_OK;
    }

    if (now_ms == filter->last_update_ms)
    {
        return VELOCITY_COMPLEMENTARY_FILTER_TIME_ERROR;
    }

    dt_s = (float)(now_ms - filter->last_update_ms) * 0.001f;
    filter->last_update_ms = now_ms;
    if ((dt_s <= 0.0f) || (dt_s > filter->config.max_dt_s))
    {
        filter->velocity_mps[0] = encoder_vx_mps;
        filter->velocity_mps[1] = encoder_vy_mps;
        return VELOCITY_COMPLEMENTARY_FILTER_TIME_ERROR;
    }

    alpha = filter->config.time_constant_s /
            (filter->config.time_constant_s + dt_s);
    predicted_vx_mps = filter->velocity_mps[0] +
                       acceleration_x_mps2 * dt_s;
    predicted_vy_mps = filter->velocity_mps[1] +
                       acceleration_y_mps2 * dt_s;

    filter->velocity_mps[0] = alpha * predicted_vx_mps +
                              (1.0f - alpha) * encoder_vx_mps;
    filter->velocity_mps[1] = alpha * predicted_vy_mps +
                              (1.0f - alpha) * encoder_vy_mps;

    return VELOCITY_COMPLEMENTARY_FILTER_OK;
}

velocity_complementary_filter_result_t velocity_complementary_filter_get(
    const velocity_complementary_filter_t *filter,
    float *vx_mps,
    float *vy_mps)
{
    if ((filter == NULL) || (vx_mps == NULL) || (vy_mps == NULL) ||
        (filter->valid == 0U) || (filter->initialized == 0U))
    {
        return VELOCITY_COMPLEMENTARY_FILTER_INVALID_PARAM;
    }

    *vx_mps = filter->velocity_mps[0];
    *vy_mps = filter->velocity_mps[1];
    return VELOCITY_COMPLEMENTARY_FILTER_OK;
}
