#ifndef ENGINEERING_INNOVATION_VELOCITY_COMPLEMENTARY_FILTER_H
#define ENGINEERING_INNOVATION_VELOCITY_COMPLEMENTARY_FILTER_H

#include <stdint.h>

#define VELOCITY_COMPLEMENTARY_FILTER_DEFAULT_TIME_CONSTANT_S 0.10f
#define VELOCITY_COMPLEMENTARY_FILTER_DEFAULT_MAX_DT_S        0.20f

typedef enum
{
    VELOCITY_COMPLEMENTARY_FILTER_OK = 0,
    VELOCITY_COMPLEMENTARY_FILTER_INVALID_PARAM,
    VELOCITY_COMPLEMENTARY_FILTER_TIME_ERROR
} velocity_complementary_filter_result_t;

typedef struct
{
    float time_constant_s;
    float max_dt_s;
} velocity_complementary_filter_config_t;

typedef struct
{
    velocity_complementary_filter_config_t config;
    float velocity_mps[2];
    uint32_t last_update_ms;
    uint8_t initialized;
    uint8_t valid;
} velocity_complementary_filter_t;

/** 初始化二维车体速度互补滤波器，X 向前、Y 向左。 */
velocity_complementary_filter_result_t velocity_complementary_filter_init(
    velocity_complementary_filter_t *filter);

/** 清除滤波状态；下一次有效更新会使用编码器速度重新初始化。 */
void velocity_complementary_filter_reset(
    velocity_complementary_filter_t *filter);

/** 将滤波器速度置零，通常在编码器确认车辆停止时调用。 */
void velocity_complementary_filter_zero(
    velocity_complementary_filter_t *filter,
    uint32_t now_ms);

/**
 * 使用加速度预测并用编码器校正：
 *   v_predict = v_fused + a * dt
 *   v_fused = alpha * v_predict + (1-alpha) * v_encoder
 * 其中 alpha = tau / (tau + dt)，因此低频由编码器约束，高频保留加速度响应。
 */
velocity_complementary_filter_result_t velocity_complementary_filter_update(
    velocity_complementary_filter_t *filter,
    float encoder_vx_mps,
    float encoder_vy_mps,
    float acceleration_x_mps2,
    float acceleration_y_mps2,
    uint32_t now_ms);

/** 读取当前融合速度。 */
velocity_complementary_filter_result_t velocity_complementary_filter_get(
    const velocity_complementary_filter_t *filter,
    float *vx_mps,
    float *vy_mps);

#endif /* ENGINEERING_INNOVATION_VELOCITY_COMPLEMENTARY_FILTER_H */
