#ifndef ENGINEERING_INNOVATION_HEADING_HOLD_CONTROLLER_H
#define ENGINEERING_INNOVATION_HEADING_HOLD_CONTROLLER_H

#include <stdint.h>

#define HEADING_HOLD_DEFAULT_KP_RAD_S_PER_RAD 4.20f
#define HEADING_HOLD_DEFAULT_MAX_CORRECTION_RAD_S 10.50f
#define HEADING_HOLD_DEFAULT_DEADBAND_DEG 0.05f
/* HWT101 yaw 正方向与底盘 wz 正方向一致时使用 +1。 */
#define HEADING_HOLD_DEFAULT_CONTROL_SIGN 1.0f

typedef enum
{
    HEADING_HOLD_CONTROLLER_OK = 0,
    HEADING_HOLD_CONTROLLER_INVALID_PARAM,
    HEADING_HOLD_CONTROLLER_DISABLED
} heading_hold_controller_result_t;

typedef struct
{
    float kp_rad_s_per_rad;
    float max_correction_rad_s;
    float deadband_deg;
    float control_sign;
} heading_hold_controller_config_t;

typedef struct
{
    heading_hold_controller_config_t config;
    float target_yaw_deg;
    float error_deg;
    float correction_rad_s;
    uint8_t enabled;
    uint8_t target_valid;
    uint8_t valid;
} heading_hold_controller_t;

/** 初始化航向保持控制器。 */
heading_hold_controller_result_t heading_hold_controller_init(
    heading_hold_controller_t *controller);

/** 清除目标航向和控制输出。 */
void heading_hold_controller_reset(
    heading_hold_controller_t *controller);

/** 启用航向保持；下一次有效更新会锁存当前 yaw 为目标航向。 */
void heading_hold_controller_enable(
    heading_hold_controller_t *controller);

/** 禁用航向保持并清零角速度修正。 */
void heading_hold_controller_disable(
    heading_hold_controller_t *controller);

/** 设置明确的目标航向，供定角旋转和分段路线保持使用。 */
heading_hold_controller_result_t heading_hold_controller_set_target(
    heading_hold_controller_t *controller,
    float target_yaw_deg);

/** 根据当前 yaw 计算车体角速度修正量，单位 rad/s。 */
heading_hold_controller_result_t heading_hold_controller_update(
    heading_hold_controller_t *controller,
    float current_yaw_deg,
    float *correction_rad_s);

/** 读取目标航向、航向误差和最近一次修正量。 */
heading_hold_controller_result_t heading_hold_controller_get_status(
    const heading_hold_controller_t *controller,
    float *target_yaw_deg,
    float *error_deg,
    float *correction_rad_s);

#endif /* ENGINEERING_INNOVATION_HEADING_HOLD_CONTROLLER_H */
