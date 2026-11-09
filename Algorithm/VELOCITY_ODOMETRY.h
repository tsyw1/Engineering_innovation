#ifndef ENGINEERING_INNOVATION_VELOCITY_ODOMETRY_H
#define ENGINEERING_INNOVATION_VELOCITY_ODOMETRY_H

#include <stdint.h>

#define VELOCITY_ODOMETRY_DEFAULT_MAX_DT_S 0.20f

typedef enum
{
    VELOCITY_ODOMETRY_OK = 0,
    VELOCITY_ODOMETRY_INVALID_PARAM,
    VELOCITY_ODOMETRY_NOT_READY,
    VELOCITY_ODOMETRY_TIME_ERROR
} velocity_odometry_result_t;

typedef struct
{
    float max_dt_s;
} velocity_odometry_config_t;

typedef struct
{
    velocity_odometry_config_t config;
    float position_world_m[2];
    uint32_t last_update_ms;
    uint8_t initialized;
    uint8_t valid;
} velocity_odometry_t;

/** 初始化二维平面里程计，车体 X 向前、Y 向左，输出世界坐标位置。 */
velocity_odometry_result_t velocity_odometry_init(
    velocity_odometry_t *odometry);

/** 清零位置和时间状态；下一次有效更新只建立时间基准，不产生位移。 */
void velocity_odometry_reset(velocity_odometry_t *odometry);

/**
 * 使用车体坐标系速度和世界坐标系 yaw 更新位置：
 *   X_world += (vx*cos(yaw) - vy*sin(yaw)) * dt
 *   Y_world += (vx*sin(yaw) + vy*cos(yaw)) * dt
 * yaw 单位为 degree，正方向按逆时针为正。
 */
velocity_odometry_result_t velocity_odometry_update(
    velocity_odometry_t *odometry,
    float vx_mps,
    float vy_mps,
    float yaw_deg,
    uint32_t now_ms);

/** 读取当前世界坐标位置，单位 m。 */
velocity_odometry_result_t velocity_odometry_get_position(
    const velocity_odometry_t *odometry,
    float *x_m,
    float *y_m);

#endif /* ENGINEERING_INNOVATION_VELOCITY_ODOMETRY_H */
