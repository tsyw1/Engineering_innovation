#ifndef ENGINEERING_INNOVATION_MECANUM_INERTIAL_NAV_H
#define ENGINEERING_INNOVATION_MECANUM_INERTIAL_NAV_H

#include "IMU_UART.h"
#include "chassis_mecanum.h"

#include <stdint.h>

#define MECANUM_INERTIAL_NAV_STATE_COUNT 6U
#define MECANUM_INERTIAL_NAV_DEFAULT_GYRO_WEIGHT 0.85f
#define MECANUM_INERTIAL_NAV_DEFAULT_BIAS_UPDATE_GAIN 0.02f

typedef enum
{
    MECANUM_INERTIAL_NAV_OK = 0,
    MECANUM_INERTIAL_NAV_INVALID_PARAM,
    MECANUM_INERTIAL_NAV_NOT_READY,
    MECANUM_INERTIAL_NAV_TARGET_REACHED
} mecanum_inertial_nav_result_t;

/**
 * 二维麦轮轮惯融合状态。
 * 位置单位 m，航向单位 rad，速度单位 m/s，角速度单位 rad/s。
 * EKF 状态顺序为 [x, y, yaw, vx_body, vy_body, gyro_bias_z]。
 */
typedef struct mecanum_inertial_nav
{
    mecanum_config_t config;

    float x_m;
    float y_m;
    float yaw_rad;
    float vx_mps;
    float vy_mps;
    float encoder_wz_rad_s;
    float gyro_wz_rad_s;
    float fused_wz_rad_s;
    float gyro_z_bias_dps;
    float gyro_weight;
    float bias_update_gain;

    float ekf_state[MECANUM_INERTIAL_NAV_STATE_COUNT];
    float ekf_cov[MECANUM_INERTIAL_NAV_STATE_COUNT *
                  MECANUM_INERTIAL_NAV_STATE_COUNT];
    uint8_t ekf_initialized;

    float target_x_m;
    float target_y_m;
    float target_yaw_rad;
    float target_max_vx_mps;
    float target_max_vy_mps;
    float target_max_wz_rad_s;
    float target_kp_position;
    float target_kp_yaw;
    float target_position_tolerance_m;
    float target_yaw_tolerance_rad;
    uint8_t target_active;

    /* 0x36 Emm 多圈位置基准，不能再使用 0x31 的 16 位单圈值。 */
    int64_t last_position_pulse[MECANUM_WHEEL_COUNT];
    uint32_t last_position_update_count[MECANUM_WHEEL_COUNT];
    uint32_t last_update_ms;
    uint32_t last_imu_frame_count;
    uint8_t encoder_initialized;
    uint8_t initialized;
} mecanum_inertial_nav_t;

mecanum_inertial_nav_result_t mecanum_inertial_nav_init(
    mecanum_inertial_nav_t *nav,
    const mecanum_config_t *config,
    float gyro_z_bias_dps,
    float gyro_weight,
    float bias_update_gain);

/** 使用当前工程默认互补滤波和零偏更新参数初始化导航。 */
mecanum_inertial_nav_result_t mecanum_inertial_nav_init_default(
    mecanum_inertial_nav_t *nav,
    const mecanum_config_t *config,
    float gyro_z_bias_dps);

void mecanum_inertial_nav_set_gyro_bias(
    mecanum_inertial_nav_t *nav,
    float gyro_z_bias_dps);

void mecanum_inertial_nav_reset(mecanum_inertial_nav_t *nav);

mecanum_inertial_nav_result_t mecanum_inertial_nav_update(
    mecanum_inertial_nav_t *nav,
    const mecanum_chassis_t *chassis,
    const jy901s_uart_t *imu_sensor);

/** 设置目标位置和目标航向；调用 control_step 前不会驱动电机。 */
mecanum_inertial_nav_result_t mecanum_inertial_nav_set_target(
    mecanum_inertial_nav_t *nav,
    float target_x_m,
    float target_y_m,
    float target_yaw_rad,
    float max_vx_mps,
    float max_vy_mps,
    float max_wz_rad_s,
    float kp_position,
    float kp_yaw,
    float position_tolerance_m,
    float yaw_tolerance_rad);

void mecanum_inertial_nav_cancel_target(mecanum_inertial_nav_t *nav,
                                        mecanum_chassis_t *chassis);

mecanum_inertial_nav_result_t mecanum_inertial_nav_control_step(
    mecanum_inertial_nav_t *nav,
    mecanum_chassis_t *chassis);

uint8_t mecanum_inertial_nav_target_reached(
    const mecanum_inertial_nav_t *nav);

#endif /* ENGINEERING_INNOVATION_MECANUM_INERTIAL_NAV_H */
