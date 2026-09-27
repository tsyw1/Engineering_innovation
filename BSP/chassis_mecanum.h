//
// 麦轮底盘控制模块，使用 motor_can 原生 CAN 电机接口。
//
// 车体坐标约定：
//   vx > 0：车体向前
//   vy > 0：车体向左
//   wz > 0：车体逆时针旋转，单位 rad/s
//
// 默认轮序：前左 FL、前右 FR、后左 RL、后右 RR。
//

#ifndef ENGINEERING_INNOVATION_CHASSIS_MECANUM_H
#define ENGINEERING_INNOVATION_CHASSIS_MECANUM_H

#include "motor_can.h"

#include <stdbool.h>
#include <stdint.h>

#define MECANUM_WHEEL_COUNT              4U
#define MECANUM_DEFAULT_PULSES_PER_REV  3200U
#define MECANUM_DEFAULT_ACCELERATION    20U
#define MECANUM_DEFAULT_MAX_WHEEL_RPM   MOTOR_EMM_MAX_SPEED_RPM

/* 根据当前 1、2、3、4 号电机方向测试得到的 FL、FR、RL、RR 补偿符号。 */
#define MECANUM_DEFAULT_DIRECTION_SIGNS { 1, -1, 1, -1 }

typedef enum
{
    MECANUM_WHEEL_FL = 0,
    MECANUM_WHEEL_FR,
    MECANUM_WHEEL_RL,
    MECANUM_WHEEL_RR
} mecanum_wheel_index_t;

typedef enum
{
    MECANUM_OK = 0,
    MECANUM_CLAMPED,
    MECANUM_INVALID_PARAM,
    MECANUM_MOTOR_ERROR,
    MECANUM_NOT_READY
} mecanum_result_t;

/** 麦轮底盘硬件和控制参数。长度单位为 m，角速度单位为 rad/s。 */
typedef struct
{
    /* 按 FL、FR、RL、RR 顺序填写四台 motor_can 对象。 */
    motor_t *motor[MECANUM_WHEEL_COUNT];

    float wheel_radius_m;       // 麦轮有效半径
    float half_length_m;        // 车体前后半轴距
    float half_width_m;         // 车体左右半轮距

    uint16_t pulses_per_rev;    // CAN 位置反馈对应的每机械转脉冲数
    uint16_t max_wheel_rpm;     // 单轮最大转速
    uint8_t acceleration;       // CAN 速度模式加速度参数

    /* +1/-1：修正电机安装方向，使正轮速代表公式中的正方向。 */
    int8_t direction_sign[MECANUM_WHEEL_COUNT];
} mecanum_config_t;

/** 底盘运行时对象，允许上层读取目标轮速、反馈和里程计状态。 */
typedef struct
{
    mecanum_config_t config;

    int16_t target_wheel_rpm[MECANUM_WHEEL_COUNT];
    int16_t actual_wheel_rpm[MECANUM_WHEEL_COUNT];

    /* 车体坐标系下累计位姿，由 CAN 位置脉冲反馈积分得到。 */
    float odom_x_m;
    float odom_y_m;
    float odom_yaw_rad;
    float last_delta_x_m;
    float last_delta_y_m;
    float last_delta_yaw_rad;

    int64_t last_input_pulses[MECANUM_WHEEL_COUNT];
    uint8_t odom_initialized;
    uint32_t last_feedback_tick;
} mecanum_chassis_t;

/** 初始化并检查四台 CAN 电机对象和几何参数，不会自动使能或运动。 */
mecanum_result_t mecanum_chassis_init(mecanum_chassis_t *chassis,
                                      const mecanum_config_t *config);

/** 同时使能/失能四台底盘电机。 */
mecanum_result_t mecanum_chassis_enable(mecanum_chassis_t *chassis,
                                        bool enable);

/** 停止四台底盘电机，不影响同一 CAN 总线上的其他电机。 */
mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *chassis);

/** 设置车体速度并下发四轮同步速度命令。 */
mecanum_result_t mecanum_chassis_set_velocity(mecanum_chassis_t *chassis,
                                              float vx_mps,
                                              float vy_mps,
                                              float wz_rad_s);

/** 直接设置四轮目标 RPM，顺序为 FL、FR、RL、RR。 */
mecanum_result_t mecanum_chassis_set_wheel_speed(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT]);

/** 请求四台电机的速度和位置反馈。CAN 响应由中断异步更新到 motor_t。 */
mecanum_result_t mecanum_chassis_read_feedback(mecanum_chassis_t *chassis);

/** 根据最新 CAN 位置脉冲反馈更新轮式里程计。 */
mecanum_result_t mecanum_chassis_update_odometry(mecanum_chassis_t *chassis);

/** 清零里程计位置和当前脉冲基准。 */
void mecanum_chassis_reset_odometry(mecanum_chassis_t *chassis);

/** 用外部 IMU 航向角覆盖轮式里程计航向角，单位 rad。 */
void mecanum_chassis_set_yaw(mecanum_chassis_t *chassis, float yaw_rad);

#endif /* ENGINEERING_INNOVATION_CHASSIS_MECANUM_H */
