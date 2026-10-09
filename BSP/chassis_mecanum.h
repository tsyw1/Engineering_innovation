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
#include "IMU_UART.h"

#include <stdbool.h>
#include <stdint.h>

struct mecanum_inertial_nav;

#define MECANUM_WHEEL_COUNT              4U
#define MECANUM_DEFAULT_ENCODER_COUNTS_PER_REV  65536UL
/* Emm 位置模式默认细分为 16：3200 个位置脉冲对应电机一圈。 */
#define MECANUM_POSITION_PULSES_PER_REV        3200UL
#define MECANUM_DEFAULT_ACCELERATION    20U
#define MECANUM_DEFAULT_MAX_WHEEL_RPM   MOTOR_EMM_MAX_SPEED_RPM
#define MECANUM_DEFAULT_WHEEL_DIAMETER_MM 75.0f
/* 根据自动前进测试修正的等效轮半径。 */
#define MECANUM_DEFAULT_WHEEL_RADIUS_M  0.03911f
#define MECANUM_DEFAULT_WHEELBASE_MM    225.0f
#define MECANUM_DEFAULT_TRACK_MM        248.0f
#define MECANUM_DEFAULT_HALF_LENGTH_M   0.1125f
#define MECANUM_DEFAULT_HALF_WIDTH_M    0.1240f

/* 仅作用于编码器/惯性导航的平移里程计，不改变速度控制的运动学换算。 */
#define MECANUM_DEFAULT_ODOMETRY_X_SCALE    1.01200f
#define MECANUM_DEFAULT_ODOMETRY_Y_SCALE    1.01200f
#define MECANUM_DEFAULT_ODOMETRY_YAW_SCALE  1.0041f

#define MECANUM_DEFAULT_MOTOR_FL_ID     1U
#define MECANUM_DEFAULT_MOTOR_FR_ID     2U
#define MECANUM_DEFAULT_MOTOR_RL_ID     3U
#define MECANUM_DEFAULT_MOTOR_RR_ID     4U
#define MECANUM_DEFAULT_FEEDBACK_PERIOD_MS 20U

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

/** 编码器独立里程计输出，供后续与 IMU 做融合。 */
typedef struct
{
    float x_m;
    float y_m;
    float yaw_rad;

    float delta_x_m;
    float delta_y_m;
    float delta_yaw_rad;

    float vx_mps;
    float vy_mps;
    float wz_rad_s;

    uint32_t update_tick;
    uint8_t valid;
} mecanum_encoder_odometry_t;

/** 麦轮底盘硬件和控制参数。长度单位为 m，角速度单位为 rad/s。 */
typedef struct
{
    /* 按 FL、FR、RL、RR 顺序填写四台 motor_can 对象。 */
    motor_t *motor[MECANUM_WHEEL_COUNT];

    float wheel_radius_m;       // 麦轮有效半径
    float half_length_m;        // 车体前后半轴距
    float half_width_m;         // 车体左右半轮距

    /* 编码器里程计标定比例，0 表示使用 1.0（兼容旧配置）。 */
    float odometry_x_scale;
    float odometry_y_scale;
    float odometry_yaw_scale;

    uint32_t pulses_per_rev;    // 0x36 位置一圈对应的计数，65536 个计数
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
    /* 最近一次相对位置细调命令，单位为电机脉冲。 */
    int32_t last_position_increment_pulse[MECANUM_WHEEL_COUNT];

    /* 车体坐标系下累计位姿，由 0x36 多圈位置反馈积分得到。 */
    float odom_x_m;
    float odom_y_m;
    float odom_yaw_rad;
    float last_delta_x_m;
    float last_delta_y_m;
    float last_delta_yaw_rad;

    uint16_t last_encoder_raw[MECANUM_WHEEL_COUNT];
    int64_t last_position_pulse[MECANUM_WHEEL_COUNT];
    uint32_t last_encoder_update_ms[MECANUM_WHEEL_COUNT];
    uint32_t last_encoder_update_count[MECANUM_WHEEL_COUNT];
    uint8_t odom_initialized;
    uint32_t last_feedback_tick;

    /* 独立保存编码器里程计，避免后续 IMU 融合覆盖原始结果。 */
    mecanum_encoder_odometry_t encoder_odometry;

    /* 固定距离动作优先使用惯性导航闭环；为空时回退到位置模式。 */
    struct mecanum_inertial_nav *inertial_nav;
    jy901s_uart_t *imu_sensor;
    uint8_t inertial_motion_enabled;
} mecanum_chassis_t;

/** 初始化并检查四台 CAN 电机对象和几何参数，不会自动使能或运动。 */
mecanum_result_t mecanum_chassis_init(mecanum_chassis_t *chassis,
                                      const mecanum_config_t *config);

/** 使用当前工程默认电机地址、轮序、方向和机械参数初始化底盘。 */
mecanum_result_t mecanum_chassis_init_default(
    mecanum_chassis_t *chassis,
    motor_bus_t *bus,
    motor_t motors[MECANUM_WHEEL_COUNT]);

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

/** 以相对当前位置模式执行一次四轮位置细调。 */
mecanum_result_t mecanum_chassis_set_position_increment(
    mecanum_chassis_t *chassis,
    const int32_t pulse[MECANUM_WHEEL_COUNT],
    uint16_t speed_rpm,
    uint8_t acceleration);

/** 绑定惯性导航和 IMU，供状态读取和显式闭环控制使用。 */
void mecanum_chassis_attach_inertial_nav(
    mecanum_chassis_t *chassis,
    struct mecanum_inertial_nav *nav,
    jy901s_uart_t *imu_sensor);

/** 请求四台电机的速度和位置反馈。CAN 响应由中断异步更新到 motor_t。 */
mecanum_result_t mecanum_chassis_read_feedback(mecanum_chassis_t *chassis);

/** 按默认周期请求一次四轮反馈。 */
mecanum_result_t mecanum_chassis_feedback_task(mecanum_chassis_t *chassis);

/** 根据最新 0x36 多圈位置反馈更新轮式里程计。 */
mecanum_result_t mecanum_chassis_update_odometry(mecanum_chassis_t *chassis);

/** 使用电机 0x36 多圈位置反馈更新编码器独立里程计。 */
mecanum_result_t mecanum_chassis_update_encoder_odometry(
    mecanum_chassis_t *chassis);

/** 读取最近一次有效的编码器里程计结果，供 IMU 融合使用。 */
mecanum_result_t mecanum_chassis_get_encoder_odometry(
    const mecanum_chassis_t *chassis,
    mecanum_encoder_odometry_t *odometry);

/*
 * 典型调用顺序：
 *   mecanum_chassis_feedback_task(&chassis);
 *   mecanum_chassis_update_encoder_odometry(&chassis);
 *   mecanum_chassis_get_encoder_odometry(&chassis, &encoder_odom);
 */

/** 清零里程计位置和当前脉冲基准。 */
void mecanum_chassis_reset_odometry(mecanum_chassis_t *chassis);

/** 用外部 IMU 航向角覆盖轮式里程计航向角，单位 rad。 */
void mecanum_chassis_set_yaw(mecanum_chassis_t *chassis, float yaw_rad);

#endif /* ENGINEERING_INNOVATION_CHASSIS_MECANUM_H */
