//
// 机械臂 Z 轴旋转驱动，固定使用 5 号 CAN 电机和 Emm 位置模式。
//

#ifndef ENGINEERING_INNOVATION_ARM_Z_AXIS_H
#define ENGINEERING_INNOVATION_ARM_Z_AXIS_H

#include "motor_can.h"

#include <stdbool.h>
#include <stdint.h>

#define ARM_Z_AXIS_MOTOR_ID                         5U
#define ARM_Z_AXIS_DEFAULT_COMMAND_PULSES_PER_REV  3200UL
#define ARM_Z_AXIS_FEEDBACK_COUNTS_PER_REV          65536.0f
#define ARM_Z_AXIS_DEFAULT_SPEED_RPM                100U
#define ARM_Z_AXIS_DEFAULT_ACCELERATION             50U

/* Z 轴旋转方向约定：电机 CCW = 机械顺时针，电机 CW = 机械逆时针。 */

typedef enum
{
    ARM_Z_AXIS_OK = 0,
    ARM_Z_AXIS_INVALID_PARAM,
    ARM_Z_AXIS_OUT_OF_RANGE,
    ARM_Z_AXIS_BUSY,
    ARM_Z_AXIS_MOTOR_ERROR,
    ARM_Z_AXIS_NOT_READY
} arm_z_axis_result_t;

typedef struct
{
    /* Emm 位置命令使用的细分脉冲数，默认 16 细分时为 3200 脉冲/电机圈。 */
    uint32_t command_pulses_per_motor_rev;
    /* 电机转数 / Z 轴输出转数，直连时填写 1.0f。 */
    float reduction_ratio;
    /*
     * +1 或 -1，用于统一软件角度正方向和电机方向。
     * 当前默认约定：正角度 -> 电机 CW -> 机械逆时针；
     * 负角度 -> 电机 CCW -> 机械顺时针。
     */
    int8_t direction_sign;
    uint16_t speed_rpm;
    uint8_t acceleration;

    bool soft_limit_enabled;
    float min_angle_deg;
    float max_angle_deg;
} arm_z_axis_config_t;

typedef struct
{
    motor_t *motor;
    arm_z_axis_config_t config;
    float target_angle_deg;
    int64_t target_command_pulse;
    uint8_t initialized;
} arm_z_axis_t;

/** 生成默认配置；reduction_ratio 为电机转数/输出轴转数。 */
arm_z_axis_result_t arm_z_axis_config_default(
    arm_z_axis_config_t *config,
    float reduction_ratio);

/** 注册并初始化固定地址为 5 的 Z 轴电机。CAN 总线必须已经启动。 */
arm_z_axis_result_t arm_z_axis_init(arm_z_axis_t *axis,
                                    motor_bus_t *bus,
                                    motor_t *motor,
                                    const arm_z_axis_config_t *config);

arm_z_axis_result_t arm_z_axis_enable(arm_z_axis_t *axis, bool enable);

/** 按机械臂输出轴角度运动；绝对零点由回零或 clear_position 确定。 */
arm_z_axis_result_t arm_z_axis_move_to(arm_z_axis_t *axis,
                                       float angle_deg,
                                       motor_sync_t sync);

arm_z_axis_result_t arm_z_axis_stop(arm_z_axis_t *axis);
arm_z_axis_result_t arm_z_axis_set_zero(arm_z_axis_t *axis);
arm_z_axis_result_t arm_z_axis_home(arm_z_axis_t *axis,
                                    uint8_t home_mode,
                                    motor_sync_t sync);
arm_z_axis_result_t arm_z_axis_home_abort(arm_z_axis_t *axis);

/** 请求实时位置和状态，返回数据由 CAN 中断异步写入 motor_t。 */
arm_z_axis_result_t arm_z_axis_read_feedback(arm_z_axis_t *axis);

/** 将最近一次 0x36 反馈换算成机械臂输出轴角度。 */
arm_z_axis_result_t arm_z_axis_get_angle(const arm_z_axis_t *axis,
                                         float *angle_deg);

bool arm_z_axis_position_reached(const arm_z_axis_t *axis);

#endif /* ENGINEERING_INNOVATION_ARM_Z_AXIS_H */
