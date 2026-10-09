//
// 机械臂升降轴驱动，固定使用 6 号 CAN 电机和 Emm 位置模式。
//

#ifndef ENGINEERING_INNOVATION_ARM_LIFT_AXIS_H
#define ENGINEERING_INNOVATION_ARM_LIFT_AXIS_H

#include "motor_can.h"

#include <stdbool.h>
#include <stdint.h>

#define ARM_LIFT_AXIS_MOTOR_ID                         6U
#define ARM_LIFT_AXIS_DEFAULT_COMMAND_PULSES_PER_REV  3200UL
#define ARM_LIFT_AXIS_FEEDBACK_COUNTS_PER_REV          65536.0f
#define ARM_LIFT_AXIS_DEFAULT_SPEED_RPM                250U
#define ARM_LIFT_AXIS_DEFAULT_ACCELERATION             50U

typedef enum
{
    ARM_LIFT_AXIS_OK = 0,
    ARM_LIFT_AXIS_INVALID_PARAM,
    ARM_LIFT_AXIS_OUT_OF_RANGE,
    ARM_LIFT_AXIS_BUSY,
    ARM_LIFT_AXIS_MOTOR_ERROR,
    ARM_LIFT_AXIS_NOT_READY
} arm_lift_axis_result_t;

typedef struct
{
    /* Emm 位置命令使用的细分脉冲数，默认 16 细分时为 3200 脉冲/电机圈。 */
    uint32_t command_pulses_per_motor_rev;
    /* 电机每转一圈，升降机构实际移动的毫米数。 */
    float travel_per_motor_rev_mm;
    /*
     * +1 或 -1，用于统一“高度增加为正”和电机方向。
     * 当前机械约定：正脉冲 -> CW -> 上升，因此通常填写 +1；
     * 负脉冲 -> CCW -> 下降。
     */
    int8_t direction_sign;
    uint16_t speed_rpm;
    uint8_t acceleration;

    bool soft_limit_enabled;
    float min_height_mm;
    float max_height_mm;
} arm_lift_axis_config_t;

typedef struct
{
    motor_t *motor;
    arm_lift_axis_config_t config;
    float target_height_mm;
    int64_t target_command_pulse;
    uint8_t initialized;
} arm_lift_axis_t;

/** 生成默认配置；travel_per_motor_rev_mm 必须按丝杆或同步带机构填写。 */
arm_lift_axis_result_t arm_lift_axis_config_default(
    arm_lift_axis_config_t *config,
    float travel_per_motor_rev_mm);

/** 注册并初始化固定地址为 6 的升降轴电机。CAN 总线必须已经启动。 */
arm_lift_axis_result_t arm_lift_axis_init(
    arm_lift_axis_t *axis,
    motor_bus_t *bus,
    motor_t *motor,
    const arm_lift_axis_config_t *config);

arm_lift_axis_result_t arm_lift_axis_enable(arm_lift_axis_t *axis,
                                            bool enable);

/** 按机构高度运动；绝对零点由回零或 clear_position 确定。 */
arm_lift_axis_result_t arm_lift_axis_move_to(arm_lift_axis_t *axis,
                                             float height_mm,
                                             motor_sync_t sync);

arm_lift_axis_result_t arm_lift_axis_stop(arm_lift_axis_t *axis);
arm_lift_axis_result_t arm_lift_axis_set_zero(arm_lift_axis_t *axis);
arm_lift_axis_result_t arm_lift_axis_home(arm_lift_axis_t *axis,
                                          uint8_t home_mode,
                                          motor_sync_t sync);
arm_lift_axis_result_t arm_lift_axis_home_abort(arm_lift_axis_t *axis);

/** 请求实时位置和状态，返回数据由 CAN 中断异步写入 motor_t。 */
arm_lift_axis_result_t arm_lift_axis_read_feedback(arm_lift_axis_t *axis);

/** 将最近一次 0x36 反馈换算成升降机构高度。 */
arm_lift_axis_result_t arm_lift_axis_get_height(
    const arm_lift_axis_t *axis,
    float *height_mm);

bool arm_lift_axis_position_reached(const arm_lift_axis_t *axis);

#endif /* ENGINEERING_INNOVATION_ARM_LIFT_AXIS_H */
