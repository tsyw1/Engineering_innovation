// 机械臂水平伸缩步进轴，使用 CAN 电机绝对位置模式。

#ifndef ENGINEERING_INNOVATION_ARM_HORIZONTAL_AXIS_H
#define ENGINEERING_INNOVATION_ARM_HORIZONTAL_AXIS_H

#include "motor_can.h"

#include <stdbool.h>
#include <stdint.h>

#define ARM_HORIZONTAL_AXIS_DEFAULT_COMMAND_PULSES_PER_REV 3200UL
#define ARM_HORIZONTAL_AXIS_DEFAULT_SPEED_RPM                70U
#define ARM_HORIZONTAL_AXIS_DEFAULT_ACCELERATION             50U

typedef enum
{
    ARM_HORIZONTAL_AXIS_OK = 0,
    ARM_HORIZONTAL_AXIS_INVALID_PARAM,
    ARM_HORIZONTAL_AXIS_OUT_OF_RANGE,
    ARM_HORIZONTAL_AXIS_BUSY,
    ARM_HORIZONTAL_AXIS_MOTOR_ERROR,
    ARM_HORIZONTAL_AXIS_NOT_READY
} arm_horizontal_axis_result_t;

typedef struct
{
    uint32_t command_pulses_per_motor_rev;
    float travel_per_motor_rev_mm;
    int8_t direction_sign;
    uint16_t speed_rpm;
    uint8_t acceleration;
    bool soft_limit_enabled;
    float min_extension_mm;
    float max_extension_mm;
} arm_horizontal_axis_config_t;

typedef struct
{
    motor_t *motor;
    arm_horizontal_axis_config_t config;
    float target_extension_mm;
    int64_t target_command_pulse;
    uint8_t motor_id;
    uint8_t initialized;
} arm_horizontal_axis_t;

arm_horizontal_axis_result_t arm_horizontal_axis_config_default(
    arm_horizontal_axis_config_t *config,
    float travel_per_motor_rev_mm);

/* motor_id 由机械接线确认后传入，不在本模块中假定具体地址。 */
arm_horizontal_axis_result_t arm_horizontal_axis_init(
    arm_horizontal_axis_t *axis,
    motor_bus_t *bus,
    motor_t *motor,
    uint8_t motor_id,
    const arm_horizontal_axis_config_t *config);

arm_horizontal_axis_result_t arm_horizontal_axis_enable(
    arm_horizontal_axis_t *axis,
    bool enable);
arm_horizontal_axis_result_t arm_horizontal_axis_move_to(
    arm_horizontal_axis_t *axis,
    float extension_mm,
    motor_sync_t sync);
arm_horizontal_axis_result_t arm_horizontal_axis_stop(
    arm_horizontal_axis_t *axis);
arm_horizontal_axis_result_t arm_horizontal_axis_set_zero(
    arm_horizontal_axis_t *axis);
arm_horizontal_axis_result_t arm_horizontal_axis_read_feedback(
    arm_horizontal_axis_t *axis);
arm_horizontal_axis_result_t arm_horizontal_axis_get_extension(
    const arm_horizontal_axis_t *axis,
    float *extension_mm);
bool arm_horizontal_axis_position_reached(
    const arm_horizontal_axis_t *axis);

#endif /* ENGINEERING_INNOVATION_ARM_HORIZONTAL_AXIS_H */
