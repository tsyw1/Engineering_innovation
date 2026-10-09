//
// 机械臂夹爪舵机驱动，固定使用 TIM1_CH1（PE9 / Servo1）。
//

#ifndef ENGINEERING_INNOVATION_GRIPPER_SERVO_H
#define ENGINEERING_INNOVATION_GRIPPER_SERVO_H

#include "servo.h"

#include <stdint.h>

#define GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG    90U
#define GRIPPER_SERVO_DEFAULT_CLOSED_ANGLE_DEG  2U

typedef enum
{
    GRIPPER_SERVO_OK = 0,
    GRIPPER_SERVO_INVALID_PARAM,
    GRIPPER_SERVO_ERROR,
    GRIPPER_SERVO_NOT_READY
} gripper_servo_result_t;

typedef enum
{
    GRIPPER_SERVO_STATE_UNKNOWN = 0,
    GRIPPER_SERVO_STATE_OPEN,
    GRIPPER_SERVO_STATE_CLOSED
} gripper_servo_state_t;

typedef struct
{
    uint16_t open_angle_deg;
    uint16_t closed_angle_deg;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
} gripper_servo_config_t;

typedef struct
{
    servo_t servo;
    gripper_servo_config_t config;
    gripper_servo_state_t state;
    uint8_t initialized;
} gripper_servo_t;

/** 生成默认配置：90 度张开，2 度闭合。 */
void gripper_servo_config_default(gripper_servo_config_t *config);

/** 初始化 TIM1_CH1 舵机，并在启动 PWM 后将夹爪复位到闭合状态。 */
gripper_servo_result_t gripper_servo_init(
    gripper_servo_t *gripper,
    const gripper_servo_config_t *config);

gripper_servo_result_t gripper_servo_open(gripper_servo_t *gripper);
gripper_servo_result_t gripper_servo_close(gripper_servo_t *gripper);
gripper_servo_result_t gripper_servo_stop(gripper_servo_t *gripper);

gripper_servo_state_t gripper_servo_get_state(
    const gripper_servo_t *gripper);

#endif /* ENGINEERING_INNOVATION_GRIPPER_SERVO_H */
