// 机械臂 Z 轴串行总线舵机接口。

#ifndef ENGINEERING_INNOVATION_ARM_Z_SERVO_AXIS_H
#define ENGINEERING_INNOVATION_ARM_Z_SERVO_AXIS_H

#include "main.h"

#include <stdint.h>

#define ARM_Z_SERVO_AXIS_DEFAULT_MIN_ANGLE_DEG  0.0f
#define ARM_Z_SERVO_AXIS_DEFAULT_MAX_ANGLE_DEG 270.0f
#define ARM_Z_SERVO_AXIS_DEFAULT_SERVO_ID        1U
#define ARM_Z_SERVO_AXIS_DEFAULT_RUN_TIME_MS   200U

typedef enum
{
    ARM_Z_SERVO_AXIS_OK = 0,
    ARM_Z_SERVO_AXIS_INVALID_PARAM,
    ARM_Z_SERVO_AXIS_NOT_READY,
    ARM_Z_SERVO_AXIS_SERVO_ERROR
} arm_z_servo_axis_result_t;

typedef struct
{
    float min_angle_deg;
    float max_angle_deg;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    float zero_angle_deg;
    uint8_t reverse;
    uint8_t servo_id;
    uint16_t run_time_ms;
} arm_z_servo_axis_config_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    arm_z_servo_axis_config_t config;
    float target_angle_deg;
    uint8_t initialized;
    uint8_t hardware_bound;
} arm_z_servo_axis_t;

void arm_z_servo_axis_config_default(arm_z_servo_axis_config_t *config);

arm_z_servo_axis_result_t arm_z_servo_axis_bind(
    arm_z_servo_axis_t *axis,
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    const arm_z_servo_axis_config_t *config);

/** 当前硬件绑定：USART2_TX / PA2，总线舵机 ID 由默认配置指定。 */
arm_z_servo_axis_result_t arm_z_servo_axis_bind_usart2(
    arm_z_servo_axis_t *axis,
    const arm_z_servo_axis_config_t *config);

arm_z_servo_axis_result_t arm_z_servo_axis_start(
    arm_z_servo_axis_t *axis);
arm_z_servo_axis_result_t arm_z_servo_axis_move_to(
    arm_z_servo_axis_t *axis,
    float angle_deg);
arm_z_servo_axis_result_t arm_z_servo_axis_move_to_position(
    arm_z_servo_axis_t *axis,
    uint16_t position);
arm_z_servo_axis_result_t arm_z_servo_axis_stop(
    arm_z_servo_axis_t *axis);
arm_z_servo_axis_result_t arm_z_servo_axis_set_zero(
    arm_z_servo_axis_t *axis);

float arm_z_servo_axis_get_target_angle(
    const arm_z_servo_axis_t *axis);

#endif /* ENGINEERING_INNOVATION_ARM_Z_SERVO_AXIS_H */
