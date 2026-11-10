//
// 三工位物料盘舵机驱动，固定使用 TIM1_CH2（PE11 / Servo2）。
//

#ifndef ENGINEERING_INNOVATION_MATERIAL_TRAY_SERVO_H
#define ENGINEERING_INNOVATION_MATERIAL_TRAY_SERVO_H

#include "servo.h"

#include <stdint.h>

#define MATERIAL_TRAY_POSITION_COUNT          3U
#define MATERIAL_TRAY_SERVO_MAX_ANGLE_DEG      180U
#define MATERIAL_TRAY_SERVO_MIN_PULSE_US       400U
#define MATERIAL_TRAY_SERVO_MAX_PULSE_US       2500U
#define MATERIAL_TRAY_DEFAULT_POSITION_1_DEG  15U
#define MATERIAL_TRAY_DEFAULT_POSITION_2_DEG  60U
#define MATERIAL_TRAY_DEFAULT_POSITION_3_DEG  104U

typedef enum
{
    MATERIAL_TRAY_SERVO_OK = 0,
    MATERIAL_TRAY_SERVO_INVALID_PARAM,
    MATERIAL_TRAY_SERVO_OUT_OF_RANGE,
    MATERIAL_TRAY_SERVO_ERROR,
    MATERIAL_TRAY_SERVO_NOT_READY
} material_tray_servo_result_t;

typedef enum
{
    MATERIAL_TRAY_POSITION_1 = 0,
    MATERIAL_TRAY_POSITION_2,
    MATERIAL_TRAY_POSITION_3,
    MATERIAL_TRAY_POSITION_INVALID
} material_tray_position_t;

typedef struct
{
    uint16_t position_angle_deg[MATERIAL_TRAY_POSITION_COUNT];
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    material_tray_position_t initial_position;
} material_tray_servo_config_t;

typedef struct
{
    servo_t servo;
    material_tray_servo_config_t config;
    material_tray_position_t current_position;
    uint8_t pwm_started;
    uint8_t initialized;
} material_tray_servo_t;

/** 生成 180 度舵机默认配置：1号位 15 度，2号位 60 度，3号位 104 度。 */
void material_tray_servo_config_default(
    material_tray_servo_config_t *config);

/** 初始化 TIM1_CH2 舵机对象，不输出 PWM，不自动转动物料盘。 */
material_tray_servo_result_t material_tray_servo_init(
    material_tray_servo_t *tray,
    const material_tray_servo_config_t *config);

material_tray_servo_result_t material_tray_servo_set_position(
    material_tray_servo_t *tray,
    material_tray_position_t position);

material_tray_servo_result_t material_tray_servo_next(
    material_tray_servo_t *tray);

material_tray_servo_result_t material_tray_servo_previous(
    material_tray_servo_t *tray);

material_tray_servo_result_t material_tray_servo_stop(
    material_tray_servo_t *tray);

material_tray_position_t material_tray_servo_get_position(
    const material_tray_servo_t *tray);

#endif /* ENGINEERING_INNOVATION_MATERIAL_TRAY_SERVO_H */
