#ifndef ENGINEERING_INNOVATION_SERVO_H
#define ENGINEERING_INNOVATION_SERVO_H

#include "main.h"

#include <stdint.h>

// 舵机所用定时器应配置为 1 MHz 计数频率，比较值 1 对应 1 us。
#define SERVO_PERIOD_US             20000U
#define SERVO_MIN_ANGLE_DEG         0U
#define SERVO_MAX_ANGLE_DEG         180U
#define SERVO_DEFAULT_MIN_PULSE_US  500U
#define SERVO_DEFAULT_MAX_PULSE_US  2500U

typedef enum
{
    SERVO_OK = 0,
    SERVO_ERROR,
    SERVO_INVALID_PARAM
} servo_result_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
} servo_t;

/** 初始化舵机对象，默认使用 500-2500 us 对应 0-180 度。 */
servo_result_t servo_init(servo_t *servo,
                          TIM_HandleTypeDef *htim,
                          uint32_t channel);

/** 启动舵机 PWM 输出。 */
servo_result_t servo_start(servo_t *servo);

/** 设置舵机角度，按默认 0~180 度范围换算。 */
servo_result_t servo_set_angle(servo_t *servo, uint16_t angle_deg);

/** 按指定机械角度范围换算舵机角度，例如 0~270 度。 */
servo_result_t servo_set_angle_with_range(servo_t *servo,
                                          uint16_t angle_deg,
                                          uint16_t max_angle_deg);

/** 直接设置脉宽，超出标定范围时自动限幅。 */
servo_result_t servo_set_pulse_us(servo_t *servo, uint16_t pulse_us);

/** 修改舵机 0 度和 180 度对应的脉宽。 */
servo_result_t servo_set_pulse_range(servo_t *servo,
                                     uint16_t min_pulse_us,
                                     uint16_t max_pulse_us);

/** 停止舵机 PWM 输出。 */
servo_result_t servo_stop(servo_t *servo);

#endif // ENGINEERING_INNOVATION_SERVO_H
