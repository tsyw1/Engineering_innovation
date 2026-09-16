#ifndef ENGINEERING_INNOVATION_SERVO_H
#define ENGINEERING_INNOVATION_SERVO_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

// TIM1 配置为 1 MHz 计数频率、20 ms 周期，因此比较值直接等于脉宽 us。
#define SERVO_PERIOD_US             20000U
#define SERVO_MIN_ANGLE_DEG         0U
#define SERVO_MAX_ANGLE_DEG         180U
#define SERVO_DEFAULT_MIN_PULSE_US  500U
#define SERVO_DEFAULT_MAX_PULSE_US  2500U

typedef enum
{
    SERVO_OK = 0,
    SERVO_INVALID_PARAM,
    SERVO_HAL_ERROR
} servo_result_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    uint16_t angle_deg;
    bool started;
} servo_t;

/** 绑定一个已经由 CubeMX 配置好的 PWM 通道。 */
/*
 * 使用示例：
 *   servo_t servo1;
 *   servo_init(&servo1, &htim1, TIM_CHANNEL_1);
 *   servo_start(&servo1);
 *   servo_set_angle(&servo1, 90U);
 */
servo_result_t servo_init(servo_t *servo,
                          TIM_HandleTypeDef *htim,
                          uint32_t channel);

/** 修改 0~180 度对应的脉宽范围，默认值为 500~2500 us。 */
servo_result_t servo_set_pulse_range(servo_t *servo,
                                     uint16_t min_pulse_us,
                                     uint16_t max_pulse_us);

/** 启动或停止该舵机对应的 PWM 输出。 */
servo_result_t servo_start(servo_t *servo);
servo_result_t servo_stop(servo_t *servo);

/** 设置舵机角度，输入范围 0~180 度。 */
servo_result_t servo_set_angle(servo_t *servo, uint16_t angle_deg);

/** 直接设置高电平脉宽，范围由 servo_set_pulse_range() 限定。 */
servo_result_t servo_set_pulse_us(servo_t *servo, uint16_t pulse_us);

#endif // ENGINEERING_INNOVATION_SERVO_H
