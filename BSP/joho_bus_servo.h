#ifndef ENGINEERING_INNOVATION_JOHO_BUS_SERVO_H
#define ENGINEERING_INNOVATION_JOHO_BUS_SERVO_H

#include "main.h"

#include <stdint.h>

/* JOHO 协议位置范围；具体机械零点仍需按实物标定。 */
#define JOHO_BUS_SERVO_POSITION_MIN  0U
#define JOHO_BUS_SERVO_POSITION_MAX  4095U

typedef enum
{
    JOHO_BUS_SERVO_OK = 0,
    JOHO_BUS_SERVO_INVALID_PARAM,
    JOHO_BUS_SERVO_UART_ERROR
} joho_bus_servo_result_t;

/**
 * 通过 USART TX 单线发送位置控制帧。
 * 当前接口不等待舵机应答，适用于 MCU TX 直连信号线的测试连接。
 */
joho_bus_servo_result_t joho_bus_servo_set_position(
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    uint16_t position,
    uint16_t run_time_ms);

/** 将机械角度按指定范围线性换算为 0~4095 的协议位置。 */
joho_bus_servo_result_t joho_bus_servo_set_angle(
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    uint16_t angle_deg,
    uint16_t max_angle_deg,
    uint16_t run_time_ms);

#endif /* ENGINEERING_INNOVATION_JOHO_BUS_SERVO_H */
