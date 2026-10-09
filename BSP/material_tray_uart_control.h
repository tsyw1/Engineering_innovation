#ifndef ENGINEERING_INNOVATION_MATERIAL_TRAY_UART_CONTROL_H
#define ENGINEERING_INNOVATION_MATERIAL_TRAY_UART_CONTROL_H

#include "main.h"

typedef enum
{
    MATERIAL_TRAY_UART_CONTROL_OK = 0,
    MATERIAL_TRAY_UART_CONTROL_SERVO_ERROR,
    MATERIAL_TRAY_UART_CONTROL_UART_ERROR
} material_tray_uart_control_result_t;

/** 初始化物料盘舵机和 USART6 单字节中断接收。 */
material_tray_uart_control_result_t material_tray_uart_control_init(void);

/** 在主循环中调用，解析 USART6 接收到的物料盘角度，范围 0~270 度。 */
void material_tray_uart_control_task(void);

/** 由统一 HAL UART 回调转发 USART6 接收完成事件。 */
void material_tray_uart_control_on_rx_complete(UART_HandleTypeDef *huart);

/** 由统一 HAL UART 回调转发 USART6 错误事件。 */
void material_tray_uart_control_on_error(UART_HandleTypeDef *huart);

#endif /* ENGINEERING_INNOVATION_MATERIAL_TRAY_UART_CONTROL_H */
