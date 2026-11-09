#ifndef ENGINEERING_INNOVATION_SERVO_UART_TEST_H
#define ENGINEERING_INNOVATION_SERVO_UART_TEST_H

#include "main.h"

typedef enum
{
    SERVO_UART_TEST_OK = 0,
    SERVO_UART_TEST_SERVO_ERROR,
    SERVO_UART_TEST_UART_ERROR
} servo_uart_test_result_t;

/** 初始化三路舵机和 USART6 单字节中断接收。 */
servo_uart_test_result_t servo_uart_test_init(void);

/** 在主循环中调用，处理 S1/S2/S3 控制命令。 */
void servo_uart_test_task(void);

/** 由统一 HAL UART 回调转发 USART6 接收完成事件。 */
void servo_uart_test_on_rx_complete(UART_HandleTypeDef *huart);

/** 由统一 HAL UART 回调转发 USART6 错误事件。 */
void servo_uart_test_on_error(UART_HandleTypeDef *huart);

#endif /* ENGINEERING_INNOVATION_SERVO_UART_TEST_H */
