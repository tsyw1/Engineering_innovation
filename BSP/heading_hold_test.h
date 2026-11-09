#ifndef ENGINEERING_INNOVATION_HEADING_HOLD_TEST_H
#define ENGINEERING_INNOVATION_HEADING_HOLD_TEST_H

#include "HWT101CT.h"
#include "chassis_mecanum.h"
#include "main.h"

typedef enum
{
    HEADING_HOLD_TEST_OK = 0,
    HEADING_HOLD_TEST_INVALID_PARAM,
    HEADING_HOLD_TEST_CONTROLLER_ERROR,
    HEADING_HOLD_TEST_UART_ERROR
} heading_hold_test_result_t;

/** 初始化航向矫正测试，并启动参数串口的单字节中断接收。 */
heading_hold_test_result_t heading_hold_test_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *hwt101ct_sensor,
    UART_HandleTypeDef *command_uart);

/** 主循环周期调用，执行航向矫正、命令处理和状态输出。 */
void heading_hold_test_task(void);

/** 由统一 HAL UART 回调转发参数串口接收完成事件。 */
void heading_hold_test_on_rx_complete(UART_HandleTypeDef *huart);

/** 由统一 HAL UART 回调转发参数串口错误事件。 */
void heading_hold_test_on_error(UART_HandleTypeDef *huart);

#endif /* ENGINEERING_INNOVATION_HEADING_HOLD_TEST_H */
