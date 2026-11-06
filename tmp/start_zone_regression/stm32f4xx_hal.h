#ifndef START_ZONE_TEST_HAL_H
#define START_ZONE_TEST_HAL_H

#include <stdint.h>

typedef struct { uint32_t unused; } CAN_HandleTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_RX } HAL_UART_StateTypeDef;
typedef struct
{
    uint32_t ErrorCode;
    HAL_UART_StateTypeDef RxState;
    uint8_t *rx_buffer;
} UART_HandleTypeDef;

#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_FE   4U
#define HAL_UART_ERROR_ORE  8U
#define __HAL_UART_CLEAR_OREFLAG(huart) mock_clear_uart_errors(huart)

void mock_clear_uart_errors(UART_HandleTypeDef *huart);

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
uint32_t __get_PRIMASK(void);
void __set_PRIMASK(uint32_t value);
void __disable_irq(void);
void __enable_irq(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data,
                                  uint16_t length, uint32_t timeout_ms);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
