#ifndef GROUND_PLACE_TEST_HAL_H
#define GROUND_PLACE_TEST_HAL_H

#include <stdint.h>

/* Host-only HAL types; production headers and control code are compiled intact. */
typedef struct { uint32_t unused; } CAN_HandleTypeDef;
typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

#endif
