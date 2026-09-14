/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_TX_Pin GPIO_PIN_2
#define Motor_TX_GPIO_Port GPIOA
#define Motor_RX_Pin GPIO_PIN_3
#define Motor_RX_GPIO_Port GPIOA
#define Servo1_Pin GPIO_PIN_9
#define Servo1_GPIO_Port GPIOE
#define Servo2_Pin GPIO_PIN_11
#define Servo2_GPIO_Port GPIOE
#define Version_TX_Pin GPIO_PIN_8
#define Version_TX_GPIO_Port GPIOD
#define Version_RX_Pin GPIO_PIN_9
#define Version_RX_GPIO_Port GPIOD
#define QR_TX_Pin GPIO_PIN_6
#define QR_TX_GPIO_Port GPIOC
#define QR_RX_Pin GPIO_PIN_7
#define QR_RX_GPIO_Port GPIOC
#define Screen_TX_Pin GPIO_PIN_12
#define Screen_TX_GPIO_Port GPIOC
#define Screen_RX_Pin GPIO_PIN_2
#define Screen_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
