/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor_can.h"

#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 地址修改代码暂时注释，电机测试完成后可重新启用。 */
#if 0
#define MOTOR_OLD_ADDRESS            2U
#define MOTOR_NEW_ADDRESS            4U
#define MOTOR_ADDRESS_WAIT_MS        500U
#endif

/* 四电机运行和编码器变化量采样参数。 */
#define MOTOR_TEST_SPEED_RPM         500
#define MOTOR_TEST_ACCELERATION      20U
#define MOTOR_TEST_RUN_TIME_MS       3000U
#define MOTOR_TEST_SAMPLE_PERIOD_MS  100U
#define MOTOR_TEST_MOTOR_COUNT       4U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static motor_bus_t s_motor_bus;
static motor_t s_test_motors[MOTOR_TEST_MOTOR_COUNT];
#if 1
static motor_t s_address_motor;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void motor_can_four_motor_encoder_test(void);
#if 0
static void motor_can_change_address_run(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void motor_can_send_text(const char *text)
{
  if (text != NULL)
  {
    /* USART1 输出测试状态，便于通过串口助手观察执行阶段。 */
    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            100U);
  }
}

#if 0
static char motor_can_hex(uint8_t value)
{
  value &= 0x0FU;
  return (value < 10U) ? (char)('0' + value) :
         (char)('A' + value - 10U);
}

static void motor_can_send_address(uint8_t address)
{
  char text[64];

  (void)snprintf(text,
                 sizeof(text),
                 "Motor CAN ID: 0x%c%c (decimal %u)\r\n",
                 motor_can_hex((uint8_t)(address >> 4U)),
                 motor_can_hex(address),
                 (unsigned int)address);
  motor_can_send_text(text);
}

static void motor_can_change_address_run(void)
{
  uint8_t discovered_address;
  motor_result_t result;

  motor_can_send_text("CAN motor address calibration start\r\n");
  motor_can_send_text("Change address 1 -> 3, save = 1\r\n");

  /* 修改前必须使用电机当前旧地址注册对象。 */
  result = motor_init(&s_address_motor,
                      &s_motor_bus,
                      MOTOR_OLD_ADDRESS);
  if (result != MOTOR_OK)
  {
    motor_can_send_text("Motor object init failed\r\n");
    return;
  }

  /* 原生 CAN 帧：扩展 ID 0x0100，数据 AE 4B 01 03 6B。 */
  result = motor_set_address(&s_address_motor,
                             MOTOR_NEW_ADDRESS,
                             true);
  if (result != MOTOR_OK)
  {
    motor_can_send_text("Address change command failed\r\n");
    return;
  }

  motor_can_send_text("Address change command sent\r\n");
  HAL_Delay(MOTOR_ADDRESS_WAIT_MS);

  /* 修改后使用广播命令读取当前地址。 */
  if (motor_broadcast_read_id(&s_motor_bus) != MOTOR_OK)
  {
    motor_can_send_text("Broadcast ID read send failed\r\n");
    return;
  }

  HAL_Delay(MOTOR_ADDRESS_WAIT_MS);
  if (motor_bus_get_discovered_id(&s_motor_bus, &discovered_address))
  {
    motor_can_send_text("Broadcast ID read response:\r\n");
    motor_can_send_address(discovered_address);
  }
  else
  {
    motor_can_send_text("No motor ID response\r\n");
  }
}
#endif /* 地址修改代码 */

static void motor_can_print_encoder(uint8_t motor_id,
                                    const motor_t *motor)
{
  char text[96];

  if (motor == NULL)
  {
    return;
  }

  (void)snprintf(text,
                 sizeof(text),
                 "M%u encoder_delta=%d encoder_abs=%u valid=%u\r\n",
                 (unsigned int)motor_id,
                 (int)motor->encoder_delta,
                 (unsigned int)motor->encoder_raw,
                 (unsigned int)motor->encoder_valid);
  motor_can_send_text(text);
}

static void motor_can_four_motor_encoder_test(void)
{
  static const uint8_t motor_ids[MOTOR_TEST_MOTOR_COUNT] = {1U,2U,3U,4U};
  uint8_t index;
  uint32_t elapsed_ms;
  motor_result_t result;

  motor_can_send_text("Four motor CAN encoder test start\r\n");
  motor_can_send_text("Motor IDs: 1, 2, 3, 4\r\n");

  for (index = 0U; index < MOTOR_TEST_MOTOR_COUNT; index++)
  {
    result = motor_init(&s_test_motors[index],
                        &s_motor_bus,
                        motor_ids[index]);
    if (result != MOTOR_OK)
    {
      motor_can_send_text("Motor object init failed\r\n");
      return;
    }
  }

  /* 先逐台使能。 */
  for (index = 0U; index < MOTOR_TEST_MOTOR_COUNT; index++)
  {
    result = motor_enable(&s_test_motors[index],
                          true,
                          MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
      motor_can_send_text("Motor enable failed\r\n");
      (void)motor_stop_all(&s_motor_bus);
      return;
    }
  }

  HAL_Delay(100U);

  /* 连续发送四个立即执行速度命令。 */
  for (index = 0U; index < MOTOR_TEST_MOTOR_COUNT; index++)
  {
    result = motor_set_speed(&s_test_motors[index],
                             MOTOR_TEST_SPEED_RPM,
                             MOTOR_TEST_ACCELERATION,
                             MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
      motor_can_send_text("Motor speed command failed\r\n");
      (void)motor_stop_all(&s_motor_bus);
      return;
    }
  }

  motor_can_send_text("Four motors running at 100 RPM\r\n");

  /* 运行期间轮询读取 0x31 编码器值，并打印相邻采样变化量。 */
  for (elapsed_ms = 0U;
       elapsed_ms < MOTOR_TEST_RUN_TIME_MS;
       elapsed_ms += MOTOR_TEST_SAMPLE_PERIOD_MS)
  {
    for (index = 0U; index < MOTOR_TEST_MOTOR_COUNT; index++)
    {
      result = motor_read_encoder(&s_test_motors[index]);
      if (result != MOTOR_OK)
      {
        motor_can_send_text("Encoder read command failed\r\n");
      }
    }

    /* 等待 CAN 接收中断完成四台电机的反馈。 */
    HAL_Delay(20U);
    for (index = 0U; index < MOTOR_TEST_MOTOR_COUNT; index++)
    {
      motor_can_print_encoder(motor_ids[index], &s_test_motors[index]);
    }

    if ((MOTOR_TEST_SAMPLE_PERIOD_MS > 20U))
    {
      HAL_Delay(MOTOR_TEST_SAMPLE_PERIOD_MS - 20U);
    }
  }

  if (motor_stop_all(&s_motor_bus) == MOTOR_OK)
  {
    motor_can_send_text("Four motor CAN encoder test stopped\r\n");
  }
  else
  {
    motor_can_send_text("Motor stop command failed\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  if (motor_bus_init(&s_motor_bus,
                     &hcan1,
                     motor_can_hal_send_frame,
                     20U) != MOTOR_OK)
  {
    Error_Handler();
  }

  if (motor_bus_start(&s_motor_bus) != MOTOR_OK)
  {
    Error_Handler();
  }

  /* 当前执行四电机运行和编码器变化量测试。 */
  motor_can_four_motor_encoder_test();
  // motor_can_change_address_run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
