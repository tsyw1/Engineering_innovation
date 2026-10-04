/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "IMU_UART.h"
#include "IMU_GYRO_CALIBRATION.h"
#include "IMU_DATA_LOGGER.h"

#include <stdio.h>
#include <string.h>

/* 手动旋转测试参数。 */
#define IMU_ROTATION_BASELINE_TIME_MS   500U
#define IMU_ROTATION_LOOP_DELAY_MS      5U
#define IMU_TEST_STATUS_PERIOD_MS       500U
#define IMU_TEST_TX_TIMEOUT_MS          100U
#define IMU_TEST_LINE_BUFFER_SIZE       192U

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void imu_yaw_prime(void);
static void imu_test_write_text(const char *text);
static void imu_test_write_status(const char *status);

/* Private user code ---------------------------------------------------------*/
static jy901s_uart_t s_imu_sensor;
static imu_gyro_calibration_t s_imu_calibration;
static imu_data_logger_t s_imu_logger;

static void imu_test_write_text(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart6,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          IMU_TEST_TX_TIMEOUT_MS);
}

static void imu_test_write_status(const char *status)
{
  char line[IMU_TEST_LINE_BUFFER_SIZE];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "imu_test,status,%s,time_ms,%lu,frames,%lu,angle_frames,%lu,gyro_frames,%lu,checksum_errors,%lu,uart_errors,%lu,bias_dps,%.6f,yaw_deg,%.6f\r\n",
                    (status != NULL) ? status : "unknown",
                    (unsigned long)HAL_GetTick(),
                    (unsigned long)s_imu_sensor.frame_count,
                    (unsigned long)s_imu_sensor.angle_frame_count,
                    (unsigned long)s_imu_sensor.angular_velocity_frame_count,
                    (unsigned long)s_imu_sensor.checksum_error_count,
                    (unsigned long)s_imu_sensor.uart_error_count,
                    (double)imu_gyro_calibration_get_bias(&s_imu_calibration),
                    (double)imu_data_logger_get_compensated_yaw_deg(&s_imu_logger));
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    imu_test_write_text(line);
  }
}

static void imu_yaw_prime(void)
{
  uint32_t start_tick = HAL_GetTick();

  while ((uint32_t)(HAL_GetTick() - start_tick) <
         IMU_ROTATION_BASELINE_TIME_MS)
  {
    imu_data_logger_task(&s_imu_logger);
    HAL_Delay(IMU_ROTATION_LOOP_DELAY_MS);
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  jy901s_uart_result_t imu_result;
  jy901s_uart_result_t calibration_result;
  uint32_t next_status_tick;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART6_UART_Init();
  imu_test_write_text("imu_test,boot,USART6_ready\r\n");
  MX_USART1_UART_Init();

  imu_result = imu_data_logger_init_default(&s_imu_logger,
                                            &s_imu_sensor,
                                            &huart1,
                                            &huart6);
  if (imu_result != JY901S_UART_OK)
  {
    imu_test_write_text("imu_test,error,logger_init_failed\r\n");
    while (1)
    {
      imu_test_write_text("imu_test,status,logger_not_ready\r\n");
      HAL_Delay(1000U);
    }
  }

  /* 上电后保持小车静止，计算 Z 轴陀螺仪零偏。 */
  imu_test_write_text("imu_test,calibration,begin_keep_stationary\r\n");
  calibration_result = imu_gyro_calibration_run_default(&s_imu_calibration,
                                                         &s_imu_sensor);
  if (calibration_result != JY901S_UART_OK)
  {
    /* 校准失败时仍继续输出原始/未补偿结果，便于定位 USART1 接收问题。 */
    s_imu_calibration.gyro_z_bias_dps = 0.0f;
    s_imu_calibration.calibrated = 0U;
    imu_test_write_text("imu_test,calibration,failed_continue_without_bias\r\n");
    imu_test_write_status("calibration_failed");
  }
  else
  {
    imu_test_write_status("calibration_ok");
  }
  imu_data_logger_set_gyro_z_bias(
      &s_imu_logger,
      imu_gyro_calibration_get_bias(&s_imu_calibration));
  imu_yaw_prime();
  imu_test_write_status("running");
  next_status_tick = HAL_GetTick() + IMU_TEST_STATUS_PERIOD_MS;

  while (1)
  {
    /* 手动旋转小车时，USART6 持续输出补偿后的偏航角。 */
    imu_data_logger_task(&s_imu_logger);
    if ((int32_t)(HAL_GetTick() - next_status_tick) >= 0)
    {
      imu_test_write_status("running");
      next_status_tick = HAL_GetTick() + IMU_TEST_STATUS_PERIOD_MS;
    }
    HAL_Delay(IMU_ROTATION_LOOP_DELAY_MS);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
