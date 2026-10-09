/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能搬运比赛流程入口
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "material_tray_servo.h"
#include "motor_can.h"
#include "QR.h"
#include "screen.h"
#include "IMU_UART.h"
#include "IMU_GYRO_CALIBRATION.h"
#include "MECANUM_INERTIAL_NAV.h"
#include "task1_start_qr.h"
#include "task2_platform_pick.h"
#include "task3_coarse_ring.h"
#include "task4_ring_return.h"
#include "task5_temporary_ring.h"
#include "task6_platform_pick.h"
#include "task7_coarse_ring.h"
#include "task8_ring_return.h"
#include "task9_temporary_stack.h"
#include "task10_return_home.h"
#include "vision_uart.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_MOTOR_POWER_SETTLE_MS  2000U
#define APP_CAN_SETTLE_MS            200U
#define APP_AUTO_START_DELAY_MS     5000U
#define APP_POSITION_REPORT_PERIOD_MS 200U

static motor_bus_t s_motor_bus;
static motor_t s_chassis_motors[MECANUM_WHEEL_COUNT];
static arm_controller_t s_arm;
static mecanum_chassis_t s_chassis;
static jy901s_uart_t s_imu_sensor;
static imu_gyro_calibration_t s_imu_calibration;
static mecanum_inertial_nav_t s_inertial_nav;
static material_tray_servo_t s_material_tray;
static material_tray_servo_config_t s_material_tray_config;
static vision_fine_adjust_t s_fine_adjust;
static screen_t s_screen;
static task1_start_qr_t s_task1;
static task2_platform_pick_t s_task2;
static task3_coarse_ring_t s_task3;
static task4_ring_return_t s_task4;
static task5_temporary_ring_t s_task5;
static task6_platform_pick_t s_task6;
static task7_coarse_ring_t s_task7;
static task8_ring_return_t s_task8;
static task9_temporary_stack_t s_task9;
static task10_return_home_t s_task10;
static uint8_t s_active_task_number;
static uint32_t s_last_position_report_tick;

void SystemClock_Config(void);

static void app_write_status(uint8_t status, const char *event)
{
    char line[128];
    int length;
    uint8_t task_number;

    if (event == NULL)
    {
        event = "unknown";
    }
    task_number = s_active_task_number;
    if ((task_number == 0U) && (strncmp(event, "task1_", 6U) == 0))
    {
        task_number = 1U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task2_", 6U) == 0))
    {
        task_number = 2U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task3_", 6U) == 0))
    {
        task_number = 3U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task4_", 6U) == 0))
    {
        task_number = 4U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task5_", 6U) == 0))
    {
        task_number = 5U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task6_", 6U) == 0))
    {
        task_number = 6U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task7_", 6U) == 0))
    {
        task_number = 7U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task8_", 6U) == 0))
    {
        task_number = 8U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task9_", 6U) == 0))
    {
        task_number = 9U;
    }
    else if ((task_number == 0U) && (strncmp(event, "task10_", 7U) == 0))
    {
        task_number = 10U;
    }
    length = snprintf(line,
                      sizeof(line),
                      "COMPETITION,STATUS,TASK%u,VALUE=%u,%s\r\n",
                      (unsigned int)task_number,
                      (unsigned int)status,
                      event);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart6,
                                (uint8_t *)line,
                                (uint16_t)length,
                                100U);
    }
}

static void app_report_error(const char *event)
{
    char line[128];
    int length;

    if (event == NULL)
    {
        event = "unknown";
    }
    length = snprintf(line,
                      sizeof(line),
                      "COMPETITION,ERROR,TASK%u,%s\r\n",
                      (unsigned int)s_active_task_number,
                      event);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart6,
                                (uint8_t *)line,
                                (uint16_t)length,
                                100U);
    }
    Error_Handler();
}

static void app_report_chassis_position(void)
{
    uint32_t now;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mdeg;
    char line[160];
    int length;

    /* 反馈由 CAN 中断异步更新；每次循环都尝试消费最新的一致样本。 */
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_chassis_update_encoder_odometry(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    /* 底盘运动期间周期性重发Z轴绝对目标，抵抗负载惯性造成的偏转。 */
    (void)arm_controller_hold_z_position(&s_arm);

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_position_report_tick) <
        APP_POSITION_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_position_report_tick = now;

    if ((s_inertial_nav.initialized == 0U) ||
        (s_inertial_nav.ekf_initialized == 0U))
    {
        length = snprintf(line,
                          sizeof(line),
                          "COMPETITION,POSITION,TASK%u,VALID=0\r\n",
                          (unsigned int)s_active_task_number);
    }
    else
    {
        x_mm = (int32_t)(s_inertial_nav.x_m * 1000.0f);
        y_mm = (int32_t)(s_inertial_nav.y_m * 1000.0f);
        yaw_mdeg = (int32_t)(s_inertial_nav.yaw_rad *
                             57295.7795f);
        length = snprintf(line,
                          sizeof(line),
                          "COMPETITION,POSITION,TASK%u,SOURCE=INERTIAL,VALID=1,X_MM=%ld,Y_MM=%ld,YAW_MDEG=%ld\r\n",
                          (unsigned int)s_active_task_number,
                          (long)x_mm,
                          (long)y_mm,
                          (long)yaw_mdeg);
    }
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart6,
                                (uint8_t *)line,
                                (uint16_t)length,
                                100U);
    }
}

static uint8_t app_prepare(void)
{
    motor_result_t motor_result;
    arm_controller_result_t arm_result;
    mecanum_result_t chassis_result;
    jy901s_uart_result_t imu_result;
    mecanum_inertial_nav_result_t nav_result;
    vision_uart_result_t vision_result;
    task1_start_qr_result_t task1_result;
    task2_platform_pick_result_t task2_result;
    task3_coarse_ring_result_t task3_result;
    task4_ring_return_result_t task4_result;
    task5_temporary_ring_result_t task5_result;
    task6_platform_pick_result_t task6_result;
    task7_coarse_ring_result_t task7_result;
    task8_ring_return_result_t task8_result;
    task9_temporary_stack_result_t task9_result;
    task10_return_home_result_t task10_result;

    /* 等待电机驱动器和 CAN 收发器完成上电自检。 */
    HAL_Delay(APP_MOTOR_POWER_SETTLE_MS);

    motor_result = motor_bus_init(&s_motor_bus,
                                  &hcan1,
                                  motor_can_hal_send_frame,
                                  100U);
    if ((motor_result != MOTOR_OK) ||
        (motor_bus_start(&s_motor_bus) != MOTOR_OK))
    {
        app_report_error("can_start_failed");
        return 0U;
    }
    HAL_Delay(APP_CAN_SETTLE_MS);

    arm_result = arm_controller_init(&s_arm, &s_motor_bus);
    if (arm_result != ARM_CONTROLLER_OK)
    {
        app_report_error("arm_init_failed");
        return 0U;
    }
    /* 上电阶段不执行机械臂调零，调零在第一段左移过程中执行。 */

    chassis_result = mecanum_chassis_init_default(&s_chassis,
                                                  &s_motor_bus,
                                                  s_chassis_motors);
    if (chassis_result != MECANUM_OK)
    {
        app_report_error("chassis_init_failed");
        return 0U;
    }

    imu_result = jy901s_uart_init(&s_imu_sensor, &huart1);
    if (imu_result != JY901S_UART_OK)
    {
        app_report_error("imu_init_failed");
        return 0U;
    }
    if (jy901s_uart_configure_output(
            &s_imu_sensor,
            JY901S_UART_OUTPUT_GYRO_ACCEL,
            JY901S_UART_RATE_100HZ) != JY901S_UART_OK)
    {
        app_report_error("imu_configure_failed");
        return 0U;
    }

    imu_result = imu_gyro_calibration_run_default(&s_imu_calibration,
                                                  &s_imu_sensor);
    if ((imu_result != JY901S_UART_OK) ||
        (s_imu_calibration.calibrated == 0U))
    {
        app_report_error("imu_calibration_failed");
        return 0U;
    }

    nav_result = mecanum_inertial_nav_init_default(
        &s_inertial_nav,
        &s_chassis.config,
        imu_gyro_calibration_get_bias(&s_imu_calibration));
    if (nav_result != MECANUM_INERTIAL_NAV_OK)
    {
        app_report_error("inertial_nav_init_failed");
        return 0U;
    }
    mecanum_chassis_attach_inertial_nav(&s_chassis,
                                        &s_inertial_nav,
                                        &s_imu_sensor);
    if (mecanum_chassis_enable(&s_chassis, true) != MECANUM_OK)
    {
        app_report_error("chassis_enable_failed");
        return 0U;
    }

    material_tray_servo_config_default(&s_material_tray_config);
    if (material_tray_servo_init(&s_material_tray,
                                 &s_material_tray_config) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        app_report_error("tray_init_failed");
        return 0U;
    }

    if (screen_init(&s_screen, &huart5) != SCREEN_OK)
    {
        app_report_error("screen_init_failed");
        return 0U;
    }
    if (screen_reset_statistics(&s_screen) != SCREEN_OK)
    {
        app_report_error("screen_reset_failed");
        return 0U;
    }

    if (QR_Init() != HAL_OK)
    {
        app_report_error("qr_uart_init_failed");
        return 0U;
    }

    vision_result = vision_uart_init(vision_uart_get_default(), &huart3);
    if ((vision_result != VISION_UART_RESULT_OK) ||
        (vision_uart_start_receive(vision_uart_get_default()) !=
         VISION_UART_RESULT_OK))
    {
        app_report_error("material_vision_uart_init_failed");
        return 0U;
    }

    s_active_task_number = 1U;
    task1_result = task1_start_qr_init(&s_task1,
                                       &s_arm,
                                       &s_chassis,
                                       &s_screen,
                                       app_write_status);
    if (task1_result != TASK1_START_QR_OK)
    {
        app_report_error("task1_init_failed");
        return 0U;
    }

    s_active_task_number = 2U;
    task2_result = task2_platform_pick_init(&s_task2,
                                            &s_arm,
                                            &s_chassis,
                                            &s_fine_adjust,
                                            &s_material_tray,
                                            &s_screen,
                                            app_write_status);
    if (task2_result != TASK2_PLATFORM_PICK_OK)
    {
        app_report_error("task2_init_failed");
        return 0U;
    }

    s_active_task_number = 3U;
    task3_result = task3_coarse_ring_init(&s_task3,
                                          &s_arm,
                                          &s_chassis,
                                          &s_fine_adjust,
                                          &s_material_tray,
                                          &s_screen,
                                          app_write_status);
    if (task3_result != TASK3_COARSE_RING_OK)
    {
        app_report_error("task3_init_failed");
        return 0U;
    }

    s_active_task_number = 4U;
    task4_result = task4_ring_return_init(&s_task4,
                                          &s_arm,
                                          &s_chassis,
                                          &s_fine_adjust,
                                          &s_material_tray,
                                          &s_screen,
                                          app_write_status);
    if (task4_result != TASK4_RING_RETURN_OK)
    {
        app_report_error("task4_init_failed");
        return 0U;
    }

    s_active_task_number = 5U;
    task5_result = task5_temporary_ring_init(&s_task5,
                                             &s_arm,
                                             &s_chassis,
                                             &s_fine_adjust,
                                             &s_material_tray,
                                             &s_screen,
                                             app_write_status);
    if (task5_result != TASK5_TEMPORARY_RING_OK)
    {
        app_report_error("task5_init_failed");
        return 0U;
    }

    s_active_task_number = 6U;
    task6_result = task6_platform_pick_init(&s_task6,
                                            &s_arm,
                                            &s_chassis,
                                            &s_fine_adjust,
                                            &s_material_tray,
                                            &s_screen,
                                            app_write_status);
    if (task6_result != TASK6_PLATFORM_PICK_OK)
    {
        app_report_error("task6_init_failed");
        return 0U;
    }

    s_active_task_number = 7U;
    task7_result = task7_coarse_ring_init(&s_task7,
                                          &s_arm,
                                          &s_chassis,
                                          &s_fine_adjust,
                                          &s_material_tray,
                                          &s_screen,
                                          app_write_status);
    if (task7_result != TASK7_COARSE_RING_OK)
    {
        app_report_error("task7_init_failed");
        return 0U;
    }

    s_active_task_number = 8U;
    task8_result = task8_ring_return_init(&s_task8,
                                          &s_arm,
                                          &s_chassis,
                                          &s_fine_adjust,
                                          &s_material_tray,
                                          &s_screen,
                                          app_write_status);
    if (task8_result != TASK8_RING_RETURN_OK)
    {
        app_report_error("task8_init_failed");
        return 0U;
    }

    s_active_task_number = 9U;
    task9_result = task9_temporary_stack_init(&s_task9,
                                              &s_arm,
                                              &s_chassis,
                                              &s_fine_adjust,
                                              &s_material_tray,
                                              &s_screen,
                                              app_write_status);
    if (task9_result != TASK9_TEMPORARY_STACK_OK)
    {
        app_report_error("task9_init_failed");
        return 0U;
    }

    s_active_task_number = 10U;
    task10_result = task10_return_home_init(&s_task10,
                                            &s_chassis,
                                            &s_screen,
                                            app_write_status);
    if (task10_result != TASK10_RETURN_HOME_OK)
    {
        app_report_error("task10_init_failed");
        return 0U;
    }

    s_active_task_number = 0U;
    app_write_status(TASK1_START_QR_STATUS_INIT,
                     "competition_flow_ready");
    return 1U;
}

int main(void)
{
    uint32_t power_on_tick;
    uint8_t auto_start_sent = 0U;
    uint8_t task2_submitted = 0U;
    uint8_t task3_submitted = 0U;
    uint8_t task4_submitted = 0U;
    uint8_t task5_submitted = 0U;
    uint8_t task6_submitted = 0U;
    uint8_t task7_submitted = 0U;
    uint8_t task8_submitted = 0U;
    uint8_t task9_submitted = 0U;
    uint8_t task10_submitted = 0U;
    uint8_t flow_done_reported = 0U;
    QR_TaskCode_t qr_task;

    HAL_Init();
    power_on_tick = HAL_GetTick();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();
    MX_USART6_UART_Init();
    MX_CAN1_Init();

    if (app_prepare() == 0U)
    {
        Error_Handler();
    }

    while (1)
    {
        app_report_chassis_position();

        /* 无实体启动按键，以上电时刻起 5 s 后自动启动一次。 */
        if ((auto_start_sent == 0U) &&
            ((uint32_t)(HAL_GetTick() - power_on_tick) >=
             APP_AUTO_START_DELAY_MS))
        {
            s_active_task_number = 1U;
            task1_start_qr_request_start(&s_task1);
            auto_start_sent = 1U;
        }

        vision_uart_task(vision_uart_get_default());
        s_active_task_number = 1U;
        (void)task1_start_qr_step(&s_task1);

        if ((task2_submitted == 0U) &&
            (task1_start_qr_get_state(&s_task1) == TASK1_START_QR_DONE) &&
            (task1_start_qr_get_task(&s_task1, &qr_task) != 0U))
        {
            s_active_task_number = 2U;
            if (task2_platform_pick_submit_qr(&s_task2, &qr_task) ==
                TASK2_PLATFORM_PICK_OK)
            {
                task2_submitted = 1U;
            }
            else
            {
                app_report_error("task2_submit_failed");
            }
        }

        if (task2_submitted != 0U)
        {
            s_active_task_number = 2U;
            (void)task2_platform_pick_step(&s_task2);
        }

        if ((task3_submitted == 0U) &&
            (task2_platform_pick_get_state(&s_task2) ==
             TASK2_PLATFORM_PICK_DONE))
        {
            s_active_task_number = 3U;
            if (task3_coarse_ring_submit_qr(&s_task3, &qr_task) ==
                TASK3_COARSE_RING_OK)
            {
                task3_submitted = 1U;
            }
        }

        if (task3_submitted != 0U)
        {
            s_active_task_number = 3U;
            (void)task3_coarse_ring_step(&s_task3);
        }

        if ((task4_submitted == 0U) &&
            (task3_coarse_ring_get_state(&s_task3) ==
             TASK3_COARSE_RING_DONE))
        {
            s_active_task_number = 4U;
            if (task4_ring_return_submit_qr(&s_task4, &qr_task) ==
                TASK4_RING_RETURN_OK)
            {
                task4_submitted = 1U;
            }
        }

        if (task4_submitted != 0U)
        {
            s_active_task_number = 4U;
            (void)task4_ring_return_step(&s_task4);
        }

        if ((task5_submitted == 0U) &&
            (task4_ring_return_get_state(&s_task4) ==
             TASK4_RING_RETURN_DONE))
        {
            s_active_task_number = 5U;
            if (task5_temporary_ring_submit_qr(&s_task5, &qr_task) ==
                TASK5_TEMPORARY_RING_OK)
            {
                task5_submitted = 1U;
            }
        }

        if (task5_submitted != 0U)
        {
            s_active_task_number = 5U;
            (void)task5_temporary_ring_step(&s_task5);
        }

        if ((task6_submitted == 0U) &&
            (task5_temporary_ring_get_state(&s_task5) ==
             TASK5_TEMPORARY_RING_DONE))
        {
            s_active_task_number = 6U;
            if (task6_platform_pick_submit_qr(&s_task6, &qr_task) ==
                TASK6_PLATFORM_PICK_OK)
            {
                task6_submitted = 1U;
            }
        }

        if (task6_submitted != 0U)
        {
            s_active_task_number = 6U;
            (void)task6_platform_pick_step(&s_task6);
        }

        if ((task7_submitted == 0U) &&
            (task6_platform_pick_get_state(&s_task6) ==
             TASK6_PLATFORM_PICK_DONE))
        {
            s_active_task_number = 7U;
            if (task7_coarse_ring_submit_qr(&s_task7, &qr_task) ==
                TASK7_COARSE_RING_OK)
            {
                task7_submitted = 1U;
            }
        }

        if (task7_submitted != 0U)
        {
            s_active_task_number = 7U;
            (void)task7_coarse_ring_step(&s_task7);
        }

        if ((task8_submitted == 0U) &&
            (task7_coarse_ring_get_state(&s_task7) ==
             TASK7_COARSE_RING_DONE))
        {
            s_active_task_number = 8U;
            if (task8_ring_return_submit_qr(&s_task8, &qr_task) ==
                TASK8_RING_RETURN_OK)
            {
                task8_submitted = 1U;
            }
        }

        if (task8_submitted != 0U)
        {
            s_active_task_number = 8U;
            (void)task8_ring_return_step(&s_task8);
        }

        if ((task9_submitted == 0U) &&
            (task8_ring_return_get_state(&s_task8) ==
             TASK8_RING_RETURN_DONE))
        {
            s_active_task_number = 9U;
            if (task9_temporary_stack_submit_qr(&s_task9, &qr_task) ==
                TASK9_TEMPORARY_STACK_OK)
            {
                task9_submitted = 1U;
            }
        }

        if (task9_submitted != 0U)
        {
            s_active_task_number = 9U;
            (void)task9_temporary_stack_step(&s_task9);
        }

        if ((task10_submitted == 0U) &&
            (task9_temporary_stack_get_state(&s_task9) ==
             TASK9_TEMPORARY_STACK_DONE))
        {
            s_active_task_number = 10U;
            if (task10_return_home_submit_qr(&s_task10, &qr_task) ==
                TASK10_RETURN_HOME_OK)
            {
                task10_submitted = 1U;
            }
        }

        if (task10_submitted != 0U)
        {
            s_active_task_number = 10U;
            (void)task10_return_home_step(&s_task10);
            if ((flow_done_reported == 0U) &&
                (task10_return_home_get_state(&s_task10) ==
                 TASK10_RETURN_HOME_DONE))
            {
                app_write_status(TASK10_RETURN_HOME_STATUS_DONE,
                                 "competition_flow_done");
                flow_done_reported = 1U;
            }
        }
        HAL_Delay(5U);
    }
}

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
