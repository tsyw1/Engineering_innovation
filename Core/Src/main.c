/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能搬运比赛完整流程入口
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "HWT101CT.h"
#include "QR.h"
#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "chassis_motion_closed_loop.h"
#include "material_tray_servo.h"
#include "motor_can.h"
#include "screen.h"
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_MOTOR_POWER_SETTLE_MS    1000U
#define APP_CAN_SETTLE_MS              200U
#define APP_MAIN_PERIOD_MS               5U
#define APP_POSITION_REPORT_PERIOD_MS   200U
#define APP_RUNTIME_REPORT_PERIOD_MS    500U
#define APP_HWT101_RATE_100HZ           0x09U
#define APP_STARTUP_HEADING_WAIT_TIMEOUT_MS 1000U

static motor_bus_t s_motor_bus;
static motor_t s_chassis_motors[MECANUM_WHEEL_COUNT];
static mecanum_chassis_t s_chassis;
static arm_controller_t s_arm;
static hwt101ct_t s_yaw_sensor;
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
static uint32_t s_last_runtime_report_tick;
static uint8_t s_chassis_ready;
static uint8_t s_uart6_ready;
static uint8_t s_task2_last_result;
static char s_error_event[96];
static char s_last_status_event[96];
static char s_last_error_detail[96];
static uint8_t s_last_status_value;

void SystemClock_Config(void);
static void app_report_runtime_status(void);

static void app_send_text(const char *text)
{
    size_t length;

    if (text == NULL)
    {
        return;
    }
    length = strlen(text);
    if (length <= UINT16_MAX)
    {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)text,
                                (uint16_t)length, 100U);
    }
}

static uint8_t app_event_task_number(const char *event)
{
    if (event == NULL) return 0U;
    if (strncmp(event, "task1_", 6U) == 0) return 1U;
    if (strncmp(event, "task2_", 6U) == 0) return 2U;
    if (strncmp(event, "task3_", 6U) == 0) return 3U;
    if (strncmp(event, "task4_", 6U) == 0) return 4U;
    if (strncmp(event, "task5_", 6U) == 0) return 5U;
    if (strncmp(event, "task6_", 6U) == 0) return 6U;
    if (strncmp(event, "task7_", 6U) == 0) return 7U;
    if (strncmp(event, "task8_", 6U) == 0) return 8U;
    if (strncmp(event, "task9_", 6U) == 0) return 9U;
    if (strncmp(event, "task10_", 7U) == 0) return 10U;
    return 0U;
}

static void app_write_status(uint8_t status, const char *event)
{
    char line[160];
    int length;
    uint8_t task_number = s_active_task_number;

    if (task_number == 0U)
    {
        task_number = app_event_task_number(event);
    }
    if (event != NULL)
    {
        /* 保存最近一次状态，错误保持帧使用它而不是通用 step_failed。 */
        (void)snprintf(s_last_status_event,
                       sizeof(s_last_status_event),
                       "%s",
                       event);
        s_last_status_value = status;
        if (status != 255U)
        {
            s_last_error_detail[0] = '\0';
        }
        if ((status == 255U) &&
            ((strstr(event, "task2_item=") != NULL) ||
             (strstr(event, "task6_item=") != NULL)))
        {
            (void)snprintf(s_last_error_detail,
                           sizeof(s_last_error_detail),
                           "%s",
                           event);
        }
    }
    length = snprintf(line, sizeof(line),
                      "COMPETITION,STATUS,TASK%u,VALUE=%u,%s\r\n",
                      (unsigned int)task_number, (unsigned int)status,
                      (event != NULL) ? event : "unknown");
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        app_send_text(line);
    }
}

static void app_report_error(const char *event)
{
    char line[160];
    const char *detail;
    int length;

    detail = (s_last_error_detail[0] != '\0') ?
             s_last_error_detail :
             ((s_last_status_event[0] != '\0') ?
             s_last_status_event :
             ((event != NULL) ? event : "unknown"));
    length = snprintf(line, sizeof(line),
                      "COMPETITION,ERROR,TASK%u,%s\r\n",
                      (unsigned int)s_active_task_number,
                      detail);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        app_send_text(line);
    }
    (void)snprintf(s_error_event, sizeof(s_error_event), "%s", detail);
    if (s_chassis_ready != 0U)
    {
        (void)mecanum_chassis_stop(&s_chassis);
        (void)mecanum_chassis_enable(&s_chassis, false);
    }
    Error_Handler();
}

static void app_report_chassis_position(void)
{
    uint32_t now_ms;
    mecanum_encoder_velocity_t velocity;
    hwt101ct_data_t yaw_data;
    char line[220];
    int length;

    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_chassis_update_encoder_velocity(&s_chassis);
    (void)arm_controller_hold_z_position(&s_arm);
    hwt101ct_receive_watchdog(&s_yaw_sensor, HAL_GetTick(), 1000U);

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_position_report_tick) <
        APP_POSITION_REPORT_PERIOD_MS)
    {
        app_report_runtime_status();
        return;
    }
    s_last_position_report_tick = now_ms;
    memset(&velocity, 0, sizeof(velocity));
    memset(&yaw_data, 0, sizeof(yaw_data));
    (void)mecanum_chassis_get_encoder_velocity(&s_chassis, &velocity);
    (void)hwt101ct_get_data(&s_yaw_sensor, &yaw_data);
    length = snprintf(line, sizeof(line),
                      "COMPETITION,POSITION,TASK%u,YAW_MDEG=%ld,VX_MMPS=%ld,VY_MMPS=%ld,WZ_MRADPS=%ld\r\n",
                      (unsigned int)s_active_task_number,
                      (long)(yaw_data.yaw_deg * 1000.0f),
                      (long)(velocity.vx_mps * 1000.0f),
                      (long)(velocity.vy_mps * 1000.0f),
                      (long)(velocity.wz_rad_s * 1000.0f));
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        app_send_text(line);
    }
    app_report_runtime_status();
}

/*
 * USART6 运行诊断：用于区分任务状态机等待、视觉无帧和机械臂/CAN故障。
 * 该输出只读软件状态，不会向底盘或机械臂下发任何控制命令。
 */
static void app_report_runtime_status(void)
{
    uint32_t now_ms;
    vision_uart_diagnostics_t vision_diagnostics;
    char line[512];
    int length;

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_runtime_report_tick) <
        APP_RUNTIME_REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_runtime_report_tick = now_ms;

    memset(&vision_diagnostics, 0, sizeof(vision_diagnostics));
    vision_uart_get_diagnostics(vision_uart_get_default(),
                                &vision_diagnostics);
    length = snprintf(
        line,
        sizeof(line),
        "COMPETITION,RUNTIME,TICK=%lu,TASK=%u,TASK2_STATE=%u,ITEM_STATE=%u,ITEM_INDEX=%u,TASK2_RESULT=%u,TARGET=%u,VISION_STATE=%u,VISION_MODE=%u,VISION_TARGET=%u,COARSE_PICK=%u,VISION_REJECT=%u,VISION_FRAMES=%lu,VISION_LINES=%lu,VISION_RX=%lu,LAST_STATUS=%u,LAST_VALID=%u,LAST_DET=%u,LAST_AGE=%lu,CHASSIS_CMD=%d/%d/%d/%d,CHASSIS_RESULT=%u,ARM_FLAGS=%u/%u,ARM_POS_VALID=%u/%u,ARM_POS_UPDATES=%lu/%lu,GRIPPER=%u\r\n",
        (unsigned long)now_ms,
        (unsigned int)s_active_task_number,
        (unsigned int)s_task2.state,
        (unsigned int)s_task2.item_state,
        (unsigned int)s_task2.item_index,
        (unsigned int)s_task2_last_result,
        (unsigned int)((s_task2.item_index <
                        TASK2_PLATFORM_PICK_ITEM_COUNT) ?
                       s_task2.target_ids[s_task2.item_index] : 0U),
        (unsigned int)s_fine_adjust.state,
        (unsigned int)s_fine_adjust.mode,
        (unsigned int)s_fine_adjust.target_id,
        (unsigned int)s_fine_adjust.coarse_pick_enabled,
        (unsigned int)s_fine_adjust.last_reject_reason,
        (unsigned long)vision_diagnostics.frame_count,
        (unsigned long)vision_diagnostics.line_count,
        (unsigned long)vision_diagnostics.rx_byte_count,
        (unsigned int)vision_diagnostics.last_status,
        (unsigned int)vision_diagnostics.last_valid,
        (unsigned int)vision_diagnostics.last_detection_count,
        (unsigned long)vision_diagnostics.last_age_ms,
        (int)s_chassis.target_wheel_rpm[0],
        (int)s_chassis.target_wheel_rpm[1],
        (int)s_chassis.target_wheel_rpm[2],
        (int)s_chassis.target_wheel_rpm[3],
        (unsigned int)s_chassis.last_speed_command_result,
        (unsigned int)s_arm.lift_motor.state_flags,
        (unsigned int)s_arm.horizontal_motor.state_flags,
        (unsigned int)s_arm.lift_motor.position_valid,
        (unsigned int)s_arm.horizontal_motor.position_valid,
        (unsigned long)s_arm.lift_motor.position_update_count,
        (unsigned long)s_arm.horizontal_motor.position_update_count,
        (unsigned int)s_arm.gripper.state);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        app_send_text(line);
    }
}

static uint8_t app_prepare(void)
{
    motor_result_t motor_result;
    mecanum_result_t chassis_result;
    arm_controller_result_t arm_result;
    material_tray_servo_result_t tray_result;
    hwt101ct_result_t yaw_result;
    chassis_motion_result_t motion_result;
    uint32_t heading_wait_start_ms;

    if ((screen_init(&s_screen, &huart5) != SCREEN_OK) ||
        (screen_start_receive(&s_screen) != SCREEN_OK))
    {
        app_report_error("screen_rx_init_failed");
        return 0U;
    }
    HAL_Delay(APP_MOTOR_POWER_SETTLE_MS);

    motor_result = motor_bus_init(&s_motor_bus, &hcan1,
                                  motor_can_hal_send_frame, 100U);
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
    chassis_result = mecanum_chassis_init_default(&s_chassis,
                                                  &s_motor_bus,
                                                  s_chassis_motors);
    if (chassis_result != MECANUM_OK)
    {
        app_report_error("chassis_init_failed");
        return 0U;
    }
    s_chassis_ready = 1U;

    yaw_result = hwt101ct_init(&s_yaw_sensor, &huart1);
    if ((yaw_result != HWT101CT_OK) ||
        (hwt101ct_configure_angle_output(&s_yaw_sensor,
                                         APP_HWT101_RATE_100HZ) !=
         HWT101CT_OK))
    {
        app_report_error("hwt101_init_failed");
        return 0U;
    }
    if (chassis_motion_closed_loop_init(&s_chassis, &s_yaw_sensor) !=
        CHASSIS_MOTION_OK)
    {
        app_report_error("chassis_motion_init_failed");
        return 0U;
    }
    /* 记录上电后的第一帧有效偏航角，整个流程都以此作为航向参考。 */
    heading_wait_start_ms = HAL_GetTick();
    do
    {
        motion_result = chassis_motion_capture_startup_heading();
        if (motion_result == CHASSIS_MOTION_OK)
        {
            break;
        }
        HAL_Delay(10U);
    } while ((HAL_GetTick() - heading_wait_start_ms) <
             APP_STARTUP_HEADING_WAIT_TIMEOUT_MS);
    if (motion_result != CHASSIS_MOTION_OK)
    {
        app_report_error("startup_heading_reference_failed");
        return 0U;
    }
    if (mecanum_chassis_enable(&s_chassis, true) != MECANUM_OK)
    {
        app_report_error("chassis_enable_failed");
        return 0U;
    }

    material_tray_servo_config_default(&s_material_tray_config);
    tray_result = material_tray_servo_init(&s_material_tray,
                                           &s_material_tray_config);
    if (tray_result != MATERIAL_TRAY_SERVO_OK)
    {
        app_report_error("tray_init_failed");
        return 0U;
    }
    if (screen_reset_statistics(&s_screen) != SCREEN_OK)
    {
        app_report_error("screen_reset_failed");
        return 0U;
    }
    /* QR_Init() 已经清空缓存并启动 UART4 单字节接收。 */
    if (QR_Init() != HAL_OK)
    {
        app_report_error("qr_uart_init_failed");
        return 0U;
    }
    if ((vision_uart_init(vision_uart_get_default(), &huart3) !=
         VISION_UART_RESULT_OK) ||
        (vision_uart_start_receive(vision_uart_get_default()) !=
         VISION_UART_RESULT_OK))
    {
        app_report_error("vision_uart_init_failed");
        return 0U;
    }

    s_active_task_number = 1U;
    if (task1_start_qr_init(&s_task1, &s_arm, &s_chassis,
                            &s_screen, app_write_status) !=
        TASK1_START_QR_OK)
    {
        app_report_error("task1_init_failed");
        return 0U;
    }
    s_active_task_number = 2U;
    if (task2_platform_pick_init(&s_task2, &s_arm, &s_chassis,
                                 &s_fine_adjust, &s_material_tray,
                                 &s_screen, app_write_status) !=
        TASK2_PLATFORM_PICK_OK)
    {
        app_report_error("task2_init_failed");
        return 0U;
    }
    s_active_task_number = 3U;
    if (task3_coarse_ring_init(&s_task3, &s_arm, &s_chassis,
                               &s_fine_adjust, &s_material_tray,
                               &s_screen, app_write_status) !=
        TASK3_COARSE_RING_OK)
    {
        app_report_error("task3_init_failed");
        return 0U;
    }
    s_active_task_number = 4U;
    if (task4_ring_return_init(&s_task4, &s_arm, &s_chassis,
                               &s_fine_adjust, &s_material_tray,
                               &s_screen, app_write_status) !=
        TASK4_RING_RETURN_OK)
    {
        app_report_error("task4_init_failed");
        return 0U;
    }
    s_active_task_number = 5U;
    if (task5_temporary_ring_init(&s_task5, &s_arm, &s_chassis,
                                  &s_fine_adjust, &s_material_tray,
                                  &s_screen, app_write_status) !=
        TASK5_TEMPORARY_RING_OK)
    {
        app_report_error("task5_init_failed");
        return 0U;
    }
    s_active_task_number = 6U;
    if (task6_platform_pick_init(&s_task6, &s_arm, &s_chassis,
                                 &s_fine_adjust, &s_material_tray,
                                 &s_screen, app_write_status) !=
        TASK6_PLATFORM_PICK_OK)
    {
        app_report_error("task6_init_failed");
        return 0U;
    }
    s_active_task_number = 7U;
    if (task7_coarse_ring_init(&s_task7, &s_arm, &s_chassis,
                               &s_fine_adjust, &s_material_tray,
                               &s_screen, app_write_status) !=
        TASK7_COARSE_RING_OK)
    {
        app_report_error("task7_init_failed");
        return 0U;
    }
    s_active_task_number = 8U;
    if (task8_ring_return_init(&s_task8, &s_arm, &s_chassis,
                               &s_fine_adjust, &s_material_tray,
                               &s_screen, app_write_status) !=
        TASK8_RING_RETURN_OK)
    {
        app_report_error("task8_init_failed");
        return 0U;
    }
    s_active_task_number = 9U;
    if (task9_temporary_stack_init(&s_task9, &s_arm, &s_chassis,
                                   &s_fine_adjust, &s_material_tray,
                                   &s_screen, app_write_status) !=
        TASK9_TEMPORARY_STACK_OK)
    {
        app_report_error("task9_init_failed");
        return 0U;
    }
    s_active_task_number = 10U;
    if (task10_return_home_init(&s_task10, &s_chassis, &s_screen,
                                app_write_status) !=
        TASK10_RETURN_HOME_OK)
    {
        app_report_error("task10_init_failed");
        return 0U;
    }
    vision_fine_adjust_set_heading_sensor(&s_fine_adjust,
                                          &s_yaw_sensor);

    s_active_task_number = 0U;
    app_write_status(TASK1_START_QR_STATUS_INIT, "competition_flow_ready");
    app_write_status(TASK1_START_QR_STATUS_WAIT_START,
                     "waiting_uart5_0x31_zone1_or_0x32_zone2");
    return 1U;
}

int main(void)
{
    uint8_t flow_started = 0U;
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
    start_zone_t start_zone = START_ZONE_NONE;
    QR_TaskCode_t qr_task;
    task1_start_qr_result_t task1_result;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();
    MX_USART6_UART_Init();
    s_uart6_ready = 1U;
    MX_CAN1_Init();

    if (app_prepare() == 0U)
    {
        Error_Handler();
    }
    memset(&qr_task, 0, sizeof(qr_task));

    while (1)
    {
        screen_rx_task(&s_screen);
        vision_uart_task(vision_uart_get_default());
        app_report_chassis_position();

        if ((flow_started == 0U) &&
            (screen_take_start_zone(&s_screen, &start_zone) != 0U))
        {
            if ((task1_start_qr_set_start_zone(&s_task1, start_zone) !=
                 TASK1_START_QR_OK) ||
                (task10_return_home_set_start_zone(&s_task10, start_zone) !=
                 TASK10_RETURN_HOME_OK))
            {
                app_report_error("start_zone_config_failed");
            }
            s_active_task_number = 1U;
            app_write_status(TASK1_START_QR_STATUS_MOVE,
                             (start_zone == START_ZONE_2) ?
                             "screen_start_zone2" : "screen_start_zone1");
            task1_start_qr_request_start(&s_task1);
            flow_started = 1U;
        }
        if (flow_started == 0U)
        {
            HAL_Delay(APP_MAIN_PERIOD_MS);
            continue;
        }

        s_active_task_number = 1U;
        task1_result = task1_start_qr_step(&s_task1);
        if ((task1_result != TASK1_START_QR_OK) &&
            (task1_result != TASK1_START_QR_WAITING))
        {
            app_report_error("task1_step_failed");
        }
        if ((task2_submitted == 0U) &&
            (task1_start_qr_get_state(&s_task1) == TASK1_START_QR_DONE) &&
            (task1_start_qr_get_task(&s_task1, &qr_task) != 0U))
        {
            s_active_task_number = 2U;
            if (task2_platform_pick_submit_qr(&s_task2, &qr_task) !=
                TASK2_PLATFORM_PICK_OK)
            {
                app_report_error("task2_submit_failed");
            }
            task2_submitted = 1U;
        }
        if (task2_submitted != 0U)
        {
            s_active_task_number = 2U;
            s_task2_last_result =
                (uint8_t)task2_platform_pick_step(&s_task2);
            if (task2_platform_pick_get_state(&s_task2) ==
                TASK2_PLATFORM_PICK_FAILED)
            {
                app_report_error("task2_step_failed");
            }
        }

        if ((task3_submitted == 0U) &&
            (task2_platform_pick_get_state(&s_task2) ==
             TASK2_PLATFORM_PICK_DONE))
        {
            s_active_task_number = 3U;
            if (task3_coarse_ring_submit_qr(&s_task3, &qr_task) !=
                TASK3_COARSE_RING_OK)
            {
                app_report_error("task3_submit_failed");
            }
            task3_submitted = 1U;
        }
        if (task3_submitted != 0U)
        {
            s_active_task_number = 3U;
            (void)task3_coarse_ring_step(&s_task3);
            if (task3_coarse_ring_get_state(&s_task3) ==
                TASK3_COARSE_RING_FAILED)
            {
                app_report_error("task3_step_failed");
            }
        }

        if ((task4_submitted == 0U) &&
            (task3_coarse_ring_get_state(&s_task3) ==
             TASK3_COARSE_RING_DONE))
        {
            s_active_task_number = 4U;
            if (task4_ring_return_submit_qr(&s_task4, &qr_task) !=
                TASK4_RING_RETURN_OK)
            {
                app_report_error("task4_submit_failed");
            }
            task4_submitted = 1U;
        }
        if (task4_submitted != 0U)
        {
            s_active_task_number = 4U;
            (void)task4_ring_return_step(&s_task4);
            if (task4_ring_return_get_state(&s_task4) ==
                TASK4_RING_RETURN_FAILED)
            {
                app_report_error("task4_step_failed");
            }
        }

        if ((task5_submitted == 0U) &&
            (task4_ring_return_get_state(&s_task4) ==
             TASK4_RING_RETURN_DONE))
        {
            s_active_task_number = 5U;
            if (task5_temporary_ring_submit_qr(&s_task5, &qr_task) !=
                TASK5_TEMPORARY_RING_OK)
            {
                app_report_error("task5_submit_failed");
            }
            task5_submitted = 1U;
        }
        if (task5_submitted != 0U)
        {
            s_active_task_number = 5U;
            (void)task5_temporary_ring_step(&s_task5);
            if (task5_temporary_ring_get_state(&s_task5) ==
                TASK5_TEMPORARY_RING_FAILED)
            {
                app_report_error("task5_step_failed");
            }
        }

        if ((task6_submitted == 0U) &&
            (task5_temporary_ring_get_state(&s_task5) ==
             TASK5_TEMPORARY_RING_DONE))
        {
            s_active_task_number = 6U;
            if (task6_platform_pick_submit_qr(&s_task6, &qr_task) !=
                TASK6_PLATFORM_PICK_OK)
            {
                app_report_error("task6_submit_failed");
            }
            task6_submitted = 1U;
        }
        if (task6_submitted != 0U)
        {
            s_active_task_number = 6U;
            (void)task6_platform_pick_step(&s_task6);
            if (task6_platform_pick_get_state(&s_task6) ==
                TASK6_PLATFORM_PICK_FAILED)
            {
                app_report_error("task6_step_failed");
            }
        }

        if ((task7_submitted == 0U) &&
            (task6_platform_pick_get_state(&s_task6) ==
             TASK6_PLATFORM_PICK_DONE))
        {
            s_active_task_number = 7U;
            if (task7_coarse_ring_submit_qr(&s_task7, &qr_task) !=
                TASK7_COARSE_RING_OK)
            {
                app_report_error("task7_submit_failed");
            }
            task7_submitted = 1U;
        }
        if (task7_submitted != 0U)
        {
            s_active_task_number = 7U;
            (void)task7_coarse_ring_step(&s_task7);
            if (task7_coarse_ring_get_state(&s_task7) ==
                TASK7_COARSE_RING_FAILED)
            {
                app_report_error("task7_step_failed");
            }
        }

        if ((task8_submitted == 0U) &&
            (task7_coarse_ring_get_state(&s_task7) ==
             TASK7_COARSE_RING_DONE))
        {
            s_active_task_number = 8U;
            if (task8_ring_return_submit_qr(&s_task8, &qr_task) !=
                TASK8_RING_RETURN_OK)
            {
                app_report_error("task8_submit_failed");
            }
            task8_submitted = 1U;
        }
        if (task8_submitted != 0U)
        {
            s_active_task_number = 8U;
            (void)task8_ring_return_step(&s_task8);
            if (task8_ring_return_get_state(&s_task8) ==
                TASK8_RING_RETURN_FAILED)
            {
                app_report_error("task8_step_failed");
            }
        }

        if ((task9_submitted == 0U) &&
            (task8_ring_return_get_state(&s_task8) ==
             TASK8_RING_RETURN_DONE))
        {
            s_active_task_number = 9U;
            if (task9_temporary_stack_submit_qr(&s_task9, &qr_task) !=
                TASK9_TEMPORARY_STACK_OK)
            {
                app_report_error("task9_submit_failed");
            }
            task9_submitted = 1U;
        }
        if (task9_submitted != 0U)
        {
            s_active_task_number = 9U;
            (void)task9_temporary_stack_step(&s_task9);
            if (task9_temporary_stack_get_state(&s_task9) ==
                TASK9_TEMPORARY_STACK_FAILED)
            {
                app_report_error("task9_step_failed");
            }
        }

        if ((task10_submitted == 0U) &&
            (task9_temporary_stack_get_state(&s_task9) ==
             TASK9_TEMPORARY_STACK_DONE))
        {
            s_active_task_number = 10U;
            if (task10_return_home_submit_qr(&s_task10, &qr_task) !=
                TASK10_RETURN_HOME_OK)
            {
                app_report_error("task10_submit_failed");
            }
            task10_submitted = 1U;
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
        HAL_Delay(APP_MAIN_PERIOD_MS);
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
  while (1)
  {
    if (s_uart6_ready != 0U)
    {
      char line[256];
      int length = snprintf(line,
                            sizeof(line),
                            "COMPETITION,ERROR,HOLD,TASK=%u,DETAIL=%s,STATUS=%u,ITEM=%u,ITEM_STATE=%u,RESULT=%u,VISION_STATE=%u,VISION_REJECT=%u,ARM_FLAGS=%u/%u,GRIPPER=%u\r\n",
                            (unsigned int)s_active_task_number,
                            (s_error_event[0] != '\0') ? s_error_event :
                            "unknown",
                            (unsigned int)s_last_status_value,
                            (unsigned int)s_task2.item_index,
                            (unsigned int)s_task2.item_state,
                            (unsigned int)s_task2_last_result,
                            (unsigned int)s_fine_adjust.state,
                            (unsigned int)s_fine_adjust.last_reject_reason,
                            (unsigned int)s_arm.lift_motor.state_flags,
                            (unsigned int)s_arm.horizontal_motor.state_flags,
                            (unsigned int)s_arm.gripper.state);
      if ((length > 0) && ((size_t)length < sizeof(line)))
      {
        app_send_text(line);
      }
    }
    HAL_Delay(500U);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
