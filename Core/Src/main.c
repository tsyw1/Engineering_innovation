/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "motor_can.h"
#include "chassis_mecanum.h"

#include <stdio.h>
#include <string.h>

#define TEST_TX_TIMEOUT_MS              100U
#define TEST_LINE_BUFFER_SIZE           192U

#if 0
/* 小车前后左右移动演示参数，现已停用。 */
#define CAR_MOVE_SPEED_MPS              0.15f
#define CAR_MOVE_TIME_MS                2000U
#define CAR_STOP_TIME_MS                1000U
#define CAR_ENCODER_PRINT_PERIOD_MS     200U
#define CAR_IDLE_PRINT_PERIOD_MS        500U
#define CAR_ENCODER_READY_TIMEOUT_MS    2000U
#define CAR_LOOP_DELAY_MS               5U
#define CAR_CAN_TIMEOUT_MS              100U
#endif

/* 麦轮底盘里程和旋转测试参数。 */
#define CHASSIS_TEST_DISTANCE_M         2.4f
#define CHASSIS_TEST_LINEAR_SPEED_MPS   0.20f
#define CHASSIS_TEST_DECEL_DISTANCE_M   0.30f
#define CHASSIS_TEST_DECEL_LINEAR_SPEED_MPS 0.05f
#define CHASSIS_TEST_ROTATION_SPEED_RAD_S 0.35f
#define CHASSIS_TEST_DECEL_ANGLE_DEG    15.0f
#define CHASSIS_TEST_DECEL_ROTATION_SPEED_RAD_S 0.10f
#define CHASSIS_TEST_LOOP_DELAY_MS      5U
#define CHASSIS_TEST_ENCODER_PRINT_PERIOD_MS 200U
#define CHASSIS_TEST_TIMEOUT_MS         30000U
#define CHASSIS_TEST_SETTLE_TIME_MS     500U
#define CHASSIS_TEST_CAN_TIMEOUT_MS     100U
#define CHASSIS_TEST_PI_F               3.14159265358979323846f

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void test_write_text(const char *text);
#if 0
static void car_write_init_diagnostics(mecanum_result_t chassis_result);
static void car_feedback_service(void);
static uint8_t car_encoders_ready(void);
static void car_write_encoder_values(const char *motion_name);
static uint8_t car_wait_encoder_ready(void);
static uint8_t car_run_direction(const char *motion_name,
                                 float vx_mps,
                                 float vy_mps);
static uint8_t car_run_motion_demo(void);
#endif

static void chassis_test_service(void);
static void chassis_test_write_odometry(const char *name);
static void chassis_test_write_realtime_odometry(const char *name);
static uint8_t chassis_test_wait_encoder_ready(void);
static uint8_t chassis_test_encoder_valid_mask(void);
static void chassis_test_write_encoder_diagnostics(const char *reason);
static uint8_t chassis_test_run_translation(const char *name,
                                            float vx_mps,
                                            float vy_mps);
static uint8_t chassis_test_run_rotation(const char *name,
                                         float target_angle_deg);
static uint8_t chassis_test_run_all(void);

/* Private user code ---------------------------------------------------------*/
static motor_bus_t s_motor_bus;
static motor_t s_motors[MECANUM_WHEEL_COUNT];
static mecanum_chassis_t s_chassis;
static uint8_t s_chassis_ready;

static void test_write_text(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  (void)HAL_UART_Transmit(&huart6,
                          (uint8_t *)text,
                          (uint16_t)strlen(text),
                          TEST_TX_TIMEOUT_MS);
}

#if 0
static void car_write_init_diagnostics(mecanum_result_t chassis_result)
{
  char line[TEST_LINE_BUFFER_SIZE];
  uint8_t next_motor_id;
  int length;

  next_motor_id = (s_motor_bus.motor_count < MECANUM_WHEEL_COUNT) ?
                  (uint8_t)(s_motor_bus.motor_count + 1U) : 0U;
  length = snprintf(line,
                    sizeof(line),
                    "car,error,chassis_init_failed,result,%u,registered,%u,next_id,%u,can_state,%u,can_error,0x%08lX,tx_free,%lu,rx_frames,%lu,rx_errors,%lu\r\n",
                    (unsigned int)chassis_result,
                    (unsigned int)s_motor_bus.motor_count,
                    (unsigned int)next_motor_id,
                    (unsigned int)HAL_CAN_GetState(&hcan1),
                    (unsigned long)HAL_CAN_GetError(&hcan1),
                    (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
                    (unsigned long)s_motor_bus.rx_frame_count,
                    (unsigned long)s_motor_bus.rx_error_count);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}
#endif

/* 处理 CAN 反馈和编码器里程计。 */
static void chassis_test_service(void)
{
  if (s_chassis_ready != 0U)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_chassis_update_encoder_odometry(&s_chassis);
  }
}

static void chassis_test_write_odometry(const char *name)
{
  mecanum_encoder_odometry_t odometry;
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  if (mecanum_chassis_get_encoder_odometry(&s_chassis,
                                           &odometry) != MECANUM_OK)
  {
    length = snprintf(line,
                      sizeof(line),
                      "chassis_test,odometry,%s,status,NOT_READY,"
                      "valid,0x%02X,updates,%lu,%lu,%lu,%lu\r\n",
                      (name != NULL) ? name : "unknown",
                      (unsigned int)chassis_test_encoder_valid_mask(),
                      (unsigned long)s_motors[0].encoder_update_count,
                      (unsigned long)s_motors[1].encoder_update_count,
                      (unsigned long)s_motors[2].encoder_update_count,
                      (unsigned long)s_motors[3].encoder_update_count);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
      test_write_text(line);
    }
    return;
  }

  length = snprintf(line,
                    sizeof(line),
                    "chassis_test,result,%s,x_m,%.4f,y_m,%.4f,yaw_deg,%.2f,"
                    "vx_mps,%.3f,vy_mps,%.3f,wz_rad_s,%.3f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)odometry.x_m,
                    (double)odometry.y_m,
                    (double)(odometry.yaw_rad * 180.0f /
                             CHASSIS_TEST_PI_F),
                    (double)odometry.vx_mps,
                    (double)odometry.vy_mps,
                    (double)odometry.wz_rad_s);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

/* 周期输出编码器计算出的底盘里程计，不直接输出电机原始编码值。 */
static void chassis_test_write_realtime_odometry(const char *name)
{
  mecanum_encoder_odometry_t odometry;
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  if (mecanum_chassis_get_encoder_odometry(&s_chassis,
                                           &odometry) != MECANUM_OK)
  {
    length = snprintf(line,
                      sizeof(line),
                      "chassis_test,odometry,%s,status,NOT_READY,"
                      "valid,0x%02X,updates,%lu,%lu,%lu,%lu\r\n",
                      (name != NULL) ? name : "unknown",
                      (unsigned int)chassis_test_encoder_valid_mask(),
                      (unsigned long)s_motors[0].encoder_update_count,
                      (unsigned long)s_motors[1].encoder_update_count,
                      (unsigned long)s_motors[2].encoder_update_count,
                      (unsigned long)s_motors[3].encoder_update_count);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
      test_write_text(line);
    }
    return;
  }

  length = snprintf(line,
                    sizeof(line),
                    "chassis_test,odometry,%s,x_m,%.4f,y_m,%.4f,"
                    "yaw_deg,%.2f,vx_mps,%.3f,vy_mps,%.3f,wz_rad_s,%.3f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)odometry.x_m,
                    (double)odometry.y_m,
                    (double)(odometry.yaw_rad * 180.0f /
                             CHASSIS_TEST_PI_F),
                    (double)odometry.vx_mps,
                    (double)odometry.vy_mps,
                    (double)odometry.wz_rad_s);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

/* 返回四个电机当前是否都收到过有效的 0x31 编码器反馈。 */
static uint8_t chassis_test_encoder_valid_mask(void)
{
  uint8_t index;
  uint8_t mask = 0U;

  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    if (s_motors[index].encoder_valid != 0U)
    {
      mask |= (uint8_t)(1U << index);
    }
  }

  return mask;
}

static void chassis_test_write_encoder_diagnostics(const char *reason)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "chassis_test,encoder_diag,%s,valid_mask,0x%02X,"
                    "updates,%lu,%lu,%lu,%lu,rx_frames,%lu,rx_errors,%lu,"
                    "can_error,0x%08lX\r\n",
                    (reason != NULL) ? reason : "unknown",
                    (unsigned int)chassis_test_encoder_valid_mask(),
                    (unsigned long)s_motors[0].encoder_update_count,
                    (unsigned long)s_motors[1].encoder_update_count,
                    (unsigned long)s_motors[2].encoder_update_count,
                    (unsigned long)s_motors[3].encoder_update_count,
                    (unsigned long)s_motor_bus.rx_frame_count,
                    (unsigned long)s_motor_bus.rx_error_count,
                    (unsigned long)s_motor_bus.last_can_error);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static uint8_t chassis_test_wait_encoder_ready(void)
{
  uint32_t start_tick;

  mecanum_chassis_reset_odometry(&s_chassis);
  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         CHASSIS_TEST_TIMEOUT_MS)
  {
    chassis_test_service();
    if (chassis_test_encoder_valid_mask() == 0x0FU)
    {
      /* 收齐四路编码器后先建立基准，下一次反馈再开始积分。 */
      (void)mecanum_chassis_update_encoder_odometry(&s_chassis);
      return 1U;
    }
    HAL_Delay(CHASSIS_TEST_LOOP_DELAY_MS);
  }

  chassis_test_write_encoder_diagnostics("ready_timeout");
  return 0U;
}

static uint8_t chassis_test_run_translation(const char *name,
                                            float vx_mps,
                                            float vy_mps)
{
  mecanum_encoder_odometry_t odometry;
  float progress_m;
  float signed_progress_m;
  float remaining_m;
  uint32_t start_tick;
  uint32_t next_odometry_print_tick;
  uint8_t direction;
  uint8_t deceleration_sent = 0U;

  test_write_text("chassis_test,segment_begin,");
  test_write_text(name);
  test_write_text("\r\n");

  if (chassis_test_wait_encoder_ready() == 0U)
  {
    test_write_text("chassis_test,error,encoder_not_ready\r\n");
    return 0U;
  }

  if (mecanum_chassis_set_velocity(&s_chassis,
                                   vx_mps,
                                   vy_mps,
                                   0.0f) == MECANUM_MOTOR_ERROR)
  {
    test_write_text("chassis_test,error,translation_command_failed\r\n");
    return 0U;
  }

  test_write_text("chassis_test,command_sent,");
  test_write_text(name);
  test_write_text("\r\n");

  direction = ((vx_mps + vy_mps) >= 0.0f) ? 1U : 0U;
  start_tick = HAL_GetTick();
  next_odometry_print_tick = start_tick;
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         CHASSIS_TEST_TIMEOUT_MS)
  {
    chassis_test_service();
    if ((int32_t)(HAL_GetTick() - next_odometry_print_tick) >= 0)
    {
      chassis_test_write_realtime_odometry(name);
      next_odometry_print_tick += CHASSIS_TEST_ENCODER_PRINT_PERIOD_MS;
    }
    if (mecanum_chassis_get_encoder_odometry(&s_chassis,
                                             &odometry) == MECANUM_OK)
    {
      progress_m = (vx_mps != 0.0f) ? odometry.x_m : odometry.y_m;
      signed_progress_m = (direction != 0U) ? progress_m : -progress_m;
      remaining_m = CHASSIS_TEST_DISTANCE_M - signed_progress_m;

      if ((deceleration_sent == 0U) &&
          (remaining_m > 0.0f) &&
          (remaining_m <= CHASSIS_TEST_DECEL_DISTANCE_M))
      {
        if (mecanum_chassis_set_velocity(
                &s_chassis,
                vx_mps * (CHASSIS_TEST_DECEL_LINEAR_SPEED_MPS /
                          CHASSIS_TEST_LINEAR_SPEED_MPS),
                vy_mps * (CHASSIS_TEST_DECEL_LINEAR_SPEED_MPS /
                          CHASSIS_TEST_LINEAR_SPEED_MPS),
                0.0f) == MECANUM_MOTOR_ERROR)
        {
          (void)mecanum_chassis_stop(&s_chassis);
          test_write_text("chassis_test,error,deceleration_command_failed\r\n");
          return 0U;
        }
        deceleration_sent = 1U;
        test_write_text("chassis_test,decelerate,");
        test_write_text(name);
        test_write_text("\r\n");
      }

      if (signed_progress_m >= CHASSIS_TEST_DISTANCE_M)
      {
        (void)mecanum_chassis_stop(&s_chassis);
        HAL_Delay(CHASSIS_TEST_SETTLE_TIME_MS);
        chassis_test_write_odometry(name);
        test_write_text("chassis_test,segment_done,");
        test_write_text(name);
        test_write_text("\r\n");
        return 1U;
      }
    }
    HAL_Delay(CHASSIS_TEST_LOOP_DELAY_MS);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  chassis_test_write_encoder_diagnostics("translation_timeout");
  test_write_text("chassis_test,error,translation_timeout\r\n");
  return 0U;
}

static uint8_t chassis_test_run_rotation(const char *name,
                                         float target_angle_deg)
{
  mecanum_encoder_odometry_t odometry;
  float target_angle_rad;
  float remaining_angle_rad;
  uint32_t start_tick;
  uint32_t next_odometry_print_tick;
  uint8_t deceleration_sent = 0U;

  test_write_text("chassis_test,segment_begin,");
  test_write_text(name);
  test_write_text("\r\n");

  if (chassis_test_wait_encoder_ready() == 0U)
  {
    test_write_text("chassis_test,error,encoder_not_ready\r\n");
    return 0U;
  }

  target_angle_rad = target_angle_deg * CHASSIS_TEST_PI_F / 180.0f;
  if (mecanum_chassis_set_velocity(&s_chassis,
                                   0.0f,
                                   0.0f,
                                   CHASSIS_TEST_ROTATION_SPEED_RAD_S) ==
      MECANUM_MOTOR_ERROR)
  {
    test_write_text("chassis_test,error,rotation_command_failed\r\n");
    return 0U;
  }

  test_write_text("chassis_test,command_sent,");
  test_write_text(name);
  test_write_text("\r\n");

  start_tick = HAL_GetTick();
  next_odometry_print_tick = start_tick;
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         CHASSIS_TEST_TIMEOUT_MS)
  {
    chassis_test_service();
    if ((int32_t)(HAL_GetTick() - next_odometry_print_tick) >= 0)
    {
      chassis_test_write_realtime_odometry(name);
      next_odometry_print_tick += CHASSIS_TEST_ENCODER_PRINT_PERIOD_MS;
    }
    if (mecanum_chassis_get_encoder_odometry(&s_chassis,
                                             &odometry) == MECANUM_OK)
    {
      remaining_angle_rad = target_angle_rad - odometry.yaw_rad;
      if ((deceleration_sent == 0U) &&
          (remaining_angle_rad > 0.0f) &&
          (remaining_angle_rad <=
           CHASSIS_TEST_DECEL_ANGLE_DEG * CHASSIS_TEST_PI_F / 180.0f))
      {
        if (mecanum_chassis_set_velocity(
                &s_chassis,
                0.0f,
                0.0f,
                CHASSIS_TEST_DECEL_ROTATION_SPEED_RAD_S) ==
            MECANUM_MOTOR_ERROR)
        {
          (void)mecanum_chassis_stop(&s_chassis);
          test_write_text("chassis_test,error,deceleration_command_failed\r\n");
          return 0U;
        }
        deceleration_sent = 1U;
        test_write_text("chassis_test,decelerate,");
        test_write_text(name);
        test_write_text("\r\n");
      }

      if (odometry.yaw_rad >= target_angle_rad)
      {
        (void)mecanum_chassis_stop(&s_chassis);
        HAL_Delay(CHASSIS_TEST_SETTLE_TIME_MS);
        chassis_test_write_odometry(name);
        test_write_text("chassis_test,segment_done,");
        test_write_text(name);
        test_write_text("\r\n");
        return 1U;
      }
    }
    HAL_Delay(CHASSIS_TEST_LOOP_DELAY_MS);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  chassis_test_write_encoder_diagnostics("rotation_timeout");
  test_write_text("chassis_test,error,rotation_timeout\r\n");
  return 0U;
}

static uint8_t chassis_test_run_all(void)
{
  if (mecanum_chassis_enable(&s_chassis, true) != MECANUM_OK)
  {
    test_write_text("chassis_test,error,enable_failed\r\n");
    return 0U;
  }

  test_write_text("chassis_test,begin\r\n");
  if ((chassis_test_run_translation("forward_2p4m",
                                    CHASSIS_TEST_LINEAR_SPEED_MPS,
                                    0.0f) == 0U) ||
      (chassis_test_run_translation("backward_2p4m",
                                    -CHASSIS_TEST_LINEAR_SPEED_MPS,
                                    0.0f) == 0U) ||
      (chassis_test_run_translation("left_2p4m",
                                    0.0f,
                                    CHASSIS_TEST_LINEAR_SPEED_MPS) == 0U) ||
      (chassis_test_run_translation("right_2p4m",
                                    0.0f,
                                    -CHASSIS_TEST_LINEAR_SPEED_MPS) == 0U) ||
      (chassis_test_run_rotation("rotate_90deg", 90.0f) == 0U) ||
      (chassis_test_run_rotation("rotate_180deg", 180.0f) == 0U) ||
      (chassis_test_run_rotation("rotate_270deg", 270.0f) == 0U) ||
      (chassis_test_run_rotation("rotate_360deg", 360.0f) == 0U))
  {
    (void)mecanum_chassis_stop(&s_chassis);
    test_write_text("chassis_test,failed\r\n");
    return 0U;
  }

  (void)mecanum_chassis_stop(&s_chassis);
  test_write_text("chassis_test,complete\r\n");
  return 1U;
}

#if 0
/* 定时前后左右移动演示，现已停用。 */
/* 周期请求四个电机的速度和编码器反馈。 */
static void car_feedback_service(void)
{
  if (s_chassis_ready != 0U)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
  }
}

static uint8_t car_encoders_ready(void)
{
  uint8_t index;

  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    if (s_motors[index].encoder_valid == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

/* USART6 输出顺序：前左、前右、后左、后右。 */
static void car_write_encoder_values(const char *motion_name)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  if (car_encoders_ready() == 0U)
  {
    return;
  }

  length = snprintf(line,
                    sizeof(line),
                    "car,encoder,%s,FL,%u,FR,%u,RL,%u,RR,%u\r\n",
                    (motion_name != NULL) ? motion_name : "unknown",
                    (unsigned int)s_motors[MECANUM_WHEEL_FL].encoder_raw,
                    (unsigned int)s_motors[MECANUM_WHEEL_FR].encoder_raw,
                    (unsigned int)s_motors[MECANUM_WHEEL_RL].encoder_raw,
                    (unsigned int)s_motors[MECANUM_WHEEL_RR].encoder_raw);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static uint8_t car_wait_encoder_ready(void)
{
  uint32_t start_tick;

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         CAR_ENCODER_READY_TIMEOUT_MS)
  {
    car_feedback_service();
    if (car_encoders_ready() != 0U)
    {
      car_write_encoder_values("ready");
      return 1U;
    }
    HAL_Delay(CAR_LOOP_DELAY_MS);
  }

  test_write_text("car,error,encoder_not_ready\r\n");
  return 0U;
}

static uint8_t car_run_direction(const char *motion_name,
                                 float vx_mps,
                                 float vy_mps)
{
  mecanum_result_t result;
  uint32_t start_tick;
  uint32_t next_print_tick;

  test_write_text("car,motion_begin,");
  test_write_text(motion_name);
  test_write_text("\r\n");

  result = mecanum_chassis_set_velocity(&s_chassis,
                                        vx_mps,
                                        vy_mps,
                                        0.0f);
  if ((result != MECANUM_OK) && (result != MECANUM_CLAMPED))
  {
    test_write_text("car,error,motion_command_failed\r\n");
    (void)mecanum_chassis_stop(&s_chassis);
    return 0U;
  }

  start_tick = HAL_GetTick();
  next_print_tick = start_tick;
  while ((uint32_t)(HAL_GetTick() - start_tick) < CAR_MOVE_TIME_MS)
  {
    car_feedback_service();
    if ((int32_t)(HAL_GetTick() - next_print_tick) >= 0)
    {
      car_write_encoder_values(motion_name);
      next_print_tick += CAR_ENCODER_PRINT_PERIOD_MS;
    }
    HAL_Delay(CAR_LOOP_DELAY_MS);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  car_feedback_service();
  car_write_encoder_values(motion_name);
  test_write_text("car,motion_end,");
  test_write_text(motion_name);
  test_write_text("\r\n");
  HAL_Delay(CAR_STOP_TIME_MS);
  return 1U;
}

static uint8_t car_run_motion_demo(void)
{
  if (mecanum_chassis_enable(&s_chassis, true) != MECANUM_OK)
  {
    test_write_text("car,error,enable_failed\r\n");
    return 0U;
  }

  if (car_wait_encoder_ready() == 0U)
  {
    (void)mecanum_chassis_stop(&s_chassis);
    return 0U;
  }

  test_write_text("car,demo_begin\r\n");
  if ((car_run_direction("forward", CAR_MOVE_SPEED_MPS, 0.0f) == 0U) ||
      (car_run_direction("backward", -CAR_MOVE_SPEED_MPS, 0.0f) == 0U) ||
      (car_run_direction("left", 0.0f, CAR_MOVE_SPEED_MPS) == 0U) ||
      (car_run_direction("right", 0.0f, -CAR_MOVE_SPEED_MPS) == 0U))
  {
    (void)mecanum_chassis_stop(&s_chassis);
    test_write_text("car,demo_failed\r\n");
    return 0U;
  }

  (void)mecanum_chassis_stop(&s_chassis);
  test_write_text("car,demo_complete\r\n");
  return 1U;
}
#endif

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  motor_result_t motor_result;
  mecanum_result_t chassis_result;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART6_UART_Init();
  test_write_text("chassis_test,boot,USART6_ready\r\n");
  MX_CAN1_Init();

  motor_result = motor_bus_init(&s_motor_bus,
                                &hcan1,
                                motor_can_hal_send_frame,
                                CHASSIS_TEST_CAN_TIMEOUT_MS);
  if ((motor_result != MOTOR_OK) ||
      (motor_bus_start(&s_motor_bus) != MOTOR_OK))
  {
    test_write_text("chassis_test,error,can_start_failed\r\n");
  }
  else
  {
    chassis_result = mecanum_chassis_init_default(&s_chassis,
                                                  &s_motor_bus,
                                                  s_motors);
    if (chassis_result != MECANUM_OK)
    {
      test_write_text("chassis_test,error,chassis_init_failed\r\n");
    }
    else
    {
      s_chassis_ready = 1U;
    }
  }

  if (s_chassis_ready != 0U)
  {
    (void)chassis_test_run_all();
  }

  while (1)
  {
    /* 测试完成后继续更新 CAN 反馈和编码器里程计。 */
    chassis_test_service();
    HAL_Delay(CHASSIS_TEST_LOOP_DELAY_MS);
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
