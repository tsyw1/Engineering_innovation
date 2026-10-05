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
#include "IMU_UART.h"
#include "IMU_GYRO_CALIBRATION.h"
#include "MECANUM_INERTIAL_NAV.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define TEST_TX_TIMEOUT_MS              100U
#define TEST_LINE_BUFFER_SIZE           192U
/* 置 1 可恢复统一底盘标定测试；默认 0 进入编码器/IMU 融合。 */
#define APP_RUN_CALIBRATION_TEST        0U
/* 默认上电自动执行四个连续 90 度旋转测试，完成后电机再次失能。 */
#define APP_RUN_MOTION_TEST             1U
#define MOTION_TEST_SPEED_MPS           0.20f
#define MOTION_TEST_DISTANCE_M          2.10f
#define MOTION_TEST_DECEL_SPEED_MPS     0.08f
#define MOTION_TEST_DECEL_DISTANCE_M    0.15f
#define MOTION_TEST_TIMEOUT_MS          20000U
#define MOTION_TEST_SETTLE_MS           1000U
#define MOTION_TEST_HEADING_HOLD        1U
#define MOTION_TEST_HEADING_KP          1.80f
#define MOTION_TEST_MAX_CORRECTION_WZ   0.15f
#define MOTION_TEST_CONTROL_PERIOD_MS   20U
#define MOTION_TEST_ROTATION_SPEED_RAD_S 0.20f
#define MOTION_TEST_ROTATION_DECEL_SPEED_RAD_S 0.08f
#define MOTION_TEST_ROTATION_DECEL_DEG  12.0f
#define MOTION_TEST_ROTATION_TIMEOUT_MS 12000U

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

/* 编码器标定测试参数。每次上电只执行一个方向，便于外部测量实际位移。 */
#define CALIBRATION_TEST_LINEAR_SPEED_MPS       0.12f
#define CALIBRATION_TEST_ROTATION_SPEED_RAD_S   0.20f
#define CALIBRATION_TEST_ROTATION_ANGLE_DEG     90.0f
#define CALIBRATION_TEST_LINEAR_RUN_TIME_MS     20000U
#define CALIBRATION_TEST_STARTUP_SETTLE_MS      1000U
#define CALIBRATION_TEST_STOP_SETTLE_MS         1000U
#define CALIBRATION_TEST_LOOP_DELAY_MS          5U
#define CALIBRATION_TEST_READY_TIMEOUT_MS       5000U
#define CALIBRATION_TEST_CAN_TIMEOUT_MS         100U
#define CALIBRATION_TEST_PI_F                   3.14159265358979323846f

/* 六个方向按固定顺序各测试一次。 */
typedef enum
{
  CALIBRATION_MOTION_FORWARD = 0,
  CALIBRATION_MOTION_BACKWARD,
  CALIBRATION_MOTION_LEFT,
  CALIBRATION_MOTION_RIGHT,
  CALIBRATION_MOTION_ROTATE_CCW,
  CALIBRATION_MOTION_ROTATE_CW
} calibration_motion_t;

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

static void calibration_test_service(void);
static uint8_t calibration_test_read_positions(
    int64_t position[MECANUM_WHEEL_COUNT],
    uint32_t update_count[MECANUM_WHEEL_COUNT],
    const uint32_t baseline_count[MECANUM_WHEEL_COUNT],
    uint8_t require_new,
    uint32_t timeout_ms);
static void calibration_test_write_snapshot(
    const char *label,
    const int64_t position[MECANUM_WHEEL_COUNT],
    const uint32_t update_count[MECANUM_WHEEL_COUNT]);
static void calibration_test_write_diagnostics(const char *reason);
static uint8_t calibration_test_run_once(calibration_motion_t motion);
static uint8_t calibration_test_run(void);

/* Private user code ---------------------------------------------------------*/
static motor_bus_t s_motor_bus;
static motor_t s_motors[MECANUM_WHEEL_COUNT];
static mecanum_chassis_t s_chassis;
static uint8_t s_chassis_ready;
static jy901s_uart_t s_imu_sensor;
static imu_gyro_calibration_t s_imu_calibration;
static mecanum_inertial_nav_t s_inertial_nav;
static uint8_t s_fusion_ready;
static uint32_t s_last_fusion_log_tick;

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

static void fusion_write_status(void)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "fusion,x_m,%.4f,y_m,%.4f,yaw_deg,%.2f,"
                    "encoder_wz,%.4f,gyro_wz,%.4f,fused_wz,%.4f,"
                    "gyro_bias_dps,%.4f,imu_frame,%lu\r\n",
                    (double)s_inertial_nav.x_m,
                    (double)s_inertial_nav.y_m,
                    (double)(s_inertial_nav.yaw_rad * 57.295779513f),
                    (double)s_inertial_nav.encoder_wz_rad_s,
                    (double)s_inertial_nav.gyro_wz_rad_s,
                    (double)s_inertial_nav.fused_wz_rad_s,
                    (double)s_inertial_nav.gyro_z_bias_dps,
                    (unsigned long)s_inertial_nav.last_imu_frame_count);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static float fusion_wrap_angle(float angle_rad)
{
  return atan2f(sinf(angle_rad), cosf(angle_rad));
}

#if 0
static uint8_t fusion_run_motion_segment(const char *name,
                                         float vx_mps,
                                         float vy_mps,
                                         float distance_m,
                                         float heading_reference_rad)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;
  float heading_error_rad;
  float correction_wz_rad_s;
  float command_speed_mps;
  float body_direction_x;
  float body_direction_y;
  float world_direction_x;
  float world_direction_y;
  float displacement_x_m;
  float displacement_y_m;
  float progress_m;
  float start_x_m;
  float start_y_m;
  float start_yaw_rad;
  uint32_t start_tick;
  uint32_t next_log_tick;
  uint32_t next_control_tick;
  uint8_t deceleration_sent;
  mecanum_result_t result;

  length = snprintf(line,
                    sizeof(line),
                    "fusion,segment_start,name,%s,vx_mps,%.3f,vy_mps,%.3f,"
                    "speed_mps,%.3f,target_distance_m,%.3f,timeout_ms,%lu,"
                    "heading_kp,%.2f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)vx_mps,
                    (double)vy_mps,
                    (double)MOTION_TEST_SPEED_MPS,
                    (double)distance_m,
                    (unsigned long)MOTION_TEST_TIMEOUT_MS,
                    (double)MOTION_TEST_HEADING_KP);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }

  /* 先保持静止建立 0x36 位置基准，避免把启动前位置算进测试位移。 */
  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_SETTLE_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    HAL_Delay(5U);
  }

  start_x_m = s_inertial_nav.x_m;
  start_y_m = s_inertial_nav.y_m;
  start_yaw_rad = s_inertial_nav.yaw_rad;
  body_direction_x = vx_mps / MOTION_TEST_SPEED_MPS;
  body_direction_y = vy_mps / MOTION_TEST_SPEED_MPS;
  world_direction_x = cosf(heading_reference_rad) * body_direction_x -
                      sinf(heading_reference_rad) * body_direction_y;
  world_direction_y = sinf(heading_reference_rad) * body_direction_x +
                      cosf(heading_reference_rad) * body_direction_y;
  command_speed_mps = MOTION_TEST_SPEED_MPS;
  progress_m = 0.0f;
  deceleration_sent = 0U;

  result = mecanum_chassis_set_velocity(&s_chassis,
                                        vx_mps,
                                        vy_mps,
                                        0.0f);
  if ((result != MECANUM_OK) && (result != MECANUM_CLAMPED))
  {
    (void)mecanum_chassis_stop(&s_chassis);
    (void)mecanum_chassis_enable(&s_chassis, false);
    test_write_text("fusion,error,motion_command_failed\r\n");
    return 0U;
  }

  start_tick = HAL_GetTick();
  next_log_tick = start_tick;
  next_control_tick = start_tick;
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_TIMEOUT_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    displacement_x_m = s_inertial_nav.x_m - start_x_m;
    displacement_y_m = s_inertial_nav.y_m - start_y_m;
    progress_m = displacement_x_m * world_direction_x +
                 displacement_y_m * world_direction_y;
    if ((deceleration_sent == 0U) &&
        ((distance_m - progress_m) <= MOTION_TEST_DECEL_DISTANCE_M))
    {
      command_speed_mps = MOTION_TEST_DECEL_SPEED_MPS;
      deceleration_sent = 1U;
      test_write_text("fusion,segment_decelerate\r\n");
    }
    if (progress_m >= distance_m)
    {
      break;
    }
    if ((MOTION_TEST_HEADING_HOLD != 0U) &&
        ((int32_t)(HAL_GetTick() - next_control_tick) >= 0))
    {
      /* 用 sin/cos 计算最短角度误差，避免跨越 +/-pi 时跳变。 */
      heading_error_rad = atan2f(
          sinf(heading_reference_rad - s_inertial_nav.yaw_rad),
          cosf(heading_reference_rad - s_inertial_nav.yaw_rad));
      correction_wz_rad_s = MOTION_TEST_HEADING_KP * heading_error_rad;
      if (correction_wz_rad_s > MOTION_TEST_MAX_CORRECTION_WZ)
      {
        correction_wz_rad_s = MOTION_TEST_MAX_CORRECTION_WZ;
      }
      else if (correction_wz_rad_s < -MOTION_TEST_MAX_CORRECTION_WZ)
      {
        correction_wz_rad_s = -MOTION_TEST_MAX_CORRECTION_WZ;
      }
      result = mecanum_chassis_set_velocity(&s_chassis,
                                            body_direction_x * command_speed_mps,
                                            body_direction_y * command_speed_mps,
                                            correction_wz_rad_s);
      if ((result != MECANUM_OK) && (result != MECANUM_CLAMPED))
      {
        (void)mecanum_chassis_stop(&s_chassis);
        (void)mecanum_chassis_enable(&s_chassis, false);
        test_write_text("fusion,error,heading_control_failed\r\n");
        return 0U;
      }
      next_control_tick += MOTION_TEST_CONTROL_PERIOD_MS;
    }
    if ((int32_t)(HAL_GetTick() - next_log_tick) >= 0)
    {
      fusion_write_status();
      next_log_tick += 500U;
    }
    HAL_Delay(5U);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  if (progress_m < distance_m)
  {
    test_write_text("fusion,error,segment_timeout\r\n");
    return 0U;
  }

  /* 停车后继续采集一段时间，等待最后几帧位置反馈进入融合。 */
  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_SETTLE_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    HAL_Delay(5U);
  }

  displacement_x_m = s_inertial_nav.x_m - start_x_m;
  displacement_y_m = s_inertial_nav.y_m - start_y_m;
  world_direction_x = cosf(heading_reference_rad) *
                      (vx_mps / MOTION_TEST_SPEED_MPS) -
                      sinf(heading_reference_rad) *
                      (vy_mps / MOTION_TEST_SPEED_MPS);
  world_direction_y = sinf(heading_reference_rad) *
                      (vx_mps / MOTION_TEST_SPEED_MPS) +
                      cosf(heading_reference_rad) *
                      (vy_mps / MOTION_TEST_SPEED_MPS);
  progress_m = displacement_x_m * world_direction_x +
               displacement_y_m * world_direction_y;

  length = snprintf(line,
                    sizeof(line),
                    "fusion,segment_end,name,%s,progress_m,%.4f,"
                    "dx_m,%.4f,dy_m,%.4f,"
                    "dyaw_deg,%.2f,x_m,%.4f,y_m,%.4f,yaw_deg,%.2f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)progress_m,
                    (double)(s_inertial_nav.x_m - start_x_m),
                    (double)(s_inertial_nav.y_m - start_y_m),
                    (double)((s_inertial_nav.yaw_rad - start_yaw_rad) *
                             57.295779513f),
                    (double)s_inertial_nav.x_m,
                    (double)s_inertial_nav.y_m,
                    (double)(s_inertial_nav.yaw_rad * 57.295779513f));
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
  return 1U;
}
#endif

static uint8_t fusion_run_rotation_segment(const char *name,
                                           float target_angle_rad)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;
  float start_yaw_rad;
  float previous_yaw_rad;
  float angle_accum_rad = 0.0f;
  float command_wz_rad_s = MOTION_TEST_ROTATION_SPEED_RAD_S;
  uint32_t start_tick;
  uint32_t next_log_tick;
  uint32_t next_control_tick;
  uint8_t deceleration_sent = 0U;
  mecanum_result_t result;

  length = snprintf(line,
                    sizeof(line),
                    "fusion,rotation_start,name,%s,target_deg,%.1f,"
                    "speed_rad_s,%.3f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)(target_angle_rad * 57.295779513f),
                    (double)MOTION_TEST_ROTATION_SPEED_RAD_S);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_SETTLE_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    HAL_Delay(5U);
  }

  start_yaw_rad = s_inertial_nav.yaw_rad;
  previous_yaw_rad = start_yaw_rad;
  result = mecanum_chassis_set_velocity(&s_chassis,
                                        0.0f,
                                        0.0f,
                                        command_wz_rad_s);
  if ((result != MECANUM_OK) && (result != MECANUM_CLAMPED))
  {
    test_write_text("fusion,error,rotation_command_failed\r\n");
    return 0U;
  }

  start_tick = HAL_GetTick();
  next_log_tick = start_tick;
  next_control_tick = start_tick;
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         MOTION_TEST_ROTATION_TIMEOUT_MS)
  {
    float angle_step_rad;
    float remaining_rad;

    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    angle_step_rad = fusion_wrap_angle(s_inertial_nav.yaw_rad -
                                       previous_yaw_rad);
    angle_accum_rad += angle_step_rad;
    previous_yaw_rad = s_inertial_nav.yaw_rad;
    remaining_rad = target_angle_rad - angle_accum_rad;

    if ((deceleration_sent == 0U) &&
        (remaining_rad <= MOTION_TEST_ROTATION_DECEL_DEG *
                          0.01745329252f))
    {
      command_wz_rad_s = MOTION_TEST_ROTATION_DECEL_SPEED_RAD_S;
      deceleration_sent = 1U;
      test_write_text("fusion,rotation_decelerate\r\n");
    }
    if (angle_accum_rad >= target_angle_rad)
    {
      break;
    }
    if ((int32_t)(HAL_GetTick() - next_control_tick) >= 0)
    {
      result = mecanum_chassis_set_velocity(&s_chassis,
                                            0.0f,
                                            0.0f,
                                            command_wz_rad_s);
      if ((result != MECANUM_OK) && (result != MECANUM_CLAMPED))
      {
        (void)mecanum_chassis_stop(&s_chassis);
        test_write_text("fusion,error,rotation_control_failed\r\n");
        return 0U;
      }
      next_control_tick += MOTION_TEST_CONTROL_PERIOD_MS;
    }
    if ((int32_t)(HAL_GetTick() - next_log_tick) >= 0)
    {
      fusion_write_status();
      next_log_tick += 500U;
    }
    HAL_Delay(5U);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  if (angle_accum_rad < target_angle_rad)
  {
    test_write_text("fusion,error,rotation_timeout\r\n");
    return 0U;
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_SETTLE_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    HAL_Delay(5U);
  }

  length = snprintf(line,
                    sizeof(line),
                    "fusion,rotation_end,name,%s,angle_deg,%.2f,"
                    "start_yaw_deg,%.2f,end_yaw_deg,%.2f\r\n",
                    (name != NULL) ? name : "unknown",
                    (double)(angle_accum_rad * 57.295779513f),
                    (double)(start_yaw_rad * 57.295779513f),
                    (double)(s_inertial_nav.yaw_rad * 57.295779513f));
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
  return 1U;
}

static uint8_t fusion_run_motion_test(void)
{
  uint32_t start_tick;
  mecanum_result_t result;

  test_write_text("fusion,test_start,sequence,rotate_90_ccw_x4\r\n");

  /* 先保持静止建立 0x36 位置基准，避免把启动前位置算进测试位移。 */
  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < MOTION_TEST_SETTLE_MS)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
    (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                      &s_chassis,
                                      &s_imu_sensor);
    HAL_Delay(5U);
  }

  result = mecanum_chassis_enable(&s_chassis, true);
  if (result != MECANUM_OK)
  {
    test_write_text("fusion,error,motor_enable_failed\r\n");
    return 0U;
  }

  if ((fusion_run_rotation_segment("rotate_90_ccw_1",
                                   90.0f * 0.01745329252f) == 0U) ||
      (fusion_run_rotation_segment("rotate_90_ccw_2",
                                   90.0f * 0.01745329252f) == 0U) ||
      (fusion_run_rotation_segment("rotate_90_ccw_3",
                                   90.0f * 0.01745329252f) == 0U) ||
      (fusion_run_rotation_segment("rotate_90_ccw_4",
                                   90.0f * 0.01745329252f) == 0U))
  {
    (void)mecanum_chassis_enable(&s_chassis, false);
    return 0U;
  }

  (void)mecanum_chassis_stop(&s_chassis);
  (void)mecanum_chassis_enable(&s_chassis, false);
  test_write_text("fusion,test_end,final_state\r\n");
  fusion_write_status();
  return 1U;
}

static const char *calibration_test_motion_name(
    calibration_motion_t motion)
{
  switch (motion)
  {
    case CALIBRATION_MOTION_FORWARD:
      return "forward";
    case CALIBRATION_MOTION_BACKWARD:
      return "backward";
    case CALIBRATION_MOTION_LEFT:
      return "left";
    case CALIBRATION_MOTION_RIGHT:
      return "right";
    case CALIBRATION_MOTION_ROTATE_CCW:
      return "rotate_ccw";
    case CALIBRATION_MOTION_ROTATE_CW:
      return "rotate_cw";
    default:
      return "unknown";
  }
}

static uint8_t calibration_test_read_position_state(
    int64_t position[MECANUM_WHEEL_COUNT],
    uint32_t update_count[MECANUM_WHEEL_COUNT])
{
  uint32_t primask;
  uint8_t index;
  uint8_t ready = 1U;

  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    if (s_motors[index].position_valid == 0U)
    {
      ready = 0U;
    }
    position[index] = s_motors[index].actual_position_pulse;
    update_count[index] = s_motors[index].position_update_count;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return ready;
}

/* 只使用四路位置反馈都更新后的快照，避免新旧数据混合。 */
static uint8_t calibration_test_read_positions(
    int64_t position[MECANUM_WHEEL_COUNT],
    uint32_t update_count[MECANUM_WHEEL_COUNT],
    const uint32_t baseline_count[MECANUM_WHEEL_COUNT],
    uint8_t require_new,
    uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  uint8_t index;

  while ((uint32_t)(HAL_GetTick() - start_tick) < timeout_ms)
  {
    uint8_t all_updated = 1U;

    calibration_test_service();
    if (calibration_test_read_position_state(position, update_count) == 0U)
    {
      HAL_Delay(CALIBRATION_TEST_LOOP_DELAY_MS);
      continue;
    }

    if (require_new != 0U)
    {
      for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
      {
        if (update_count[index] == baseline_count[index])
        {
          all_updated = 0U;
          break;
        }
      }
      if (all_updated == 0U)
      {
        HAL_Delay(CALIBRATION_TEST_LOOP_DELAY_MS);
        continue;
      }
    }
    return 1U;
  }

  return 0U;
}

static void calibration_test_service(void)
{
  if (s_chassis_ready != 0U)
  {
    (void)mecanum_chassis_feedback_task(&s_chassis);
  }
}

static void calibration_test_write_snapshot(
    const char *label,
    const int64_t position[MECANUM_WHEEL_COUNT],
    const uint32_t update_count[MECANUM_WHEEL_COUNT])
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "calibration,%s,tick,%lu,fl,%ld,fr,%ld,rl,%ld,rr,%ld,count,%lu,%lu,%lu,%lu\r\n",
                    (label != NULL) ? label : "unknown",
                    (unsigned long)HAL_GetTick(),
                    (long)(int32_t)position[MECANUM_WHEEL_FL],
                    (long)(int32_t)position[MECANUM_WHEEL_FR],
                    (long)(int32_t)position[MECANUM_WHEEL_RL],
                    (long)(int32_t)position[MECANUM_WHEEL_RR],
                    (unsigned long)update_count[MECANUM_WHEEL_FL],
                    (unsigned long)update_count[MECANUM_WHEEL_FR],
                    (unsigned long)update_count[MECANUM_WHEEL_RL],
                    (unsigned long)update_count[MECANUM_WHEEL_RR]);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static void calibration_test_write_diagnostics(const char *reason)
{
  char line[TEST_LINE_BUFFER_SIZE];
  int length;

  length = snprintf(line,
                    sizeof(line),
                    "calibration,error,%s,valid,%u,%u,%u,%u,updates,%lu,%lu,%lu,%lu,rx_frames,%lu,rx_errors,%lu,can_error,0x%08lX\r\n",
                    (reason != NULL) ? reason : "unknown",
                    (unsigned int)s_motors[0].position_valid,
                    (unsigned int)s_motors[1].position_valid,
                    (unsigned int)s_motors[2].position_valid,
                    (unsigned int)s_motors[3].position_valid,
                    (unsigned long)s_motors[0].position_update_count,
                    (unsigned long)s_motors[1].position_update_count,
                    (unsigned long)s_motors[2].position_update_count,
                    (unsigned long)s_motors[3].position_update_count,
                    (unsigned long)s_motor_bus.rx_frame_count,
                    (unsigned long)s_motor_bus.rx_error_count,
                    (unsigned long)s_motor_bus.last_can_error);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static void calibration_test_write_result(
    calibration_motion_t motion,
    uint32_t elapsed_ms,
    const int64_t delta[MECANUM_WHEEL_COUNT])
{
  float wheel_distance[MECANUM_WHEEL_COUNT];
  float geometry_factor;
  float dx;
  float dy;
  float dyaw;
  char line[TEST_LINE_BUFFER_SIZE];
  int length;
  uint8_t index;

  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    wheel_distance[index] = (float)delta[index] *
                            (2.0f * CALIBRATION_TEST_PI_F *
                             s_chassis.config.wheel_radius_m) /
                            (float)s_chassis.config.pulses_per_rev;
    wheel_distance[index] *=
        (float)s_chassis.config.direction_sign[index];
  }

  geometry_factor = s_chassis.config.half_length_m +
                    s_chassis.config.half_width_m;
  dx = (wheel_distance[MECANUM_WHEEL_FL] +
        wheel_distance[MECANUM_WHEEL_FR] +
        wheel_distance[MECANUM_WHEEL_RL] +
        wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
  dy = (-wheel_distance[MECANUM_WHEEL_FL] +
        wheel_distance[MECANUM_WHEEL_FR] +
        wheel_distance[MECANUM_WHEEL_RL] -
        wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
  dyaw = (-wheel_distance[MECANUM_WHEEL_FL] +
          wheel_distance[MECANUM_WHEEL_FR] -
          wheel_distance[MECANUM_WHEEL_RL] +
          wheel_distance[MECANUM_WHEEL_RR]) /
         (4.0f * geometry_factor);

  dx *= (s_chassis.config.odometry_x_scale > 0.0f) ?
        s_chassis.config.odometry_x_scale : 1.0f;
  dy *= (s_chassis.config.odometry_y_scale > 0.0f) ?
        s_chassis.config.odometry_y_scale : 1.0f;
  dyaw *= (s_chassis.config.odometry_yaw_scale > 0.0f) ?
          s_chassis.config.odometry_yaw_scale : 1.0f;

  length = snprintf(line,
                    sizeof(line),
                    "calibration,result,%s,elapsed_ms,%lu,dfl,%ld,dfr,%ld,drl,%ld,drr,%ld\r\n",
                    calibration_test_motion_name(motion),
                    (unsigned long)elapsed_ms,
                    (long)(int32_t)delta[MECANUM_WHEEL_FL],
                    (long)(int32_t)delta[MECANUM_WHEEL_FR],
                    (long)(int32_t)delta[MECANUM_WHEEL_RL],
                    (long)(int32_t)delta[MECANUM_WHEEL_RR]);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }

  length = snprintf(line,
                    sizeof(line),
                    "calibration,estimate,%s,dx_m,%.6f,dy_m,%.6f,dyaw_deg,%.4f,config_radius_m,%.6f,geometry_m,%.6f\r\n",
                    calibration_test_motion_name(motion),
                    (double)dx,
                    (double)dy,
                    (double)(dyaw * 180.0f / CALIBRATION_TEST_PI_F),
                    (double)s_chassis.config.wheel_radius_m,
                    (double)geometry_factor);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }
}

static uint8_t calibration_test_run_once(calibration_motion_t motion)
{
  int64_t start_position[MECANUM_WHEEL_COUNT];
  int64_t end_position[MECANUM_WHEEL_COUNT];
  int64_t delta[MECANUM_WHEEL_COUNT];
  uint32_t start_count[MECANUM_WHEEL_COUNT];
  uint32_t end_count[MECANUM_WHEEL_COUNT];
  uint32_t run_time_ms;
  uint32_t start_tick;
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  float wz_rad_s = 0.0f;
  float rotation_angle_rad;
  uint8_t index;

  switch (motion)
  {
    case CALIBRATION_MOTION_FORWARD:
      vx_mps = CALIBRATION_TEST_LINEAR_SPEED_MPS;
      run_time_ms = CALIBRATION_TEST_LINEAR_RUN_TIME_MS;
      break;
    case CALIBRATION_MOTION_BACKWARD:
      vx_mps = -CALIBRATION_TEST_LINEAR_SPEED_MPS;
      run_time_ms = CALIBRATION_TEST_LINEAR_RUN_TIME_MS;
      break;
    case CALIBRATION_MOTION_LEFT:
      vy_mps = CALIBRATION_TEST_LINEAR_SPEED_MPS;
      run_time_ms = CALIBRATION_TEST_LINEAR_RUN_TIME_MS;
      break;
    case CALIBRATION_MOTION_RIGHT:
      vy_mps = -CALIBRATION_TEST_LINEAR_SPEED_MPS;
      run_time_ms = CALIBRATION_TEST_LINEAR_RUN_TIME_MS;
      break;
    case CALIBRATION_MOTION_ROTATE_CCW:
      wz_rad_s = CALIBRATION_TEST_ROTATION_SPEED_RAD_S;
      rotation_angle_rad = CALIBRATION_TEST_ROTATION_ANGLE_DEG *
                           CALIBRATION_TEST_PI_F / 180.0f;
      run_time_ms = (uint32_t)((rotation_angle_rad / wz_rad_s) *
                               1000.0f + 0.5f);
      break;
    case CALIBRATION_MOTION_ROTATE_CW:
      wz_rad_s = -CALIBRATION_TEST_ROTATION_SPEED_RAD_S;
      rotation_angle_rad = CALIBRATION_TEST_ROTATION_ANGLE_DEG *
                           CALIBRATION_TEST_PI_F / 180.0f;
      run_time_ms = (uint32_t)((rotation_angle_rad /
                                (-wz_rad_s)) * 1000.0f + 0.5f);
      break;
    default:
      return 0U;
  }

  (void)mecanum_chassis_stop(&s_chassis);
  (void)mecanum_chassis_enable(&s_chassis, false);
  HAL_Delay(CALIBRATION_TEST_STARTUP_SETTLE_MS);

  if (calibration_test_read_positions(start_position,
                                      start_count,
                                      start_count,
                                      0U,
                                      CALIBRATION_TEST_READY_TIMEOUT_MS) == 0U)
  {
    calibration_test_write_diagnostics("start_position_timeout");
    return 0U;
  }
  calibration_test_write_snapshot("start", start_position, start_count);

  if (mecanum_chassis_enable(&s_chassis, true) != MECANUM_OK)
  {
    calibration_test_write_diagnostics("enable_failed");
    return 0U;
  }
  if (mecanum_chassis_set_velocity(&s_chassis,
                                   vx_mps,
                                   vy_mps,
                                   wz_rad_s) == MECANUM_MOTOR_ERROR)
  {
    (void)mecanum_chassis_stop(&s_chassis);
    calibration_test_write_diagnostics("command_failed");
    return 0U;
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < run_time_ms)
  {
    calibration_test_service();
    HAL_Delay(CALIBRATION_TEST_LOOP_DELAY_MS);
  }

  (void)mecanum_chassis_stop(&s_chassis);
  HAL_Delay(CALIBRATION_TEST_STOP_SETTLE_MS);
  if (calibration_test_read_positions(end_position,
                                      end_count,
                                      start_count,
                                      1U,
                                      CALIBRATION_TEST_READY_TIMEOUT_MS) == 0U)
  {
    (void)mecanum_chassis_enable(&s_chassis, false);
    calibration_test_write_diagnostics("end_position_timeout");
    return 0U;
  }
  (void)mecanum_chassis_enable(&s_chassis, false);

  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    delta[index] = end_position[index] - start_position[index];
  }

  calibration_test_write_snapshot("end", end_position, end_count);
  calibration_test_write_result(motion,
                                (uint32_t)(HAL_GetTick() - start_tick),
                                delta);
  test_write_text("calibration,segment_complete\r\n");
  return 1U;
}

static uint8_t calibration_test_run(void)
{
  static const calibration_motion_t motion_sequence[] =
  {
    CALIBRATION_MOTION_FORWARD,
    CALIBRATION_MOTION_BACKWARD,
    CALIBRATION_MOTION_LEFT,
    CALIBRATION_MOTION_RIGHT,
    CALIBRATION_MOTION_ROTATE_CCW,
    CALIBRATION_MOTION_ROTATE_CW
  };
  char line[TEST_LINE_BUFFER_SIZE];
  int length;
  uint8_t index;

  length = snprintf(line,
                    sizeof(line),
                    "calibration,begin,mode,all,linear_speed_mps,%.3f,rotation_speed_rad_s,%.3f,rotation_angle_deg,%.1f,linear_run_ms,%lu\r\n",
                    (double)CALIBRATION_TEST_LINEAR_SPEED_MPS,
                    (double)CALIBRATION_TEST_ROTATION_SPEED_RAD_S,
                    (double)CALIBRATION_TEST_ROTATION_ANGLE_DEG,
                    (unsigned long)CALIBRATION_TEST_LINEAR_RUN_TIME_MS);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    test_write_text(line);
  }

  for (index = 0U;
       index < (uint8_t)(sizeof(motion_sequence) /
                         sizeof(motion_sequence[0]));
       index++)
  {
    if (calibration_test_run_once(motion_sequence[index]) == 0U)
    {
      (void)mecanum_chassis_stop(&s_chassis);
      (void)mecanum_chassis_enable(&s_chassis, false);
      test_write_text("calibration,sequence_failed\r\n");
      return 0U;
    }
  }

  test_write_text("calibration,sequence_complete\r\n");
  return 1U;
}

#if 0

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
                      (unsigned long)s_motors[0].position_update_count,
                      (unsigned long)s_motors[1].position_update_count,
                      (unsigned long)s_motors[2].position_update_count,
                      (unsigned long)s_motors[3].position_update_count);
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
                      (unsigned long)s_motors[0].position_update_count,
                      (unsigned long)s_motors[1].position_update_count,
                      (unsigned long)s_motors[2].position_update_count,
                      (unsigned long)s_motors[3].position_update_count);
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

/* 返回四个电机当前是否都收到过有效的 0x36 位置反馈。 */
static uint8_t chassis_test_encoder_valid_mask(void)
{
  uint8_t index;
  uint8_t mask = 0U;

  for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
  {
    if (s_motors[index].position_valid != 0U)
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
                    (unsigned long)s_motors[0].position_update_count,
                    (unsigned long)s_motors[1].position_update_count,
                    (unsigned long)s_motors[2].position_update_count,
                    (unsigned long)s_motors[3].position_update_count,
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
                                    CHASSIS_TEST_FORWARD_VY_CORRECTION_MPS) == 0U) ||
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

#endif

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  motor_result_t motor_result;
  mecanum_result_t chassis_result;
  jy901s_uart_result_t imu_result;
  mecanum_inertial_nav_result_t nav_result;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  test_write_text("fusion,boot,USART6_ready\r\n");
  MX_CAN1_Init();

  motor_result = motor_bus_init(&s_motor_bus,
                                &hcan1,
                                motor_can_hal_send_frame,
                                 CALIBRATION_TEST_CAN_TIMEOUT_MS);
  if ((motor_result != MOTOR_OK) ||
      (motor_bus_start(&s_motor_bus) != MOTOR_OK))
  {
    test_write_text("calibration,error,can_start_failed\r\n");
  }
  else
  {
    chassis_result = mecanum_chassis_init_default(&s_chassis,
                                                  &s_motor_bus,
                                                  s_motors);
    if (chassis_result != MECANUM_OK)
    {
       test_write_text("calibration,error,chassis_init_failed\r\n");
    }
    else
    {
      s_chassis_ready = 1U;
    }
  }

  if ((s_chassis_ready != 0U) &&
      (APP_RUN_CALIBRATION_TEST == 0U))
  {
    /* IMU 校准阶段先失能电机，校准成功后自动测试会重新使能。 */
    chassis_result = mecanum_chassis_enable(&s_chassis, false);
    if (chassis_result != MECANUM_OK)
    {
      test_write_text("fusion,error,motor_disable_failed\r\n");
      s_chassis_ready = 0U;
    }
    else
    {
      test_write_text("fusion,motors_disabled,imu_calibration\r\n");
    }
  }

  if (s_chassis_ready != 0U)
  {
    if (APP_RUN_CALIBRATION_TEST != 0U)
    {
      (void)calibration_test_run();
    }
    else
    {
      imu_result = jy901s_uart_init(&s_imu_sensor, &huart1);
      if (imu_result != JY901S_UART_OK)
      {
        test_write_text("fusion,error,imu_init_failed\r\n");
      }
      else
      {
        /* JY901 RSW：加速度(bit1)+角速度(bit2)，100 Hz 输出。 */
        imu_result = jy901s_uart_configure_output(
            &s_imu_sensor,
            JY901S_UART_OUTPUT_GYRO_ACCEL,
            JY901S_UART_RATE_100HZ);
        if (imu_result != JY901S_UART_OK)
        {
          test_write_text("fusion,error,imu_config_failed\r\n");
        }
        else
        {
          test_write_text("fusion,imu_ready,keep_robot_static\r\n");
          imu_result = imu_gyro_calibration_run_default(
              &s_imu_calibration,
              &s_imu_sensor);
          if ((imu_result != JY901S_UART_OK) ||
              (s_imu_calibration.calibrated == 0U))
          {
            test_write_text("fusion,error,imu_calibration_failed\r\n");
          }
          else
          {
            nav_result = mecanum_inertial_nav_init_default(
                &s_inertial_nav,
                &s_chassis.config,
                imu_gyro_calibration_get_bias(&s_imu_calibration));
            if (nav_result != MECANUM_INERTIAL_NAV_OK)
            {
              test_write_text("fusion,error,nav_init_failed\r\n");
            }
            else
            {
              s_fusion_ready = 1U;
              s_last_fusion_log_tick = HAL_GetTick();
              test_write_text("fusion,ready\r\n");
              if (APP_RUN_MOTION_TEST != 0U)
              {
                (void)fusion_run_motion_test();
              }
            }
          }
        }
      }
    }
  }

  while (1)
  {
    if (s_chassis_ready != 0U)
    {
      (void)mecanum_chassis_feedback_task(&s_chassis);
    }
    if (s_fusion_ready != 0U)
    {
      (void)mecanum_inertial_nav_update(&s_inertial_nav,
                                        &s_chassis,
                                        &s_imu_sensor);
      if ((uint32_t)(HAL_GetTick() - s_last_fusion_log_tick) >= 500U)
      {
        s_last_fusion_log_tick = HAL_GetTick();
        fusion_write_status();
      }
    }
    HAL_Delay(5U);
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
