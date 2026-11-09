#include "square_odometry_test.h"

#include "HEADING_HOLD_CONTROLLER.h"
#include "VELOCITY_ODOMETRY.h"
#include "chassis_odometry_motion_config.h"

#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define SQUARE_ODOMETRY_TEST_SIDE_M             2.10f
#define SQUARE_ODOMETRY_TEST_LEFT_COMP_M        CHASSIS_ODOMETRY_LEFT_COMP_M
#define SQUARE_ODOMETRY_TEST_CONTROL_PERIOD_MS  CHASSIS_ODOMETRY_CONTROL_PERIOD_MS
#define SQUARE_ODOMETRY_TEST_OUTPUT_PERIOD_MS   100U
#define SQUARE_ODOMETRY_TEST_SETTLE_MS          CHASSIS_ODOMETRY_SETTLE_MS
#define SQUARE_ODOMETRY_TEST_CORNER_STOP_MS     CHASSIS_ODOMETRY_CORNER_STOP_MS
#define SQUARE_ODOMETRY_TEST_STOP_STABLE_MS     CHASSIS_ODOMETRY_STOP_STABLE_MS
#define SQUARE_ODOMETRY_TEST_STOP_TIMEOUT_MS    CHASSIS_ODOMETRY_STOP_TIMEOUT_MS
#define SQUARE_ODOMETRY_TEST_STOP_SPEED_MPS     CHASSIS_ODOMETRY_STOP_SPEED_MPS
#define SQUARE_ODOMETRY_TEST_SEGMENT_TIMEOUT_MS CHASSIS_ODOMETRY_SEGMENT_TIMEOUT_MS
#define SQUARE_ODOMETRY_TEST_MAX_YAW_AGE_MS     CHASSIS_ODOMETRY_MAX_YAW_AGE_MS
#define SQUARE_ODOMETRY_TEST_YAW_RESTART_MS     CHASSIS_ODOMETRY_YAW_RESTART_MS
#define SQUARE_ODOMETRY_TEST_MAX_ENCODER_AGE_MS CHASSIS_ODOMETRY_MAX_ENCODER_AGE_MS
#define SQUARE_ODOMETRY_TEST_DISTANCE_TOL_M     CHASSIS_ODOMETRY_DISTANCE_TOL_M
#define SQUARE_ODOMETRY_TEST_MAX_SPEED_MPS      CHASSIS_ODOMETRY_MAX_SPEED_MPS
#define SQUARE_ODOMETRY_TEST_ACCEL_MPS2         CHASSIS_ODOMETRY_ACCEL_MPS2
#define SQUARE_ODOMETRY_TEST_DECEL_MPS2         CHASSIS_ODOMETRY_DECEL_MPS2
#define SQUARE_ODOMETRY_TEST_MIN_SPEED_MPS      CHASSIS_ODOMETRY_MIN_SPEED_MPS
#define SQUARE_ODOMETRY_TEST_CROSS_TRACK_KP     CHASSIS_ODOMETRY_CROSS_TRACK_KP
#define SQUARE_ODOMETRY_TEST_CROSS_TRACK_KI     CHASSIS_ODOMETRY_CROSS_TRACK_KI
#define SQUARE_ODOMETRY_TEST_CROSS_INTEGRAL_LIMIT_M_S \
                                                    CHASSIS_ODOMETRY_CROSS_INTEGRAL_LIMIT_M_S
#define SQUARE_ODOMETRY_TEST_MAX_CROSS_SPEED_MPS CHASSIS_ODOMETRY_MAX_CROSS_SPEED_MPS
#define SQUARE_ODOMETRY_TEST_CROSS_SPEED_RATIO  CHASSIS_ODOMETRY_CROSS_SPEED_RATIO
#define SQUARE_ODOMETRY_TEST_DEG_TO_RAD         0.01745329251994329577f
#define SQUARE_ODOMETRY_TEST_TX_TIMEOUT_MS      100U

typedef enum
{
    SQUARE_STATE_WAIT_YAW = 0,
    SQUARE_STATE_SETTLING,
    SQUARE_STATE_MOVE,
    SQUARE_STATE_CORNER_STOP,
    SQUARE_STATE_FINISHED,
    SQUARE_STATE_ERROR
} square_state_t;

typedef enum
{
    SQUARE_FAULT_NONE = 0,
    SQUARE_FAULT_ENCODER_TIMEOUT,
    SQUARE_FAULT_YAW_TIMEOUT,
    SQUARE_FAULT_ODOMETRY,
    SQUARE_FAULT_HEADING_CONTROLLER,
    SQUARE_FAULT_MOTOR_COMMAND,
    SQUARE_FAULT_STOP_TIMEOUT,
    SQUARE_FAULT_SEGMENT_TIMEOUT
} square_fault_t;

static mecanum_chassis_t *s_chassis;
static hwt101ct_t *s_hwt101ct_sensor;
static velocity_odometry_t s_odometry;
static heading_hold_controller_t s_heading_hold;
static square_state_t s_state;
static uint8_t s_segment;
static uint8_t s_segment_start_valid;
static float s_segment_start_x;
static float s_segment_start_y;
static float s_target_yaw_deg;
static uint32_t s_state_start_ms;
static uint32_t s_last_control_ms;
static uint32_t s_last_output_ms;
static uint32_t s_last_encoder_ok_ms;
static uint32_t s_stop_stable_start_ms;
static uint8_t s_stop_stable_active;
static square_fault_t s_fault;
static uint8_t s_initialized;
static float s_progress_m;
static float s_remaining_m;
static float s_cross_track_error_m;
static float s_cross_track_integral_m_s;
static float s_along_track_speed_mps;
static float s_cross_correction_mps;
static float s_total_distance_m;
static float s_last_distance_x_m;
static float s_last_distance_y_m;
static uint8_t s_distance_position_valid;

static void square_send_text(const char *text)
{
    if (text != NULL)
    {
        (void)HAL_UART_Transmit(&huart6,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                SQUARE_ODOMETRY_TEST_TX_TIMEOUT_MS);
    }
}

static uint32_t square_age_ms(uint32_t now_ms, uint32_t update_ms)
{
    int32_t elapsed_ms = (int32_t)(now_ms - update_ms);

    /* 中断可能在主循环保存 now_ms 后更新数据时间戳。 */
    return (elapsed_ms < 0) ? 0U : (uint32_t)elapsed_ms;
}

static float square_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static const char *square_state_name(void)
{
    switch (s_state)
    {
        case SQUARE_STATE_SETTLING:
            return "SETTLE";
        case SQUARE_STATE_MOVE:
            return "MOVE";
        case SQUARE_STATE_CORNER_STOP:
            return "CORNER";
        case SQUARE_STATE_FINISHED:
            return "FINISHED";
        case SQUARE_STATE_ERROR:
            return "ERROR";
        case SQUARE_STATE_WAIT_YAW:
        default:
            return "WAIT_YAW";
    }
}

static const char *square_fault_name(void)
{
    switch (s_fault)
    {
        case SQUARE_FAULT_ENCODER_TIMEOUT:
            return "ENCODER_TIMEOUT";
        case SQUARE_FAULT_YAW_TIMEOUT:
            return "YAW_TIMEOUT";
        case SQUARE_FAULT_ODOMETRY:
            return "ODOMETRY";
        case SQUARE_FAULT_HEADING_CONTROLLER:
            return "HEADING_CONTROLLER";
        case SQUARE_FAULT_MOTOR_COMMAND:
            return "MOTOR_COMMAND";
        case SQUARE_FAULT_STOP_TIMEOUT:
            return "STOP_TIMEOUT";
        case SQUARE_FAULT_SEGMENT_TIMEOUT:
            return "SEGMENT_TIMEOUT";
        case SQUARE_FAULT_NONE:
        default:
            return "NONE";
    }
}

static void square_stop(void)
{
    if (s_chassis != NULL)
    {
        (void)mecanum_chassis_stop(s_chassis);
    }
}

static uint8_t square_get_yaw(uint32_t now_ms, hwt101ct_data_t *data)
{
    if ((data == NULL) ||
        (hwt101ct_get_data(s_hwt101ct_sensor, data) != HWT101CT_OK) ||
        (data->yaw_valid == 0U))
    {
        return 0U;
    }

    return (square_age_ms(now_ms, data->yaw_update_ms) <=
            SQUARE_ODOMETRY_TEST_MAX_YAW_AGE_MS) ? 1U : 0U;
}

static void square_set_state(square_state_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;
    s_segment_start_valid = 0U;
    if (state == SQUARE_STATE_CORNER_STOP)
    {
        s_stop_stable_start_ms = 0U;
        s_stop_stable_active = 0U;
    }

    if ((state == SQUARE_STATE_FINISHED) ||
        (state == SQUARE_STATE_ERROR))
    {
        heading_hold_controller_disable(&s_heading_hold);
        square_stop();
    }
}

static void square_set_error(square_fault_t fault, uint32_t now_ms)
{
    s_fault = fault;
    square_set_state(SQUARE_STATE_ERROR, now_ms);
}

static void square_get_body_direction(float *vx_sign, float *vy_sign)
{
    *vx_sign = 0.0f;
    *vy_sign = 0.0f;

    switch (s_segment)
    {
        case 0U:
            *vx_sign = 1.0f;
            break;
        case 1U:
            *vy_sign = 1.0f;
            break;
        case 2U:
            *vx_sign = -1.0f;
            break;
        case 3U:
            *vy_sign = -1.0f;
            break;
        default:
            break;
    }
}

static float square_get_segment_target_m(void)
{
    /* 麦轮向左横移存在少量实际位移损失，只补偿左移这一段。 */
    if (s_segment == 1U)
    {
        return SQUARE_ODOMETRY_TEST_SIDE_M +
               SQUARE_ODOMETRY_TEST_LEFT_COMP_M;
    }

    return SQUARE_ODOMETRY_TEST_SIDE_M;
}

static uint8_t square_corner_is_stable(uint32_t now_ms)
{
    mecanum_encoder_velocity_t velocity;

    if (mecanum_chassis_get_encoder_velocity(s_chassis, &velocity) !=
        MECANUM_OK)
    {
        s_stop_stable_active = 0U;
        return 0U;
    }

    if ((fabsf(velocity.vx_mps) > SQUARE_ODOMETRY_TEST_STOP_SPEED_MPS) ||
        (fabsf(velocity.vy_mps) > SQUARE_ODOMETRY_TEST_STOP_SPEED_MPS))
    {
        s_stop_stable_active = 0U;
        return 0U;
    }

    if (s_stop_stable_active == 0U)
    {
        s_stop_stable_start_ms = now_ms;
        s_stop_stable_active = 1U;
        return 0U;
    }

    return (square_age_ms(now_ms, s_stop_stable_start_ms) >=
            SQUARE_ODOMETRY_TEST_STOP_STABLE_MS) ? 1U : 0U;
}

static uint8_t square_update_odometry(uint32_t now_ms)
{
    mecanum_encoder_velocity_t velocity;
    hwt101ct_data_t yaw_data;
    velocity_odometry_result_t odometry_result;
    float x_m;
    float y_m;
    float delta_x_m;
    float delta_y_m;

    (void)mecanum_chassis_feedback_task(s_chassis);
    if (mecanum_chassis_update_encoder_velocity(s_chassis) == MECANUM_OK)
    {
        s_last_encoder_ok_ms = HAL_GetTick();
    }

    /* CAN 命令包含帧间延时，期间中断可能刷新传感器时间戳。 */
    now_ms = HAL_GetTick();

    if (square_age_ms(now_ms, s_last_encoder_ok_ms) >
        SQUARE_ODOMETRY_TEST_MAX_ENCODER_AGE_MS)
    {
        s_fault = SQUARE_FAULT_ENCODER_TIMEOUT;
        return 0U;
    }
    if (mecanum_chassis_get_encoder_velocity(s_chassis, &velocity) !=
        MECANUM_OK)
    {
        s_fault = SQUARE_FAULT_ENCODER_TIMEOUT;
        return 0U;
    }
    if (square_get_yaw(now_ms, &yaw_data) == 0U)
    {
        s_fault = SQUARE_FAULT_YAW_TIMEOUT;
        return 0U;
    }

    odometry_result = velocity_odometry_update(&s_odometry,
                                               velocity.vx_mps,
                                               velocity.vy_mps,
                                               yaw_data.yaw_deg,
                                               now_ms);
    /* 主循环可能在同一毫秒内重复调用；这种时间戳错误不代表传感器失效。 */
    if (odometry_result == VELOCITY_ODOMETRY_TIME_ERROR)
    {
        return 1U;
    }
    if (odometry_result != VELOCITY_ODOMETRY_OK)
    {
        s_fault = SQUARE_FAULT_ODOMETRY;
        return 0U;
    }

    if (velocity_odometry_get_position(&s_odometry, &x_m, &y_m) ==
        VELOCITY_ODOMETRY_OK)
    {
        if (s_distance_position_valid != 0U)
        {
            delta_x_m = x_m - s_last_distance_x_m;
            delta_y_m = y_m - s_last_distance_y_m;
            s_total_distance_m += sqrtf(delta_x_m * delta_x_m +
                                        delta_y_m * delta_y_m);
        }
        s_last_distance_x_m = x_m;
        s_last_distance_y_m = y_m;
        s_distance_position_valid = 1U;
    }
    return 1U;
}

static void square_start_segment(uint32_t now_ms)
{
    float x_m;
    float y_m;

    if ((velocity_odometry_get_position(&s_odometry, &x_m, &y_m) ==
         VELOCITY_ODOMETRY_OK))
    {
        s_segment_start_x = x_m;
        s_segment_start_y = y_m;
        s_segment_start_valid = 1U;
        s_progress_m = 0.0f;
        s_remaining_m = square_get_segment_target_m();
        s_cross_track_error_m = 0.0f;
        s_cross_track_integral_m_s = 0.0f;
        s_cross_correction_mps = 0.0f;
        s_last_control_ms = now_ms - SQUARE_ODOMETRY_TEST_CONTROL_PERIOD_MS;
        square_send_text((s_segment == 0U) ?
                         "SQUARE START FORWARD\r\n" :
                         "SQUARE NEXT SEGMENT\r\n");
    }
}

static void square_apply_motion(uint32_t now_ms,
                                const hwt101ct_data_t *yaw_data)
{
    mecanum_encoder_velocity_t encoder_velocity;
    float x_m;
    float y_m;
    float vx_sign;
    float vy_sign;
    float direction_x;
    float direction_y;
    float normal_x;
    float normal_y;
    float progress_m;
    float remaining_m;
    float cross_track_error_m;
    float speed_mps;
    float ramp_speed_mps;
    float cross_speed_limit_mps;
    float cross_correction_mps;
    float control_dt_s;
    float velocity_world_x_mps;
    float velocity_world_y_mps;
    float velocity_body_x_mps;
    float velocity_body_y_mps;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float actual_world_vx_mps;
    float actual_world_vy_mps;
    float correction_wz_rad_s = 0.0f;
    uint32_t elapsed_ms;

    if ((yaw_data == NULL) ||
        ((uint32_t)(now_ms - s_last_control_ms) <
         SQUARE_ODOMETRY_TEST_CONTROL_PERIOD_MS))
    {
        return;
    }
    control_dt_s = (float)(now_ms - s_last_control_ms) * 0.001f;
    s_last_control_ms = now_ms;

    if (s_segment_start_valid == 0U)
    {
        square_start_segment(now_ms);
        return;
    }

    if (velocity_odometry_get_position(&s_odometry, &x_m, &y_m) !=
        VELOCITY_ODOMETRY_OK)
    {
        return;
    }

    square_get_body_direction(&vx_sign, &vy_sign);
    direction_x = vx_sign * cosf(s_target_yaw_deg * SQUARE_ODOMETRY_TEST_DEG_TO_RAD) -
                  vy_sign * sinf(s_target_yaw_deg * SQUARE_ODOMETRY_TEST_DEG_TO_RAD);
    direction_y = vx_sign * sinf(s_target_yaw_deg * SQUARE_ODOMETRY_TEST_DEG_TO_RAD) +
                  vy_sign * cosf(s_target_yaw_deg * SQUARE_ODOMETRY_TEST_DEG_TO_RAD);
    normal_x = -direction_y;
    normal_y = direction_x;
    /* 用世界坐标位移在当前边方向上的投影作为该边的已行驶距离，
       不受横向误差影响，也不依赖固定运行时间。 */
    progress_m = (x_m - s_segment_start_x) * direction_x +
                 (y_m - s_segment_start_y) * direction_y;
    cross_track_error_m = (x_m - s_segment_start_x) * normal_x +
                          (y_m - s_segment_start_y) * normal_y;
    remaining_m = square_get_segment_target_m() - progress_m;
    s_progress_m = progress_m;
    s_remaining_m = remaining_m;
    s_cross_track_error_m = cross_track_error_m;

    yaw_rad = yaw_data->yaw_deg * SQUARE_ODOMETRY_TEST_DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    if (mecanum_chassis_get_encoder_velocity(s_chassis,
                                             &encoder_velocity) == MECANUM_OK)
    {
        actual_world_vx_mps = encoder_velocity.vx_mps * cos_yaw -
                              encoder_velocity.vy_mps * sin_yaw;
        actual_world_vy_mps = encoder_velocity.vx_mps * sin_yaw +
                              encoder_velocity.vy_mps * cos_yaw;
        s_along_track_speed_mps = actual_world_vx_mps * direction_x +
                                  actual_world_vy_mps * direction_y;
    }

    if (remaining_m <= SQUARE_ODOMETRY_TEST_DISTANCE_TOL_M)
    {
        (void)mecanum_chassis_set_velocity(s_chassis, 0.0f, 0.0f, 0.0f);
        square_set_state(SQUARE_STATE_CORNER_STOP, now_ms);
        return;
    }

    elapsed_ms = now_ms - s_state_start_ms;
    if (elapsed_ms > SQUARE_ODOMETRY_TEST_SEGMENT_TIMEOUT_MS)
    {
        square_set_error(SQUARE_FAULT_SEGMENT_TIMEOUT, now_ms);
        square_send_text("SQUARE ERROR SEGMENT_TIMEOUT\r\n");
        return;
    }
    ramp_speed_mps = SQUARE_ODOMETRY_TEST_ACCEL_MPS2 *
                     (float)elapsed_ms * 0.001f;
    speed_mps = sqrtf(2.0f * SQUARE_ODOMETRY_TEST_DECEL_MPS2 * remaining_m);
    if (speed_mps > SQUARE_ODOMETRY_TEST_MAX_SPEED_MPS)
    {
        speed_mps = SQUARE_ODOMETRY_TEST_MAX_SPEED_MPS;
    }
    if (speed_mps > ramp_speed_mps)
    {
        speed_mps = ramp_speed_mps;
    }
    if (speed_mps < SQUARE_ODOMETRY_TEST_MIN_SPEED_MPS)
    {
        speed_mps = SQUARE_ODOMETRY_TEST_MIN_SPEED_MPS;
    }

    /* 横向误差为正表示偏到路线左侧；积分项消除持续性的侧向漂移。 */
    s_cross_track_integral_m_s = square_clamp(
        s_cross_track_integral_m_s + cross_track_error_m * control_dt_s,
        -SQUARE_ODOMETRY_TEST_CROSS_INTEGRAL_LIMIT_M_S,
        SQUARE_ODOMETRY_TEST_CROSS_INTEGRAL_LIMIT_M_S);
    cross_speed_limit_mps = speed_mps *
                            SQUARE_ODOMETRY_TEST_CROSS_SPEED_RATIO;
    if (cross_speed_limit_mps > SQUARE_ODOMETRY_TEST_MAX_CROSS_SPEED_MPS)
    {
        cross_speed_limit_mps = SQUARE_ODOMETRY_TEST_MAX_CROSS_SPEED_MPS;
    }
    cross_correction_mps = square_clamp(
        -SQUARE_ODOMETRY_TEST_CROSS_TRACK_KP * cross_track_error_m -
        SQUARE_ODOMETRY_TEST_CROSS_TRACK_KI * s_cross_track_integral_m_s,
        -cross_speed_limit_mps,
        cross_speed_limit_mps);
    s_cross_correction_mps = cross_correction_mps;

    velocity_world_x_mps = direction_x * speed_mps +
                           normal_x * cross_correction_mps;
    velocity_world_y_mps = direction_y * speed_mps +
                           normal_y * cross_correction_mps;
    /* 路线在世界坐标中定义，按当前实测航向转换为车体速度指令。 */
    velocity_body_x_mps = velocity_world_x_mps * cos_yaw +
                          velocity_world_y_mps * sin_yaw;
    velocity_body_y_mps = -velocity_world_x_mps * sin_yaw +
                           velocity_world_y_mps * cos_yaw;

    if (heading_hold_controller_update(&s_heading_hold,
                                       yaw_data->yaw_deg,
                                       &correction_wz_rad_s) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        square_set_error(SQUARE_FAULT_HEADING_CONTROLLER, now_ms);
        return;
    }

    if (mecanum_chassis_set_velocity(s_chassis,
                                     velocity_body_x_mps,
                                     velocity_body_y_mps,
                                     correction_wz_rad_s) == MECANUM_MOTOR_ERROR)
    {
        square_set_error(SQUARE_FAULT_MOTOR_COMMAND, now_ms);
    }
}

static void square_output(uint32_t now_ms)
{
    float x_m = 0.0f;
    float y_m = 0.0f;
    float target_yaw = 0.0f;
    float error_deg = 0.0f;
    float correction = 0.0f;
    hwt101ct_data_t yaw_data;
    uint32_t motor_age_ms[MECANUM_WHEEL_COUNT];
    uint8_t index;
    int length;
    char text[448];

    if ((uint32_t)(now_ms - s_last_output_ms) <
        SQUARE_ODOMETRY_TEST_OUTPUT_PERIOD_MS)
    {
        return;
    }
    s_last_output_ms = now_ms;
    memset(&yaw_data, 0, sizeof(yaw_data));
    (void)hwt101ct_get_data(s_hwt101ct_sensor, &yaw_data);
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_age_ms[index] = square_age_ms(
            now_ms,
            s_chassis->config.motor[index]->speed_update_ms);
    }
    (void)velocity_odometry_get_position(&s_odometry, &x_m, &y_m);
    (void)heading_hold_controller_get_status(&s_heading_hold,
                                              &target_yaw,
                                              &error_deg,
                                              &correction);
    length = snprintf(text,
                      sizeof(text),
                      "SQUARE=%s FAULT=%s SEG=%u X=%.3f Y=%.3f YAW=%.2f "
                      "TARGET=%.2f ERR=%+.2f WZ=%+.3f "
                      "DIST=%.3f PROG=%.3f REM=%.3f CROSS=%+.3f "
                      "VALONG=%+.3f VCROSS=%+.3f "
                      "MAGE=%lu,%lu,%lu,%lu "
                      "YAGE=%lums YF=%lu RX=%lu CK=%lu UE=%lu RR=%lu\r\n",
                      square_state_name(),
                      square_fault_name(),
                      (unsigned int)s_segment,
                      (double)x_m,
                      (double)y_m,
                      (double)yaw_data.yaw_deg,
                      (double)target_yaw,
                      (double)error_deg,
                      (double)correction,
                      (double)s_total_distance_m,
                      (double)s_progress_m,
                      (double)s_remaining_m,
                      (double)s_cross_track_error_m,
                      (double)s_along_track_speed_mps,
                      (double)s_cross_correction_mps,
                      (unsigned long)motor_age_ms[0],
                      (unsigned long)motor_age_ms[1],
                      (unsigned long)motor_age_ms[2],
                      (unsigned long)motor_age_ms[3],
                      (unsigned long)square_age_ms(now_ms,
                                                   yaw_data.yaw_update_ms),
                      (unsigned long)s_hwt101ct_sensor->yaw_frame_count,
                      (unsigned long)s_hwt101ct_sensor->rx_byte_count,
                      (unsigned long)s_hwt101ct_sensor->checksum_error_count,
                      (unsigned long)s_hwt101ct_sensor->uart_error_count,
                      (unsigned long)s_hwt101ct_sensor->rx_restart_count);
    if ((length > 0) && ((size_t)length < sizeof(text)))
    {
        square_send_text(text);
    }
}

square_odometry_test_result_t square_odometry_test_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *hwt101ct_sensor)
{
    uint32_t now_ms;

    if ((chassis == NULL) || (hwt101ct_sensor == NULL))
    {
        return SQUARE_ODOMETRY_TEST_INVALID_PARAM;
    }

    s_chassis = chassis;
    s_hwt101ct_sensor = hwt101ct_sensor;
    (void)velocity_odometry_init(&s_odometry);
    (void)heading_hold_controller_init(&s_heading_hold);
    now_ms = HAL_GetTick();
    s_segment = 0U;
    s_segment_start_valid = 0U;
    s_target_yaw_deg = 0.0f;
    s_last_control_ms = now_ms;
    s_last_output_ms = now_ms;
    s_last_encoder_ok_ms = now_ms;
    s_stop_stable_start_ms = 0U;
    s_stop_stable_active = 0U;
    s_fault = SQUARE_FAULT_NONE;
    s_progress_m = 0.0f;
    s_remaining_m = SQUARE_ODOMETRY_TEST_SIDE_M;
    s_cross_track_error_m = 0.0f;
    s_cross_track_integral_m_s = 0.0f;
    s_along_track_speed_mps = 0.0f;
    s_cross_correction_mps = 0.0f;
    s_total_distance_m = 0.0f;
    s_last_distance_x_m = 0.0f;
    s_last_distance_y_m = 0.0f;
    s_distance_position_valid = 0U;
    s_initialized = 1U;
    square_set_state(SQUARE_STATE_WAIT_YAW, now_ms);
    square_send_text("SQUARE_ODOMETRY_READY, KEEP ROBOT STILL\r\n");
    return SQUARE_ODOMETRY_TEST_OK;
}

void square_odometry_test_task(void)
{
    uint32_t now_ms;
    hwt101ct_data_t yaw_data;
    float correction_wz_rad_s = 0.0f;

    if ((s_initialized == 0U) || (s_chassis == NULL) ||
        (s_hwt101ct_sensor == NULL))
    {
        return;
    }

    now_ms = HAL_GetTick();
    hwt101ct_receive_watchdog(s_hwt101ct_sensor,
                              now_ms,
                              SQUARE_ODOMETRY_TEST_YAW_RESTART_MS);
    if (square_update_odometry(now_ms) == 0U)
    {
        if (s_state == SQUARE_STATE_MOVE)
        {
            square_set_error(s_fault, now_ms);
        }
        square_output(now_ms);
        return;
    }

    /* 反馈读取会阻塞数毫秒，状态机继续使用最新系统时间。 */
    now_ms = HAL_GetTick();

    if (s_state == SQUARE_STATE_WAIT_YAW)
    {
        if (square_get_yaw(now_ms, &yaw_data) != 0U)
        {
            square_stop();
            s_fault = SQUARE_FAULT_NONE;
            s_state = SQUARE_STATE_SETTLING;
            s_state_start_ms = now_ms;
            square_send_text("SQUARE SETTLING\r\n");
        }
    }
    else if (s_state == SQUARE_STATE_SETTLING)
    {
        if ((uint32_t)(now_ms - s_state_start_ms) >=
            SQUARE_ODOMETRY_TEST_SETTLE_MS)
        {
            if (hwt101ct_set_current_yaw_zero(s_hwt101ct_sensor) !=
                HWT101CT_OK)
            {
                square_set_error(SQUARE_FAULT_YAW_TIMEOUT, now_ms);
            }
            else
            {
                velocity_odometry_reset(&s_odometry);
                s_total_distance_m = 0.0f;
                s_last_distance_x_m = 0.0f;
                s_last_distance_y_m = 0.0f;
                s_distance_position_valid = 0U;
                heading_hold_controller_enable(&s_heading_hold);
                (void)heading_hold_controller_update(&s_heading_hold,
                                                     0.0f,
                                                     &correction_wz_rad_s);
                s_target_yaw_deg = 0.0f;
                s_segment = 0U;
                s_fault = SQUARE_FAULT_NONE;
                square_set_state(SQUARE_STATE_MOVE, now_ms);
                square_send_text("SQUARE START TARGET_YAW=0\r\n");
            }
        }
    }
    else if (s_state == SQUARE_STATE_MOVE)
    {
        if (square_get_yaw(now_ms, &yaw_data) == 0U)
        {
            square_set_error(SQUARE_FAULT_YAW_TIMEOUT, now_ms);
        }
        else
        {
            square_apply_motion(now_ms, &yaw_data);
        }
    }
    else if (s_state == SQUARE_STATE_CORNER_STOP)
    {
        uint32_t stop_elapsed_ms;
        uint8_t stop_stable;

        if ((uint32_t)(now_ms - s_last_control_ms) <
            SQUARE_ODOMETRY_TEST_CONTROL_PERIOD_MS)
        {
            square_output(now_ms);
            return;
        }
        s_last_control_ms = now_ms;
        /* 每条边到位后只等待车体停稳，避免低速二次校航向造成过调。 */
        (void)mecanum_chassis_set_velocity(s_chassis,
                                           0.0f,
                                           0.0f,
                                           0.0f);

        stop_elapsed_ms = square_age_ms(now_ms, s_state_start_ms);
        stop_stable = square_corner_is_stable(now_ms);
        if ((s_state == SQUARE_STATE_CORNER_STOP) &&
            (stop_elapsed_ms >= SQUARE_ODOMETRY_TEST_STOP_TIMEOUT_MS))
        {
            square_set_error(SQUARE_FAULT_STOP_TIMEOUT, now_ms);
            square_send_text("SQUARE ERROR STOP_TIMEOUT\r\n");
        }

        if ((s_state == SQUARE_STATE_CORNER_STOP) &&
            (stop_elapsed_ms >= SQUARE_ODOMETRY_TEST_CORNER_STOP_MS) &&
            (stop_stable != 0U))
        {
            s_segment++;
            if (s_segment >= 4U)
            {
                square_set_state(SQUARE_STATE_FINISHED, now_ms);
                square_send_text("SQUARE FINISHED\r\n");
            }
            else
            {
                square_set_state(SQUARE_STATE_MOVE, now_ms);
            }
        }
    }

    square_output(now_ms);
}
