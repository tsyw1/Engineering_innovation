#include "chassis_velocity_test.h"

#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHASSIS_VELOCITY_TEST_SAMPLE_PERIOD_MS   10U
#define CHASSIS_VELOCITY_TEST_COMMAND_PERIOD_MS  10U
#define CHASSIS_VELOCITY_TEST_OUTPUT_PERIOD_MS 100U
#define CHASSIS_VELOCITY_TEST_ACCEL_PERIOD_MS  1000U
#define CHASSIS_VELOCITY_TEST_CRUISE_PERIOD_MS 1000U
#define CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS \
    (2U * CHASSIS_VELOCITY_TEST_ACCEL_PERIOD_MS + \
     CHASSIS_VELOCITY_TEST_CRUISE_PERIOD_MS)
#define CHASSIS_VELOCITY_TEST_STOP_PERIOD_MS   1000U
#define CHASSIS_VELOCITY_TEST_ZERO_SETTLE_MS   1000U
#define CHASSIS_VELOCITY_TEST_TX_TIMEOUT_MS     100U
#define CHASSIS_VELOCITY_TEST_LINEAR_SPEED_MPS  0.40f
#define CHASSIS_VELOCITY_TEST_LINEAR_ACCEL_MPS2 0.40f
#define CHASSIS_VELOCITY_TEST_STOP_SPEED_MPS     0.02f
#define CHASSIS_VELOCITY_TEST_STOP_ACCEL_MPS2    0.25f
#define CHASSIS_VELOCITY_TEST_STOP_CONFIRM_MS   250U
#define CHASSIS_VELOCITY_TEST_HEADING_MIN_SPEED_MPS 0.05f
#define CHASSIS_VELOCITY_TEST_MAX_HWT_YAW_AGE_MS 200U

typedef enum
{
    CHASSIS_VELOCITY_TEST_WAIT_CALIBRATION = 0,
    CHASSIS_VELOCITY_TEST_ZERO_SETTLE,
    CHASSIS_VELOCITY_TEST_FORWARD,
    CHASSIS_VELOCITY_TEST_STOP_AFTER_FORWARD,
    CHASSIS_VELOCITY_TEST_BACKWARD,
    CHASSIS_VELOCITY_TEST_STOP_AFTER_BACKWARD,
    CHASSIS_VELOCITY_TEST_LEFT,
    CHASSIS_VELOCITY_TEST_STOP_AFTER_LEFT,
    CHASSIS_VELOCITY_TEST_RIGHT,
    CHASSIS_VELOCITY_TEST_STOP_AFTER_RIGHT,
    CHASSIS_VELOCITY_TEST_FINISHED
} chassis_velocity_test_state_t;

static mecanum_chassis_t *s_chassis;
static const hwt101ct_t *s_hwt101ct_sensor;
static const jy901s_t *s_jy901s_sensor;
static jy901s_accel_velocity_t *s_accel_velocity;
static velocity_complementary_filter_t s_velocity_filter;
static velocity_odometry_t s_velocity_odometry;
static heading_hold_controller_t s_heading_hold;
static chassis_velocity_test_state_t s_state;
static uint32_t s_state_start_ms;
static uint32_t s_last_sample_ms;
static uint32_t s_last_command_ms;
static uint32_t s_last_output_ms;
static uint32_t s_stop_candidate_start_ms;
static uint8_t s_stop_zeroed;
static uint8_t s_stop_candidate_active;
static jy901s_result_t s_last_jy901s_result;
static jy901s_data_t s_last_jy901s_data;

static void chassis_velocity_test_send_text(const char *text)
{
    if (text != NULL)
    {
        (void)HAL_UART_Transmit(&huart6,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                CHASSIS_VELOCITY_TEST_TX_TIMEOUT_MS);
    }
}

static const char *chassis_velocity_test_state_name(void)
{
    switch (s_state)
    {
    case CHASSIS_VELOCITY_TEST_FORWARD:
        return "FORWARD";
    case CHASSIS_VELOCITY_TEST_BACKWARD:
        return "BACKWARD";
    case CHASSIS_VELOCITY_TEST_LEFT:
        return "LEFT";
    case CHASSIS_VELOCITY_TEST_RIGHT:
        return "RIGHT";
    case CHASSIS_VELOCITY_TEST_FINISHED:
        return "FINISHED";
    case CHASSIS_VELOCITY_TEST_WAIT_CALIBRATION:
        return "CALIBRATING";
    case CHASSIS_VELOCITY_TEST_ZERO_SETTLE:
        return "ZEROING";
    default:
        return "STOP";
    }
}

static void chassis_velocity_test_set_state(
    chassis_velocity_test_state_t state,
    uint32_t now_ms)
{
    s_state = state;
    s_state_start_ms = now_ms;
    if (state != CHASSIS_VELOCITY_TEST_FINISHED)
    {
        s_stop_zeroed = 0U;
        s_stop_candidate_start_ms = 0U;
        s_stop_candidate_active = 0U;
    }

    if (s_chassis == NULL)
    {
        return;
    }

    switch (state)
    {
    case CHASSIS_VELOCITY_TEST_FORWARD:
    case CHASSIS_VELOCITY_TEST_BACKWARD:
    case CHASSIS_VELOCITY_TEST_LEFT:
    case CHASSIS_VELOCITY_TEST_RIGHT:
        heading_hold_controller_enable(&s_heading_hold);
        s_last_command_ms = now_ms - CHASSIS_VELOCITY_TEST_COMMAND_PERIOD_MS;
        break;
    case CHASSIS_VELOCITY_TEST_STOP_AFTER_FORWARD:
    case CHASSIS_VELOCITY_TEST_STOP_AFTER_BACKWARD:
    case CHASSIS_VELOCITY_TEST_STOP_AFTER_LEFT:
    case CHASSIS_VELOCITY_TEST_STOP_AFTER_RIGHT:
    case CHASSIS_VELOCITY_TEST_FINISHED:
        heading_hold_controller_disable(&s_heading_hold);
        (void)mecanum_chassis_stop(s_chassis);
        break;
    case CHASSIS_VELOCITY_TEST_WAIT_CALIBRATION:
        heading_hold_controller_disable(&s_heading_hold);
    default:
        break;
    }
}

static float chassis_velocity_test_get_profile_speed(uint32_t elapsed_ms)
{
    float speed_mps;

    if (elapsed_ms < CHASSIS_VELOCITY_TEST_ACCEL_PERIOD_MS)
    {
        speed_mps = CHASSIS_VELOCITY_TEST_LINEAR_ACCEL_MPS2 *
                    (float)elapsed_ms * 0.001f;
    }
    else if (elapsed_ms <
             (CHASSIS_VELOCITY_TEST_ACCEL_PERIOD_MS +
              CHASSIS_VELOCITY_TEST_CRUISE_PERIOD_MS))
    {
        speed_mps = CHASSIS_VELOCITY_TEST_LINEAR_SPEED_MPS;
    }
    else if (elapsed_ms < CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS)
    {
        speed_mps = CHASSIS_VELOCITY_TEST_LINEAR_ACCEL_MPS2 *
                    (float)(CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS -
                            elapsed_ms) *
                    0.001f;
    }
    else
    {
        speed_mps = 0.0f;
    }

    if (speed_mps > CHASSIS_VELOCITY_TEST_LINEAR_SPEED_MPS)
    {
        speed_mps = CHASSIS_VELOCITY_TEST_LINEAR_SPEED_MPS;
    }

    return speed_mps;
}

static void chassis_velocity_test_apply_motion_profile(uint32_t now_ms)
{
    float speed_mps;
    float correction_wz_rad_s = 0.0f;
    hwt101ct_data_t hwt101ct_data;

    if ((s_state != CHASSIS_VELOCITY_TEST_FORWARD) &&
        (s_state != CHASSIS_VELOCITY_TEST_BACKWARD) &&
        (s_state != CHASSIS_VELOCITY_TEST_LEFT) &&
        (s_state != CHASSIS_VELOCITY_TEST_RIGHT))
    {
        return;
    }

    if ((uint32_t)(now_ms - s_last_command_ms) <
        CHASSIS_VELOCITY_TEST_COMMAND_PERIOD_MS)
    {
        return;
    }
    s_last_command_ms = now_ms;

    speed_mps = chassis_velocity_test_get_profile_speed(
        now_ms - s_state_start_ms);

    if ((speed_mps >= CHASSIS_VELOCITY_TEST_HEADING_MIN_SPEED_MPS) &&
        (hwt101ct_get_data(s_hwt101ct_sensor, &hwt101ct_data) == HWT101CT_OK) &&
        (hwt101ct_data.yaw_valid != 0U) &&
        ((uint32_t)(now_ms - hwt101ct_data.yaw_update_ms) <=
         CHASSIS_VELOCITY_TEST_MAX_HWT_YAW_AGE_MS))
    {
        (void)heading_hold_controller_update(&s_heading_hold,
                                              hwt101ct_data.yaw_deg,
                                              &correction_wz_rad_s);
    }

    switch (s_state)
    {
    case CHASSIS_VELOCITY_TEST_FORWARD:
        (void)mecanum_chassis_set_velocity(s_chassis,
                                           speed_mps,
                                           0.0f,
                                           correction_wz_rad_s);
        break;
    case CHASSIS_VELOCITY_TEST_BACKWARD:
        (void)mecanum_chassis_set_velocity(s_chassis,
                                           -speed_mps,
                                           0.0f,
                                           correction_wz_rad_s);
        break;
    case CHASSIS_VELOCITY_TEST_LEFT:
        (void)mecanum_chassis_set_velocity(s_chassis,
                                           0.0f,
                                           speed_mps,
                                           correction_wz_rad_s);
        break;
    case CHASSIS_VELOCITY_TEST_RIGHT:
        (void)mecanum_chassis_set_velocity(s_chassis,
                                           0.0f,
                                           -speed_mps,
                                           correction_wz_rad_s);
        break;
    default:
        return;
    }
}

static void chassis_velocity_test_update_state(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (s_state == CHASSIS_VELOCITY_TEST_WAIT_CALIBRATION)
    {
        if (jy901s_accel_velocity_is_calibrated(s_accel_velocity) != 0U)
        {
            chassis_velocity_test_set_state(
                CHASSIS_VELOCITY_TEST_ZERO_SETTLE,
                now_ms);
            chassis_velocity_test_send_text(
                "VELOCITY_TEST_ZEROING, KEEP ROBOT STILL\r\n");
        }
        return;
    }

    if (s_state == CHASSIS_VELOCITY_TEST_ZERO_SETTLE)
    {
        if ((now_ms - s_state_start_ms) >=
            CHASSIS_VELOCITY_TEST_ZERO_SETTLE_MS)
        {
            jy901s_accel_velocity_zero_velocity_update(s_accel_velocity);
            chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_FORWARD,
                                            now_ms);
            chassis_velocity_test_send_text(
                "VELOCITY_TEST_START FORWARD\r\n");
        }
        return;
    }

    if (s_state == CHASSIS_VELOCITY_TEST_FINISHED)
    {
        return;
    }

    elapsed_ms = now_ms - s_state_start_ms;
    if ((s_state == CHASSIS_VELOCITY_TEST_FORWARD) &&
        (elapsed_ms >= CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS))
    {
        chassis_velocity_test_set_state(
            CHASSIS_VELOCITY_TEST_STOP_AFTER_FORWARD, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_STOP_AFTER_FORWARD) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_STOP_PERIOD_MS))
    {
        chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_BACKWARD,
                                        now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_BACKWARD) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS))
    {
        chassis_velocity_test_set_state(
            CHASSIS_VELOCITY_TEST_STOP_AFTER_BACKWARD, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_STOP_AFTER_BACKWARD) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_STOP_PERIOD_MS))
    {
        chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_LEFT, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_LEFT) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS))
    {
        chassis_velocity_test_set_state(
            CHASSIS_VELOCITY_TEST_STOP_AFTER_LEFT, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_STOP_AFTER_LEFT) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_STOP_PERIOD_MS))
    {
        chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_RIGHT, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_RIGHT) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_MOTION_PERIOD_MS))
    {
        chassis_velocity_test_set_state(
            CHASSIS_VELOCITY_TEST_STOP_AFTER_RIGHT, now_ms);
    }
    else if ((s_state == CHASSIS_VELOCITY_TEST_STOP_AFTER_RIGHT) &&
             (elapsed_ms >= CHASSIS_VELOCITY_TEST_STOP_PERIOD_MS))
    {
        chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_FINISHED,
                                        now_ms);
        chassis_velocity_test_send_text(
            "VELOCITY_TEST_FINISHED, ROBOT STOPPED\r\n");
    }
}

static void chassis_velocity_test_apply_stop_zeroing(uint32_t now_ms)
{
    mecanum_encoder_velocity_t velocity;
    float acceleration_x_mps2;
    float acceleration_y_mps2;

    if ((s_state != CHASSIS_VELOCITY_TEST_STOP_AFTER_FORWARD) &&
        (s_state != CHASSIS_VELOCITY_TEST_STOP_AFTER_BACKWARD) &&
        (s_state != CHASSIS_VELOCITY_TEST_STOP_AFTER_LEFT) &&
        (s_state != CHASSIS_VELOCITY_TEST_STOP_AFTER_RIGHT) &&
        (s_state != CHASSIS_VELOCITY_TEST_FINISHED))
    {
        s_stop_candidate_active = 0U;
        return;
    }

    if (s_stop_zeroed != 0U)
    {
        jy901s_accel_velocity_zero_velocity_update(s_accel_velocity);
        velocity_complementary_filter_zero(&s_velocity_filter, now_ms);
        return;
    }

    if (mecanum_chassis_get_encoder_velocity(s_chassis, &velocity) !=
        MECANUM_OK)
    {
        s_stop_candidate_active = 0U;
        return;
    }

    if (jy901s_accel_velocity_get_filtered_acceleration(
            s_accel_velocity,
            &acceleration_x_mps2,
            &acceleration_y_mps2) != JY901S_ACCEL_VELOCITY_OK)
    {
        s_stop_candidate_active = 0U;
        return;
    }

    if ((fabsf(velocity.vx_mps) <= CHASSIS_VELOCITY_TEST_STOP_SPEED_MPS) &&
        (fabsf(velocity.vy_mps) <= CHASSIS_VELOCITY_TEST_STOP_SPEED_MPS) &&
        (fabsf(acceleration_x_mps2) <=
         CHASSIS_VELOCITY_TEST_STOP_ACCEL_MPS2) &&
        (fabsf(acceleration_y_mps2) <=
         CHASSIS_VELOCITY_TEST_STOP_ACCEL_MPS2))
    {
        if (s_stop_candidate_active == 0U)
        {
            s_stop_candidate_start_ms = now_ms;
            s_stop_candidate_active = 1U;
            return;
        }

        if ((uint32_t)(now_ms - s_stop_candidate_start_ms) <
            CHASSIS_VELOCITY_TEST_STOP_CONFIRM_MS)
        {
            return;
        }

        jy901s_accel_velocity_zero_velocity_update(s_accel_velocity);
        velocity_complementary_filter_zero(&s_velocity_filter, now_ms);
        s_stop_zeroed = 1U;
        s_stop_candidate_active = 0U;
        chassis_velocity_test_send_text(
            "VELOCITY_TEST_ZERO_VELOCITY_CORRECTED\r\n");
        return;
    }

    /* 任一静止条件被破坏后重新累计连续静止时间。 */
    s_stop_candidate_start_ms = 0U;
    s_stop_candidate_active = 0U;
}

static void chassis_velocity_test_update_fused_velocity(uint32_t now_ms)
{
    mecanum_encoder_velocity_t encoder_velocity;
    float acceleration_x_mps2;
    float acceleration_y_mps2;

    if (mecanum_chassis_get_encoder_velocity(s_chassis,
                                             &encoder_velocity) != MECANUM_OK)
    {
        return;
    }

    if (jy901s_accel_velocity_get_filtered_acceleration(s_accel_velocity,
                                                        &acceleration_x_mps2,
                                                        &acceleration_y_mps2) !=
        JY901S_ACCEL_VELOCITY_OK)
    {
        return;
    }

    (void)velocity_complementary_filter_update(
        &s_velocity_filter,
        encoder_velocity.vx_mps,
        encoder_velocity.vy_mps,
        acceleration_x_mps2,
        acceleration_y_mps2,
        now_ms);
}

static void chassis_velocity_test_update_odometry(uint32_t now_ms)
{
    hwt101ct_data_t hwt101ct_data;
    float fused_vx_mps;
    float fused_vy_mps;

    /* 里程计航向只使用 HWT101CT；JY901S 只参与平面线性加速度解算。 */
    if (hwt101ct_get_data(s_hwt101ct_sensor, &hwt101ct_data) != HWT101CT_OK)
    {
        return;
    }

    if ((hwt101ct_data.yaw_valid == 0U) ||
        ((uint32_t)(now_ms - hwt101ct_data.yaw_update_ms) >
         CHASSIS_VELOCITY_TEST_MAX_HWT_YAW_AGE_MS) ||
        (velocity_complementary_filter_get(&s_velocity_filter,
                                            &fused_vx_mps,
                                            &fused_vy_mps) !=
         VELOCITY_COMPLEMENTARY_FILTER_OK))
    {
        return;
    }

    (void)velocity_odometry_update(&s_velocity_odometry,
                                   fused_vx_mps,
                                   fused_vy_mps,
                                   hwt101ct_data.yaw_deg,
                                   now_ms);
}

static uint8_t chassis_velocity_test_update_sensors(uint32_t now_ms)
{
    jy901s_accel_velocity_result_t acceleration_result;

    if ((uint32_t)(now_ms - s_last_sample_ms) <
        CHASSIS_VELOCITY_TEST_SAMPLE_PERIOD_MS)
    {
        return 0U;
    }

    s_last_sample_ms = now_ms;
    s_last_jy901s_result = jy901s_read_motion(s_jy901s_sensor,
                                               &s_last_jy901s_data);
    if (s_last_jy901s_result == JY901S_OK)
    {
        acceleration_result = jy901s_accel_velocity_update(
            s_accel_velocity,
            &s_last_jy901s_data,
            now_ms);
        return (acceleration_result == JY901S_ACCEL_VELOCITY_OK) ? 1U : 0U;
    }

    return 0U;
}

static void chassis_velocity_test_output(uint32_t now_ms)
{
    hwt101ct_data_t hwt101ct_data;
    mecanum_encoder_velocity_t encoder_velocity;
    float accel_vx = 0.0f;
    float accel_vy = 0.0f;
    float fused_vx = 0.0f;
    float fused_vy = 0.0f;
    float odom_x = 0.0f;
    float odom_y = 0.0f;
    float heading_target_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float heading_correction_rad_s = 0.0f;
    mecanum_result_t encoder_result;
    jy901s_accel_velocity_result_t accel_result;
    hwt101ct_result_t hwt_result;
    int length;
    char text[640];

    if ((uint32_t)(now_ms - s_last_output_ms) <
        CHASSIS_VELOCITY_TEST_OUTPUT_PERIOD_MS)
    {
        return;
    }
    s_last_output_ms = now_ms;

    hwt_result = hwt101ct_get_data(s_hwt101ct_sensor, &hwt101ct_data);
    encoder_result = mecanum_chassis_get_encoder_velocity(
        s_chassis, &encoder_velocity);
    accel_result = jy901s_accel_velocity_get(s_accel_velocity,
                                             &accel_vx,
                                             &accel_vy);
    (void)velocity_complementary_filter_get(&s_velocity_filter,
                                            &fused_vx,
                                            &fused_vy);
    (void)velocity_odometry_get_position(&s_velocity_odometry,
                                         &odom_x,
                                         &odom_y);
    (void)heading_hold_controller_get_status(&s_heading_hold,
                                              &heading_target_deg,
                                              &heading_error_deg,
                                              &heading_correction_rad_s);

    if (hwt_result != HWT101CT_OK)
    {
        memset(&hwt101ct_data, 0, sizeof(hwt101ct_data));
    }
    if (encoder_result != MECANUM_OK)
    {
        memset(&encoder_velocity, 0, sizeof(encoder_velocity));
    }
    if (accel_result != JY901S_ACCEL_VELOCITY_OK)
    {
        accel_vx = 0.0f;
        accel_vy = 0.0f;
    }

    length = snprintf(text,
                      sizeof(text),
                      "TEST=%s YAW=%.2f "
                      "ROLL=%.2f PITCH=%.2f "
                      "ACC_X=%+.3f ACC_Y=%+.3f ACC_Z=%+.3f "
                      "BIAS_X=%+.3f BIAS_Y=%+.3f BIAS_Z=%+.3f "
                      "GRAV_X=%+.3f GRAV_Y=%+.3f GRAV_Z=%+.3f "
                      "LIN_SENSOR_X=%+.3f LIN_SENSOR_Y=%+.3f LIN_SENSOR_Z=%+.3f "
                      "LIN_ACC_X=%+.3f LIN_ACC_Y=%+.3f "
                      "V_ENCODER_X=%+.3f V_ENCODER_Y=%+.3f "
                      "V_ACC_X=%+.3f V_ACC_Y=%+.3f "
                      "V_FUSED_X=%+.3f V_FUSED_Y=%+.3f "
                       "ODOM_X=%+.3f ODOM_Y=%+.3f "
                       "YAW_TARGET=%+.2f YAW_ERR=%+.2f WZ_CORR=%+.3f "
                      "ENC_STATUS=%ld ACC_STATUS=%ld JY_STATUS=%ld\r\n",
                      chassis_velocity_test_state_name(),
                      (double)hwt101ct_data.yaw_deg,
                      (double)s_last_jy901s_data.angle_deg[0],
                      (double)s_last_jy901s_data.angle_deg[1],
                      (double)s_last_jy901s_data.acceleration_mps2[0],
                      (double)s_last_jy901s_data.acceleration_mps2[1],
                      (double)s_last_jy901s_data.acceleration_mps2[2],
                      (double)s_accel_velocity->acceleration_bias_sensor_mps2[0],
                      (double)s_accel_velocity->acceleration_bias_sensor_mps2[1],
                      (double)s_accel_velocity->acceleration_bias_sensor_mps2[2],
                      (double)s_accel_velocity->gravity_sensor_mps2[0],
                      (double)s_accel_velocity->gravity_sensor_mps2[1],
                      (double)s_accel_velocity->gravity_sensor_mps2[2],
                      (double)s_accel_velocity->acceleration_sensor_mps2[0],
                      (double)s_accel_velocity->acceleration_sensor_mps2[1],
                      (double)s_accel_velocity->acceleration_sensor_mps2[2],
                      (double)s_accel_velocity->acceleration_vehicle_mps2[0],
                      (double)s_accel_velocity->acceleration_vehicle_mps2[1],
                      (double)encoder_velocity.vx_mps,
                      (double)encoder_velocity.vy_mps,
                      (double)accel_vx,
                      (double)accel_vy,
                      (double)fused_vx,
                      (double)fused_vy,
                       (double)odom_x,
                       (double)odom_y,
                       (double)heading_target_deg,
                       (double)heading_error_deg,
                       (double)heading_correction_rad_s,
                      (long)encoder_result,
                      (long)accel_result,
                      (long)s_last_jy901s_result);
    if ((length > 0) && ((size_t)length < sizeof(text)))
    {
        chassis_velocity_test_send_text(text);
    }
}

chassis_velocity_test_result_t chassis_velocity_test_init(
    mecanum_chassis_t *chassis,
    const hwt101ct_t *hwt101ct_sensor,
    const jy901s_t *jy901s_sensor,
    jy901s_accel_velocity_t *accel_velocity)
{
    uint32_t now_ms;

    if ((chassis == NULL) || (hwt101ct_sensor == NULL) ||
        (jy901s_sensor == NULL) || (accel_velocity == NULL))
    {
        return CHASSIS_VELOCITY_TEST_INVALID_PARAM;
    }

    s_chassis = chassis;
    s_hwt101ct_sensor = hwt101ct_sensor;
    s_jy901s_sensor = jy901s_sensor;
    s_accel_velocity = accel_velocity;
    (void)velocity_complementary_filter_init(&s_velocity_filter);
    (void)velocity_odometry_init(&s_velocity_odometry);
    (void)heading_hold_controller_init(&s_heading_hold);
    now_ms = HAL_GetTick();
    s_last_sample_ms = now_ms;
    s_last_command_ms = now_ms;
    s_last_output_ms = now_ms;
    s_last_jy901s_result = JY901S_NOT_READY;
    memset(&s_last_jy901s_data, 0, sizeof(s_last_jy901s_data));
    chassis_velocity_test_set_state(CHASSIS_VELOCITY_TEST_WAIT_CALIBRATION,
                                    now_ms);
    chassis_velocity_test_send_text(
        "VELOCITY_TEST_READY, CALIBRATING JY901S, ROBOT STOPPED\r\n");
    return CHASSIS_VELOCITY_TEST_OK;
}

void chassis_velocity_test_task(void)
{
    uint32_t now_ms;
    uint8_t sensor_updated;

    if ((s_chassis == NULL) || (s_hwt101ct_sensor == NULL) ||
        (s_jy901s_sensor == NULL) || (s_accel_velocity == NULL))
    {
        return;
    }

    now_ms = HAL_GetTick();
    sensor_updated = chassis_velocity_test_update_sensors(now_ms);
    (void)mecanum_chassis_feedback_task(s_chassis);
    (void)mecanum_chassis_update_encoder_velocity(s_chassis);
    if (sensor_updated != 0U)
    {
        chassis_velocity_test_update_fused_velocity(now_ms);
    }
    chassis_velocity_test_apply_stop_zeroing(now_ms);
    if (sensor_updated != 0U)
    {
        /* 零速修正后再积分，避免把停车前的残余速度算入位置。 */
        chassis_velocity_test_update_odometry(now_ms);
    }
    chassis_velocity_test_update_state(now_ms);
    chassis_velocity_test_apply_motion_profile(now_ms);
    chassis_velocity_test_output(now_ms);
}
