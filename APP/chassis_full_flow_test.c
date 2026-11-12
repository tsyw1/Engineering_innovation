#include "chassis_full_flow_test.h"
#include "chassis_odometry_motion_config.h"

#include <math.h>
#include <string.h>

/* 与 square_odometry_test 保持一致的底盘闭环参数。 */
#define FLOW_TEST_CONTROL_PERIOD_MS     CHASSIS_ODOMETRY_CONTROL_PERIOD_MS
#define FLOW_TEST_SETTLE_MS            CHASSIS_ODOMETRY_SETTLE_MS
#define FLOW_TEST_CORNER_STOP_MS       CHASSIS_ODOMETRY_CORNER_STOP_MS
#define FLOW_TEST_STOP_STABLE_MS       CHASSIS_ODOMETRY_STOP_STABLE_MS
#define FLOW_TEST_STOP_TIMEOUT_MS      CHASSIS_ODOMETRY_STOP_TIMEOUT_MS
#define FLOW_TEST_SEGMENT_TIMEOUT_MS   CHASSIS_ODOMETRY_SEGMENT_TIMEOUT_MS
#define FLOW_TEST_YAW_RESTART_MS       CHASSIS_ODOMETRY_YAW_RESTART_MS
#define FLOW_TEST_MAX_YAW_AGE_MS       CHASSIS_ODOMETRY_MAX_YAW_AGE_MS
#define FLOW_TEST_MAX_ENCODER_AGE_MS   CHASSIS_ODOMETRY_MAX_ENCODER_AGE_MS
#define FLOW_TEST_DISTANCE_TOL_M       CHASSIS_ODOMETRY_DISTANCE_TOL_M
#define FLOW_TEST_MAX_SPEED_MPS        CHASSIS_ODOMETRY_MAX_SPEED_MPS
#define FLOW_TEST_ACCEL_MPS2           CHASSIS_ODOMETRY_ACCEL_MPS2
#define FLOW_TEST_DECEL_MPS2           CHASSIS_ODOMETRY_DECEL_MPS2
#define FLOW_TEST_MIN_SPEED_MPS        CHASSIS_ODOMETRY_MIN_SPEED_MPS
#define FLOW_TEST_CROSS_TRACK_KP       CHASSIS_ODOMETRY_CROSS_TRACK_KP
#define FLOW_TEST_CROSS_TRACK_KI       CHASSIS_ODOMETRY_CROSS_TRACK_KI
#define FLOW_TEST_CROSS_INTEGRAL_LIMIT_M_S \
                                         CHASSIS_ODOMETRY_CROSS_INTEGRAL_LIMIT_M_S
#define FLOW_TEST_MAX_CROSS_SPEED_MPS  CHASSIS_ODOMETRY_MAX_CROSS_SPEED_MPS
#define FLOW_TEST_CROSS_SPEED_RATIO    CHASSIS_ODOMETRY_CROSS_SPEED_RATIO
#define FLOW_TEST_LEFT_COMP_M          CHASSIS_ODOMETRY_LEFT_COMP_M
#define FLOW_TEST_STOP_SPEED_MPS       CHASSIS_ODOMETRY_STOP_SPEED_MPS
#define FLOW_TEST_STOP_WZ_RAD_S         0.05f
#define FLOW_TEST_ROTATE_TOL_DEG        0.50f
#define FLOW_TEST_ROTATE_SPEED_SCALE    2.00f
#define FLOW_TEST_DEG_TO_RAD     0.01745329251994329577f
#define FLOW_TEST_TRAY_SETTLE_MS       300U
#define FLOW_TEST_PLATFORM_TRAY_SETTLE_MS 100U
#define FLOW_TEST_SENSOR_RESYNC_TIMEOUT_MS 3000U

typedef enum
{
    FLOW_TEST_ACTION_LINEAR = 0,
    FLOW_TEST_ACTION_ROTATE,
    FLOW_TEST_ACTION_DWELL,
    FLOW_TEST_ACTION_START_LATERAL,
    FLOW_TEST_ACTION_RETURN_LONGITUDINAL,
    FLOW_TEST_ACTION_ARM
} flow_test_action_type_t;

typedef enum
{
    FLOW_TEST_ARM_PLATFORM_PICK = 0,
    FLOW_TEST_ARM_RING_PLACE,
    FLOW_TEST_ARM_RING_PICK,
    FLOW_TEST_ARM_PLATFORM_PICK_ALIGNED,
    FLOW_TEST_ARM_PLATFORM_PREPOSITION,
    FLOW_TEST_ARM_TRAVEL_SAFE
} flow_test_arm_operation_t;

typedef enum
{
    FLOW_TEST_MODE_NONE = 0,
    FLOW_TEST_MODE_LINEAR,
    FLOW_TEST_MODE_ROTATE,
    FLOW_TEST_MODE_DWELL,
    FLOW_TEST_MODE_STOPPING
} flow_test_mode_t;

typedef struct
{
    flow_test_action_type_t type;
    uint8_t task_number;
    int16_t x_mm;
    int16_t y_mm;
    int16_t yaw_deg;
    uint16_t dwell_ms;
    flow_test_arm_operation_t arm_operation;
    uint8_t target_ring;
    uint8_t tray_position;
    const char *event;
} flow_test_action_t;

#define FLOW_LINEAR(task, x, y, name) \
    { FLOW_TEST_ACTION_LINEAR, (task), (x), (y), 0, 0U, \
      FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U, (name) }
#define FLOW_ROTATE(task, yaw, name) \
    { FLOW_TEST_ACTION_ROTATE, (task), 0, 0, (yaw), 0U, \
      FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U, (name) }
#define FLOW_DWELL(task, name) \
    { FLOW_TEST_ACTION_DWELL, (task), 0, 0, 0, \
      CHASSIS_FULL_FLOW_TEST_POINT_DWELL_MS, \
      FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U, (name) }
#define FLOW_START_LATERAL(task, name) \
    { FLOW_TEST_ACTION_START_LATERAL, (task), 0, 0, 0, 0U, \
      FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U, (name) }
#define FLOW_RETURN_LONGITUDINAL(task, name) \
    { FLOW_TEST_ACTION_RETURN_LONGITUDINAL, (task), 0, 0, 0, 0U, \
      FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U, (name) }
#define FLOW_ARM(task, operation, ring, tray, name) \
    { FLOW_TEST_ACTION_ARM, (task), 0, 0, 0, 0U, (operation), \
      (ring), (tray), (name) }

/* 固定路线：机械臂动作使用已标定的固定位置，不调用视觉或二维码。 */
static const flow_test_action_t s_flow_actions[] =
{
    FLOW_LINEAR(1U,  100,    0, "task1_forward_100mm"),
    FLOW_START_LATERAL(1U,       "task1_start_zone_lateral"),
    FLOW_ARM(1U, FLOW_TEST_ARM_PLATFORM_PREPOSITION, 0U, 0U,
             "task1_preposition_platform_z"),
    FLOW_LINEAR(1U,  150,    0, "task1_forward_qr_150mm"),
    FLOW_DWELL(1U,                 "task1_fixed_qr_point"),

    FLOW_LINEAR(2U,  800,    0, "task2_forward_800mm"),
    FLOW_LINEAR(2U,    0, -920, "task2_right_920mm"),
    FLOW_ARM(2U, FLOW_TEST_ARM_PLATFORM_PICK_ALIGNED, 0U, 0U,
             "task2_platform_pick_item_1"),
    FLOW_ARM(2U, FLOW_TEST_ARM_PLATFORM_PICK, 0U, 1U,
             "task2_platform_pick_item_2"),
    FLOW_ARM(2U, FLOW_TEST_ARM_PLATFORM_PICK, 0U, 2U,
             "task2_platform_pick_item_3"),

    FLOW_LINEAR(3U,    0,  100, "task3_left_100mm"),
    FLOW_ROTATE(3U,  180,         "task3_rotate_ccw_180deg"),
    FLOW_LINEAR(3U,    0, -1670, "task3_right_1670mm"),
    FLOW_ARM(3U, FLOW_TEST_ARM_RING_PLACE, 1U, 0U,
             "task3_place_item_to_ring_1"),
    FLOW_ARM(3U, FLOW_TEST_ARM_RING_PLACE, 2U, 1U,
             "task3_place_item_to_ring_2"),
    FLOW_ARM(3U, FLOW_TEST_ARM_RING_PLACE, 3U, 2U,
             "task3_place_item_to_ring_3"),

    FLOW_ARM(4U, FLOW_TEST_ARM_RING_PICK, 1U, 0U,
             "task4_pick_item_from_ring_1"),
    FLOW_ARM(4U, FLOW_TEST_ARM_RING_PICK, 2U, 1U,
             "task4_pick_item_from_ring_2"),
    FLOW_ARM(4U, FLOW_TEST_ARM_RING_PICK, 3U, 2U,
             "task4_pick_item_from_ring_3"),

    FLOW_LINEAR(5U, -860,    0, "task5_backward_from_ring_2_860mm"),
    FLOW_ROTATE(5U,  -90,         "task5_rotate_cw_90deg"),
    FLOW_LINEAR(5U, -850,    0, "task5_backward_to_area_850mm"),
    FLOW_ARM(5U, FLOW_TEST_ARM_RING_PLACE, 1U, 0U,
             "task5_place_item_to_temporary_ring_1"),
    FLOW_ARM(5U, FLOW_TEST_ARM_RING_PLACE, 2U, 1U,
             "task5_place_item_to_temporary_ring_2"),
    FLOW_ARM(5U, FLOW_TEST_ARM_RING_PLACE, 3U, 2U,
             "task5_place_item_to_temporary_ring_3"),
    FLOW_ARM(5U, FLOW_TEST_ARM_TRAVEL_SAFE, 0U, 0U,
             "task5_restore_arm_travel_safe"),

    FLOW_LINEAR(6U, -900,    0, "task6_backward_from_ring_2_900mm"),
    FLOW_ROTATE(6U,  -90,         "task6_rotate_cw_90deg"),
    FLOW_LINEAR(6U, -850,    0, "task6_backward_to_area_850mm"),
    FLOW_ARM(6U, FLOW_TEST_ARM_PLATFORM_PICK, 0U, 0U,
             "task6_platform_pick_item_1"),
    FLOW_ARM(6U, FLOW_TEST_ARM_PLATFORM_PICK, 0U, 1U,
             "task6_platform_pick_item_2"),
    FLOW_ARM(6U, FLOW_TEST_ARM_PLATFORM_PICK, 0U, 2U,
             "task6_platform_pick_item_3"),

    FLOW_LINEAR(7U,    0,  100, "task7_left_100mm"),
    FLOW_ROTATE(7U,  180,         "task7_rotate_ccw_180deg"),
    FLOW_LINEAR(7U,    0, -1670, "task7_right_1670mm"),
    FLOW_ARM(7U, FLOW_TEST_ARM_RING_PLACE, 1U, 0U,
             "task7_place_item_to_ring_1"),
    FLOW_ARM(7U, FLOW_TEST_ARM_RING_PLACE, 2U, 1U,
             "task7_place_item_to_ring_2"),
    FLOW_ARM(7U, FLOW_TEST_ARM_RING_PLACE, 3U, 2U,
             "task7_place_item_to_ring_3"),

    FLOW_ARM(8U, FLOW_TEST_ARM_RING_PICK, 1U, 0U,
             "task8_pick_item_from_ring_1"),
    FLOW_ARM(8U, FLOW_TEST_ARM_RING_PICK, 2U, 1U,
             "task8_pick_item_from_ring_2"),
    FLOW_ARM(8U, FLOW_TEST_ARM_RING_PICK, 3U, 2U,
             "task8_pick_item_from_ring_3"),

    FLOW_LINEAR(9U, -860,    0, "task9_backward_from_ring_2_860mm"),
    FLOW_ROTATE(9U,  -90,         "task9_rotate_cw_90deg"),
    FLOW_LINEAR(9U, -860,    0, "task9_backward_to_area_860mm"),
    FLOW_DWELL(9U,                 "task9_center_hold_for_stack_ring_1"),
    FLOW_DWELL(9U,                 "task9_center_hold_for_stack_ring_2"),
    FLOW_DWELL(9U,                 "task9_center_hold_for_stack_ring_3"),

    FLOW_LINEAR(10U,    0, 1900, "task10_left_1900mm"),
    FLOW_RETURN_LONGITUDINAL(10U,  "task10_return_selected_zone")
};

static uint16_t flow_action_count(void)
{
    return (uint16_t)(sizeof(s_flow_actions) / sizeof(s_flow_actions[0]));
}

static uint32_t flow_age_ms(uint32_t now_ms, uint32_t update_ms)
{
    int32_t elapsed_ms = (int32_t)(now_ms - update_ms);

    return (elapsed_ms < 0) ? 0U : (uint32_t)elapsed_ms;
}

static float flow_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float flow_clamp(float value, float min_value, float max_value)
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

static float flow_wrap_yaw_deg(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg <= -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static void flow_status(chassis_full_flow_test_t *test,
                        uint8_t task_number,
                        const char *event)
{
    uint16_t step_number;

    if ((test == NULL) || (test->write_status == NULL))
    {
        return;
    }
    step_number = (test->action_index < flow_action_count()) ?
                  (uint16_t)(test->action_index + 1U) :
                  flow_action_count();
    test->write_status(task_number, step_number, event);
}

static chassis_full_flow_test_result_t flow_fail(
    chassis_full_flow_test_t *test,
    chassis_full_flow_test_result_t result,
    const char *event)
{
    uint8_t task_number = 0U;

    if (test != NULL)
    {
        if (test->action_index < flow_action_count())
        {
            task_number = s_flow_actions[test->action_index].task_number;
        }
        heading_hold_controller_disable(&test->heading_hold);
        (void)mecanum_chassis_stop(test->chassis);
        (void)mecanum_chassis_enable(test->chassis, false);
        test->action_active = 0U;
        test->action_mode = FLOW_TEST_MODE_NONE;
        test->state = CHASSIS_FULL_FLOW_TEST_FAILED;
        flow_status(test, task_number, event);
    }
    return result;
}

static uint8_t flow_get_yaw(chassis_full_flow_test_t *test,
                            uint32_t now_ms,
                            hwt101ct_data_t *yaw_data)
{
    if ((test == NULL) || (yaw_data == NULL) ||
        (hwt101ct_get_data(test->yaw_sensor, yaw_data) != HWT101CT_OK) ||
        (yaw_data->yaw_valid == 0U))
    {
        return 0U;
    }
    return (flow_age_ms(now_ms, yaw_data->yaw_update_ms) <=
            FLOW_TEST_MAX_YAW_AGE_MS) ? 1U : 0U;
}

static chassis_full_flow_test_result_t flow_update_odometry(
    chassis_full_flow_test_t *test,
    uint32_t now_ms)
{
    mecanum_encoder_velocity_t velocity;
    hwt101ct_data_t yaw_data;
    velocity_odometry_result_t odometry_result;

    (void)mecanum_chassis_feedback_task(test->chassis);
    if (mecanum_chassis_update_encoder_velocity(test->chassis) == MECANUM_OK)
    {
        test->last_encoder_ok_tick = HAL_GetTick();
    }
    now_ms = HAL_GetTick();

    if (flow_age_ms(now_ms, test->last_encoder_ok_tick) >
        FLOW_TEST_MAX_ENCODER_AGE_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }
    if (mecanum_chassis_get_encoder_velocity(test->chassis, &velocity) !=
        MECANUM_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }
    if (flow_get_yaw(test, now_ms, &yaw_data) == 0U)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }

    odometry_result = velocity_odometry_update(&test->odometry,
                                               velocity.vx_mps,
                                               velocity.vy_mps,
                                               yaw_data.yaw_deg,
                                               now_ms);
    if (odometry_result == VELOCITY_ODOMETRY_TIME_ERROR)
    {
        return CHASSIS_FULL_FLOW_TEST_OK;
    }
    if (odometry_result != VELOCITY_ODOMETRY_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_ODOMETRY_ERROR;
    }
    return CHASSIS_FULL_FLOW_TEST_OK;
}

static float flow_profile_speed(uint32_t elapsed_ms,
                                float remaining_m,
                                float max_speed_mps)
{
    float ramp_speed_mps;
    float speed_mps;

    ramp_speed_mps = FLOW_TEST_ACCEL_MPS2 * (float)elapsed_ms * 0.001f;
    speed_mps = sqrtf(2.0f * FLOW_TEST_DECEL_MPS2 * remaining_m);
    if (speed_mps > max_speed_mps)
    {
        speed_mps = max_speed_mps;
    }
    if (speed_mps > ramp_speed_mps)
    {
        speed_mps = ramp_speed_mps;
    }
    if (speed_mps < FLOW_TEST_MIN_SPEED_MPS)
    {
        speed_mps = FLOW_TEST_MIN_SPEED_MPS;
    }
    return speed_mps;
}

static uint8_t flow_stop_is_stable(chassis_full_flow_test_t *test,
                                   uint32_t now_ms)
{
    mecanum_encoder_velocity_t velocity;

    if (mecanum_chassis_get_encoder_velocity(test->chassis, &velocity) !=
        MECANUM_OK)
    {
        test->stop_stable_active = 0U;
        return 0U;
    }
    if ((flow_abs(velocity.vx_mps) > FLOW_TEST_STOP_SPEED_MPS) ||
        (flow_abs(velocity.vy_mps) > FLOW_TEST_STOP_SPEED_MPS) ||
        (flow_abs(velocity.wz_rad_s) > FLOW_TEST_STOP_WZ_RAD_S))
    {
        test->stop_stable_active = 0U;
        return 0U;
    }
    if (test->stop_stable_active == 0U)
    {
        test->stop_stable_start_tick = now_ms;
        test->stop_stable_active = 1U;
        return 0U;
    }
    return (flow_age_ms(now_ms, test->stop_stable_start_tick) >=
            FLOW_TEST_STOP_STABLE_MS) ? 1U : 0U;
}

static void flow_enter_stop(chassis_full_flow_test_t *test,
                            uint32_t now_ms)
{
    test->action_mode = FLOW_TEST_MODE_STOPPING;
    test->stop_start_tick = now_ms;
    test->stop_stable_start_tick = 0U;
    test->stop_stable_active = 0U;
    (void)mecanum_chassis_set_velocity(test->chassis, 0.0f, 0.0f, 0.0f);
}

static void flow_finish_action(chassis_full_flow_test_t *test)
{
    test->action_active = 0U;
    test->action_mode = FLOW_TEST_MODE_NONE;
    test->action_index++;
}

static chassis_full_flow_test_result_t flow_run_arm_action(
    chassis_full_flow_test_t *test,
    const flow_test_action_t *action)
{
    arm_controller_result_t arm_result;
    uint32_t tray_settle_ms;

    if ((test == NULL) || (action == NULL) || (test->arm == NULL) ||
        (test->tray == NULL))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }

    if (action->arm_operation == FLOW_TEST_ARM_PLATFORM_PREPOSITION)
    {
        arm_result = arm_controller_preposition_platform_z(test->arm);
        goto arm_action_done;
    }
    if (action->arm_operation == FLOW_TEST_ARM_TRAVEL_SAFE)
    {
        arm_result = arm_controller_prepare_travel_safe(test->arm);
        goto arm_action_done;
    }

    if (action->tray_position >= MATERIAL_TRAY_POSITION_COUNT)
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }

    tray_settle_ms = (action->arm_operation == FLOW_TEST_ARM_PLATFORM_PICK) ||
                     (action->arm_operation ==
                      FLOW_TEST_ARM_PLATFORM_PICK_ALIGNED) ?
                     FLOW_TEST_PLATFORM_TRAY_SETTLE_MS :
                     FLOW_TEST_TRAY_SETTLE_MS;

    /* 每次机械臂动作都先选定物料盘工位，避免沿用上一次位置。 */
    if (material_tray_servo_set_position(
            test->tray,
            (material_tray_position_t)action->tray_position) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_ARM_ERROR;
    }
    HAL_Delay(tray_settle_ms);

    switch (action->arm_operation)
    {
        case FLOW_TEST_ARM_PLATFORM_PICK:
            arm_result = arm_controller_prepare_platform_pick(test->arm);
            if (arm_result == ARM_CONTROLLER_OK)
            {
                arm_result = arm_controller_pick_and_place(test->arm);
            }
            break;

        case FLOW_TEST_ARM_PLATFORM_PICK_ALIGNED:
            arm_result = arm_controller_prepare_platform_pick_aligned(
                test->arm);
            if (arm_result == ARM_CONTROLLER_OK)
            {
                arm_result = arm_controller_pick_and_place(test->arm);
            }
            break;

        case FLOW_TEST_ARM_RING_PLACE:
            if ((action->target_ring < 1U) || (action->target_ring > 3U) ||
                (arm_controller_set_target_ring(test->arm,
                                                 action->target_ring) !=
                 ARM_CONTROLLER_OK))
            {
                return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
            }
            arm_result = arm_controller_place_ring_item_from_tray(test->arm);
            break;

        case FLOW_TEST_ARM_RING_PICK:
            if ((action->target_ring < 1U) || (action->target_ring > 3U) ||
                (arm_controller_set_target_ring(test->arm,
                                                 action->target_ring) !=
                 ARM_CONTROLLER_OK))
            {
                return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
            }
            arm_result = arm_controller_prepare_ground_pick(test->arm);
            if (arm_result == ARM_CONTROLLER_OK)
            {
                arm_result = arm_controller_ground_pick_to_tray(test->arm);
            }
            break;

        default:
            return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }

arm_action_done:
    return (arm_result == ARM_CONTROLLER_OK) ?
           CHASSIS_FULL_FLOW_TEST_OK : CHASSIS_FULL_FLOW_TEST_ARM_ERROR;
}

static chassis_full_flow_test_result_t flow_start_linear(
    chassis_full_flow_test_t *test,
    int16_t x_mm,
    int16_t y_mm,
    uint32_t now_ms)
{
    float x_m;
    float y_m;

    if (((x_mm == 0) && (y_mm == 0)) ||
        ((x_mm != 0) && (y_mm != 0)) ||
        (velocity_odometry_get_position(&test->odometry, &x_m, &y_m) !=
         VELOCITY_ODOMETRY_OK))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    if (heading_hold_controller_set_target(&test->heading_hold,
                                           test->target_yaw_deg) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }

    test->direction_body_x = (x_mm > 0) ? 1.0f :
                             ((x_mm < 0) ? -1.0f : 0.0f);
    test->direction_body_y = (y_mm > 0) ? 1.0f :
                             ((y_mm < 0) ? -1.0f : 0.0f);
    test->segment_target_m = (float)((x_mm != 0) ?
                             ((x_mm < 0) ? -x_mm : x_mm) :
                             ((y_mm < 0) ? -y_mm : y_mm)) * 0.001f;
    /* 麦轮向左横移存在少量实际位移损失，只补偿左移这一段。 */
    if (test->direction_body_y > 0.0f)
    {
        test->segment_target_m += FLOW_TEST_LEFT_COMP_M;
    }
    test->segment_start_x_m = x_m;
    test->segment_start_y_m = y_m;
    test->progress_m = 0.0f;
    test->remaining_m = test->segment_target_m;
    test->cross_track_error_m = 0.0f;
    test->cross_track_integral_m_s = 0.0f;
    test->cross_correction_mps = 0.0f;
    test->action_start_tick = now_ms;
    test->last_control_tick = now_ms - FLOW_TEST_CONTROL_PERIOD_MS;
    test->action_active = 1U;
    test->action_mode = FLOW_TEST_MODE_LINEAR;
    return CHASSIS_FULL_FLOW_TEST_OK;
}

static chassis_full_flow_test_result_t flow_start_rotation(
    chassis_full_flow_test_t *test,
    int16_t yaw_deg,
    uint32_t now_ms)
{
    if (yaw_deg == 0)
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    test->target_yaw_deg = flow_wrap_yaw_deg(test->target_yaw_deg +
                                             (float)yaw_deg);
    test->rotation_direction = (yaw_deg > 0) ? 1 : -1;
    if (heading_hold_controller_set_target(&test->heading_hold,
                                           test->target_yaw_deg) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }
    test->action_start_tick = now_ms;
    test->last_control_tick = now_ms - FLOW_TEST_CONTROL_PERIOD_MS;
    test->action_active = 1U;
    test->action_mode = FLOW_TEST_MODE_ROTATE;
    return CHASSIS_FULL_FLOW_TEST_OK;
}

static chassis_full_flow_test_result_t flow_start_action(
    chassis_full_flow_test_t *test,
    uint32_t now_ms)
{
    const flow_test_action_t *action;
    chassis_full_flow_test_result_t action_result;
    int16_t x_mm;
    int16_t y_mm;

    if (test->action_index >= flow_action_count())
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    action = &s_flow_actions[test->action_index];
    flow_status(test, action->task_number, action->event);

    switch (action->type)
    {
        case FLOW_TEST_ACTION_LINEAR:
            return flow_start_linear(test,
                                     action->x_mm,
                                     action->y_mm,
                                     now_ms);

        case FLOW_TEST_ACTION_ROTATE:
            return flow_start_rotation(test, action->yaw_deg, now_ms);

        case FLOW_TEST_ACTION_DWELL:
            test->action_start_tick = now_ms;
            test->action_duration_ms = action->dwell_ms;
            test->action_active = 1U;
            test->action_mode = FLOW_TEST_MODE_DWELL;
            return CHASSIS_FULL_FLOW_TEST_OK;

        case FLOW_TEST_ACTION_START_LATERAL:
            y_mm = (test->start_zone == START_ZONE_2) ? -1050 : 1050;
            return flow_start_linear(test, 0, y_mm, now_ms);

        case FLOW_TEST_ACTION_RETURN_LONGITUDINAL:
            x_mm = (test->start_zone == START_ZONE_2) ? 980 : -1050;
            return flow_start_linear(test, x_mm, 0, now_ms);

        case FLOW_TEST_ACTION_ARM:
            action_result = flow_run_arm_action(test, action);
            if (action_result != CHASSIS_FULL_FLOW_TEST_OK)
            {
                return action_result;
            }
            /* 机械臂动作是同步阻塞调用，期间底盘反馈可能超过新鲜度阈值。
             * 重启航向串口接收，并给 CAN/航向传感器一个重新同步窗口。 */
            if (hwt101ct_restart_receive(test->yaw_sensor) != HWT101CT_OK)
            {
                return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
            }
            test->sensor_resync_active = 1U;
            test->sensor_resync_start_tick = HAL_GetTick();
            /* 机械臂接口同步等待到位，动作完成后立即进入下一步。 */
            flow_finish_action(test);
            return CHASSIS_FULL_FLOW_TEST_OK;

        default:
            return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
}

static chassis_full_flow_test_result_t flow_apply_linear(
    chassis_full_flow_test_t *test,
    const hwt101ct_data_t *yaw_data,
    uint32_t now_ms)
{
    float x_m;
    float y_m;
    float target_yaw_rad;
    float current_yaw_rad;
    float target_cos;
    float target_sin;
    float current_cos;
    float current_sin;
    float direction_x;
    float direction_y;
    float normal_x;
    float normal_y;
    float cross_speed_limit_mps;
    float cross_correction_mps;
    float speed_mps;
    float velocity_world_x_mps;
    float velocity_world_y_mps;
    float velocity_body_x_mps;
    float velocity_body_y_mps;
    float correction_wz_rad_s;
    float control_dt_s;
    mecanum_result_t chassis_result;
    uint32_t elapsed_ms;

    if ((uint32_t)(now_ms - test->last_control_tick) <
        FLOW_TEST_CONTROL_PERIOD_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    control_dt_s = (float)(now_ms - test->last_control_tick) * 0.001f;
    test->last_control_tick = now_ms;
    if (velocity_odometry_get_position(&test->odometry, &x_m, &y_m) !=
        VELOCITY_ODOMETRY_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_ODOMETRY_ERROR;
    }

    target_yaw_rad = test->target_yaw_deg * FLOW_TEST_DEG_TO_RAD;
    target_cos = cosf(target_yaw_rad);
    target_sin = sinf(target_yaw_rad);
    direction_x = test->direction_body_x * target_cos -
                  test->direction_body_y * target_sin;
    direction_y = test->direction_body_x * target_sin +
                  test->direction_body_y * target_cos;
    normal_x = -direction_y;
    normal_y = direction_x;
    test->progress_m = (x_m - test->segment_start_x_m) * direction_x +
                       (y_m - test->segment_start_y_m) * direction_y;
    test->cross_track_error_m =
        (x_m - test->segment_start_x_m) * normal_x +
        (y_m - test->segment_start_y_m) * normal_y;
    test->remaining_m = test->segment_target_m - test->progress_m;

    if (test->remaining_m <= FLOW_TEST_DISTANCE_TOL_M)
    {
        flow_enter_stop(test, now_ms);
        return CHASSIS_FULL_FLOW_TEST_OK;
    }
    elapsed_ms = now_ms - test->action_start_tick;
    if (elapsed_ms > FLOW_TEST_SEGMENT_TIMEOUT_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_TIMEOUT;
    }

    speed_mps = flow_profile_speed(elapsed_ms,
                                   test->remaining_m,
                                   FLOW_TEST_MAX_SPEED_MPS);
    test->cross_track_integral_m_s = flow_clamp(
        test->cross_track_integral_m_s +
        test->cross_track_error_m * control_dt_s,
        -FLOW_TEST_CROSS_INTEGRAL_LIMIT_M_S,
        FLOW_TEST_CROSS_INTEGRAL_LIMIT_M_S);
    cross_speed_limit_mps = speed_mps * FLOW_TEST_CROSS_SPEED_RATIO;
    if (cross_speed_limit_mps > FLOW_TEST_MAX_CROSS_SPEED_MPS)
    {
        cross_speed_limit_mps = FLOW_TEST_MAX_CROSS_SPEED_MPS;
    }
    cross_correction_mps = flow_clamp(
        -FLOW_TEST_CROSS_TRACK_KP * test->cross_track_error_m -
        FLOW_TEST_CROSS_TRACK_KI * test->cross_track_integral_m_s,
        -cross_speed_limit_mps,
        cross_speed_limit_mps);
    test->cross_correction_mps = cross_correction_mps;
    velocity_world_x_mps = direction_x * speed_mps +
                           normal_x * cross_correction_mps;
    velocity_world_y_mps = direction_y * speed_mps +
                           normal_y * cross_correction_mps;

    current_yaw_rad = yaw_data->yaw_deg * FLOW_TEST_DEG_TO_RAD;
    current_cos = cosf(current_yaw_rad);
    current_sin = sinf(current_yaw_rad);
    velocity_body_x_mps = velocity_world_x_mps * current_cos +
                          velocity_world_y_mps * current_sin;
    velocity_body_y_mps = -velocity_world_x_mps * current_sin +
                           velocity_world_y_mps * current_cos;
    if (heading_hold_controller_update(&test->heading_hold,
                                       yaw_data->yaw_deg,
                                       &correction_wz_rad_s) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }
    chassis_result = mecanum_chassis_set_velocity(test->chassis,
                                                  velocity_body_x_mps,
                                                  velocity_body_y_mps,
                                                  correction_wz_rad_s);
    return (chassis_result == MECANUM_MOTOR_ERROR) ?
           CHASSIS_FULL_FLOW_TEST_CHASSIS_ERROR :
           CHASSIS_FULL_FLOW_TEST_OK;
}

static chassis_full_flow_test_result_t flow_apply_rotation(
    chassis_full_flow_test_t *test,
    const hwt101ct_data_t *yaw_data,
    uint32_t now_ms)
{
    float error_deg;
    float remaining_wheel_m;
    float wheel_speed_mps;
    float max_wz_rad_s;
    float correction_wz_rad_s;
    float geometry_factor;
    uint32_t elapsed_ms;
    mecanum_result_t chassis_result;

    if ((uint32_t)(now_ms - test->last_control_tick) <
        FLOW_TEST_CONTROL_PERIOD_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    test->last_control_tick = now_ms;
    error_deg = flow_wrap_yaw_deg(test->target_yaw_deg -
                                  yaw_data->yaw_deg);
    if (flow_abs(error_deg) <= FLOW_TEST_ROTATE_TOL_DEG)
    {
        flow_enter_stop(test, now_ms);
        return CHASSIS_FULL_FLOW_TEST_OK;
    }
    elapsed_ms = now_ms - test->action_start_tick;
    if (elapsed_ms > FLOW_TEST_SEGMENT_TIMEOUT_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_TIMEOUT;
    }

    geometry_factor = test->chassis->config.half_length_m +
                      test->chassis->config.half_width_m;
    if (geometry_factor <= 0.0f)
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    remaining_wheel_m = geometry_factor * flow_abs(error_deg) *
                        FLOW_TEST_DEG_TO_RAD;
    wheel_speed_mps = flow_profile_speed(
        elapsed_ms,
        remaining_wheel_m,
        FLOW_TEST_MAX_SPEED_MPS * FLOW_TEST_ROTATE_SPEED_SCALE);
    max_wz_rad_s = wheel_speed_mps / geometry_factor;
    if (heading_hold_controller_update(&test->heading_hold,
                                       yaw_data->yaw_deg,
                                       &correction_wz_rad_s) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR;
    }
    correction_wz_rad_s *= FLOW_TEST_ROTATE_SPEED_SCALE;
    /* 180 度目标在 +/-180 边界存在最短路径歧义，保持路线指定方向。 */
    if ((flow_abs(error_deg) > 90.0f) &&
        (((correction_wz_rad_s > 0.0f) ? 1 : -1) !=
         test->rotation_direction))
    {
        correction_wz_rad_s = flow_abs(correction_wz_rad_s) *
                              (float)test->rotation_direction;
    }
    correction_wz_rad_s = flow_clamp(correction_wz_rad_s,
                                     -max_wz_rad_s,
                                     max_wz_rad_s);
    chassis_result = mecanum_chassis_set_velocity(test->chassis,
                                                  0.0f,
                                                  0.0f,
                                                  correction_wz_rad_s);
    return (chassis_result == MECANUM_MOTOR_ERROR) ?
           CHASSIS_FULL_FLOW_TEST_CHASSIS_ERROR :
           CHASSIS_FULL_FLOW_TEST_OK;
}

static chassis_full_flow_test_result_t flow_apply_stop(
    chassis_full_flow_test_t *test,
    uint32_t now_ms)
{
    uint32_t stop_elapsed_ms;
    mecanum_result_t chassis_result;

    if ((uint32_t)(now_ms - test->last_control_tick) <
        FLOW_TEST_CONTROL_PERIOD_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    test->last_control_tick = now_ms;
    /* 到位后只等待车体停稳，不再执行停车后的二次航向校正。 */
    chassis_result = mecanum_chassis_set_velocity(test->chassis,
                                                  0.0f,
                                                  0.0f,
                                                  0.0f);
    if (chassis_result == MECANUM_MOTOR_ERROR)
    {
        return CHASSIS_FULL_FLOW_TEST_CHASSIS_ERROR;
    }

    stop_elapsed_ms = flow_age_ms(now_ms, test->stop_start_tick);
    if (stop_elapsed_ms >= FLOW_TEST_STOP_TIMEOUT_MS)
    {
        return CHASSIS_FULL_FLOW_TEST_TIMEOUT;
    }
    if ((stop_elapsed_ms >= FLOW_TEST_CORNER_STOP_MS) &&
        (flow_stop_is_stable(test, now_ms) != 0U))
    {
        flow_finish_action(test);
        return CHASSIS_FULL_FLOW_TEST_OK;
    }
    return CHASSIS_FULL_FLOW_TEST_WAITING;
}

chassis_full_flow_test_result_t chassis_full_flow_test_init(
    chassis_full_flow_test_t *test,
    mecanum_chassis_t *chassis,
    hwt101ct_t *yaw_sensor,
    arm_controller_t *arm,
    material_tray_servo_t *tray,
    chassis_full_flow_test_status_fn write_status)
{
    uint32_t now_ms;

    if ((test == NULL) || (chassis == NULL) || (yaw_sensor == NULL) ||
        (arm == NULL) || (tray == NULL))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    memset(test, 0, sizeof(*test));
    test->chassis = chassis;
    test->yaw_sensor = yaw_sensor;
    test->arm = arm;
    test->tray = tray;
    test->write_status = write_status;
    if ((velocity_odometry_init(&test->odometry) != VELOCITY_ODOMETRY_OK) ||
        (heading_hold_controller_init(&test->heading_hold) !=
         HEADING_HOLD_CONTROLLER_OK))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    now_ms = HAL_GetTick();
    test->last_control_tick = now_ms;
    test->last_encoder_ok_tick = now_ms;
    test->start_zone = START_ZONE_NONE;
    test->state = CHASSIS_FULL_FLOW_TEST_WAIT_START;
    flow_status(test, 0U, "competition_chassis_flow_ready");
    return CHASSIS_FULL_FLOW_TEST_OK;
}

chassis_full_flow_test_result_t chassis_full_flow_test_request_start(
    chassis_full_flow_test_t *test,
    start_zone_t start_zone)
{
    if ((test == NULL) ||
        ((start_zone != START_ZONE_1) && (start_zone != START_ZONE_2)) ||
        (test->state != CHASSIS_FULL_FLOW_TEST_WAIT_START))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    test->start_zone = start_zone;
    test->action_index = 0U;
    test->action_active = 0U;
    test->action_mode = FLOW_TEST_MODE_NONE;
    test->state = CHASSIS_FULL_FLOW_TEST_WAIT_YAW;
    flow_status(test,
                1U,
                (start_zone == START_ZONE_2) ?
                "screen_start_zone2_wait_yaw" :
                "screen_start_zone1_wait_yaw");
    return CHASSIS_FULL_FLOW_TEST_OK;
}

chassis_full_flow_test_result_t chassis_full_flow_test_task(
    chassis_full_flow_test_t *test)
{
    chassis_full_flow_test_result_t update_result;
    chassis_full_flow_test_result_t action_result;
    hwt101ct_data_t yaw_data;
    uint32_t now_ms;
    float correction_wz_rad_s;

    if ((test == NULL) || (test->chassis == NULL) ||
        (test->yaw_sensor == NULL))
    {
        return CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }
    if (test->state == CHASSIS_FULL_FLOW_TEST_DONE)
    {
        return CHASSIS_FULL_FLOW_TEST_DONE_RESULT;
    }
    if (test->state == CHASSIS_FULL_FLOW_TEST_FAILED)
    {
        return CHASSIS_FULL_FLOW_TEST_CHASSIS_ERROR;
    }

    now_ms = HAL_GetTick();
    hwt101ct_receive_watchdog(test->yaw_sensor,
                              now_ms,
                              FLOW_TEST_YAW_RESTART_MS);
    update_result = flow_update_odometry(test, now_ms);
    now_ms = HAL_GetTick();

    if (test->state == CHASSIS_FULL_FLOW_TEST_WAIT_START)
    {
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    if (test->state == CHASSIS_FULL_FLOW_TEST_WAIT_YAW)
    {
        if ((update_result == CHASSIS_FULL_FLOW_TEST_OK) &&
            (flow_get_yaw(test, now_ms, &yaw_data) != 0U))
        {
            (void)mecanum_chassis_stop(test->chassis);
            test->state = CHASSIS_FULL_FLOW_TEST_SETTLING;
            test->state_start_tick = now_ms;
            flow_status(test, 1U, "chassis_flow_settling");
        }
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    if (test->state == CHASSIS_FULL_FLOW_TEST_SETTLING)
    {
        if (update_result != CHASSIS_FULL_FLOW_TEST_OK)
        {
            return CHASSIS_FULL_FLOW_TEST_WAITING;
        }
        if (flow_age_ms(now_ms, test->state_start_tick) < FLOW_TEST_SETTLE_MS)
        {
            return CHASSIS_FULL_FLOW_TEST_WAITING;
        }
        if (hwt101ct_set_current_yaw_zero(test->yaw_sensor) != HWT101CT_OK)
        {
            return flow_fail(test,
                             CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR,
                             "chassis_flow_yaw_zero_failed");
        }
        velocity_odometry_reset(&test->odometry);
        heading_hold_controller_enable(&test->heading_hold);
        if ((heading_hold_controller_update(&test->heading_hold,
                                            0.0f,
                                            &correction_wz_rad_s) !=
             HEADING_HOLD_CONTROLLER_OK) ||
            (heading_hold_controller_set_target(&test->heading_hold, 0.0f) !=
             HEADING_HOLD_CONTROLLER_OK))
        {
            return flow_fail(test,
                             CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR,
                             "chassis_flow_heading_init_failed");
        }
        test->target_yaw_deg = 0.0f;
        test->state = CHASSIS_FULL_FLOW_TEST_RUNNING;
        flow_status(test, 1U, "chassis_flow_closed_loop_start");
        return CHASSIS_FULL_FLOW_TEST_OK;
    }

    if (update_result == CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR)
    {
        if (test->sensor_resync_active != 0U)
        {
            if (flow_age_ms(now_ms, test->sensor_resync_start_tick) <
                FLOW_TEST_SENSOR_RESYNC_TIMEOUT_MS)
            {
                return CHASSIS_FULL_FLOW_TEST_WAITING;
            }
        }
        return flow_fail(test,
                         update_result,
                         "chassis_flow_sensor_timeout");
    }
    if (test->sensor_resync_active != 0U)
    {
        test->sensor_resync_active = 0U;
    }
    if (update_result == CHASSIS_FULL_FLOW_TEST_ODOMETRY_ERROR)
    {
        return flow_fail(test,
                         update_result,
                         "chassis_flow_odometry_failed");
    }
    if (update_result == CHASSIS_FULL_FLOW_TEST_WAITING)
    {
        return update_result;
    }
    if (flow_get_yaw(test, now_ms, &yaw_data) == 0U)
    {
        return flow_fail(test,
                         CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR,
                         "chassis_flow_yaw_timeout");
    }

    if (test->action_active == 0U)
    {
        if (test->action_index >= flow_action_count())
        {
            heading_hold_controller_disable(&test->heading_hold);
            (void)mecanum_chassis_stop(test->chassis);
            (void)mecanum_chassis_enable(test->chassis, false);
            test->state = CHASSIS_FULL_FLOW_TEST_DONE;
            flow_status(test, 10U, "competition_chassis_flow_done");
            return CHASSIS_FULL_FLOW_TEST_DONE_RESULT;
        }
        action_result = flow_start_action(test, now_ms);
        if (action_result != CHASSIS_FULL_FLOW_TEST_OK)
        {
            return flow_fail(test,
                             action_result,
                             "chassis_flow_action_start_failed");
        }
        return CHASSIS_FULL_FLOW_TEST_OK;
    }

    if (test->action_mode == FLOW_TEST_MODE_DWELL)
    {
        if (flow_age_ms(now_ms, test->action_start_tick) >=
            test->action_duration_ms)
        {
            flow_finish_action(test);
            return CHASSIS_FULL_FLOW_TEST_OK;
        }
        return CHASSIS_FULL_FLOW_TEST_WAITING;
    }
    if (test->action_mode == FLOW_TEST_MODE_LINEAR)
    {
        action_result = flow_apply_linear(test, &yaw_data, now_ms);
    }
    else if (test->action_mode == FLOW_TEST_MODE_ROTATE)
    {
        action_result = flow_apply_rotation(test, &yaw_data, now_ms);
    }
    else if (test->action_mode == FLOW_TEST_MODE_STOPPING)
    {
        action_result = flow_apply_stop(test, now_ms);
    }
    else
    {
        action_result = CHASSIS_FULL_FLOW_TEST_INVALID_PARAM;
    }

    if ((action_result != CHASSIS_FULL_FLOW_TEST_OK) &&
        (action_result != CHASSIS_FULL_FLOW_TEST_WAITING))
    {
        return flow_fail(test,
                         action_result,
                         (action_result == CHASSIS_FULL_FLOW_TEST_TIMEOUT) ?
                         "chassis_flow_motion_timeout" :
                         "chassis_flow_motion_failed");
    }
    return action_result;
}

chassis_full_flow_test_state_t chassis_full_flow_test_get_state(
    const chassis_full_flow_test_t *test)
{
    return (test == NULL) ? CHASSIS_FULL_FLOW_TEST_FAILED : test->state;
}

uint16_t chassis_full_flow_test_get_step(
    const chassis_full_flow_test_t *test)
{
    return (test == NULL) ? 0U : test->action_index;
}

uint16_t chassis_full_flow_test_get_step_count(void)
{
    return flow_action_count();
}
