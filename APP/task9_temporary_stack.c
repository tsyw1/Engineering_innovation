 #include "task9_temporary_stack.h"

#include "chassis_motion_closed_loop.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证直线和旋转等待时间按实际速度计算。 */
#define TASK9_MOVE_SPEED_RPM          60U
#define TASK9_MOVE_ACCELERATION       240U
#define TASK9_MOVE_SETTLE_MS         400U
#define TASK9_ARM_PREPOSITION_TIMEOUT_MS 3000U
#define TASK9_ROTATE_CLOCKWISE_SIGN  (-1)
#define TASK9_TEMPORARY_STACK_TOLERANCE_PIXEL       3
#define TASK9_TEMPORARY_STACK_POSITION_SETTLE_MS   40U

static void task9_status(task9_temporary_stack_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static void task9_apply_fine_adjust_config(task9_temporary_stack_t *task)
{
    if ((task == NULL) || (task->fine_adjust == NULL) ||
        (task->fine_adjust_config_saved != 0U))
    {
        return;
    }

    task->saved_tolerance_x_pixel =
        task->fine_adjust->config.tolerance_x_pixel;
    task->saved_tolerance_y_pixel =
        task->fine_adjust->config.tolerance_y_pixel;
    task->saved_position_settle_ms =
        task->fine_adjust->config.position_settle_ms;
    task->fine_adjust->config.tolerance_x_pixel =
        TASK9_TEMPORARY_STACK_TOLERANCE_PIXEL;
    task->fine_adjust->config.tolerance_y_pixel =
        TASK9_TEMPORARY_STACK_TOLERANCE_PIXEL;
    task->fine_adjust->config.position_settle_ms =
        TASK9_TEMPORARY_STACK_POSITION_SETTLE_MS;
    task->fine_adjust_config_saved = 1U;
}

static void task9_restore_fine_adjust_config(task9_temporary_stack_t *task)
{
    if ((task == NULL) || (task->fine_adjust == NULL) ||
        (task->fine_adjust_config_saved == 0U))
    {
        return;
    }

    task->fine_adjust->config.tolerance_x_pixel =
        task->saved_tolerance_x_pixel;
    task->fine_adjust->config.tolerance_y_pixel =
        task->saved_tolerance_y_pixel;
    task->fine_adjust->config.position_settle_ms =
        task->saved_position_settle_ms;
    task->fine_adjust_config_saved = 0U;
}

static task9_temporary_stack_result_t task9_fail(
    task9_temporary_stack_t *task,
    task9_temporary_stack_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        task9_restore_fine_adjust_config(task);
        (void)chassis_motion_set_service_callback(NULL, NULL);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK9_TEMPORARY_STACK_FAILED;
        task9_status(task, TASK9_TEMPORARY_STACK_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task9_pulses_per_mm(const mecanum_chassis_t *chassis)
{
    float circumference_mm;

    if ((chassis == NULL) ||
        (chassis->config.wheel_radius_m <= 0.0f) ||
        (chassis->config.pulses_per_rev == 0U))
    {
        return 0U;
    }
    circumference_mm = 2.0f * 3.14159265359f *
                       chassis->config.wheel_radius_m * 1000.0f;
    return (uint32_t)(((float)MECANUM_POSITION_PULSES_PER_REV /
                       circumference_mm) + 0.5f);
}

static uint32_t task9_linear_duration_ms(const mecanum_chassis_t *chassis,
                                         uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK9_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK9_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK9_MOVE_SETTLE_MS;
}

static uint32_t task9_rotation_pulses(const mecanum_chassis_t *chassis,
                                      uint16_t angle_deg)
{
    float wheel_distance_m;

    if ((chassis == NULL) ||
        (chassis->config.wheel_radius_m <= 0.0f) ||
        (chassis->config.pulses_per_rev == 0U))
    {
        return 0U;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    return (uint32_t)((wheel_distance_m /
                       (2.0f * 3.14159265359f *
                        chassis->config.wheel_radius_m) *
                       (float)MECANUM_POSITION_PULSES_PER_REV) + 0.5f);
}

static uint32_t task9_rotation_duration_ms(const mecanum_chassis_t *chassis,
                                           uint16_t angle_deg)
{
    float wheel_distance_m;
    float wheel_speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK9_MOVE_SETTLE_MS;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    wheel_speed_mps = ((float)TASK9_MOVE_SPEED_RPM / 60.0f) *
                      2.0f * 3.14159265359f *
                      chassis->config.wheel_radius_m;
    return (uint32_t)(wheel_distance_m / wheel_speed_mps * 1000.0f) +
           TASK9_MOVE_SETTLE_MS;
}

static void task9_wait_motion_done(const task9_temporary_stack_t *task)
{
    uint32_t elapsed_ms;

    if (task == NULL)
    {
        return;
    }
    elapsed_ms = HAL_GetTick() - task->move_start_tick;
    if (elapsed_ms < task->move_duration_ms)
    {
        HAL_Delay(task->move_duration_ms - elapsed_ms);
    }
}

static task9_temporary_stack_result_t task9_move_linear(
    task9_temporary_stack_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    if ((task == NULL) || (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK9_TEMPORARY_STACK_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK9_TEMPORARY_STACK_OK;
}

static task9_temporary_stack_result_t task9_rotate_clockwise_90(
    task9_temporary_stack_t *task)
{
    if (task == NULL)
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    if (chassis_motion_rotate_deg(
            task->chassis,
            (int16_t)TASK9_TEMPORARY_STACK_ROTATE_DEG *
            TASK9_ROTATE_CLOCKWISE_SIGN) != CHASSIS_MOTION_OK)
    {
        return TASK9_TEMPORARY_STACK_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK9_TEMPORARY_STACK_OK;
}

static uint16_t task9_backward_distance_mm(uint8_t ring)
{
    switch (ring)
    {
        case 1U:
            return TASK9_TEMPORARY_STACK_BACKWARD_RING_1_MM;
        case 2U:
            return TASK9_TEMPORARY_STACK_BACKWARD_RING_2_MM;
        case 3U:
            return TASK9_TEMPORARY_STACK_BACKWARD_RING_3_MM;
        default:
            return 0U;
    }
}

static task9_temporary_stack_result_t task9_select_target_ring(
    task9_temporary_stack_t *task)
{
    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    /* 底盘固定在暂存区2号环，目标环位交给机械臂水平轴处理。 */
    task->current_ring = TASK9_TEMPORARY_STACK_CENTER;
    return TASK9_TEMPORARY_STACK_OK;
}

static uint8_t task9_find_target(const task9_temporary_stack_t *task,
                                 uint8_t *ring)
{
    uint8_t index;
    uint8_t material_color;

    if ((task == NULL) || (ring == NULL))
    {
        return 0U;
    }
    material_color = task->qr_task.second_batch_color[task->item_index];
    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if (task->qr_task.first_batch_color[index] == material_color)
        {
            *ring = task->qr_task.first_batch_position[index];
            return 1U;
        }
    }
    return 0U;
}

static uint8_t task9_find_material_at_ring(
    const task9_temporary_stack_t *task,
    uint8_t ring,
    uint8_t *material_id)
{
    uint8_t index;

    if ((task == NULL) || (material_id == NULL) ||
        (ring < 1U) || (ring > 3U))
    {
        return 0U;
    }
    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if (task->qr_task.first_batch_position[index] == ring)
        {
            *material_id = task->qr_task.first_batch_color[index];
            return 1U;
        }
    }
    return 0U;
}

static task9_temporary_stack_result_t task9_set_tray_position(
    task9_temporary_stack_t *task)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK9_TEMPORARY_STACK_ITEM_COUNT))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)task->item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK9_TEMPORARY_STACK_TRAY_ERROR;
    }
    return TASK9_TEMPORARY_STACK_OK;
}

task9_temporary_stack_result_t task9_temporary_stack_init(
    task9_temporary_stack_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task9_temporary_stack_status_fn write_status)
{
    vision_fine_adjust_config_t config;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    memset(task, 0, sizeof(*task));
    task->arm = arm;
    task->chassis = chassis;
    task->fine_adjust = fine_adjust;
    task->tray = tray;
    task->screen = screen;
    task->write_status = write_status;
    vision_fine_adjust_config_default(&config);
    config.timeout_ms = 0U;
    if (vision_fine_adjust_init(fine_adjust,
                                vision_uart_get_default(),
                                chassis,
                                &config) != VISION_FINE_ADJUST_OK)
    {
        return TASK9_TEMPORARY_STACK_VISION_ERROR;
    }
    task->current_ring = TASK9_TEMPORARY_STACK_CENTER;
    task->state = TASK9_TEMPORARY_STACK_WAIT_TASK;
    task9_status(task,
                 TASK9_TEMPORARY_STACK_STATUS_INIT,
                 "task9_temporary_stack_ready");
    return TASK9_TEMPORARY_STACK_OK;
}

task9_temporary_stack_result_t task9_temporary_stack_submit_qr(
    task9_temporary_stack_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK9_TEMPORARY_STACK_WAIT_TASK))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    for (index = 0U; index < TASK9_TEMPORARY_STACK_ITEM_COUNT; index++)
    {
        if ((qr_task->first_batch_color[index] < 1U) ||
            (qr_task->first_batch_color[index] > 6U) ||
            (qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U) ||
            (qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U) ||
            (qr_task->second_batch_position[index] < 1U) ||
            (qr_task->second_batch_position[index] > 3U))
        {
            return TASK9_TEMPORARY_STACK_INVALID_PARAM;
        }
    }
    if (vision_fine_adjust_preselect_mode(
            task->fine_adjust,
            VISION_UART_MODE_MATERIAL) != VISION_FINE_ADJUST_OK)
    {
        return TASK9_TEMPORARY_STACK_VISION_ERROR;
    }
    task9_apply_fine_adjust_config(task);
    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    /* 跨区行驶和首次视觉校准期间预置第1工位。 */
    if (task9_set_tray_position(task) != TASK9_TEMPORARY_STACK_OK)
    {
        return task9_fail(task,
                          TASK9_TEMPORARY_STACK_TRAY_ERROR,
                          "task9_tray_position_failed");
    }
    /* 任务8结束后底盘仍停在粗加工区2号环。 */
    task->current_ring = TASK9_TEMPORARY_STACK_CENTER;
    task->task_ready = 1U;
    task->state = TASK9_TEMPORARY_STACK_BACKWARD_FROM_RING;
    task9_status(task,
                 TASK9_TEMPORARY_STACK_STATUS_MOVE,
                 "task9_temporary_stack_sequence_loaded");
    return TASK9_TEMPORARY_STACK_OK;
}

task9_temporary_stack_result_t task9_temporary_stack_step(
    task9_temporary_stack_t *task)
{
    task9_temporary_stack_result_t result;
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;
    uint16_t distance_mm;
    uint8_t center_material_id;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK9_TEMPORARY_STACK_INVALID_PARAM;
    }
    switch (task->state)
    {
        case TASK9_TEMPORARY_STACK_WAIT_TASK:
            return TASK9_TEMPORARY_STACK_WAITING;

        case TASK9_TEMPORARY_STACK_BACKWARD_FROM_RING:
            distance_mm = task9_backward_distance_mm(task->current_ring);
            if (distance_mm == 0U)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_INVALID_PARAM,
                                  "task9_backward_distance_invalid");
            }
            result = task9_move_linear(task, distance_mm, -1, 0);
            if (result != TASK9_TEMPORARY_STACK_OK)
            {
                return task9_fail(task, result,
                                  "task9_backward_from_ring_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_WAIT_BACKWARD_FROM_RING;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_WAIT_BACKWARD_FROM_RING:
            task9_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK9_TEMPORARY_STACK_ROTATE;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_ROTATE:
            task9_status(task,
                         TASK9_TEMPORARY_STACK_STATUS_ROTATE,
                         "task9_rotate_clockwise_90_start");
            result = task9_rotate_clockwise_90(task);
            if (result != TASK9_TEMPORARY_STACK_OK)
            {
                return task9_fail(task, result,
                                  "task9_rotate_clockwise_90_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_WAIT_ROTATE;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_WAIT_ROTATE:
            task9_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK9_TEMPORARY_STACK_BACKWARD_TO_AREA;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_BACKWARD_TO_AREA:
            if ((arm_controller_set_target_ring(
                     task->arm,
                     TASK9_TEMPORARY_STACK_CENTER) != ARM_CONTROLLER_OK) ||
                (chassis_motion_set_service_callback(
                     arm_controller_preposition_service,
                     task->arm) != CHASSIS_MOTION_OK) ||
                (arm_controller_start_stack_pick_preposition(task->arm) !=
                 ARM_CONTROLLER_OK))
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_ARM_ERROR,
                                  "task9_arm_preposition_start_failed");
            }
            task9_status(task,
                         TASK9_TEMPORARY_STACK_STATUS_PREPARE,
                         "task9_arm_preposition_during_travel_start");
            task9_status(task,
                         TASK9_TEMPORARY_STACK_STATUS_MOVE,
                         "task9_backward_to_temporary_area_start");
            result = task9_move_linear(task,
                                       TASK9_TEMPORARY_STACK_BACKWARD_TO_AREA_MM,
                                       -1,
                                       0);
            if (result != TASK9_TEMPORARY_STACK_OK)
            {
                return task9_fail(task, result,
                                  "task9_backward_to_temporary_area_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_WAIT_BACKWARD_TO_AREA;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_WAIT_BACKWARD_TO_AREA:
            task9_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            if (arm_controller_wait_preposition(
                    task->arm,
                    TASK9_ARM_PREPOSITION_TIMEOUT_MS) != ARM_CONTROLLER_OK)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_ARM_ERROR,
                                  "task9_arm_preposition_failed");
            }
            (void)chassis_motion_set_service_callback(NULL, NULL);
            task->current_ring = TASK9_TEMPORARY_STACK_CENTER;
            task->state = TASK9_TEMPORARY_STACK_PREPARE_ITEM;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_PREPARE_ITEM:
            if (task9_find_target(task,
                                  &task->target_ring) == 0U)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_INVALID_PARAM,
                                  "task9_qr_target_ring_not_found");
            }
            result = task9_select_target_ring(task);
            if (result != TASK9_TEMPORARY_STACK_OK)
            {
                return task9_fail(task, result,
                                  "task9_target_ring_invalid");
            }
            task9_status(task,
                         TASK9_TEMPORARY_STACK_STATUS_PREPARE,
                         "task9_stack_prepare_start");
            if (task->item_index == 0U)
            {
                /* 首件码垛视觉姿态已在跨区行驶期间完成。 */
                if (task9_find_material_at_ring(
                        task,
                        TASK9_TEMPORARY_STACK_CENTER,
                        &center_material_id) == 0U)
                {
                    return task9_fail(task,
                                      TASK9_TEMPORARY_STACK_INVALID_PARAM,
                                      "task9_center_material_not_found");
                }
                if (vision_fine_adjust_start_preselected(
                        task->fine_adjust,
                        VISION_UART_MODE_MATERIAL,
                        center_material_id) !=
                    VISION_FINE_ADJUST_OK)
                {
                    return task9_fail(task,
                                      TASK9_TEMPORARY_STACK_VISION_ERROR,
                                      "task9_center_material_mode_start_failed");
                }
                task->state = TASK9_TEMPORARY_STACK_ALIGN;
                return TASK9_TEMPORARY_STACK_OK;
            }

            if (arm_controller_set_target_ring(task->arm,
                                               task->target_ring) !=
                ARM_CONTROLLER_OK)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_ARM_ERROR,
                                  "task9_set_target_ring_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_PLACE_ITEM;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                if (arm_controller_set_target_ring(task->arm,
                                                   task->target_ring) !=
                    ARM_CONTROLLER_OK)
                {
                    return task9_fail(task,
                                      TASK9_TEMPORARY_STACK_ARM_ERROR,
                                      "task9_set_target_ring_failed");
                }
                (void)mecanum_chassis_stop(task->chassis);
                task->state = TASK9_TEMPORARY_STACK_PLACE_ITEM;
                task9_status(task,
                             TASK9_TEMPORARY_STACK_STATUS_FINE_ADJUST,
                             "task9_center_ring_fine_adjust_done");
                return TASK9_TEMPORARY_STACK_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_VISION_ERROR,
                                  "task9_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_CHASSIS_ERROR,
                                  "task9_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_ARM_ERROR,
                                  "task9_fine_adjust_failed");
            }
            return TASK9_TEMPORARY_STACK_WAITING;

        case TASK9_TEMPORARY_STACK_PLACE_ITEM:
            task9_status(task,
                         TASK9_TEMPORARY_STACK_STATUS_PLACE,
                         "task9_stack_item_start");
            arm_result = ((task->item_index + 1U) <
                          TASK9_TEMPORARY_STACK_ITEM_COUNT) ?
                arm_controller_stack_item_from_tray_keep_low(task->arm) :
                arm_controller_stack_item_from_tray(task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_ARM_ERROR,
                                  "task9_stack_item_failed");
            }
            if (screen_record_successful_place(task->screen) != SCREEN_OK)
            {
                return task9_fail(task,
                                  TASK9_TEMPORARY_STACK_SCREEN_ERROR,
                                  "task9_place_count_display_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_RETURN_TRAY;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_RETURN_TRAY:
            /* 件间保持码垛高度、暂存区朝向和开爪；最后一件已抬升，再回盘关爪。 */
            if ((task->item_index + 1U) >= TASK9_TEMPORARY_STACK_ITEM_COUNT)
            {
                arm_result = arm_controller_return_to_tray_direction(task->arm);
                if (arm_result != ARM_CONTROLLER_OK)
                {
                    return task9_fail(task,
                                      TASK9_TEMPORARY_STACK_ARM_ERROR,
                                      "task9_return_to_tray_failed");
                }
            }
            task->item_index++;
            if (task->item_index >= TASK9_TEMPORARY_STACK_ITEM_COUNT)
            {
                (void)mecanum_chassis_stop(task->chassis);
                task9_restore_fine_adjust_config(task);
                task->state = TASK9_TEMPORARY_STACK_DONE;
                task9_status(task,
                             TASK9_TEMPORARY_STACK_STATUS_DONE,
                             "task9_three_items_stacked");
                return TASK9_TEMPORARY_STACK_OK;
            }
            /* 机械臂已离开料盘，立即预置下一工位。 */
            result = task9_set_tray_position(task);
            if (result != TASK9_TEMPORARY_STACK_OK)
            {
                return task9_fail(task, result,
                                  "task9_tray_position_failed");
            }
            task->state = TASK9_TEMPORARY_STACK_PREPARE_ITEM;
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_DONE:
            return TASK9_TEMPORARY_STACK_OK;

        case TASK9_TEMPORARY_STACK_FAILED:
        default:
            return TASK9_TEMPORARY_STACK_ARM_ERROR;
    }
}

task9_temporary_stack_state_t task9_temporary_stack_get_state(
    const task9_temporary_stack_t *task)
{
    return (task == NULL) ?
           TASK9_TEMPORARY_STACK_FAILED : task->state;
}
