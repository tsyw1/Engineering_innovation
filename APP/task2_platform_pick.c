#include "task2_platform_pick.h"

#include "chassis_motion_closed_loop.h"

#include <stdio.h>
#include <string.h>

#define TASK2_ARM_PREPOSITION_TIMEOUT_MS 3000U
/* 物料盘取料按视觉图像中的轴向偏差判定，边界值也算已对准。 */
#define TASK2_FIRST_MATERIAL_TOLERANCE_X_PIXEL 10
#define TASK2_FIRST_MATERIAL_TOLERANCE_Y_PIXEL 10
#define TASK2_MATERIAL_TOLERANCE_X_PIXEL 73
#define TASK2_MATERIAL_TOLERANCE_Y_PIXEL 70
#define TASK2_FINE_ADJUST_POSITION_PULSES_PER_PIXEL 0.32f
#define TASK2_FINE_ADJUST_MAX_STEP_PULSES 64U
#define TASK2_RETURN_POSITION_SPEED_RPM 180U
#define TASK2_RETURN_POSITION_SETTLE_MS 120U

static void task2_apply_material_pick_tolerance(
    task2_platform_pick_t *task)
{
    if ((task == NULL) || (task->fine_adjust == NULL))
    {
        return;
    }

    /* 每次取料前重新写入，避免被其他任务对同一个视觉对象的初始化覆盖。 */
    if (task->item_index == 0U)
    {
        task->fine_adjust->config.tolerance_x_pixel =
            TASK2_FIRST_MATERIAL_TOLERANCE_X_PIXEL;
        task->fine_adjust->config.tolerance_y_pixel =
            TASK2_FIRST_MATERIAL_TOLERANCE_Y_PIXEL;
    }
    else
    {
        task->fine_adjust->config.tolerance_x_pixel =
            TASK2_MATERIAL_TOLERANCE_X_PIXEL;
        task->fine_adjust->config.tolerance_y_pixel =
            TASK2_MATERIAL_TOLERANCE_Y_PIXEL;
    }
    task->fine_adjust->config.position_pulses_per_pixel =
        TASK2_FINE_ADJUST_POSITION_PULSES_PER_PIXEL;
    task->fine_adjust->config.max_step_pulses =
        TASK2_FINE_ADJUST_MAX_STEP_PULSES;
    vision_fine_adjust_set_target_match_required(task->fine_adjust, 1U);
    vision_fine_adjust_set_coarse_pick(task->fine_adjust, 0U);
}

static void task2_restore_fine_adjust_defaults(
    task2_platform_pick_t *task)
{
    if ((task == NULL) || (task->fine_adjust == NULL))
    {
        return;
    }
    task->fine_adjust->config.position_pulses_per_pixel =
        VISION_FINE_ADJUST_DEFAULT_POSITION_PULSES_PER_PIXEL;
    task->fine_adjust->config.max_step_pulses =
        VISION_FINE_ADJUST_DEFAULT_MAX_STEP_PULSES;
    vision_fine_adjust_set_target_match_required(task->fine_adjust, 0U);
}

static void task2_status(task2_platform_pick_t *task,
                         uint8_t status,
                         const char *event);

static task2_platform_pick_result_t task2_move(
    task2_platform_pick_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign);

static void task2_wait_move_done(const task2_platform_pick_t *task);

static task2_platform_pick_result_t task2_return_to_entry(
    task2_platform_pick_t *task)
{
    int32_t pulses[MECANUM_WHEEL_COUNT];
    int32_t reverse_pulses[MECANUM_WHEEL_COUNT];
    uint32_t max_pulses = 0U;
    uint8_t index;
    uint32_t duration_ms;
    mecanum_result_t chassis_result;

    if ((task == NULL) || (task->fine_adjust == NULL) ||
        (task->chassis == NULL))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    vision_fine_adjust_take_motion_pulses(task->fine_adjust, pulses);
    vision_fine_adjust_abort(task->fine_adjust);
    vision_fine_adjust_set_coarse_pick(task->fine_adjust, 0U);
    vision_fine_adjust_set_motion_enabled(task->fine_adjust, 1U);
    vision_fine_adjust_set_heading_enabled(task->fine_adjust, 1U);
    task2_restore_fine_adjust_defaults(task);

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int32_t magnitude = (pulses[index] < 0) ?
                           -pulses[index] : pulses[index];
        reverse_pulses[index] = -pulses[index];
        if ((uint32_t)magnitude > max_pulses)
        {
            max_pulses = (uint32_t)magnitude;
        }
    }

    if (max_pulses != 0U)
    {
        task2_status(task,
                     TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                     "task2_reverse_fine_adjust_start");
        chassis_result = mecanum_chassis_set_position_increment(
            task->chassis,
            reverse_pulses,
            TASK2_RETURN_POSITION_SPEED_RPM,
            task->chassis->config.acceleration);
        if ((chassis_result != MECANUM_OK) &&
            (chassis_result != MECANUM_CLAMPED))
        {
            return TASK2_PLATFORM_PICK_CHASSIS_ERROR;
        }
        duration_ms = (max_pulses * 60000U +
                       ((uint32_t)TASK2_RETURN_POSITION_SPEED_RPM *
                        MECANUM_POSITION_PULSES_PER_REV) - 1U) /
                      ((uint32_t)TASK2_RETURN_POSITION_SPEED_RPM *
                       MECANUM_POSITION_PULSES_PER_REV);
        HAL_Delay(duration_ms + TASK2_RETURN_POSITION_SETTLE_MS);
        task2_status(task,
                     TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                     "task2_reverse_fine_adjust_done");
    }

    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                 "task2_backward_70mm_start");
    if (task2_move(task,
                   TASK2_PLATFORM_PICK_FORWARD_MM,
                   -1,
                   0) != TASK2_PLATFORM_PICK_OK)
    {
        return TASK2_PLATFORM_PICK_CHASSIS_ERROR;
    }
    task2_wait_move_done(task);
    (void)mecanum_chassis_stop(task->chassis);
    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                 "task2_backward_70mm_done");
    return TASK2_PLATFORM_PICK_OK;
}

static void task2_status(task2_platform_pick_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static void task2_stop(task2_platform_pick_t *task)
{
    if (task == NULL)
    {
        return;
    }
    if (task->fine_adjust != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        vision_fine_adjust_set_coarse_pick(task->fine_adjust, 0U);
        vision_fine_adjust_set_motion_enabled(task->fine_adjust, 1U);
        vision_fine_adjust_set_heading_enabled(task->fine_adjust, 1U);
        task2_restore_fine_adjust_defaults(task);
    }
    if (task->chassis != NULL)
    {
        (void)chassis_motion_set_service_callback(NULL, NULL);
        (void)chassis_motion_set_heading_disturbance_compensation(0U);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
    }
}

static task2_platform_pick_result_t task2_fail(
    task2_platform_pick_t *task,
    task2_platform_pick_result_t result,
    const char *event)
{
    task2_stop(task);
    if (task != NULL)
    {
        task->state = TASK2_PLATFORM_PICK_FAILED;
        task->item_state = TASK2_PLATFORM_PICK_ITEMS_FAILED;
        task2_status(task, TASK2_PLATFORM_PICK_STATUS_ERROR, event);
    }
    return result;
}

static task2_platform_pick_result_t task2_move(
    task2_platform_pick_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    if ((task == NULL) || (task->chassis == NULL) ||
        (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK2_PLATFORM_PICK_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK2_PLATFORM_PICK_OK;
}

static void task2_wait_move_done(const task2_platform_pick_t *task)
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

static task2_platform_pick_result_t task2_start_tray_position_at(
    task2_platform_pick_t *task,
    uint8_t item_index);

static task2_platform_pick_result_t task2_start_tray_position(
    task2_platform_pick_t *task)
{
    return task2_start_tray_position_at(
        task,
        (task != NULL) ? task->item_index : TASK2_PLATFORM_PICK_ITEM_COUNT);
}

static task2_platform_pick_result_t task2_start_tray_position_at(
    task2_platform_pick_t *task,
    uint8_t item_index)
{
    const material_tray_position_t positions[TASK2_PLATFORM_PICK_ITEM_COUNT] =
    {
        MATERIAL_TRAY_POSITION_1,
        MATERIAL_TRAY_POSITION_2,
        MATERIAL_TRAY_POSITION_3
    };

    if ((task == NULL) || (task->tray == NULL) ||
        (item_index >= TASK2_PLATFORM_PICK_ITEM_COUNT))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_TRAY_POSITION,
                 "task2_tray_position_start");
    if (material_tray_servo_set_position(
            task->tray,
            positions[item_index]) != MATERIAL_TRAY_SERVO_OK)
    {
        return task2_fail(task,
                          TASK2_PLATFORM_PICK_TRAY_ERROR,
                          "task2_tray_position_failed");
    }
    return TASK2_PLATFORM_PICK_OK;
}

static arm_controller_result_t task2_start_next_tray_position_callback(
    void *context)
{
    task2_platform_pick_t *task = (task2_platform_pick_t *)context;
    task2_platform_pick_result_t result;

    if ((task == NULL) ||
        ((task->item_index + 1U) >= TASK2_PLATFORM_PICK_ITEM_COUNT))
    {
        return ARM_CONTROLLER_OK;
    }
    result = task2_start_tray_position_at(task,
                                          (uint8_t)(task->item_index + 1U));
    return (result == TASK2_PLATFORM_PICK_OK) ?
           ARM_CONTROLLER_OK : ARM_CONTROLLER_MOTOR_ERROR;
}

static void task2_finish_tray_position(task2_platform_pick_t *task)
{
    if (task == NULL)
    {
        return;
    }
    HAL_Delay(TASK2_PLATFORM_PICK_TRAY_SETTLE_MS);
    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_TRAY_POSITION,
                 "task2_tray_position_done");
}

static task2_platform_pick_result_t task2_set_tray_position(
    task2_platform_pick_t *task)
{
    task2_platform_pick_result_t result;

    result = task2_start_tray_position(task);
    if (result != TASK2_PLATFORM_PICK_OK)
    {
        return result;
    }
    task2_finish_tray_position(task);
    return TASK2_PLATFORM_PICK_OK;
}

static task2_platform_pick_result_t task2_start_item(
    task2_platform_pick_t *task)
{
    char event[96];
    int length;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->fine_adjust == NULL) ||
        (task->item_index >= TASK2_PLATFORM_PICK_ITEM_COUNT) ||
        (task->target_ids[task->item_index] == 0U))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    task2_apply_material_pick_tolerance(task);
    /* 原料区取料视觉阶段只修正平移，航向误差延后到后续 180 度旋转。 */
    vision_fine_adjust_set_heading_enabled(task->fine_adjust, 0U);

    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                 "task2_platform_prepare_pick_start");
    if (task->item_index == 0U)
    {
        /* 首件机械臂姿态已在两段跨区行驶期间完成。 */
        vision_fine_adjust_set_motion_enabled(task->fine_adjust, 1U);
        adjust_result = vision_fine_adjust_start_preselected(
            task->fine_adjust,
            VISION_UART_MODE_MATERIAL,
            task->target_ids[task->item_index]);
    }
    else
    {
        /* 后两件只禁止底盘修正，仍启动视觉等待目标进入73/70像素容差。 */
        if (arm_controller_prepare_platform_pick_from_tray(task->arm) !=
            ARM_CONTROLLER_OK)
        {
            adjust_result = VISION_FINE_ADJUST_ARM_ERROR;
        }
        else
        {
            vision_fine_adjust_set_motion_enabled(task->fine_adjust, 0U);
            adjust_result = vision_fine_adjust_start(
                task->fine_adjust,
                VISION_UART_MODE_MATERIAL,
                task->target_ids[task->item_index]);
        }
    }
    if (adjust_result != VISION_FINE_ADJUST_OK)
    {
        length = snprintf(event,
                          sizeof(event),
                          "task2_item=%u,prepare_result=%u",
                          (unsigned int)(task->item_index + 1U),
                          (unsigned int)adjust_result);
        if ((length > 0) && ((size_t)length < sizeof(event)))
        {
            task2_status(task, TASK2_PLATFORM_PICK_STATUS_ERROR, event);
        }
        return task2_fail(task,
                          TASK2_PLATFORM_PICK_ARM_ERROR,
                          "task2_platform_prepare_pick_failed");
    }

    length = snprintf(event,
                      sizeof(event),
                      "task2_item=%u,target_id=%u,%s",
                      (unsigned int)(task->item_index + 1U),
                      (unsigned int)task->target_ids[task->item_index],
                      (task->item_index == 0U) ?
                       "fine_adjust_started" : "vision_wait_started");
    if ((length > 0) && ((size_t)length < sizeof(event)))
    {
        task2_status(task, TASK2_PLATFORM_PICK_STATUS_FINE_ADJUST, event);
    }
    return TASK2_PLATFORM_PICK_OK;
}

static task2_platform_pick_result_t task2_start_first_item(
    task2_platform_pick_t *task)
{
    if (task == NULL)
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }
    task->item_index = 0U;
    if (task2_set_tray_position(task) != TASK2_PLATFORM_PICK_OK)
    {
        return TASK2_PLATFORM_PICK_TRAY_ERROR;
    }
    if (task2_start_item(task) != TASK2_PLATFORM_PICK_OK)
    {
        return TASK2_PLATFORM_PICK_ARM_ERROR;
    }
    task->item_state = TASK2_PLATFORM_PICK_ITEMS_ALIGN;
    return TASK2_PLATFORM_PICK_OK;
}

static task2_platform_pick_result_t task2_step_items(
    task2_platform_pick_t *task)
{
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->fine_adjust == NULL) || (task->screen == NULL))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    switch (task->item_state)
    {
        case TASK2_PLATFORM_PICK_ITEMS_WAIT:
            return TASK2_PLATFORM_PICK_WAITING;

        case TASK2_PLATFORM_PICK_ITEMS_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                task->item_state = TASK2_PLATFORM_PICK_ITEMS_PICK;
                task2_status(task,
                             TASK2_PLATFORM_PICK_STATUS_FINE_ADJUST,
                             "task2_fine_adjust_done");
                return TASK2_PLATFORM_PICK_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_VISION_ERROR,
                                  "task2_fine_adjust_timeout");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_ARM_ERROR,
                                  "task2_fine_adjust_failed");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_CHASSIS_ERROR,
                                  "task2_fine_adjust_chassis_error");
            }
            return TASK2_PLATFORM_PICK_WAITING;

        case TASK2_PLATFORM_PICK_ITEMS_PICK:
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PICK,
                         "task2_platform_pick_place_start");
            arm_result = arm_controller_pick_and_place_with_callback(
                task->arm,
                task2_start_next_tray_position_callback,
                task);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                char event[96];
                int length;

                length = snprintf(event,
                                  sizeof(event),
                                  "task2_item=%u,arm_pick_result=%u",
                                  (unsigned int)(task->item_index + 1U),
                                  (unsigned int)arm_result);
                if ((length > 0) && ((size_t)length < sizeof(event)))
                {
                    task2_status(task,
                                 TASK2_PLATFORM_PICK_STATUS_ERROR,
                                 event);
                }
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_ARM_ERROR,
                                  "task2_platform_pick_place_failed");
            }
            if (screen_record_successful_pick(task->screen) != SCREEN_OK)
            {
                task2_status(task,
                             TASK2_PLATFORM_PICK_STATUS_ERROR,
                             "task2_pick_count_display_failed");
            }
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PLACE,
                         "task2_platform_pick_to_tray_done");

            task->item_index++;
            if (task->item_index >= TASK2_PLATFORM_PICK_ITEM_COUNT)
            {
                task->item_state = TASK2_PLATFORM_PICK_ITEMS_DONE;
                if (task2_return_to_entry(task) != TASK2_PLATFORM_PICK_OK)
                {
                    return task2_fail(task,
                                      TASK2_PLATFORM_PICK_CHASSIS_ERROR,
                                      "task2_return_to_entry_failed");
                }
                task2_status(task,
                             TASK2_PLATFORM_PICK_STATUS_DONE,
                             "task2_platform_three_items_done");
                return TASK2_PLATFORM_PICK_OK;
            }

            if (task2_start_item(task) != TASK2_PLATFORM_PICK_OK)
            {
                return TASK2_PLATFORM_PICK_ARM_ERROR;
            }
            task2_finish_tray_position(task);
            task->item_state = TASK2_PLATFORM_PICK_ITEMS_ALIGN;
            return TASK2_PLATFORM_PICK_OK;

        case TASK2_PLATFORM_PICK_ITEMS_DONE:
            return TASK2_PLATFORM_PICK_OK;

        case TASK2_PLATFORM_PICK_ITEMS_FAILED:
        default:
            return TASK2_PLATFORM_PICK_ARM_ERROR;
    }
}

task2_platform_pick_result_t task2_platform_pick_init(
    task2_platform_pick_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task2_platform_pick_status_fn write_status)
{
    vision_fine_adjust_config_t config;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    memset(task, 0, sizeof(*task));
    task->arm = arm;
    task->chassis = chassis;
    task->fine_adjust = fine_adjust;
    task->tray = tray;
    task->screen = screen;
    task->write_status = write_status;

    vision_fine_adjust_config_default(&config);
    config.tolerance_x_pixel = TASK2_MATERIAL_TOLERANCE_X_PIXEL;
    config.tolerance_y_pixel = TASK2_MATERIAL_TOLERANCE_Y_PIXEL;
    config.timeout_ms = 0U;
    adjust_result = vision_fine_adjust_init(fine_adjust,
                                            vision_uart_get_default(),
                                            chassis,
                                            &config);
    if (adjust_result != VISION_FINE_ADJUST_OK)
    {
        return TASK2_PLATFORM_PICK_VISION_ERROR;
    }

    task->item_state = TASK2_PLATFORM_PICK_ITEMS_WAIT;
    task->state = TASK2_PLATFORM_PICK_WAIT_TASK;
    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_INIT,
                 "task2_platform_pick_ready");
    return TASK2_PLATFORM_PICK_OK;
}

task2_platform_pick_result_t task2_platform_pick_submit_qr(
    task2_platform_pick_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK2_PLATFORM_PICK_WAIT_TASK))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }
    for (index = 0U; index < TASK2_PLATFORM_PICK_ITEM_COUNT; index++)
    {
        if ((qr_task->first_batch_color[index] < 1U) ||
            (qr_task->first_batch_color[index] > 6U))
        {
            return TASK2_PLATFORM_PICK_INVALID_PARAM;
        }
        task->target_ids[index] = qr_task->first_batch_color[index];
    }

    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->task_ready = 1U;
    task->state = TASK2_PLATFORM_PICK_RIGHT;
    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_TASK_CODE,
                 "task2_platform_pick_sequence_loaded");
    return TASK2_PLATFORM_PICK_OK;
}

task2_platform_pick_result_t task2_platform_pick_step(
    task2_platform_pick_t *task)
{
    task2_platform_pick_result_t result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK2_PLATFORM_PICK_WAIT_TASK:
            return TASK2_PLATFORM_PICK_WAITING;

        case TASK2_PLATFORM_PICK_RIGHT:
            vision_fine_adjust_clear_motion_pulses(task->fine_adjust);
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_arm_preposition_during_right_start");
            if ((chassis_motion_set_heading_disturbance_compensation(1U) !=
                 CHASSIS_MOTION_OK) ||
                (chassis_motion_lock_current_heading() !=
                 CHASSIS_MOTION_OK) ||
                (chassis_motion_set_service_callback(
                     arm_controller_preposition_service,
                     task->arm) != CHASSIS_MOTION_OK) ||
                (arm_controller_start_platform_pick_preposition(task->arm) !=
                 ARM_CONTROLLER_OK))
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_ARM_ERROR,
                                  "task2_arm_preposition_start_failed");
            }
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_right_920mm_start");
            result = task2_move(task,
                                TASK2_PLATFORM_PICK_RIGHT_MM,
                                0,
                                -1);
            if (result != TASK2_PLATFORM_PICK_OK)
            {
                return task2_fail(task, result,
                                  "task2_right_920mm_failed");
            }
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_forward_70mm_start");
            result = task2_move(task,
                                TASK2_PLATFORM_PICK_FORWARD_MM,
                                1,
                                0);
            if (result != TASK2_PLATFORM_PICK_OK)
            {
                return task2_fail(task, result,
                                  "task2_forward_70mm_failed");
            }
            task2_wait_move_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            if (arm_controller_wait_preposition(
                    task->arm,
                    TASK2_ARM_PREPOSITION_TIMEOUT_MS) != ARM_CONTROLLER_OK)
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_ARM_ERROR,
                                  "task2_arm_preposition_failed");
            }
            (void)chassis_motion_set_heading_disturbance_compensation(0U);
            (void)chassis_motion_set_service_callback(NULL, NULL);
            if (vision_fine_adjust_preselect_mode(
                    task->fine_adjust,
                    VISION_UART_MODE_MATERIAL) != VISION_FINE_ADJUST_OK)
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_VISION_ERROR,
                                  "task2_material_mode_select_failed");
            }
            if (task2_start_first_item(task) != TASK2_PLATFORM_PICK_OK)
            {
                return task2_fail(task,
                                  TASK2_PLATFORM_PICK_ARM_ERROR,
                                  "task2_first_platform_item_start_failed");
            }
            task->state = TASK2_PLATFORM_PICK_ITEMS;
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_material_area_reached");
            return TASK2_PLATFORM_PICK_OK;

        case TASK2_PLATFORM_PICK_ITEMS:
            result = task2_step_items(task);
            if (task->item_state == TASK2_PLATFORM_PICK_ITEMS_DONE)
            {
                task->state = TASK2_PLATFORM_PICK_DONE;
                return TASK2_PLATFORM_PICK_OK;
            }
            return result;

        case TASK2_PLATFORM_PICK_DONE:
            return TASK2_PLATFORM_PICK_OK;

        case TASK2_PLATFORM_PICK_FAILED:
        default:
            return TASK2_PLATFORM_PICK_ARM_ERROR;
    }
}

task2_platform_pick_state_t task2_platform_pick_get_state(
    const task2_platform_pick_t *task)
{
    return (task == NULL) ?
           TASK2_PLATFORM_PICK_FAILED : task->state;
}
