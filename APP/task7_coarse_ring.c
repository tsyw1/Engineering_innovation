#include "task7_coarse_ring.h"

#include "chassis_motion_closed_loop.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证直线和旋转等待时间按实际速度计算。 */
#define TASK7_MOVE_SPEED_RPM          60U
#define TASK7_MOVE_ACCELERATION       240U
#define TASK7_MOVE_SETTLE_MS         400U
#define TASK7_ROTATE_SIGN              1
#define TASK7_COARSE_RING_TOLERANCE_PIXEL       3
#define TASK7_COARSE_RING_POSITION_SETTLE_MS   40U

static void task7_status(task7_coarse_ring_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static void task7_apply_fine_adjust_config(task7_coarse_ring_t *task)
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
        TASK7_COARSE_RING_TOLERANCE_PIXEL;
    task->fine_adjust->config.tolerance_y_pixel =
        TASK7_COARSE_RING_TOLERANCE_PIXEL;
    task->fine_adjust->config.position_settle_ms =
        TASK7_COARSE_RING_POSITION_SETTLE_MS;
    task->fine_adjust_config_saved = 1U;
}

static void task7_restore_fine_adjust_config(task7_coarse_ring_t *task)
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

static task7_coarse_ring_result_t task7_fail(
    task7_coarse_ring_t *task,
    task7_coarse_ring_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        task7_restore_fine_adjust_config(task);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK7_COARSE_RING_FAILED;
        task7_status(task, TASK7_COARSE_RING_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task7_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task7_linear_duration_ms(const mecanum_chassis_t *chassis,
                                         uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK7_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK7_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK7_MOVE_SETTLE_MS;
}

static uint32_t task7_rotation_pulses(const mecanum_chassis_t *chassis,
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

static uint32_t task7_rotation_duration_ms(const mecanum_chassis_t *chassis,
                                           uint16_t angle_deg)
{
    float wheel_distance_m;
    float wheel_speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK7_MOVE_SETTLE_MS;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    wheel_speed_mps = ((float)TASK7_MOVE_SPEED_RPM / 60.0f) *
                      2.0f * 3.14159265359f *
                      chassis->config.wheel_radius_m;
    return (uint32_t)(wheel_distance_m / wheel_speed_mps * 1000.0f) +
           TASK7_MOVE_SETTLE_MS;
}

static void task7_wait_motion_done(const task7_coarse_ring_t *task)
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

static task7_coarse_ring_result_t task7_move_linear(
    task7_coarse_ring_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    if ((task == NULL) || (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }
    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK7_COARSE_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK7_COARSE_RING_OK;
}

static task7_coarse_ring_result_t task7_rotate_180(task7_coarse_ring_t *task)
{
    if (task == NULL)
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }
    if (chassis_motion_rotate_deg(task->chassis,
                                  (int16_t)TASK7_COARSE_RING_ROTATE_DEG *
                                  TASK7_ROTATE_SIGN) != CHASSIS_MOTION_OK)
    {
        return TASK7_COARSE_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK7_COARSE_RING_OK;
}

static task7_coarse_ring_result_t task7_select_target_ring(
    task7_coarse_ring_t *task)
{
    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }
    /* 底盘固定在2号环，目标环位交给机械臂水平轴处理。 */
    task->current_ring = TASK7_COARSE_RING_CENTER;
    return TASK7_COARSE_RING_OK;
}

static task7_coarse_ring_result_t task7_set_tray_position(
    task7_coarse_ring_t *task)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK7_COARSE_RING_ITEM_COUNT))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }
    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)task->item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK7_COARSE_RING_TRAY_ERROR;
    }
    return TASK7_COARSE_RING_OK;
}

task7_coarse_ring_result_t task7_coarse_ring_init(
    task7_coarse_ring_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task7_coarse_ring_status_fn write_status)
{
    vision_fine_adjust_config_t config;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
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
    adjust_result = vision_fine_adjust_init(fine_adjust,
                                            vision_uart_get_default(),
                                            chassis,
                                            &config);
    if (adjust_result != VISION_FINE_ADJUST_OK)
    {
        return TASK7_COARSE_RING_VISION_ERROR;
    }
    task->current_ring = TASK7_COARSE_RING_CENTER;
    task->state = TASK7_COARSE_RING_WAIT_TASK;
    task7_status(task,
                 TASK7_COARSE_RING_STATUS_INIT,
                 "task7_coarse_ring_ready");
    return TASK7_COARSE_RING_OK;
}

task7_coarse_ring_result_t task7_coarse_ring_submit_qr(
    task7_coarse_ring_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK7_COARSE_RING_WAIT_TASK))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }
    for (index = 0U; index < TASK7_COARSE_RING_ITEM_COUNT; index++)
    {
        if ((qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U) ||
            (qr_task->second_batch_position[index] < 1U) ||
            (qr_task->second_batch_position[index] > 3U))
        {
            return TASK7_COARSE_RING_INVALID_PARAM;
        }
    }
    if (vision_fine_adjust_preselect_mode(
            task->fine_adjust,
            VISION_UART_MODE_TARGET_RING) != VISION_FINE_ADJUST_OK)
    {
        return TASK7_COARSE_RING_VISION_ERROR;
    }
    task7_apply_fine_adjust_config(task);
    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    /* 跨区行驶和首次视觉校准期间预置第1工位。 */
    if (task7_set_tray_position(task) != TASK7_COARSE_RING_OK)
    {
        return task7_fail(task,
                          TASK7_COARSE_RING_TRAY_ERROR,
                          "task7_tray_position_failed");
    }
    task->current_ring = TASK7_COARSE_RING_CENTER;
    task->task_ready = 1U;
    task->state = TASK7_COARSE_RING_LEFT;
    task7_status(task,
                 TASK7_COARSE_RING_STATUS_MOVE,
                 "task7_left_100mm_start");
    return TASK7_COARSE_RING_OK;
}

task7_coarse_ring_result_t task7_coarse_ring_step(
    task7_coarse_ring_t *task)
{
    task7_coarse_ring_result_t result;
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK7_COARSE_RING_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK7_COARSE_RING_WAIT_TASK:
            return TASK7_COARSE_RING_WAITING;

        case TASK7_COARSE_RING_LEFT:
            result = task7_move_linear(task,
                                       TASK7_COARSE_RING_LEFT_MM,
                                       0,
                                       1);
            if (result != TASK7_COARSE_RING_OK)
            {
                return task7_fail(task, result,
                                  "task7_left_100mm_failed");
            }
            task7_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK7_COARSE_RING_ROTATE;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_ROTATE:
            task7_status(task,
                         TASK7_COARSE_RING_STATUS_ROTATE,
                         "task7_rotate_180_start");
            result = task7_rotate_180(task);
            if (result != TASK7_COARSE_RING_OK)
            {
                return task7_fail(task, result,
                                  "task7_rotate_180_failed");
            }
            task7_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK7_COARSE_RING_RIGHT;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_RIGHT:
            result = task7_move_linear(task,
                                       TASK7_COARSE_RING_RIGHT_MM,
                                       0,
                                       -1);
            if (result != TASK7_COARSE_RING_OK)
            {
                return task7_fail(task, result,
                                  "task7_right_1670mm_failed");
            }
            task7_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK7_COARSE_RING_PREPARE_ITEM;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_PREPARE_ITEM:
            task->target_ring = task->qr_task.second_batch_position[
                task->item_index];
            result = task7_select_target_ring(task);
            if (result != TASK7_COARSE_RING_OK)
            {
                return task7_fail(task, result,
                                  "task7_target_ring_invalid");
            }
            task7_status(task,
                         TASK7_COARSE_RING_STATUS_PREPARE,
                         "task7_ground_place_prepare_start");
            if (task->item_index == 0U)
            {
                /* 只校准粗加工区2号中心环一次，随后保持车身不动。 */
                adjust_result =
                    vision_fine_adjust_prepare_place_and_start_preselected(
                        task->fine_adjust,
                        task->arm,
                        TASK7_COARSE_RING_CENTER);
                if (adjust_result != VISION_FINE_ADJUST_OK)
                {
                    return task7_fail(task,
                        (adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ?
                        TASK7_COARSE_RING_ARM_ERROR :
                        TASK7_COARSE_RING_VISION_ERROR,
                                      "task7_ground_place_prepare_failed");
                }
                task->state = TASK7_COARSE_RING_ALIGN;
                return TASK7_COARSE_RING_OK;
            }

            if (arm_controller_set_target_ring(task->arm,
                                               task->target_ring) !=
                ARM_CONTROLLER_OK)
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_ARM_ERROR,
                                  "task7_set_target_ring_failed");
            }
            task->state = TASK7_COARSE_RING_PLACE_ITEM;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                if (arm_controller_set_target_ring(task->arm,
                                                   task->target_ring) !=
                    ARM_CONTROLLER_OK)
                {
                    return task7_fail(task,
                                      TASK7_COARSE_RING_ARM_ERROR,
                                      "task7_set_target_ring_failed");
                }
                (void)mecanum_chassis_stop(task->chassis);
                task->state = TASK7_COARSE_RING_PLACE_ITEM;
                task7_status(task,
                             TASK7_COARSE_RING_STATUS_FINE_ADJUST,
                             "task7_center_ring_fine_adjust_done");
                return TASK7_COARSE_RING_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_VISION_ERROR,
                                  "task7_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_CHASSIS_ERROR,
                                  "task7_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_ARM_ERROR,
                                  "task7_fine_adjust_failed");
            }
            return TASK7_COARSE_RING_WAITING;

        case TASK7_COARSE_RING_PLACE_ITEM:
            task7_status(task,
                         TASK7_COARSE_RING_STATUS_PLACE,
                         "task7_pick_from_tray_and_place_start");
            if (task->item_index + 1U < TASK7_COARSE_RING_ITEM_COUNT)
            {
                arm_result = arm_controller_place_ring_item_from_tray_keep_low(
                    task->arm);
            }
            else
            {
                arm_result = arm_controller_place_ring_item_from_tray(task->arm);
            }
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_ARM_ERROR,
                                  "task7_pick_from_tray_failed");
            }
            if (screen_record_successful_place(task->screen) != SCREEN_OK)
            {
                return task7_fail(task,
                                  TASK7_COARSE_RING_SCREEN_ERROR,
                                  "task7_place_count_display_failed");
            }
            task->state = TASK7_COARSE_RING_FINISH_ITEM;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_FINISH_ITEM:
            task->item_index++;
            if (task->item_index >= TASK7_COARSE_RING_ITEM_COUNT)
            {
                /* 最后一件完成后保持最高点、面向粗加工区。 */
                task7_restore_fine_adjust_config(task);
                task->state = TASK7_COARSE_RING_DONE;
                task7_status(task,
                             TASK7_COARSE_RING_STATUS_DONE,
                             "task7_three_items_placed");
                return TASK7_COARSE_RING_OK;
            }
            /* 机械臂已离开料盘，立即预置下一工位。 */
            result = task7_set_tray_position(task);
            if (result != TASK7_COARSE_RING_OK)
            {
                return task7_fail(task, result,
                                  "task7_tray_position_failed");
            }
            /* 前两件保持低位、面向靶环、夹爪打开，底盘继续停在2号环。 */
            task->state = TASK7_COARSE_RING_PREPARE_ITEM;
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_DONE:
            return TASK7_COARSE_RING_OK;

        case TASK7_COARSE_RING_FAILED:
        default:
            return TASK7_COARSE_RING_ARM_ERROR;
    }
}

task7_coarse_ring_state_t task7_coarse_ring_get_state(
    const task7_coarse_ring_t *task)
{
    return (task == NULL) ?
           TASK7_COARSE_RING_FAILED : task->state;
}
