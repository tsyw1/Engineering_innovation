#include "task5_temporary_ring.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证直线和旋转等待时间按实际速度计算。 */
#define TASK5_MOVE_SPEED_RPM          60U
#define TASK5_MOVE_ACCELERATION       240U
#define TASK5_MOVE_SETTLE_MS         400U
/* 位置脉冲符号约定：+1 为逆时针，-1 为顺时针。 */
#define TASK5_ROTATE_CLOCKWISE_SIGN   (-1)

static void task5_status(task5_temporary_ring_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task5_temporary_ring_result_t task5_fail(
    task5_temporary_ring_t *task,
    task5_temporary_ring_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK5_TEMPORARY_RING_FAILED;
        task5_status(task, TASK5_TEMPORARY_RING_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task5_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task5_linear_duration_ms(const mecanum_chassis_t *chassis,
                                         uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK5_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK5_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK5_MOVE_SETTLE_MS;
}

static uint32_t task5_rotation_pulses(const mecanum_chassis_t *chassis,
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

static uint32_t task5_rotation_duration_ms(const mecanum_chassis_t *chassis,
                                           uint16_t angle_deg)
{
    float wheel_distance_m;
    float wheel_speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK5_MOVE_SETTLE_MS;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    wheel_speed_mps = ((float)TASK5_MOVE_SPEED_RPM / 60.0f) *
                      2.0f * 3.14159265359f *
                      chassis->config.wheel_radius_m;
    return (uint32_t)(wheel_distance_m / wheel_speed_mps * 1000.0f) +
           TASK5_MOVE_SETTLE_MS;
}

static void task5_wait_motion_done(const task5_temporary_ring_t *task)
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

static task5_temporary_ring_result_t task5_move_linear(
    task5_temporary_ring_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    uint32_t pulses_per_mm;
    int32_t body_x_pulse;
    int32_t body_y_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];

    if ((task == NULL) || (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }
    pulses_per_mm = task5_pulses_per_mm(task->chassis);
    if (pulses_per_mm == 0U)
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }

    body_x_pulse = (int32_t)pulses_per_mm * (int32_t)distance_mm *
                   (int32_t)body_x_sign;
    body_y_pulse = (int32_t)pulses_per_mm * (int32_t)distance_mm *
                   (int32_t)body_y_sign;
    wheel_pulse[MECANUM_WHEEL_FL] = body_x_pulse - body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] = body_x_pulse - body_y_pulse;

    if (mecanum_chassis_set_position_increment(
            task->chassis,
            wheel_pulse,
            TASK5_MOVE_SPEED_RPM,
            TASK5_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK5_TEMPORARY_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task5_linear_duration_ms(task->chassis,
                                                       distance_mm);
    return TASK5_TEMPORARY_RING_OK;
}

static task5_temporary_ring_result_t task5_rotate_clockwise_90(
    task5_temporary_ring_t *task)
{
    uint32_t pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];

    if (task == NULL)
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }
    pulse = task5_rotation_pulses(task->chassis,
                                  TASK5_TEMPORARY_RING_ROTATE_DEG);
    if (pulse == 0U)
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }

    wheel_pulse[MECANUM_WHEEL_FL] =
        -(int32_t)pulse * TASK5_ROTATE_CLOCKWISE_SIGN;
    wheel_pulse[MECANUM_WHEEL_FR] =
        (int32_t)pulse * TASK5_ROTATE_CLOCKWISE_SIGN;
    wheel_pulse[MECANUM_WHEEL_RL] =
        -(int32_t)pulse * TASK5_ROTATE_CLOCKWISE_SIGN;
    wheel_pulse[MECANUM_WHEEL_RR] =
        (int32_t)pulse * TASK5_ROTATE_CLOCKWISE_SIGN;
    if (mecanum_chassis_set_position_increment(
            task->chassis,
            wheel_pulse,
            TASK5_MOVE_SPEED_RPM,
            TASK5_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK5_TEMPORARY_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task5_rotation_duration_ms(
        task->chassis,
        TASK5_TEMPORARY_RING_ROTATE_DEG);
    return TASK5_TEMPORARY_RING_OK;
}

static uint16_t task5_backward_distance_mm(uint8_t ring)
{
    switch (ring)
    {
        case 1U:
            return TASK5_TEMPORARY_RING_BACKWARD_RING_1_MM;
        case 2U:
            return TASK5_TEMPORARY_RING_BACKWARD_RING_2_MM;
        case 3U:
            return TASK5_TEMPORARY_RING_BACKWARD_RING_3_MM;
        default:
            return 0U;
    }
}

static int16_t task5_ring_offset_mm(uint8_t ring)
{
    if (ring == 1U)
    {
        return (int16_t)TASK5_TEMPORARY_RING_SPACING_MM;
    }
    if (ring == 3U)
    {
        return -(int16_t)TASK5_TEMPORARY_RING_SPACING_MM;
    }
    return 0;
}

static uint8_t task5_target_ring_for_item(
    const task5_temporary_ring_t *task)
{
    if ((task == NULL) ||
        (task->item_index >= TASK5_TEMPORARY_RING_ITEM_COUNT))
    {
        return 0U;
    }
    return task->qr_task.first_batch_position[task->item_index];
}

static task5_temporary_ring_result_t task5_move_to_ring(
    task5_temporary_ring_t *task)
{
    int16_t delta_mm;

    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U) || (task->current_ring < 1U) ||
        (task->current_ring > 3U))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }

    delta_mm = task5_ring_offset_mm(task->target_ring) -
               task5_ring_offset_mm(task->current_ring);
    if (delta_mm == 0)
    {
        return TASK5_TEMPORARY_RING_OK;
    }
    return task5_move_linear(task,
                             (uint16_t)((delta_mm > 0) ?
                                        delta_mm : -delta_mm),
                             (delta_mm > 0) ? 1 : -1,
                             0);
}

static task5_temporary_ring_result_t task5_set_tray_position(
    task5_temporary_ring_t *task)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK5_TEMPORARY_RING_ITEM_COUNT))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }
    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)task->item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK5_TEMPORARY_RING_TRAY_ERROR;
    }
    HAL_Delay(TASK5_TEMPORARY_RING_TRAY_SETTLE_MS);
    return TASK5_TEMPORARY_RING_OK;
}

task5_temporary_ring_result_t task5_temporary_ring_init(
    task5_temporary_ring_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task5_temporary_ring_status_fn write_status)
{
    vision_fine_adjust_config_t config;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
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
        return TASK5_TEMPORARY_RING_VISION_ERROR;
    }
    task->state = TASK5_TEMPORARY_RING_WAIT_TASK;
    task5_status(task,
                 TASK5_TEMPORARY_RING_STATUS_INIT,
                 "task5_temporary_ring_ready");
    return TASK5_TEMPORARY_RING_OK;
}

task5_temporary_ring_result_t task5_temporary_ring_submit_qr(
    task5_temporary_ring_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK5_TEMPORARY_RING_WAIT_TASK))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }
    for (index = 0U; index < TASK5_TEMPORARY_RING_ITEM_COUNT; index++)
    {
        if ((qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U) ||
            (qr_task->second_batch_position[index] < 1U) ||
            (qr_task->second_batch_position[index] > 3U) ||
            (qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U))
        {
            return TASK5_TEMPORARY_RING_INVALID_PARAM;
        }
    }

    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    task->current_ring = qr_task->first_batch_position[
        TASK5_TEMPORARY_RING_ITEM_COUNT - 1U];
    task->target_ring = TASK5_TEMPORARY_RING_CENTER;
    task->task_ready = 1U;
    task->state = TASK5_TEMPORARY_RING_BACKWARD_FROM_RING;
    task5_status(task,
                 TASK5_TEMPORARY_RING_STATUS_MOVE,
                 "task5_temporary_ring_sequence_loaded");
    return TASK5_TEMPORARY_RING_OK;
}

task5_temporary_ring_result_t task5_temporary_ring_step(
    task5_temporary_ring_t *task)
{
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;
    task5_temporary_ring_result_t result;
    uint16_t distance_mm;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK5_TEMPORARY_RING_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK5_TEMPORARY_RING_WAIT_TASK:
            return TASK5_TEMPORARY_RING_WAITING;

        case TASK5_TEMPORARY_RING_BACKWARD_FROM_RING:
            distance_mm = task5_backward_distance_mm(task->current_ring);
            if (distance_mm == 0U)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_INVALID_PARAM,
                                  "task5_backward_distance_invalid");
            }
            task5_status(task,
                         TASK5_TEMPORARY_RING_STATUS_MOVE,
                         "task5_backward_from_ring_start");
            result = task5_move_linear(task, distance_mm, -1, 0);
            if (result != TASK5_TEMPORARY_RING_OK)
            {
                return task5_fail(task, result,
                                  "task5_backward_from_ring_failed");
            }
            task->state = TASK5_TEMPORARY_RING_WAIT_BACKWARD_FROM_RING;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_WAIT_BACKWARD_FROM_RING:
            task5_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK5_TEMPORARY_RING_ROTATE;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_ROTATE:
            task5_status(task,
                         TASK5_TEMPORARY_RING_STATUS_ROTATE,
                         "task5_rotate_clockwise_90_start");
            result = task5_rotate_clockwise_90(task);
            if (result != TASK5_TEMPORARY_RING_OK)
            {
                return task5_fail(task, result,
                                  "task5_rotate_clockwise_90_failed");
            }
            task->state = TASK5_TEMPORARY_RING_WAIT_ROTATE;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_WAIT_ROTATE:
            task5_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK5_TEMPORARY_RING_BACKWARD_TO_AREA;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_BACKWARD_TO_AREA:
            task5_status(task,
                         TASK5_TEMPORARY_RING_STATUS_MOVE,
                         "task5_backward_to_temporary_area_start");
            result = task5_move_linear(
                task,
                TASK5_TEMPORARY_RING_BACKWARD_TO_AREA_MM,
                -1,
                0);
            if (result != TASK5_TEMPORARY_RING_OK)
            {
                return task5_fail(task, result,
                                  "task5_backward_to_temporary_area_failed");
            }
            task->state = TASK5_TEMPORARY_RING_WAIT_BACKWARD_TO_AREA;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_WAIT_BACKWARD_TO_AREA:
            task5_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->current_ring = TASK5_TEMPORARY_RING_CENTER;
            task->state = TASK5_TEMPORARY_RING_PREPARE_ITEM;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_PREPARE_ITEM:
            /* 第一批物料按任务3的放置顺序转移到对应暂存环。 */
            task->target_ring = task5_target_ring_for_item(task);
            if ((task->target_ring < 1U) || (task->target_ring > 3U))
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_INVALID_PARAM,
                                  "task5_target_ring_invalid");
            }
            result = task5_move_to_ring(task);
            if (result != TASK5_TEMPORARY_RING_OK)
            {
                return task5_fail(task, result,
                                  "task5_move_to_target_ring_failed");
            }
            if (task->target_ring != task->current_ring)
            {
                task5_wait_motion_done(task);
                (void)mecanum_chassis_stop(task->chassis);
                task->current_ring = task->target_ring;
            }
            task5_status(task,
                         TASK5_TEMPORARY_RING_STATUS_PREPARE,
                         "task5_ground_place_prepare_start");
            /* 仅第一件需要下降；后两件沿用前一件的低位放置姿态。 */
            if (task->item_index == 0U)
            {
                arm_result = arm_controller_prepare_ground_place_aligned(
                    task->arm);
                if (arm_result != ARM_CONTROLLER_OK)
                {
                    return task5_fail(task,
                                      TASK5_TEMPORARY_RING_ARM_ERROR,
                                      "task5_ground_place_prepare_failed");
                }
            }
            if (vision_fine_adjust_start(
                    task->fine_adjust,
                    VISION_UART_MODE_TARGET_RING,
                    task->target_ring) != VISION_FINE_ADJUST_OK)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_VISION_ERROR,
                                  "task5_target_ring_mode_start_failed");
            }
            task->state = TASK5_TEMPORARY_RING_ALIGN;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                task->state = TASK5_TEMPORARY_RING_PLACE_ITEM;
                task5_status(task,
                             TASK5_TEMPORARY_RING_STATUS_FINE_ADJUST,
                             "task5_fine_adjust_done");
                return TASK5_TEMPORARY_RING_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_VISION_ERROR,
                                  "task5_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_CHASSIS_ERROR,
                                  "task5_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_ARM_ERROR,
                                  "task5_fine_adjust_failed");
            }
            return TASK5_TEMPORARY_RING_WAITING;

        case TASK5_TEMPORARY_RING_PLACE_ITEM:
            result = task5_set_tray_position(task);
            if (result != TASK5_TEMPORARY_RING_OK)
            {
                return task5_fail(task, result,
                                  "task5_tray_position_failed");
            }
            task5_status(task,
                         TASK5_TEMPORARY_RING_STATUS_PLACE,
                         "task5_pick_from_tray_and_place_start");
            /* 任务五放到地面靶环，使用地面靶环放置高度 -7500 度；
             * 码垛高度 -4663.7 度只由任务九使用。 */
            if (task->item_index + 1U < TASK5_TEMPORARY_RING_ITEM_COUNT)
            {
                arm_result = arm_controller_place_ring_item_from_tray_keep_low(
                    task->arm);
            }
            else
            {
                /* 最后一件仍抬到最高点，衔接任务6跨区取料。 */
                arm_result = arm_controller_place_ring_item_from_tray(
                    task->arm);
            }
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_ARM_ERROR,
                                  "task5_pick_from_tray_failed");
            }
            if (screen_record_successful_place(task->screen) != SCREEN_OK)
            {
                return task5_fail(task,
                                  TASK5_TEMPORARY_RING_SCREEN_ERROR,
                                  "task5_place_count_display_failed");
            }
            task->item_index++;
            if (task->item_index >= TASK5_TEMPORARY_RING_ITEM_COUNT)
            {
                (void)mecanum_chassis_stop(task->chassis);
                task->state = TASK5_TEMPORARY_RING_DONE;
                task5_status(task,
                             TASK5_TEMPORARY_RING_STATUS_DONE,
                             "task5_three_items_placed");
                return TASK5_TEMPORARY_RING_OK;
            }
            task->state = TASK5_TEMPORARY_RING_PREPARE_ITEM;
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_DONE:
            return TASK5_TEMPORARY_RING_OK;

        case TASK5_TEMPORARY_RING_FAILED:
        default:
            return TASK5_TEMPORARY_RING_ARM_ERROR;
    }
}

task5_temporary_ring_state_t task5_temporary_ring_get_state(
    const task5_temporary_ring_t *task)
{
    return (task == NULL) ?
           TASK5_TEMPORARY_RING_FAILED : task->state;
}
