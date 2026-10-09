#include "task4_ring_return.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证等待时间按实际速度计算。 */
#define TASK4_MOVE_SPEED_RPM         60U
#define TASK4_MOVE_ACCELERATION      240U
#define TASK4_MOVE_SETTLE_MS        600U
#define TASK4_PRE_ALIGN_RIGHT_MM      10U
#define TASK4_PRE_ALIGN_CLOCKWISE_DEG 0.3f

static void task4_status(task4_ring_return_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task4_ring_return_result_t task4_fail(
    task4_ring_return_t *task,
    task4_ring_return_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK4_RING_RETURN_FAILED;
        task4_status(task, TASK4_RING_RETURN_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task4_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task4_move_duration_ms(const mecanum_chassis_t *chassis,
                                       uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK4_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK4_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK4_MOVE_SETTLE_MS;
}

static void task4_wait_move_done(const task4_ring_return_t *task)
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

static int16_t task4_ring_offset_mm(uint8_t ring)
{
    if (ring == 1U)
    {
        return (int16_t)TASK4_RING_RETURN_SPACING_MM;
    }
    if (ring == 3U)
    {
        return -(int16_t)TASK4_RING_RETURN_SPACING_MM;
    }
    return 0;
}

static uint8_t task4_target_ring_for_item(
    const task4_ring_return_t *task,
    uint8_t item_index)
{
    if ((task == NULL) || (item_index >= TASK4_RING_RETURN_ITEM_COUNT))
    {
        return 0U;
    }
    return (task->use_placement_order != 0U) ?
           task->qr_task.first_batch_position[item_index] :
           task->qr_task.second_batch_position[item_index];
}

static uint8_t task4_target_color_for_item(
    const task4_ring_return_t *task,
    uint8_t item_index)
{
    if ((task == NULL) || (item_index >= TASK4_RING_RETURN_ITEM_COUNT))
    {
        return 0U;
    }
    return (task->use_placement_order != 0U) ?
           task->qr_task.first_batch_color[item_index] :
           task->qr_task.second_batch_color[item_index];
}

static task4_ring_return_result_t task4_move_linear(
    task4_ring_return_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign)
{
    uint32_t pulses_per_mm;
    int32_t body_x_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];

    if ((task == NULL) || (distance_mm == 0U) || (body_x_sign == 0))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }
    pulses_per_mm = task4_pulses_per_mm(task->chassis);
    if (pulses_per_mm == 0U)
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    body_x_pulse = (int32_t)pulses_per_mm * (int32_t)distance_mm *
                   (int32_t)body_x_sign;
    wheel_pulse[MECANUM_WHEEL_FL] = body_x_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] = body_x_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] = body_x_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] = body_x_pulse;

    if (mecanum_chassis_set_position_increment(
            task->chassis,
            wheel_pulse,
            TASK4_MOVE_SPEED_RPM,
            TASK4_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK4_RING_RETURN_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task4_move_duration_ms(task->chassis,
                                                     distance_mm);
    return TASK4_RING_RETURN_OK;
}

static task4_ring_return_result_t task4_apply_pre_align_offset(
    task4_ring_return_t *task)
{
    uint32_t pulses_per_mm;
    int32_t body_y_pulse;
    int32_t body_yaw_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];
    float wheel_distance_m;

    if ((task == NULL) || (task->chassis == NULL) ||
        (task->chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }
    pulses_per_mm = task4_pulses_per_mm(task->chassis);
    if (pulses_per_mm == 0U)
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    /* 车体右移为负Y；顺时针旋转为负航向。 */
    body_y_pulse = -(int32_t)(pulses_per_mm *
                              TASK4_PRE_ALIGN_RIGHT_MM);
    wheel_distance_m = (task->chassis->config.half_length_m +
                        task->chassis->config.half_width_m) *
                       (TASK4_PRE_ALIGN_CLOCKWISE_DEG *
                        3.14159265359f / 180.0f);
    body_yaw_pulse = -(int32_t)((wheel_distance_m /
                                (2.0f * 3.14159265359f *
                                 task->chassis->config.wheel_radius_m) *
                                (float)MECANUM_POSITION_PULSES_PER_REV) +
                               0.5f);

    wheel_pulse[MECANUM_WHEEL_FL] = -body_y_pulse - body_yaw_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] = body_y_pulse + body_yaw_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] = body_y_pulse - body_yaw_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] = -body_y_pulse + body_yaw_pulse;
    if (mecanum_chassis_set_position_increment(
            task->chassis,
            wheel_pulse,
            TASK4_MOVE_SPEED_RPM,
            TASK4_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK4_RING_RETURN_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task4_move_duration_ms(
        task->chassis,
        TASK4_PRE_ALIGN_RIGHT_MM);
    return TASK4_RING_RETURN_OK;
}

static task4_ring_return_result_t task4_move_to_ring(
    task4_ring_return_t *task)
{
    int16_t delta_mm;
    task4_ring_return_result_t result;

    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U) || (task->current_ring < 1U) ||
        (task->current_ring > 3U))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    delta_mm = task4_ring_offset_mm(task->target_ring) -
               task4_ring_offset_mm(task->current_ring);
    if (delta_mm == 0)
    {
        return TASK4_RING_RETURN_OK;
    }

    /* 任务3完成180度转向后，靶环轴线对应车体前后：
     * body_x_sign=+1 向后，body_x_sign=-1 向前。 */
    result = task4_move_linear(
        task,
        (uint16_t)((delta_mm > 0) ? delta_mm : -delta_mm),
        (delta_mm > 0) ? 1 : -1);
    return result;
}

static task4_ring_return_result_t task4_set_tray_position(
    task4_ring_return_t *task)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK4_RING_RETURN_ITEM_COUNT))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)task->item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK4_RING_RETURN_TRAY_ERROR;
    }
    HAL_Delay(TASK4_RING_RETURN_TRAY_SETTLE_MS);
    return TASK4_RING_RETURN_OK;
}

task4_ring_return_result_t task4_ring_return_init(
    task4_ring_return_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task4_ring_return_status_fn write_status)
{
    vision_fine_adjust_config_t config;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    memset(task, 0, sizeof(*task));
    task->arm = arm;
    task->chassis = chassis;
    task->fine_adjust = fine_adjust;
    task->tray = tray;
    task->screen = screen;
    task->write_status = write_status;
    task->use_placement_order = 1U;

    vision_fine_adjust_config_default(&config);
    config.timeout_ms = 0U;
    adjust_result = vision_fine_adjust_init(fine_adjust,
                                            vision_uart_get_default(),
                                            chassis,
                                            &config);
    if (adjust_result != VISION_FINE_ADJUST_OK)
    {
        return TASK4_RING_RETURN_VISION_ERROR;
    }

    task->state = TASK4_RING_RETURN_WAIT_TASK;
    task4_status(task,
                 TASK4_RING_RETURN_STATUS_INIT,
                 "task4_ring_return_ready");
    return TASK4_RING_RETURN_OK;
}

task4_ring_return_result_t task4_ring_return_submit_qr(
    task4_ring_return_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK4_RING_RETURN_WAIT_TASK))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }
    for (index = 0U; index < TASK4_RING_RETURN_ITEM_COUNT; index++)
    {
        if ((qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U) ||
            (qr_task->second_batch_position[index] < 1U) ||
            (qr_task->second_batch_position[index] > 3U) ||
            (qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U))
        {
            return TASK4_RING_RETURN_INVALID_PARAM;
        }
    }

    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    /* 任务3最后一件物料所在靶环就是任务4的当前位置。 */
    task->current_ring = qr_task->first_batch_position[
        TASK4_RING_RETURN_ITEM_COUNT - 1U];
    task->target_ring = task->current_ring;
    task->task_ready = 1U;
    task->state = TASK4_RING_RETURN_MOVE_TO_ITEM;
    task4_status(task,
                 TASK4_RING_RETURN_STATUS_MOVE,
                 "task4_ring_return_sequence_loaded");
    return TASK4_RING_RETURN_OK;
}

task4_ring_return_result_t task4_ring_return_step(
    task4_ring_return_t *task)
{
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;
    task4_ring_return_result_t result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK4_RING_RETURN_WAIT_TASK:
            return TASK4_RING_RETURN_WAITING;

        case TASK4_RING_RETURN_MOVE_TO_ITEM:
            task->target_ring = task4_target_ring_for_item(
                task,
                task->item_index);
            /* 任务4按任务3的放置顺序取回，任务8可复用第二批目标顺序。 */
            result = task4_set_tray_position(task);
            if (result != TASK4_RING_RETURN_OK)
            {
                return task4_fail(task, result,
                                  "task4_tray_position_failed");
            }
            task4_status(task,
                         TASK4_RING_RETURN_STATUS_MOVE,
                         "task4_move_to_ring_start");
            result = task4_move_to_ring(task);
            if (result != TASK4_RING_RETURN_OK)
            {
                return task4_fail(task, result,
                                  "task4_move_to_ring_failed");
            }
            if (task->target_ring == task->current_ring)
            {
                task->state = TASK4_RING_RETURN_PREPARE_ITEM;
            }
            else
            {
                task->state = TASK4_RING_RETURN_WAIT_MOVE;
            }
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_WAIT_MOVE:
            task4_wait_move_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->current_ring = task->target_ring;
            task->state = TASK4_RING_RETURN_PREPARE_ITEM;
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_PREPARE_ITEM:
            if (task->use_placement_order != 0U)
            {
                result = task4_apply_pre_align_offset(task);
                if (result != TASK4_RING_RETURN_OK)
                {
                    return task4_fail(task,
                                      result,
                                      "task4_pre_align_offset_failed");
                }
                task4_wait_move_done(task);
                (void)mecanum_chassis_stop(task->chassis);
            }
            task4_status(task,
                         TASK4_RING_RETURN_STATUS_PREPARE,
                         "task4_ground_pick_prepare_start");
            arm_result = arm_controller_prepare_ground_pick_aligned(
                task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_ARM_ERROR,
                                  "task4_ground_pick_prepare_failed");
            }
            if (vision_fine_adjust_start(
                    task->fine_adjust,
                    VISION_UART_MODE_MATERIAL,
                    task4_target_color_for_item(task,
                                                task->item_index)) !=
                VISION_FINE_ADJUST_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_VISION_ERROR,
                                  "task4_material_mode_start_failed");
            }
            task->state = TASK4_RING_RETURN_ALIGN;
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                task->state = TASK4_RING_RETURN_PICK_AND_RETURN;
                task4_status(task,
                             TASK4_RING_RETURN_STATUS_FINE_ADJUST,
                             "task4_fine_adjust_done");
                return TASK4_RING_RETURN_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_VISION_ERROR,
                                  "task4_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_CHASSIS_ERROR,
                                  "task4_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_ARM_ERROR,
                                  "task4_fine_adjust_failed");
            }
            return TASK4_RING_RETURN_WAITING;

        case TASK4_RING_RETURN_PICK_AND_RETURN:
            task4_status(task,
                         TASK4_RING_RETURN_STATUS_PICK,
                         "task4_ground_pick_start");
            arm_result = arm_controller_ground_pick_to_tray_and_return(
                task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_ARM_ERROR,
                                  "task4_ground_pick_to_tray_failed");
            }
            if (screen_record_successful_pick(task->screen) != SCREEN_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_SCREEN_ERROR,
                                  "task4_pick_count_display_failed");
            }
            task4_status(task,
                         TASK4_RING_RETURN_STATUS_PLACE,
                         "task4_place_to_tray_done");
            task->item_index++;
            if (task->item_index >= TASK4_RING_RETURN_ITEM_COUNT)
            {
                (void)mecanum_chassis_stop(task->chassis);
                task->state = TASK4_RING_RETURN_DONE;
                task4_status(task,
                             TASK4_RING_RETURN_STATUS_DONE,
                             "task4_three_items_returned");
                return TASK4_RING_RETURN_OK;
            }
            task->state = TASK4_RING_RETURN_MOVE_TO_ITEM;
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_DONE:
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_FAILED:
        default:
            return TASK4_RING_RETURN_ARM_ERROR;
    }
}

task4_ring_return_state_t task4_ring_return_get_state(
    const task4_ring_return_t *task)
{
    return (task == NULL) ?
           TASK4_RING_RETURN_FAILED : task->state;
}
