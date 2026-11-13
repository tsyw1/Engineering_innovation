#include "task4_ring_return.h"

#include <string.h>

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

static task4_ring_return_result_t task4_select_target_ring(
    task4_ring_return_t *task)
{
    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }
    /* 任务4和任务8均固定在2号环，目标环位交给水平轴。 */
    task->current_ring = TASK4_RING_RETURN_CENTER;
    return TASK4_RING_RETURN_OK;
}

static task4_ring_return_result_t task4_start_tray_position_at(
    task4_ring_return_t *task,
    uint8_t item_index)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (item_index >= TASK4_RING_RETURN_ITEM_COUNT))
    {
        return TASK4_RING_RETURN_INVALID_PARAM;
    }

    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK4_RING_RETURN_TRAY_ERROR;
    }
    return TASK4_RING_RETURN_OK;
}

static task4_ring_return_result_t task4_start_tray_position(
    task4_ring_return_t *task)
{
    return task4_start_tray_position_at(
        task,
        (task != NULL) ? task->item_index : TASK4_RING_RETURN_ITEM_COUNT);
}

static void task4_finish_tray_position(task4_ring_return_t *task)
{
    if (task == NULL)
    {
        return;
    }

    /* 首件料盘定位后，按原料区取料流程等待舵机稳定。 */
    HAL_Delay(TASK4_RING_RETURN_TRAY_SETTLE_MS);
}

static arm_controller_result_t task4_start_next_tray_position_callback(
    void *context)
{
    task4_ring_return_t *task = (task4_ring_return_t *)context;
    task4_ring_return_result_t result;
    uint8_t next_item_index;
    uint8_t next_target_ring;

    if ((task == NULL) ||
        ((task->item_index + 1U) >= TASK4_RING_RETURN_ITEM_COUNT))
    {
        return ARM_CONTROLLER_OK;
    }

    next_item_index = (uint8_t)(task->item_index + 1U);
    next_target_ring = task4_target_ring_for_item(task, next_item_index);
    /* 回调发生在升降轴开始回收、水平轴尚未收回时；这里只启动料盘，
     * 目标环 Z 轴转向留到下一件准备阶段，避免低位/伸出状态转向。 */
    if ((next_target_ring < 1U) || (next_target_ring > 3U) ||
        (arm_controller_set_target_ring(task->arm, next_target_ring) !=
         ARM_CONTROLLER_OK))
    {
        return ARM_CONTROLLER_MOTOR_ERROR;
    }

    result = task4_start_tray_position_at(task, next_item_index);
    return (result == TASK4_RING_RETURN_OK) ?
           ARM_CONTROLLER_OK : ARM_CONTROLLER_MOTOR_ERROR;
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
    /* 地面取料期间预置第1工位，任务8复用相同流程。 */
    if (task4_start_tray_position(task) != TASK4_RING_RETURN_OK)
    {
        return task4_fail(task,
                          TASK4_RING_RETURN_TRAY_ERROR,
                          "task4_tray_position_failed");
    }
    task4_finish_tray_position(task);
    /* 任务3结束后底盘仍停在2号环。 */
    task->current_ring = TASK4_RING_RETURN_CENTER;
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
            result = task4_select_target_ring(task);
            if (result != TASK4_RING_RETURN_OK)
            {
                return task4_fail(task, result,
                                  "task4_target_ring_invalid");
            }
            task->state = TASK4_RING_RETURN_PREPARE_ITEM;
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_WAIT_MOVE:
            /* 兼容旧状态：当前版本不再执行环间或预对准移车。 */
            task->current_ring = TASK4_RING_RETURN_CENTER;
            task->state = TASK4_RING_RETURN_PREPARE_ITEM;
            return TASK4_RING_RETURN_OK;

        case TASK4_RING_RETURN_PREPARE_ITEM:
            task4_status(task,
                         TASK4_RING_RETURN_STATUS_PREPARE,
                         "task4_ground_pick_prepare_start");
            if (arm_controller_set_target_ring(task->arm,
                                               task->target_ring) !=
                ARM_CONTROLLER_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_ARM_ERROR,
                                  "task4_set_target_ring_failed");
            }
            arm_result = (task->item_index == 0U) ?
                         arm_controller_prepare_ground_pick(task->arm) :
                         arm_controller_prepare_ground_pick_from_tray(
                             task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task4_fail(task,
                                  TASK4_RING_RETURN_ARM_ERROR,
                                  "task4_ground_pick_prepare_failed");
            }
            /* 任务4和任务8均固定底盘，按二维码环位直接取回。 */
            task->state = TASK4_RING_RETURN_PICK_AND_RETURN;
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
            /* 开爪稳定 50 ms 后，回收升降轴的同时启动下一工位料盘；
             * 下一件准备阶段再按目标环安全转向。 */
            arm_result = arm_controller_ground_pick_to_tray_with_callback(
                task->arm,
                task4_start_next_tray_position_callback,
                task);
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
                /* 最后一件同样保持最高点、水平轴0度，不再转向中心环。 */
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
