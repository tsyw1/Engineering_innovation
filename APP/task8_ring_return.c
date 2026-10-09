#include "task8_ring_return.h"

#include <string.h>

static task8_ring_return_result_t task8_from_task4_result(
    task4_ring_return_result_t result)
{
    switch (result)
    {
        case TASK4_RING_RETURN_OK:
            return TASK8_RING_RETURN_OK;
        case TASK4_RING_RETURN_WAITING:
            return TASK8_RING_RETURN_WAITING;
        case TASK4_RING_RETURN_INVALID_PARAM:
            return TASK8_RING_RETURN_INVALID_PARAM;
        case TASK4_RING_RETURN_CHASSIS_ERROR:
            return TASK8_RING_RETURN_CHASSIS_ERROR;
        case TASK4_RING_RETURN_ARM_ERROR:
            return TASK8_RING_RETURN_ARM_ERROR;
        case TASK4_RING_RETURN_VISION_ERROR:
            return TASK8_RING_RETURN_VISION_ERROR;
        case TASK4_RING_RETURN_TRAY_ERROR:
            return TASK8_RING_RETURN_TRAY_ERROR;
        case TASK4_RING_RETURN_SCREEN_ERROR:
            return TASK8_RING_RETURN_SCREEN_ERROR;
        default:
            return TASK8_RING_RETURN_ARM_ERROR;
    }
}

task8_ring_return_result_t task8_ring_return_init(
    task8_ring_return_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task4_ring_return_status_fn write_status)
{
    task4_ring_return_result_t result;

    if (task == NULL)
    {
        return TASK8_RING_RETURN_INVALID_PARAM;
    }
    memset(task, 0, sizeof(*task));
    result = task4_ring_return_init(&task->implementation,
                                    arm,
                                    chassis,
                                    fine_adjust,
                                    tray,
                                    screen,
                                    write_status);
    if (result != TASK4_RING_RETURN_OK)
    {
        task->state = TASK8_RING_RETURN_FAILED;
        return task8_from_task4_result(result);
    }
    /* 任务8取回任务7放置的第二批物料，保持原有第二批目标顺序。 */
    task->implementation.use_placement_order = 0U;
    task->state = TASK8_RING_RETURN_WAIT_TASK;
    return TASK8_RING_RETURN_OK;
}

task8_ring_return_result_t task8_ring_return_submit_qr(
    task8_ring_return_t *task,
    const QR_TaskCode_t *qr_task)
{
    task4_ring_return_result_t result;

    if ((task == NULL) || (qr_task == NULL))
    {
        return TASK8_RING_RETURN_INVALID_PARAM;
    }
    result = task4_ring_return_submit_qr(&task->implementation, qr_task);
    if (result != TASK4_RING_RETURN_OK)
    {
        return task8_from_task4_result(result);
    }

    /* 任务7按 second_batch_position 放置，任务8从最后一个环位开始。 */
    task->implementation.current_ring = qr_task->second_batch_position[
        QR_BATCH_ITEM_COUNT - 1U];
    task->state = TASK8_RING_RETURN_RUNNING;
    return TASK8_RING_RETURN_OK;
}

task8_ring_return_result_t task8_ring_return_step(
    task8_ring_return_t *task)
{
    task4_ring_return_result_t result;
    task4_ring_return_state_t implementation_state;

    if (task == NULL)
    {
        return TASK8_RING_RETURN_INVALID_PARAM;
    }
    if (task->state == TASK8_RING_RETURN_WAIT_TASK)
    {
        return TASK8_RING_RETURN_WAITING;
    }
    if (task->state == TASK8_RING_RETURN_DONE)
    {
        return TASK8_RING_RETURN_OK;
    }
    if (task->state == TASK8_RING_RETURN_FAILED)
    {
        return TASK8_RING_RETURN_ARM_ERROR;
    }

    result = task4_ring_return_step(&task->implementation);
    implementation_state = task4_ring_return_get_state(
        &task->implementation);
    if (implementation_state == TASK4_RING_RETURN_DONE)
    {
        /* 任务9从物料盘最高点开始，最后一次取料后逆时针回盘。 */
        if (arm_controller_return_to_tray_direction(
                task->implementation.arm) != ARM_CONTROLLER_OK)
        {
            task->state = TASK8_RING_RETURN_FAILED;
            return TASK8_RING_RETURN_ARM_ERROR;
        }
        task->state = TASK8_RING_RETURN_DONE;
    }
    else if (implementation_state == TASK4_RING_RETURN_FAILED)
    {
        task->state = TASK8_RING_RETURN_FAILED;
    }
    return task8_from_task4_result(result);
}

task8_ring_return_state_t task8_ring_return_get_state(
    const task8_ring_return_t *task)
{
    return (task == NULL) ?
           TASK8_RING_RETURN_FAILED : task->state;
}
