#include "task1_start_qr.h"

#include "chassis_motion_closed_loop.h"

#include <stdio.h>
#include <string.h>

static void task1_status(task1_start_qr_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task1_start_qr_result_t task1_fail(task1_start_qr_t *task,
                                          task1_start_qr_result_t result,
                                          const char *event)
{
    if (task != NULL)
    {
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK1_START_QR_FAILED;
        task1_status(task, TASK1_START_QR_STATUS_ERROR, event);
    }
    return result;
}

static task1_start_qr_result_t task1_move(task1_start_qr_t *task,
                                          uint16_t distance_mm,
                                          int8_t body_x_sign,
                                          int8_t body_y_sign)
{
    if ((task == NULL) || (task->chassis == NULL) ||
        (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK1_START_QR_INVALID_PARAM;
    }

    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK1_START_QR_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK1_START_QR_OK;
}

static void task1_wait_move_done(const task1_start_qr_t *task)
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

static void task1_stop_and_settle(task1_start_qr_t *task)
{
    if ((task != NULL) && (task->chassis != NULL))
    {
        /* 闭环直线运动返回前已经确认底盘稳定停止，不再额外等待 80 ms。 */
        (void)mecanum_chassis_stop(task->chassis);
    }
}

task1_start_qr_result_t task1_start_qr_init(
    task1_start_qr_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    screen_t *screen,
    task1_start_qr_status_fn write_status)
{
    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (screen == NULL))
    {
        return TASK1_START_QR_INVALID_PARAM;
    }

    memset(task, 0, sizeof(*task));
    task->arm = arm;
    task->chassis = chassis;
    task->screen = screen;
    task->write_status = write_status;
    task->start_zone = START_ZONE_1;
    task->state = TASK1_START_QR_WAIT_START;
    task1_status(task, TASK1_START_QR_STATUS_INIT, "task1_ready");
    task1_status(task,
                 TASK1_START_QR_STATUS_WAIT_START,
                 "task1_waiting_start");
    return TASK1_START_QR_OK;
}

task1_start_qr_result_t task1_start_qr_set_start_zone(
    task1_start_qr_t *task, start_zone_t start_zone)
{
    if ((task == NULL) ||
        ((start_zone != START_ZONE_1) && (start_zone != START_ZONE_2)) ||
        (task->state != TASK1_START_QR_WAIT_START) ||
        (task->start_requested != 0U))
    {
        return TASK1_START_QR_INVALID_PARAM;
    }
    task->start_zone = start_zone;
    return TASK1_START_QR_OK;
}

void task1_start_qr_request_start(task1_start_qr_t *task)
{
    if ((task != NULL) && (task->state == TASK1_START_QR_WAIT_START))
    {
        task->start_requested = 1U;
    }
}

task1_start_qr_result_t task1_start_qr_step(task1_start_qr_t *task)
{
    QR_Status_t qr_status;
    char event[64];
    int length;
    task1_start_qr_result_t result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->screen == NULL))
    {
        return TASK1_START_QR_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK1_START_QR_WAIT_START:
            if (task->start_requested == 0U)
            {
                return TASK1_START_QR_WAITING;
            }
            task->start_requested = 0U;
            task->state = TASK1_START_QR_FORWARD_FIRST;
            task1_status(task, TASK1_START_QR_STATUS_MOVE,
                         "task1_forward_100mm_start");
            return TASK1_START_QR_OK;

        case TASK1_START_QR_FORWARD_FIRST:
            result = task1_move(task, TASK1_START_QR_FORWARD_FIRST_MM, 1, 0);
            if (result != TASK1_START_QR_OK)
            {
                return task1_fail(task, result, "task1_forward_100mm_failed");
            }
            task1_wait_move_done(task);
            task1_stop_and_settle(task);
            task->state = TASK1_START_QR_LATERAL_AND_ZERO;
            task1_status(task, TASK1_START_QR_STATUS_MOVE,
                         (task->start_zone == START_ZONE_2) ?
                         "task1_right_1050mm_start" : "task1_left_1050mm_start");
            return TASK1_START_QR_OK;

        case TASK1_START_QR_LATERAL_AND_ZERO:
            /* 起点区2位于场地下侧，车头同样朝左，向车体右侧横移至二维码。 */
            result = task1_move(task,
                                (task->start_zone == START_ZONE_2) ?
                                TASK1_START_QR_RIGHT_MM : TASK1_START_QR_LEFT_MM,
                                0,
                                (task->start_zone == START_ZONE_2) ? -1 : 1);
            if (result != TASK1_START_QR_OK)
            {
                return task1_fail(task, result,
                                  (task->start_zone == START_ZONE_2) ?
                                  "task1_right_1050mm_failed" : "task1_left_1050mm_failed");
            }
            task1_status(task, TASK1_START_QR_STATUS_ARM_ZERO,
                         (task->start_zone == START_ZONE_2) ?
                         "task1_arm_zero_during_right_start" : "task1_arm_zero_during_left_start");
            if (arm_controller_zero(task->arm) != ARM_CONTROLLER_OK)
            {
                return task1_fail(task,
                                  TASK1_START_QR_ARM_ERROR,
                                  "task1_arm_zero_failed");
            }
            task1_wait_move_done(task);
            task1_stop_and_settle(task);
            task1_status(task, TASK1_START_QR_STATUS_ARM_ZERO,
                         (task->start_zone == START_ZONE_2) ?
                         "task1_arm_zero_during_right_done" : "task1_arm_zero_during_left_done");
            /* 任务 2 首次平台取料需要面向平台；先确认水平轴收回 0 度，
             * 再旋转 Z 轴且不下降，避免任务 2 再次重复旋转。 */
            task1_status(task, TASK1_START_QR_STATUS_ARM_ZERO,
                         "task1_preposition_platform_z_start");
            if (arm_controller_preposition_platform_z(task->arm) !=
                ARM_CONTROLLER_OK)
            {
                return task1_fail(task,
                                  TASK1_START_QR_ARM_ERROR,
                                  "task1_preposition_platform_z_failed");
            }
            task1_status(task, TASK1_START_QR_STATUS_ARM_ZERO,
                         "task1_preposition_platform_z_done");
            task->state = TASK1_START_QR_FORWARD_QR;
            return TASK1_START_QR_OK;

        case TASK1_START_QR_FORWARD_QR:
            task1_status(task, TASK1_START_QR_STATUS_MOVE,
                         "task1_forward_150mm_start");
            /* 清掉上一轮残留码，但保留本次驶向二维码过程中的新帧。 */
            QR_ClearTask();
            QR_ClearError();
            result = task1_move(task, TASK1_START_QR_FORWARD_QR_MM, 1, 0);
            if (result != TASK1_START_QR_OK)
            {
                return task1_fail(task, result, "task1_forward_150mm_failed");
            }
            task1_wait_move_done(task);
            task1_stop_and_settle(task);
            task->state = TASK1_START_QR_WAIT_QR;
            task1_status(task, TASK1_START_QR_STATUS_QR,
                         "task1_qr_receive_enabled");
            return TASK1_START_QR_OK;

        case TASK1_START_QR_WAIT_QR:
            qr_status = QR_TakeTask(&task->qr_task);
            if (qr_status == QR_STATUS_NOT_READY)
            {
                if (QR_IsErrorReady() != 0U)
                {
                    qr_status = QR_GetError();
                    QR_ClearError();
                    length = snprintf(event,
                                      sizeof(event),
                                      "task1_qr_error=%u",
                                      (unsigned int)qr_status);
                    if ((length > 0) &&
                        ((size_t)length < sizeof(event)))
                    {
                        task1_status(task, TASK1_START_QR_STATUS_ERROR, event);
                    }
                }
                return TASK1_START_QR_WAITING;
            }
            if (qr_status != QR_STATUS_OK)
            {
                return task1_fail(task,
                                  TASK1_START_QR_QR_ERROR,
                                  "task1_qr_take_failed");
            }
            task->state = TASK1_START_QR_PROCESS_QR;
            return TASK1_START_QR_OK;

        case TASK1_START_QR_PROCESS_QR:
            if (screen_show_task_code(task->screen, task->qr_task.raw_code) !=
                SCREEN_OK)
            {
                return task1_fail(task,
                                  TASK1_START_QR_SCREEN_ERROR,
                                  "task1_task_code_screen_failed");
            }
            if (task_plan_build(&task->qr_task, &task->plan) == 0U)
            {
                return task1_fail(task,
                                  TASK1_START_QR_PLAN_ERROR,
                                  "task1_task_code_plan_failed");
            }
            task->task_ready = 1U;
            task->state = TASK1_START_QR_DONE;
            task1_status(task,
                         TASK1_START_QR_STATUS_TASK_READY,
                         "task1_task_code_processed");
            return TASK1_START_QR_OK;

        case TASK1_START_QR_DONE:
            return TASK1_START_QR_OK;

        case TASK1_START_QR_FAILED:
        default:
            return TASK1_START_QR_QR_ERROR;
    }
}

task1_start_qr_state_t task1_start_qr_get_state(
    const task1_start_qr_t *task)
{
    return (task == NULL) ? TASK1_START_QR_FAILED : task->state;
}

uint8_t task1_start_qr_get_task(const task1_start_qr_t *task,
                                QR_TaskCode_t *qr_task)
{
    if ((task == NULL) || (qr_task == NULL) || (task->task_ready == 0U))
    {
        return 0U;
    }
    memcpy(qr_task, &task->qr_task, sizeof(*qr_task));
    return 1U;
}

uint8_t task1_start_qr_get_plan(const task1_start_qr_t *task,
                                task_execution_plan_t *plan)
{
    if ((task == NULL) || (plan == NULL) || (task->task_ready == 0U))
    {
        return 0U;
    }
    memcpy(plan, &task->plan, sizeof(*plan));
    return 1U;
}
