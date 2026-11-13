#include "task10_return_home.h"

#include "chassis_motion_closed_loop.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证等待时间按实际速度计算。 */
#define TASK10_MOVE_SPEED_RPM          60U
#define TASK10_MOVE_ACCELERATION       240U
#define TASK10_MOVE_SETTLE_MS         400U

static void task10_status(task10_return_home_t *task,
                          uint8_t status,
                          const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task10_return_home_result_t task10_fail(
    task10_return_home_t *task,
    task10_return_home_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK10_RETURN_HOME_FAILED;
        task10_status(task, TASK10_RETURN_HOME_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task10_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task10_linear_duration_ms(const mecanum_chassis_t *chassis,
                                          uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK10_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK10_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK10_MOVE_SETTLE_MS;
}

static void task10_wait_motion_done(const task10_return_home_t *task)
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

static task10_return_home_result_t task10_move_linear(
    task10_return_home_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    if ((task == NULL) || (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK10_RETURN_HOME_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK10_RETURN_HOME_OK;
}

static task10_return_home_result_t task10_refresh_statistics(
    task10_return_home_t *task)
{
    if ((task == NULL) || (task->screen == NULL))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    if (screen_show_pick_count(task->screen,
                               task->screen->successful_pick_count) !=
        SCREEN_OK)
    {
        return TASK10_RETURN_HOME_SCREEN_ERROR;
    }
    if (screen_show_place_count(task->screen,
                                task->screen->successful_place_count) !=
        SCREEN_OK)
    {
        return TASK10_RETURN_HOME_SCREEN_ERROR;
    }
    return TASK10_RETURN_HOME_OK;
}

task10_return_home_result_t task10_return_home_init(
    task10_return_home_t *task,
    mecanum_chassis_t *chassis,
    screen_t *screen,
    task10_return_home_status_fn write_status)
{
    if ((task == NULL) || (chassis == NULL) || (screen == NULL))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    memset(task, 0, sizeof(*task));
    task->chassis = chassis;
    task->screen = screen;
    task->write_status = write_status;
    task->start_zone = START_ZONE_1;
    task->state = TASK10_RETURN_HOME_WAIT_TASK;
    task10_status(task,
                  TASK10_RETURN_HOME_STATUS_INIT,
                  "task10_return_home_ready");
    return TASK10_RETURN_HOME_OK;
}

task10_return_home_result_t task10_return_home_set_start_zone(
    task10_return_home_t *task, start_zone_t start_zone)
{
    if ((task == NULL) ||
        ((start_zone != START_ZONE_1) && (start_zone != START_ZONE_2)) ||
        (task->state != TASK10_RETURN_HOME_WAIT_TASK))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    task->start_zone = start_zone;
    return TASK10_RETURN_HOME_OK;
}

task10_return_home_result_t task10_return_home_submit_qr(
    task10_return_home_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) ||
        (qr_task->valid == 0U) ||
        (task->state != TASK10_RETURN_HOME_WAIT_TASK))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if ((qr_task->first_batch_color[index] < 1U) ||
            (qr_task->first_batch_color[index] > 6U) ||
            (qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U) ||
            (qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U))
        {
            return TASK10_RETURN_HOME_INVALID_PARAM;
        }
    }
    /* 任务9码垛期间底盘始终停在暂存区2号环。 */
    task->current_ring = TASK10_RETURN_HOME_CENTER_RING;
    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->task_ready = 1U;
    task->state = TASK10_RETURN_HOME_MOVE_TO_CENTER;
    task10_status(task,
                  TASK10_RETURN_HOME_STATUS_MOVE,
                  "task10_return_home_sequence_loaded");
    return TASK10_RETURN_HOME_OK;
}

task10_return_home_result_t task10_return_home_step(
    task10_return_home_t *task)
{
    task10_return_home_result_t result;
    if ((task == NULL) || (task->chassis == NULL) ||
        (task->screen == NULL))
    {
        return TASK10_RETURN_HOME_INVALID_PARAM;
    }
    switch (task->state)
    {
        case TASK10_RETURN_HOME_WAIT_TASK:
            return TASK10_RETURN_HOME_WAITING;

        case TASK10_RETURN_HOME_MOVE_TO_CENTER:
        case TASK10_RETURN_HOME_WAIT_CENTER_MOVE:
            /* 兼容旧状态：任务9已固定结束在2号环，不再执行回中运动。 */
            task->current_ring = TASK10_RETURN_HOME_CENTER_RING;
            task->state = TASK10_RETURN_HOME_LEFT;
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_LEFT:
            task10_status(task,
                          TASK10_RETURN_HOME_STATUS_MOVE,
                          "task10_left_1870mm_start");
            result = task10_move_linear(task,
                                        TASK10_RETURN_HOME_LEFT_MM,
                                        0,
                                        1);
            if (result != TASK10_RETURN_HOME_OK)
            {
                return task10_fail(task, result,
                                   "task10_left_1870mm_failed");
            }
            task->state = TASK10_RETURN_HOME_WAIT_LEFT;
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_WAIT_LEFT:
            task10_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK10_RETURN_HOME_LONGITUDINAL;
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_LONGITUDINAL:
            task10_status(task,
                          TASK10_RETURN_HOME_STATUS_MOVE,
                          (task->start_zone == START_ZONE_2) ?
                          "task10_forward_950mm_start" : "task10_backward_1020mm_start");
            /* 区1后退1020 mm；启停区2以前向移动950 mm。 */
            result = task10_move_linear(task,
                                        (task->start_zone == START_ZONE_2) ?
                                        TASK10_RETURN_HOME_ZONE2_FORWARD_MM : TASK10_RETURN_HOME_BACKWARD_MM,
                                        (task->start_zone == START_ZONE_2) ? 1 : -1,
                                        0);
            if (result != TASK10_RETURN_HOME_OK)
            {
                return task10_fail(task, result,
                                   (task->start_zone == START_ZONE_2) ?
                                   "task10_forward_950mm_failed" : "task10_backward_1020mm_failed");
            }
            task->state = TASK10_RETURN_HOME_WAIT_LONGITUDINAL;
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_WAIT_LONGITUDINAL:
            task10_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            if (task10_refresh_statistics(task) != TASK10_RETURN_HOME_OK)
            {
                return task10_fail(task,
                                   TASK10_RETURN_HOME_SCREEN_ERROR,
                                   "task10_statistics_refresh_failed");
            }
            (void)mecanum_chassis_enable(task->chassis, false);
            task->state = TASK10_RETURN_HOME_DONE;
            task10_status(task,
                          TASK10_RETURN_HOME_STATUS_DONE,
                          "task10_return_home_done");
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_DONE:
            return TASK10_RETURN_HOME_OK;

        case TASK10_RETURN_HOME_FAILED:
        default:
            return TASK10_RETURN_HOME_CHASSIS_ERROR;
    }
}

task10_return_home_state_t task10_return_home_get_state(
    const task10_return_home_t *task)
{
    return (task == NULL) ?
           TASK10_RETURN_HOME_FAILED : task->state;
}
