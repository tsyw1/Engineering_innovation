#include "task2_platform_pick.h"

#include <stdio.h>
#include <string.h>

/* 与底盘位置模式 0.25 m/s 限速一致，保证等待时间按实际速度计算。 */
#define TASK2_MOVE_SPEED_RPM         60U
#define TASK2_MOVE_ACCELERATION      240U
#define TASK2_MOVE_SETTLE_MS        600U

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
    }
    if (task->chassis != NULL)
    {
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

static uint32_t task2_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task2_move_duration_ms(const mecanum_chassis_t *chassis,
                                       uint16_t distance_mm)
{
    float speed_mps;
    float duration_ms;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK2_MOVE_SETTLE_MS;
    }

    speed_mps = ((float)TASK2_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    duration_ms = ((float)distance_mm / 1000.0f) / speed_mps * 1000.0f;
    return (uint32_t)duration_ms + TASK2_MOVE_SETTLE_MS;
}

static task2_platform_pick_result_t task2_move(
    task2_platform_pick_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    int32_t body_x_pulse;
    int32_t body_y_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];
    uint32_t pulses_per_mm;

    if ((task == NULL) || (task->chassis == NULL) ||
        (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    pulses_per_mm = task2_pulses_per_mm(task->chassis);
    if (pulses_per_mm == 0U)
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
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
            TASK2_MOVE_SPEED_RPM,
            TASK2_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK2_PLATFORM_PICK_CHASSIS_ERROR;
    }

    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task2_move_duration_ms(task->chassis,
                                                     distance_mm);
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

static task2_platform_pick_result_t task2_set_tray_position(
    task2_platform_pick_t *task)
{
    const material_tray_position_t positions[TASK2_PLATFORM_PICK_ITEM_COUNT] =
    {
        MATERIAL_TRAY_POSITION_1,
        MATERIAL_TRAY_POSITION_2,
        MATERIAL_TRAY_POSITION_3
    };

    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK2_PLATFORM_PICK_ITEM_COUNT))
    {
        return TASK2_PLATFORM_PICK_INVALID_PARAM;
    }

    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_TRAY_POSITION,
                 "task2_tray_position_start");
    if (material_tray_servo_set_position(
            task->tray,
            positions[task->item_index]) != MATERIAL_TRAY_SERVO_OK)
    {
        return task2_fail(task,
                          TASK2_PLATFORM_PICK_TRAY_ERROR,
                          "task2_tray_position_failed");
    }
    HAL_Delay(TASK2_PLATFORM_PICK_TRAY_SETTLE_MS);
    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_TRAY_POSITION,
                 "task2_tray_position_done");
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

    task2_status(task,
                 TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                 "task2_platform_prepare_pick_start");
    adjust_result = vision_fine_adjust_prepare_platform_pick_and_start(
        task->fine_adjust,
        task->arm,
        task->target_ids[task->item_index]);
    if (adjust_result != VISION_FINE_ADJUST_OK)
    {
        return task2_fail(task,
                          TASK2_PLATFORM_PICK_ARM_ERROR,
                          "task2_platform_prepare_pick_failed");
    }

    length = snprintf(event,
                      sizeof(event),
                      "task2_item=%u,target_id=%u,fine_adjust_started",
                      (unsigned int)(task->item_index + 1U),
                      (unsigned int)task->target_ids[task->item_index]);
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
            arm_result = arm_controller_pick_and_place(task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
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
                /* 任务3还需要继续使用底盘，正常完成只停车不失能。 */
                vision_fine_adjust_abort(task->fine_adjust);
                (void)mecanum_chassis_stop(task->chassis);
                task2_status(task,
                             TASK2_PLATFORM_PICK_STATUS_DONE,
                             "task2_platform_three_items_done");
                return TASK2_PLATFORM_PICK_OK;
            }

            /* 夹爪已在物料盘上方打开，保持 100 ms 后再切换物料盘位置。 */
            HAL_Delay(TASK2_PLATFORM_PICK_TRAY_OPEN_SETTLE_MS);
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
    task->state = TASK2_PLATFORM_PICK_FORWARD;
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

        case TASK2_PLATFORM_PICK_FORWARD:
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_forward_800mm_start");
            result = task2_move(task,
                                TASK2_PLATFORM_PICK_FORWARD_MM,
                                1,
                                0);
            if (result != TASK2_PLATFORM_PICK_OK)
            {
                return task2_fail(task, result,
                                  "task2_forward_800mm_failed");
            }
            task2_wait_move_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK2_PLATFORM_PICK_RIGHT;
            return TASK2_PLATFORM_PICK_OK;

        case TASK2_PLATFORM_PICK_RIGHT:
            task2_status(task,
                         TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK,
                         "task2_right_960mm_start");
            result = task2_move(task,
                                TASK2_PLATFORM_PICK_RIGHT_MM,
                                0,
                                -1);
            if (result != TASK2_PLATFORM_PICK_OK)
            {
                return task2_fail(task, result,
                                  "task2_right_960mm_failed");
            }
            task2_wait_move_done(task);
            (void)mecanum_chassis_stop(task->chassis);
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
