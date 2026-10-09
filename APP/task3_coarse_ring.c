#include "task3_coarse_ring.h"

#include <string.h>

/* 与底盘位置模式限速一致，保证直线和旋转等待时间按实际速度计算。 */
#define TASK3_MOVE_SPEED_RPM          60U
#define TASK3_MOVE_ACCELERATION       240U
#define TASK3_MOVE_SETTLE_MS         600U
#define TASK3_ROTATE_SIGN              1

static void task3_status(task3_coarse_ring_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task3_coarse_ring_result_t task3_fail(
    task3_coarse_ring_t *task,
    task3_coarse_ring_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK3_COARSE_RING_FAILED;
        task3_status(task, TASK3_COARSE_RING_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task3_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task3_linear_duration_ms(const mecanum_chassis_t *chassis,
                                         uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK3_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK3_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK3_MOVE_SETTLE_MS;
}

static uint32_t task3_rotation_pulses(const mecanum_chassis_t *chassis,
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

static uint32_t task3_rotation_duration_ms(const mecanum_chassis_t *chassis,
                                           uint16_t angle_deg)
{
    float wheel_distance_m;
    float wheel_speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK3_MOVE_SETTLE_MS;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    wheel_speed_mps = ((float)TASK3_MOVE_SPEED_RPM / 60.0f) *
                      2.0f * 3.14159265359f *
                      chassis->config.wheel_radius_m;
    return (uint32_t)(wheel_distance_m / wheel_speed_mps * 1000.0f) +
           TASK3_MOVE_SETTLE_MS;
}

static void task3_wait_motion_done(const task3_coarse_ring_t *task)
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

static task3_coarse_ring_result_t task3_move_linear(
    task3_coarse_ring_t *task,
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
        return TASK3_COARSE_RING_INVALID_PARAM;
    }
    pulses_per_mm = task3_pulses_per_mm(task->chassis);
    if (pulses_per_mm == 0U)
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
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
            TASK3_MOVE_SPEED_RPM,
            TASK3_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK3_COARSE_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task3_linear_duration_ms(task->chassis,
                                                       distance_mm);
    return TASK3_COARSE_RING_OK;
}

static task3_coarse_ring_result_t task3_rotate_180(
    task3_coarse_ring_t *task)
{
    uint32_t pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];

    if (task == NULL)
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }
    pulse = task3_rotation_pulses(task->chassis,
                                  TASK3_COARSE_RING_ROTATE_DEG);
    if (pulse == 0U)
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }

    wheel_pulse[MECANUM_WHEEL_FL] = -(int32_t)pulse * TASK3_ROTATE_SIGN;
    wheel_pulse[MECANUM_WHEEL_FR] = (int32_t)pulse * TASK3_ROTATE_SIGN;
    wheel_pulse[MECANUM_WHEEL_RL] = -(int32_t)pulse * TASK3_ROTATE_SIGN;
    wheel_pulse[MECANUM_WHEEL_RR] = (int32_t)pulse * TASK3_ROTATE_SIGN;
    if (mecanum_chassis_set_position_increment(
            task->chassis,
            wheel_pulse,
            TASK3_MOVE_SPEED_RPM,
            TASK3_MOVE_ACCELERATION) != MECANUM_OK)
    {
        return TASK3_COARSE_RING_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = task3_rotation_duration_ms(
        task->chassis,
        TASK3_COARSE_RING_ROTATE_DEG);
    return TASK3_COARSE_RING_OK;
}

static int16_t task3_ring_offset_mm(uint8_t ring)
{
    if (ring == 1U)
    {
        return (int16_t)TASK3_COARSE_RING_SPACING_MM;
    }
    if (ring == 3U)
    {
        return -(int16_t)TASK3_COARSE_RING_SPACING_MM;
    }
    return 0;
}

static task3_coarse_ring_result_t task3_move_to_ring(
    task3_coarse_ring_t *task)
{
    int16_t delta_mm;
    int8_t direction;

    if ((task == NULL) || (task->target_ring < 1U) ||
        (task->target_ring > 3U) || (task->current_ring < 1U) ||
        (task->current_ring > 3U))
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }

    delta_mm = task3_ring_offset_mm(task->target_ring) -
               task3_ring_offset_mm(task->current_ring);
    if (delta_mm == 0)
    {
        return TASK3_COARSE_RING_OK;
    }
    direction = (delta_mm > 0) ? 1 : -1;
    return task3_move_linear(task,
                             (uint16_t)((delta_mm > 0) ?
                                        delta_mm : -delta_mm),
                             direction,
                             0);
}

static task3_coarse_ring_result_t task3_set_tray_position(
    task3_coarse_ring_t *task)
{
    material_tray_position_t position;

    if ((task == NULL) || (task->item_index >= 3U))
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }
    position = (material_tray_position_t)task->item_index;
    if (material_tray_servo_set_position(task->tray, position) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return task3_fail(task,
                          TASK3_COARSE_RING_TRAY_ERROR,
                          "task3_tray_position_failed");
    }
    HAL_Delay(500U);
    return TASK3_COARSE_RING_OK;
}

task3_coarse_ring_result_t task3_coarse_ring_init(
    task3_coarse_ring_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task3_coarse_ring_status_fn write_status)
{
    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }
    memset(task, 0, sizeof(*task));
    task->arm = arm;
    task->chassis = chassis;
    task->fine_adjust = fine_adjust;
    task->tray = tray;
    task->screen = screen;
    task->write_status = write_status;
    task->current_ring = TASK3_COARSE_RING_CENTER;
    task->state = TASK3_COARSE_RING_WAIT_TASK;
    task3_status(task, TASK3_COARSE_RING_STATUS_INIT,
                 "task3_coarse_ring_ready");
    return TASK3_COARSE_RING_OK;
}

task3_coarse_ring_result_t task3_coarse_ring_submit_qr(
    task3_coarse_ring_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK3_COARSE_RING_WAIT_TASK))
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }
    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if ((qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U))
        {
            return TASK3_COARSE_RING_INVALID_PARAM;
        }
    }
    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    task->current_ring = TASK3_COARSE_RING_CENTER;
    task->task_ready = 1U;
    task->state = TASK3_COARSE_RING_LEFT;
    task3_status(task, TASK3_COARSE_RING_STATUS_MOVE,
                 "task3_left_100mm_start");
    return TASK3_COARSE_RING_OK;
}

task3_coarse_ring_result_t task3_coarse_ring_step(
    task3_coarse_ring_t *task)
{
    task3_coarse_ring_result_t result;
    vision_fine_adjust_result_t adjust_result;
    arm_controller_result_t arm_result;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK3_COARSE_RING_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK3_COARSE_RING_WAIT_TASK:
            return TASK3_COARSE_RING_WAITING;

        case TASK3_COARSE_RING_LEFT:
            result = task3_move_linear(task,
                                       TASK3_COARSE_RING_LEFT_MM,
                                       0,
                                       1);
            if (result != TASK3_COARSE_RING_OK)
            {
                return task3_fail(task, result, "task3_left_100mm_failed");
            }
            task3_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK3_COARSE_RING_ROTATE;
            task3_status(task, TASK3_COARSE_RING_STATUS_ROTATE,
                         "task3_rotate_180_start");
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_ROTATE:
            result = task3_rotate_180(task);
            if (result != TASK3_COARSE_RING_OK)
            {
                return task3_fail(task, result, "task3_rotate_180_failed");
            }
            task3_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK3_COARSE_RING_RIGHT;
            task3_status(task, TASK3_COARSE_RING_STATUS_MOVE,
                         "task3_right_1720mm_start");
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_RIGHT:
            result = task3_move_linear(task,
                                       TASK3_COARSE_RING_RIGHT_MM,
                                       0,
                                       -1);
            if (result != TASK3_COARSE_RING_OK)
            {
                return task3_fail(task, result, "task3_right_1720mm_failed");
            }
            task3_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK3_COARSE_RING_PREPARE_ITEM;
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_PREPARE_ITEM:
            task->target_ring = task->qr_task.first_batch_position[
                task->item_index];
            result = task3_move_to_ring(task);
            if (result != TASK3_COARSE_RING_OK)
            {
                return task3_fail(task, result, "task3_ring_move_failed");
            }
            if (result == TASK3_COARSE_RING_OK &&
                task3_ring_offset_mm(task->target_ring) !=
                task3_ring_offset_mm(task->current_ring))
            {
                task3_wait_motion_done(task);
                (void)mecanum_chassis_stop(task->chassis);
                task->current_ring = task->target_ring;
            }
            task3_status(task, TASK3_COARSE_RING_STATUS_PREPARE,
                         "task3_ground_place_prepare_start");
            result = (vision_fine_adjust_prepare_place_and_start(
                task->fine_adjust,
                task->arm,
                task->target_ring) == VISION_FINE_ADJUST_OK) ?
                     TASK3_COARSE_RING_OK : TASK3_COARSE_RING_ARM_ERROR;
            if (result != TASK3_COARSE_RING_OK)
            {
                return task3_fail(task, result,
                                  "task3_ground_place_prepare_failed");
            }
            task->state = TASK3_COARSE_RING_ALIGN;
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                task->state = TASK3_COARSE_RING_PLACE_ITEM;
                task3_status(task, TASK3_COARSE_RING_STATUS_FINE_ADJUST,
                             "task3_fine_adjust_done");
                return TASK3_COARSE_RING_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_VISION_ERROR,
                                  "task3_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_CHASSIS_ERROR,
                                  "task3_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_ARM_ERROR,
                                  "task3_fine_adjust_failed");
            }
            return TASK3_COARSE_RING_WAITING;

        case TASK3_COARSE_RING_PLACE_ITEM:
            if (task3_set_tray_position(task) != TASK3_COARSE_RING_OK)
            {
                return TASK3_COARSE_RING_TRAY_ERROR;
            }
            task3_status(task, TASK3_COARSE_RING_STATUS_PLACE,
                         "task3_pick_from_tray_and_place_start");
            arm_result = arm_controller_place_ring_item_from_tray(task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_ARM_ERROR,
                                  "task3_pick_from_tray_and_place_failed");
            }
            if (screen_record_successful_place(task->screen) != SCREEN_OK)
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_SCREEN_ERROR,
                                  "task3_place_count_display_failed");
            }
            task->state = TASK3_COARSE_RING_FINISH_ITEM;
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_FINISH_ITEM:
            task->item_index++;
            if (task->item_index >= QR_BATCH_ITEM_COUNT)
            {
                /* 最后一件完成后保持最高点、面向靶环，交给任务4取料。 */
                task->state = TASK3_COARSE_RING_DONE;
                task3_status(task, TASK3_COARSE_RING_STATUS_DONE,
                             "task3_three_rings_done");
                return TASK3_COARSE_RING_OK;
            }

            /* 非最后一件回到物料盘方向，为下一个顺序物料做准备。 */
            arm_result = arm_controller_finish_place(task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task3_fail(task,
                                  TASK3_COARSE_RING_ARM_ERROR,
                                  "task3_finish_place_failed");
            }
            task->state = TASK3_COARSE_RING_PREPARE_ITEM;
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_MOVE_NEXT:
            task->state = TASK3_COARSE_RING_PREPARE_ITEM;
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_DONE:
            return TASK3_COARSE_RING_OK;

        case TASK3_COARSE_RING_FAILED:
        default:
            return TASK3_COARSE_RING_ARM_ERROR;
    }
}

task3_coarse_ring_state_t task3_coarse_ring_get_state(
    const task3_coarse_ring_t *task)
{
    return (task == NULL) ? TASK3_COARSE_RING_FAILED : task->state;
}
