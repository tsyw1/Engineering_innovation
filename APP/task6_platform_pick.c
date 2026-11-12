#include "task6_platform_pick.h"

#include "chassis_motion_closed_loop.h"

#include <stdio.h>
#include <string.h>

/* 与底盘位置模式限速一致，保证直线和旋转等待时间按实际速度计算。 */
#define TASK6_MOVE_SPEED_RPM          60U
#define TASK6_MOVE_ACCELERATION       240U
#define TASK6_MOVE_SETTLE_MS         400U
#define TASK6_ARM_PREPOSITION_TIMEOUT_MS 3000U
/* 位置脉冲符号约定：+1 为逆时针，-1 为顺时针。 */
#define TASK6_ROTATE_CLOCKWISE_SIGN   (-1)

static void task6_status(task6_platform_pick_t *task,
                         uint8_t status,
                         const char *event)
{
    if ((task != NULL) && (task->write_status != NULL))
    {
        task->write_status(status, event);
    }
}

static task6_platform_pick_result_t task6_fail(
    task6_platform_pick_t *task,
    task6_platform_pick_result_t result,
    const char *event)
{
    if (task != NULL)
    {
        vision_fine_adjust_abort(task->fine_adjust);
        vision_fine_adjust_set_coarse_pick(task->fine_adjust, 0U);
        (void)chassis_motion_set_service_callback(NULL, NULL);
        (void)mecanum_chassis_stop(task->chassis);
        (void)mecanum_chassis_enable(task->chassis, false);
        task->state = TASK6_PLATFORM_PICK_FAILED;
        task6_status(task, TASK6_PLATFORM_PICK_STATUS_ERROR, event);
    }
    return result;
}

static uint32_t task6_pulses_per_mm(const mecanum_chassis_t *chassis)
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

static uint32_t task6_linear_duration_ms(const mecanum_chassis_t *chassis,
                                         uint16_t distance_mm)
{
    float speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK6_MOVE_SETTLE_MS;
    }
    speed_mps = ((float)TASK6_MOVE_SPEED_RPM / 60.0f) *
                2.0f * 3.14159265359f * chassis->config.wheel_radius_m;
    return (uint32_t)(((float)distance_mm / 1000.0f) /
                      speed_mps * 1000.0f) + TASK6_MOVE_SETTLE_MS;
}

static uint32_t task6_rotation_pulses(const mecanum_chassis_t *chassis,
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

static uint32_t task6_rotation_duration_ms(const mecanum_chassis_t *chassis,
                                           uint16_t angle_deg)
{
    float wheel_distance_m;
    float wheel_speed_mps;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return TASK6_MOVE_SETTLE_MS;
    }
    wheel_distance_m = (chassis->config.half_length_m +
                        chassis->config.half_width_m) *
                       ((float)angle_deg * 3.14159265359f / 180.0f);
    wheel_speed_mps = ((float)TASK6_MOVE_SPEED_RPM / 60.0f) *
                      2.0f * 3.14159265359f *
                      chassis->config.wheel_radius_m;
    return (uint32_t)(wheel_distance_m / wheel_speed_mps * 1000.0f) +
           TASK6_MOVE_SETTLE_MS;
}

static void task6_wait_motion_done(const task6_platform_pick_t *task)
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

static task6_platform_pick_result_t task6_move_linear(
    task6_platform_pick_t *task,
    uint16_t distance_mm,
    int8_t body_x_sign,
    int8_t body_y_sign)
{
    if ((task == NULL) || (distance_mm == 0U) ||
        ((body_x_sign == 0) && (body_y_sign == 0)))
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
    }
    if (chassis_motion_move_linear_mm(
            task->chassis,
            (int16_t)distance_mm * body_x_sign,
            (int16_t)distance_mm * body_y_sign) != CHASSIS_MOTION_OK)
    {
        return TASK6_PLATFORM_PICK_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK6_PLATFORM_PICK_OK;
}

static task6_platform_pick_result_t task6_rotate_clockwise_90(
    task6_platform_pick_t *task)
{
    if (task == NULL)
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
    }
    if (chassis_motion_rotate_deg(
            task->chassis,
            (int16_t)TASK6_PLATFORM_PICK_ROTATE_DEG *
            TASK6_ROTATE_CLOCKWISE_SIGN) != CHASSIS_MOTION_OK)
    {
        return TASK6_PLATFORM_PICK_CHASSIS_ERROR;
    }
    task->move_start_tick = HAL_GetTick();
    task->move_duration_ms = 0U;
    return TASK6_PLATFORM_PICK_OK;
}

static uint16_t task6_backward_distance_mm(uint8_t ring)
{
    switch (ring)
    {
        case 1U:
            return TASK6_PLATFORM_PICK_BACKWARD_RING_1_MM;
        case 2U:
            return TASK6_PLATFORM_PICK_BACKWARD_RING_2_MM;
        case 3U:
            return TASK6_PLATFORM_PICK_BACKWARD_RING_3_MM;
        default:
            return 0U;
    }
}

static task6_platform_pick_result_t task6_start_tray_position(
    task6_platform_pick_t *task)
{
    if ((task == NULL) || (task->tray == NULL) ||
        (task->item_index >= TASK6_PLATFORM_PICK_ITEM_COUNT))
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
    }
    if (material_tray_servo_set_position(
            task->tray,
            (material_tray_position_t)task->item_index) !=
        MATERIAL_TRAY_SERVO_OK)
    {
        return TASK6_PLATFORM_PICK_TRAY_ERROR;
    }
    return TASK6_PLATFORM_PICK_OK;
}

static void task6_finish_tray_position(task6_platform_pick_t *task)
{
    if (task == NULL)
    {
        return;
    }
    HAL_Delay(TASK6_PLATFORM_PICK_TRAY_SETTLE_MS);
}

task6_platform_pick_result_t task6_platform_pick_init(
    task6_platform_pick_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task6_platform_pick_status_fn write_status)
{
    vision_fine_adjust_config_t config;
    vision_fine_adjust_result_t adjust_result;

    if ((task == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
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
        return TASK6_PLATFORM_PICK_VISION_ERROR;
    }
    task->state = TASK6_PLATFORM_PICK_WAIT_TASK;
    task6_status(task,
                 TASK6_PLATFORM_PICK_STATUS_INIT,
                 "task6_platform_pick_ready");
    return TASK6_PLATFORM_PICK_OK;
}

task6_platform_pick_result_t task6_platform_pick_submit_qr(
    task6_platform_pick_t *task,
    const QR_TaskCode_t *qr_task)
{
    uint8_t index;

    if ((task == NULL) || (qr_task == NULL) || (qr_task->valid == 0U) ||
        (task->state != TASK6_PLATFORM_PICK_WAIT_TASK))
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
    }
    for (index = 0U; index < TASK6_PLATFORM_PICK_ITEM_COUNT; index++)
    {
        if ((qr_task->second_batch_color[index] < 1U) ||
            (qr_task->second_batch_color[index] > 6U) ||
            (qr_task->second_batch_position[index] < 1U) ||
            (qr_task->second_batch_position[index] > 3U) ||
            (qr_task->first_batch_color[index] < 1U) ||
            (qr_task->first_batch_color[index] > 6U) ||
            (qr_task->first_batch_position[index] < 1U) ||
            (qr_task->first_batch_position[index] > 3U))
        {
            return TASK6_PLATFORM_PICK_INVALID_PARAM;
        }
    }
    if (vision_fine_adjust_preselect_mode(
            task->fine_adjust,
            VISION_UART_MODE_MATERIAL) != VISION_FINE_ADJUST_OK)
    {
        return TASK6_PLATFORM_PICK_VISION_ERROR;
    }
    /* 任务5结束时料盘停在第三工位，行驶前先转回第一工位并利用行驶时间稳定。 */
    if (material_tray_servo_set_position(
            task->tray,
            MATERIAL_TRAY_POSITION_1) != MATERIAL_TRAY_SERVO_OK)
    {
        return TASK6_PLATFORM_PICK_TRAY_ERROR;
    }
    /* 任务5结束后底盘仍停在暂存区2号环。 */
    task->current_ring = TASK6_PLATFORM_PICK_CENTER;
    memcpy(&task->qr_task, qr_task, sizeof(task->qr_task));
    task->item_index = 0U;
    task->task_ready = 1U;
    task->state = TASK6_PLATFORM_PICK_BACKWARD_FROM_RING;
    task6_status(task,
                 TASK6_PLATFORM_PICK_STATUS_MOVE,
                 "task6_platform_pick_sequence_loaded");
    return TASK6_PLATFORM_PICK_OK;
}

task6_platform_pick_result_t task6_platform_pick_step(
    task6_platform_pick_t *task)
{
    task6_platform_pick_result_t result;
    vision_fine_adjust_result_t adjust_result;
    vision_fine_adjust_result_t prepare_result;
    arm_controller_result_t arm_result;
    uint16_t distance_mm;

    if ((task == NULL) || (task->arm == NULL) ||
        (task->chassis == NULL) || (task->fine_adjust == NULL) ||
        (task->tray == NULL) || (task->screen == NULL))
    {
        return TASK6_PLATFORM_PICK_INVALID_PARAM;
    }

    switch (task->state)
    {
        case TASK6_PLATFORM_PICK_WAIT_TASK:
            return TASK6_PLATFORM_PICK_WAITING;

        case TASK6_PLATFORM_PICK_BACKWARD_FROM_RING:
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_PREPARE,
                         "task6_arm_travel_safe_start");
            if (arm_controller_prepare_travel_safe(task->arm) !=
                ARM_CONTROLLER_OK)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_arm_travel_safe_failed");
            }
            distance_mm = task6_backward_distance_mm(task->current_ring);
            if (distance_mm == 0U)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_INVALID_PARAM,
                                  "task6_backward_distance_invalid");
            }
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_MOVE,
                         "task6_backward_from_ring_start");
            result = task6_move_linear(task, distance_mm, -1, 0);
            if (result != TASK6_PLATFORM_PICK_OK)
            {
                return task6_fail(task, result,
                                  "task6_backward_from_ring_failed");
            }
            task->state = TASK6_PLATFORM_PICK_WAIT_BACKWARD_FROM_RING;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_WAIT_BACKWARD_FROM_RING:
            task6_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK6_PLATFORM_PICK_ROTATE;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_ROTATE:
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_ROTATE,
                         "task6_rotate_clockwise_90_start");
            result = task6_rotate_clockwise_90(task);
            if (result != TASK6_PLATFORM_PICK_OK)
            {
                return task6_fail(task, result,
                                  "task6_rotate_clockwise_90_failed");
            }
            task->state = TASK6_PLATFORM_PICK_WAIT_ROTATE;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_WAIT_ROTATE:
            task6_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            task->state = TASK6_PLATFORM_PICK_BACKWARD_TO_AREA;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_BACKWARD_TO_AREA:
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_PREPARE,
                         "task6_arm_preposition_during_travel_start");
            if ((chassis_motion_set_service_callback(
                     arm_controller_preposition_service,
                     task->arm) != CHASSIS_MOTION_OK) ||
                (arm_controller_start_platform_pick_preposition(task->arm) !=
                 ARM_CONTROLLER_OK))
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_arm_preposition_start_failed");
            }
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_MOVE,
                         "task6_backward_to_material_area_start");
            result = task6_move_linear(
                task,
                TASK6_PLATFORM_PICK_BACKWARD_TO_AREA_MM,
                -1,
                0);
            if (result != TASK6_PLATFORM_PICK_OK)
            {
                return task6_fail(task, result,
                                  "task6_backward_to_material_area_failed");
            }
            task->state = TASK6_PLATFORM_PICK_WAIT_BACKWARD_TO_AREA;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_WAIT_BACKWARD_TO_AREA:
            task6_wait_motion_done(task);
            (void)mecanum_chassis_stop(task->chassis);
            if (arm_controller_wait_preposition(
                    task->arm,
                    TASK6_ARM_PREPOSITION_TIMEOUT_MS) != ARM_CONTROLLER_OK)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_arm_preposition_failed");
            }
            (void)chassis_motion_set_service_callback(NULL, NULL);
            task->state = TASK6_PLATFORM_PICK_PREPARE_ITEM;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_PREPARE_ITEM:
            result = task6_start_tray_position(task);
            if (result != TASK6_PLATFORM_PICK_OK)
            {
                return task6_fail(task, result,
                                  "task6_tray_position_failed");
            }
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_PREPARE,
                         "task6_platform_prepare_pick_start");
            /* 首件保持原有精调；第二、第三件只在偏差过大时才重新微调。 */
            vision_fine_adjust_set_coarse_pick(
                task->fine_adjust,
                (task->item_index == 0U) ? 0U : 1U);
            if (task->item_index == 0U)
            {
                /* 首件机械臂姿态已在跨区行驶期间完成。 */
                prepare_result = vision_fine_adjust_start_preselected(
                    task->fine_adjust,
                    VISION_UART_MODE_MATERIAL,
                    task->qr_task.second_batch_color[task->item_index]);
            }
            else
            {
                prepare_result =
                    vision_fine_adjust_prepare_platform_pick_and_start(
                        task->fine_adjust,
                        task->arm,
                        task->qr_task.second_batch_color[task->item_index]);
            }
            if (prepare_result != VISION_FINE_ADJUST_OK)
            {
                char event[96];
                int length;

                length = snprintf(event,
                                  sizeof(event),
                                  "task6_item=%u,prepare_result=%u",
                                  (unsigned int)(task->item_index + 1U),
                                  (unsigned int)prepare_result);
                if ((length > 0) && ((size_t)length < sizeof(event)))
                {
                    task6_status(task,
                                 TASK6_PLATFORM_PICK_STATUS_ERROR,
                                 event);
                }
                /* 首件使用行驶期间预选的模式；后续物料重新发送模式命令，
                 * 确保视觉端重新发布当前物料帧。 */
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_platform_prepare_pick_failed");
            }
            task6_finish_tray_position(task);
            task->state = TASK6_PLATFORM_PICK_ALIGN;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_ALIGN:
            adjust_result = vision_fine_adjust_step(task->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                task->state = TASK6_PLATFORM_PICK_PICK_ITEM;
                task6_status(task,
                             TASK6_PLATFORM_PICK_STATUS_FINE_ADJUST,
                             "task6_fine_adjust_done");
                return TASK6_PLATFORM_PICK_OK;
            }
            if (adjust_result == VISION_FINE_ADJUST_TIMEOUT_RESULT)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_VISION_ERROR,
                                  "task6_fine_adjust_timeout");
            }
            if (adjust_result == VISION_FINE_ADJUST_CHASSIS_ERROR)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_CHASSIS_ERROR,
                                  "task6_fine_adjust_chassis_error");
            }
            if ((adjust_result == VISION_FINE_ADJUST_ARM_ERROR) ||
                (adjust_result == VISION_FINE_ADJUST_UART_ERROR))
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_fine_adjust_failed");
            }
            return TASK6_PLATFORM_PICK_WAITING;

        case TASK6_PLATFORM_PICK_PICK_ITEM:
            task6_status(task,
                         TASK6_PLATFORM_PICK_STATUS_PICK,
                         "task6_platform_pick_start");
            arm_result = arm_controller_pick_and_place(task->arm);
            if (arm_result != ARM_CONTROLLER_OK)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_ARM_ERROR,
                                  "task6_platform_pick_failed");
            }
            if (screen_record_successful_pick(task->screen) != SCREEN_OK)
            {
                return task6_fail(task,
                                  TASK6_PLATFORM_PICK_SCREEN_ERROR,
                                  "task6_pick_count_display_failed");
            }
            task->item_index++;
            if (task->item_index >= TASK6_PLATFORM_PICK_ITEM_COUNT)
            {
                vision_fine_adjust_abort(task->fine_adjust);
                vision_fine_adjust_set_coarse_pick(task->fine_adjust, 0U);
                (void)mecanum_chassis_stop(task->chassis);
                task->state = TASK6_PLATFORM_PICK_DONE;
                task6_status(task,
                             TASK6_PLATFORM_PICK_STATUS_DONE,
                             "task6_platform_three_items_done");
                return TASK6_PLATFORM_PICK_OK;
            }
            /* 下一次准备动作会先升到最高点；物料盘旋转在准备期间启动。 */
            task->state = TASK6_PLATFORM_PICK_PREPARE_ITEM;
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_DONE:
            return TASK6_PLATFORM_PICK_OK;

        case TASK6_PLATFORM_PICK_FAILED:
        default:
            return TASK6_PLATFORM_PICK_ARM_ERROR;
    }
}

task6_platform_pick_state_t task6_platform_pick_get_state(
    const task6_platform_pick_t *task)
{
    return (task == NULL) ?
           TASK6_PLATFORM_PICK_FAILED : task->state;
}
