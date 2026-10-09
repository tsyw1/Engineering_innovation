#include "vision_fine_adjust.h"

#include <string.h>

static float vision_fine_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void vision_fine_stop(vision_fine_adjust_t *adjust)
{
    if ((adjust != NULL) && (adjust->chassis != NULL) &&
        (adjust->command_active != 0U))
    {
        (void)mecanum_chassis_stop(adjust->chassis);
        adjust->command_active = 0U;
    }
}

static vision_fine_adjust_result_t vision_fine_hold_or_wait(
    vision_fine_adjust_t *adjust,
    vision_fine_adjust_reject_reason_t reason)
{
    adjust->last_reject_reason = reason;
    /* 位置模式命令会自行完成，没有新视觉帧时不追加任何运动。 */
    adjust->state = VISION_FINE_ADJUST_WAITING_FRAME;
    return VISION_FINE_ADJUST_WAITING;
}

static int32_t vision_fine_error_to_pulse(
    float error,
    int8_t sign,
    float pulses_per_pixel,
    uint8_t max_pulses)
{
    float magnitude;
    int32_t pulse;

    magnitude = vision_fine_abs(error) * pulses_per_pixel;
    pulse = (int32_t)(magnitude + 0.5f);
    if (pulse < 1)
    {
        pulse = 1;
    }
    if (pulse > (int32_t)max_pulses)
    {
        pulse = (int32_t)max_pulses;
    }
    return (error * (float)sign < 0.0f) ? pulse : -pulse;
}

static uint8_t vision_fine_find_any_detection(
    const vision_uart_frame_t *frame,
    vision_detection_t *detection)
{
    uint8_t index;
    uint8_t found = 0U;
    float best_confidence = 0.0f;

    if ((frame == NULL) || (detection == NULL))
    {
        return 0U;
    }

    /* 目标 ID 暂时不匹配时，使用当前帧中置信度最高的有效目标校准。 */
    for (index = 0U; index < frame->detection_count; index++)
    {
        if (frame->detections[index].valid == 0U)
        {
            continue;
        }
        if ((found == 0U) ||
            (frame->detections[index].confidence > best_confidence))
        {
            *detection = frame->detections[index];
            best_confidence = frame->detections[index].confidence;
            found = 1U;
        }
    }
    return found;
}

static void vision_fine_limit_pulses(
    int32_t pulse[MECANUM_WHEEL_COUNT],
    uint8_t max_pulses)
{
    int32_t max_abs = 0;
    uint8_t index;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int32_t value = (pulse[index] < 0) ? -pulse[index] : pulse[index];
        if (value > max_abs)
        {
            max_abs = value;
        }
    }
    if (max_abs <= (int32_t)max_pulses)
    {
        return;
    }
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int32_t value = pulse[index] * (int32_t)max_pulses;
        if (value >= 0)
        {
            pulse[index] = (value + max_abs / 2) / max_abs;
        }
        else
        {
            pulse[index] = (value - max_abs / 2) / max_abs;
        }
    }
}

void vision_fine_adjust_config_default(vision_fine_adjust_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->vx_per_pixel = 0.0008f;
    config->vy_per_pixel = 0.0008f;
    /* 按摄像头安装关系：图像 x 向右对应车体后方，图像 y 向下对应车体左侧。 */
    /* error_to_pulse() 中 sign=+1 表示输出方向与图像误差相反。 */
    config->image_x_to_body_x_sign = 1;
    config->image_y_to_body_y_sign = -1;
    config->max_vx_mps = 0.08f;
    config->max_vy_mps = 0.08f;
    config->tolerance_x_pixel =
        VISION_FINE_ADJUST_DEFAULT_TOLERANCE_X_PIXEL;
    config->tolerance_y_pixel =
        VISION_FINE_ADJUST_DEFAULT_TOLERANCE_Y_PIXEL;
    config->max_age_ms = 150U;
    /* 设为0表示细调不设置总超时时间，持续等待视觉误差收敛。 */
    config->timeout_ms = 0U;
    /* 适当增大单步脉冲，避免每次只克服很小的静摩擦位移。 */
    config->position_pulses_per_pixel = 0.48f;
    config->max_step_pulses = 96U;
    /* 位置由脉冲数控制，速度只保证小脉冲能够可靠执行。 */
    config->position_speed_rpm = 300U;
    config->position_acceleration = 220U;
    config->position_settle_ms = 10U;
}

vision_fine_adjust_result_t vision_fine_adjust_init(
    vision_fine_adjust_t *adjust,
    vision_uart_t *vision,
    mecanum_chassis_t *chassis,
    const vision_fine_adjust_config_t *config)
{
    if ((adjust == NULL) || (vision == NULL) || (chassis == NULL) ||
        (config == NULL) || (config->vx_per_pixel <= 0.0f) ||
        (config->vy_per_pixel <= 0.0f) || (config->max_vx_mps <= 0.0f) ||
        (config->max_vy_mps <= 0.0f) || (config->tolerance_x_pixel < 0) ||
        (config->tolerance_y_pixel < 0) ||
        (config->position_pulses_per_pixel <= 0.0f) ||
        (config->max_step_pulses == 0U) ||
        (config->position_speed_rpm == 0U) ||
        (config->position_settle_ms == 0U))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    memset(adjust, 0, sizeof(*adjust));
    adjust->vision = vision;
    adjust->chassis = chassis;
    adjust->config = *config;
    adjust->state = VISION_FINE_ADJUST_IDLE;
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    return VISION_FINE_ADJUST_OK;
}

vision_fine_adjust_result_t vision_fine_adjust_start(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id)
{
    if ((adjust == NULL) || (adjust->vision == NULL) ||
        (adjust->chassis == NULL) || (target_id == 0U))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if ((mode != VISION_UART_MODE_MATERIAL) &&
        (mode != VISION_UART_MODE_TARGET_RING))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (((mode == VISION_UART_MODE_MATERIAL) &&
         ((target_id < 1U) || (target_id > 6U))) ||
        ((mode == VISION_UART_MODE_TARGET_RING) &&
         ((target_id < 1U) || (target_id > 3U))))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    vision_fine_stop(adjust);
    if (vision_uart_set_mode(adjust->vision, mode) != VISION_UART_RESULT_OK)
    {
        adjust->state = VISION_FINE_ADJUST_ERROR;
        return VISION_FINE_ADJUST_UART_ERROR;
    }
    adjust->mode = mode;
    adjust->target_id = target_id;
    adjust->start_tick = HAL_GetTick();
    adjust->last_command_tick = adjust->start_tick;
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    adjust->state = VISION_FINE_ADJUST_WAITING_FRAME;
    return VISION_FINE_ADJUST_OK;
}

vision_fine_adjust_result_t vision_fine_adjust_step(
    vision_fine_adjust_t *adjust)
{
    vision_uart_frame_t frame;
    vision_detection_t detection;
    int32_t body_x_pulse;
    int32_t body_y_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];
    uint32_t local_age_ms;
    vision_uart_result_t frame_result;
    mecanum_result_t chassis_result;
    uint8_t target_found;

    if ((adjust == NULL) || (adjust->vision == NULL) ||
        (adjust->chassis == NULL))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (adjust->state == VISION_FINE_ADJUST_ALIGNED)
    {
        return VISION_FINE_ADJUST_REACHED;
    }
    if ((adjust->state == VISION_FINE_ADJUST_TIMEOUT) ||
        (adjust->state == VISION_FINE_ADJUST_ERROR))
    {
        return VISION_FINE_ADJUST_TIMEOUT_RESULT;
    }
    if ((adjust->config.timeout_ms != 0U) &&
        ((uint32_t)(HAL_GetTick() - adjust->start_tick) >=
         adjust->config.timeout_ms))
    {
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_TIMEOUT;
        return VISION_FINE_ADJUST_TIMEOUT_RESULT;
    }

    /* 上一条位置命令尚未留出稳定时间时，不取走新的视觉帧。 */
    if ((adjust->command_active != 0U) &&
        ((uint32_t)(HAL_GetTick() - adjust->last_command_tick) <
         adjust->config.position_settle_ms))
    {
        adjust->state = VISION_FINE_ADJUST_RUNNING;
        return VISION_FINE_ADJUST_WAITING;
    }

    frame_result = vision_uart_take_frame(adjust->vision, &frame);
    if (frame_result != VISION_UART_RESULT_OK)
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_NO_FRAME);
    }
    /* 以单片机实际收到帧的时间判断新鲜度，避免视觉端 age_ms 的时间基准
       或帧周期略大于150 ms时误丢弃有效目标。 */
    local_age_ms = (uint32_t)(HAL_GetTick() - frame.received_tick);
    if (frame.mode != adjust->mode)
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_MODE);
    }
    if (frame.valid == 0U)
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_INVALID);
    }
    if (frame.status != VISION_UART_STATUS_OK)
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_STATUS);
    }
    if (local_age_ms > adjust->config.max_age_ms)
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_AGE);
    }
    target_found = (vision_uart_find_detection(&frame,
                                               adjust->target_id,
                                               &detection) ==
                    VISION_UART_RESULT_OK) ? 1U : 0U;
    if ((target_found == 0U) &&
        (vision_fine_find_any_detection(&frame, &detection) == 0U))
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_TARGET);
    }

    if ((vision_fine_abs((float)detection.dx) <=
         (float)adjust->config.tolerance_x_pixel) &&
        (vision_fine_abs((float)detection.dy) <=
         (float)adjust->config.tolerance_y_pixel) &&
        (target_found != 0U))
    {
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_ALIGNED;
        return VISION_FINE_ADJUST_REACHED;
    }

    /* 图像 x 控制车体前后，图像 y 控制车体左右。 */
    body_x_pulse = vision_fine_error_to_pulse(
        detection.dx,
        adjust->config.image_x_to_body_x_sign,
        adjust->config.position_pulses_per_pixel,
        adjust->config.max_step_pulses);
    body_y_pulse = vision_fine_error_to_pulse(
        detection.dy,
        adjust->config.image_y_to_body_y_sign,
        adjust->config.position_pulses_per_pixel,
        adjust->config.max_step_pulses);
    wheel_pulse[MECANUM_WHEEL_FL] = body_x_pulse - body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] = body_x_pulse - body_y_pulse;
    vision_fine_limit_pulses(wheel_pulse,
                             adjust->config.max_step_pulses);
    chassis_result = mecanum_chassis_set_position_increment(
        adjust->chassis,
        wheel_pulse,
        adjust->config.position_speed_rpm,
        adjust->config.position_acceleration);
    if ((chassis_result != MECANUM_OK) &&
        (chassis_result != MECANUM_CLAMPED))
    {
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_ERROR;
        return VISION_FINE_ADJUST_CHASSIS_ERROR;
    }
    adjust->command_active = 1U;
    adjust->last_command_tick = HAL_GetTick();
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    adjust->state = VISION_FINE_ADJUST_RUNNING;
    return VISION_FINE_ADJUST_OK;
}

vision_fine_adjust_result_t vision_fine_adjust_prepare_pick_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t material_color)
{
    if (arm == NULL)
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (arm_controller_prepare_ground_pick(arm) != ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    return vision_fine_adjust_start(adjust,
                                    VISION_UART_MODE_MATERIAL,
                                    material_color);
}

vision_fine_adjust_result_t
vision_fine_adjust_prepare_platform_pick_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t material_color)
{
    if (arm == NULL)
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (arm_controller_prepare_platform_pick(arm) != ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    return vision_fine_adjust_start(adjust,
                                    VISION_UART_MODE_MATERIAL,
                                    material_color);
}

vision_fine_adjust_result_t vision_fine_adjust_prepare_pick_for_task(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    const task_execution_plan_t *plan,
    task_plan_batch_t batch,
    uint8_t sequence_index)
{
    const task_plan_item_t *item;

    item = task_plan_get_item(plan, batch, sequence_index);
    if (item == NULL)
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }

    return vision_fine_adjust_prepare_pick_and_start(adjust,
                                                     arm,
                                                     item->color_code);
}

vision_fine_adjust_result_t vision_fine_adjust_prepare_place_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t target_ring)
{
    if (arm == NULL)
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (arm_controller_prepare_ground_place(arm) != ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    return vision_fine_adjust_start(adjust,
                                    VISION_UART_MODE_TARGET_RING,
                                    target_ring);
}

void vision_fine_adjust_abort(vision_fine_adjust_t *adjust)
{
    if (adjust != NULL)
    {
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_IDLE;
    }
}

vision_fine_adjust_state_t vision_fine_adjust_get_state(
    const vision_fine_adjust_t *adjust)
{
    return (adjust == NULL) ? VISION_FINE_ADJUST_ERROR : adjust->state;
}

vision_fine_adjust_reject_reason_t
vision_fine_adjust_get_last_reject_reason(const vision_fine_adjust_t *adjust)
{
    return (adjust == NULL) ? VISION_FINE_ADJUST_REJECT_INVALID :
           adjust->last_reject_reason;
}
