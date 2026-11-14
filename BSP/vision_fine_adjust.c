#include "vision_fine_adjust.h"

#include "chassis_motion_closed_loop.h"

#include <math.h>
#include <string.h>

static void vision_fine_stop(vision_fine_adjust_t *adjust)
{
    if ((adjust != NULL) && (adjust->chassis != NULL) &&
        (adjust->command_active != 0U))
    {
        (void)mecanum_chassis_stop(adjust->chassis);
        adjust->command_active = 0U;
    }
}

#if !VISION_FINE_ADJUST_BYPASS
static void vision_fine_reset_stability(vision_fine_adjust_t *adjust)
{
    if (adjust == NULL)
    {
        return;
    }
    adjust->stable_frame_count = 0U;
    adjust->stability_confirmed = 0U;
    adjust->stable_frame_valid = 0U;
    adjust->stable_detection_id = 0U;
    adjust->stable_last_seq = 0U;
    adjust->stable_dx = 0.0f;
    adjust->stable_dy = 0.0f;
}
#endif

#if !VISION_FINE_ADJUST_BYPASS
static float vision_fine_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static vision_fine_adjust_result_t vision_fine_hold_or_wait(
    vision_fine_adjust_t *adjust,
    vision_fine_adjust_reject_reason_t reason)
{
    adjust->last_reject_reason = reason;
    /* 没有可用或新鲜的视觉结果时，不再追加新的位置脉冲。 */
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

static float vision_fine_wrap_yaw_deg(float value)
{
    while (value > 180.0f)
    {
        value -= 360.0f;
    }
    while (value <= -180.0f)
    {
        value += 360.0f;
    }
    return value;
}

static int32_t vision_fine_heading_pulse(
    const vision_fine_adjust_t *adjust,
    float heading_tolerance_deg,
    float heading_kp_pulses_per_deg,
    uint8_t heading_max_correction_pulses)
{
    hwt101ct_data_t yaw_data;
    float error_deg;
    float correction;
    uint32_t now_ms;

    if ((adjust == NULL) || (adjust->yaw_sensor == NULL) ||
        (adjust->heading_target_valid == 0U) ||
        (heading_kp_pulses_per_deg <= 0.0f) ||
        (heading_max_correction_pulses == 0U) ||
        (hwt101ct_get_data(adjust->yaw_sensor, &yaw_data) != HWT101CT_OK) ||
        (yaw_data.yaw_valid == 0U))
    {
        return 0;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - yaw_data.yaw_update_ms) >
        adjust->config.heading_max_age_ms)
    {
        return 0;
    }

    error_deg = vision_fine_wrap_yaw_deg(adjust->heading_target_deg -
                                         yaw_data.yaw_deg);
    if (fabsf(error_deg) <= heading_tolerance_deg)
    {
        return 0;
    }
    correction = error_deg * heading_kp_pulses_per_deg;
    if (correction > (float)heading_max_correction_pulses)
    {
        correction = (float)heading_max_correction_pulses;
    }
    else if (correction <
             -(float)heading_max_correction_pulses)
    {
        correction = -(float)heading_max_correction_pulses;
    }
    if (correction >= 0.0f)
    {
        correction = (float)((int32_t)(correction + 0.5f));
        return (correction < 1.0f) ? 1 : (int32_t)correction;
    }
    correction = (float)((int32_t)(correction - 0.5f));
    return (correction > -1.0f) ? -1 : (int32_t)correction;
}

static void vision_fine_capture_heading(vision_fine_adjust_t *adjust)
{
    hwt101ct_data_t yaw_data;
    float target_yaw_deg;
    uint32_t now_ms;

    if ((adjust == NULL) || (adjust->yaw_sensor == NULL) ||
        (hwt101ct_get_data(adjust->yaw_sensor, &yaw_data) != HWT101CT_OK) ||
        (yaw_data.yaw_valid == 0U))
    {
        if (adjust != NULL)
        {
            adjust->heading_target_valid = 0U;
        }
        return;
    }
    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - yaw_data.yaw_update_ms) >
        adjust->config.heading_max_age_ms)
    {
        adjust->heading_target_valid = 0U;
        return;
    }
    /* 视觉阶段沿用底盘闭环的启动/规划航向，不能把当前残余偏差重新设为目标。 */
    if (chassis_motion_get_target_heading(&target_yaw_deg) ==
        CHASSIS_MOTION_OK)
    {
        adjust->heading_target_deg = target_yaw_deg;
    }
    else
    {
        adjust->heading_target_deg = yaw_data.yaw_deg;
    }
    adjust->heading_target_valid = 1U;
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

static uint8_t vision_fine_update_stability(
    vision_fine_adjust_t *adjust,
    const vision_uart_frame_t *frame,
    const vision_detection_t *detection,
    uint8_t frame_is_new)
{
    uint8_t stable;

    if ((adjust == NULL) || (frame == NULL) || (detection == NULL))
    {
        return 0U;
    }
    /* 靶环及后续物料流程保持原有判定；只给首件平移微调增加稳定帧门槛。 */
    if ((adjust->mode != VISION_UART_MODE_MATERIAL) ||
        (adjust->motion_enabled == 0U) ||
        (adjust->stability_confirmed != 0U))
    {
        return 1U;
    }
    /* 同一帧在控制周期内被复用时不能重复累计稳定帧数。 */
    if (frame_is_new == 0U)
    {
        return 0U;
    }

    stable = (adjust->stable_frame_valid != 0U) &&
             (frame->sequence_continuous != 0U) &&
             (frame->seq == (adjust->stable_last_seq + 1U)) &&
             (detection->id == adjust->stable_detection_id) &&
             (vision_fine_abs(detection->dx - adjust->stable_dx) <=
              VISION_FINE_ADJUST_STABLE_ERROR_DELTA_PIXEL) &&
             (vision_fine_abs(detection->dy - adjust->stable_dy) <=
              VISION_FINE_ADJUST_STABLE_ERROR_DELTA_PIXEL);

    if (stable == 0U)
    {
        adjust->stable_frame_count = 1U;
    }
    else if (adjust->stable_frame_count <
             VISION_FINE_ADJUST_STABLE_FRAME_COUNT)
    {
        adjust->stable_frame_count++;
    }
    adjust->stable_frame_valid = 1U;
    adjust->stable_detection_id = detection->id;
    adjust->stable_last_seq = frame->seq;
    adjust->stable_dx = detection->dx;
    adjust->stable_dy = detection->dy;

    if (adjust->stable_frame_count >= VISION_FINE_ADJUST_STABLE_FRAME_COUNT)
    {
        adjust->stability_confirmed = 1U;
        return 1U;
    }
    return 0U;
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
#endif

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
    /* 细调阶段使用较大的单步脉冲，减少视觉闭环的无效往返等待。 */
    config->position_pulses_per_pixel =
        VISION_FINE_ADJUST_DEFAULT_POSITION_PULSES_PER_PIXEL;
    config->max_step_pulses = VISION_FINE_ADJUST_DEFAULT_MAX_STEP_PULSES;
    config->position_speed_rpm = 180U;
    config->position_acceleration = 255U;
    config->position_settle_ms = 8U;
    config->heading_tolerance_deg =
        VISION_FINE_ADJUST_DEFAULT_HEADING_TOLERANCE_DEG;
    config->heading_kp_pulses_per_deg =
        VISION_FINE_ADJUST_DEFAULT_HEADING_KP_PULSES_PER_DEG;
    config->heading_max_correction_pulses =
        VISION_FINE_ADJUST_DEFAULT_HEADING_MAX_CORRECTION_PULSES;
    config->heading_max_age_ms =
        VISION_FINE_ADJUST_DEFAULT_HEADING_MAX_AGE_MS;
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
        (config->position_settle_ms == 0U) ||
        (config->heading_tolerance_deg < 0.0f) ||
        (config->heading_kp_pulses_per_deg < 0.0f) ||
        (config->heading_max_age_ms == 0U))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    memset(adjust, 0, sizeof(*adjust));
    adjust->vision = vision;
    adjust->chassis = chassis;
    adjust->config = *config;
    adjust->motion_enabled = 1U;
    adjust->heading_adjust_enabled = 1U;
    adjust->require_target_match = 0U;
    adjust->state = VISION_FINE_ADJUST_IDLE;
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    return VISION_FINE_ADJUST_OK;
}

void vision_fine_adjust_set_heading_sensor(
    vision_fine_adjust_t *adjust,
    hwt101ct_t *yaw_sensor)
{
    if (adjust == NULL)
    {
        return;
    }
    adjust->yaw_sensor = yaw_sensor;
    adjust->heading_target_valid = 0U;
}

void vision_fine_adjust_set_heading_enabled(
    vision_fine_adjust_t *adjust,
    uint8_t enabled)
{
    if (adjust == NULL)
    {
        return;
    }
    adjust->heading_adjust_enabled = (enabled != 0U) ? 1U : 0U;
    if (adjust->heading_adjust_enabled == 0U)
    {
        adjust->heading_target_valid = 0U;
    }
}

void vision_fine_adjust_set_target_match_required(
    vision_fine_adjust_t *adjust,
    uint8_t enabled)
{
    if (adjust != NULL)
    {
        adjust->require_target_match = (enabled != 0U) ? 1U : 0U;
    }
}

static vision_fine_adjust_result_t vision_fine_adjust_start_internal(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id,
    uint8_t mode_preselected)
{
    float target_yaw_deg;

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
    /* 原料取料阶段的航向修正仍延后到夹取完成后的旋转闭环。 */
    if ((mode != VISION_UART_MODE_TARGET_RING) &&
        (adjust->heading_adjust_enabled != 0U) &&
        (chassis_motion_get_target_heading(&target_yaw_deg) ==
         CHASSIS_MOTION_OK))
    {
        if (chassis_motion_correct_heading() != CHASSIS_MOTION_OK)
        {
            adjust->state = VISION_FINE_ADJUST_ERROR;
            return VISION_FINE_ADJUST_CHASSIS_ERROR;
        }
    }
#if VISION_FINE_ADJUST_BYPASS
    /* 联调旁路：保持接口和状态机语义，但不依赖视觉模块。 */
    (void)mode_preselected;
    adjust->vision->current_mode = mode;
    adjust->mode = mode;
    adjust->target_id = target_id;
    adjust->start_tick = HAL_GetTick();
    adjust->last_command_tick = adjust->start_tick;
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    adjust->state = VISION_FINE_ADJUST_ALIGNED;
    return VISION_FINE_ADJUST_OK;
#else
    if (mode_preselected != 0U)
    {
        if (adjust->vision->current_mode != mode)
        {
            adjust->state = VISION_FINE_ADJUST_ERROR;
            return VISION_FINE_ADJUST_UART_ERROR;
        }
        /* 只保留停车后收到的新帧，不重复发送已经提前发送的模式命令。 */
        vision_uart_clear_frame(adjust->vision);
    }
    else if (vision_uart_set_mode(adjust->vision, mode) !=
             VISION_UART_RESULT_OK)
    {
        adjust->state = VISION_FINE_ADJUST_ERROR;
        return VISION_FINE_ADJUST_UART_ERROR;
    }
    adjust->mode = mode;
    adjust->target_id = target_id;
    adjust->start_tick = HAL_GetTick();
    adjust->last_command_tick = adjust->start_tick;
    adjust->last_frame_valid = 0U;
    vision_fine_reset_stability(adjust);
    if (adjust->heading_adjust_enabled == 0U)
    {
        adjust->heading_target_valid = 0U;
    }
    else
    {
        vision_fine_capture_heading(adjust);
    }
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    adjust->state = VISION_FINE_ADJUST_WAITING_FRAME;
    return VISION_FINE_ADJUST_OK;
#endif
}

vision_fine_adjust_result_t vision_fine_adjust_start(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id)
{
    return vision_fine_adjust_start_internal(adjust, mode, target_id, 0U);
}

vision_fine_adjust_result_t vision_fine_adjust_preselect_mode(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode)
{
    if ((adjust == NULL) || (adjust->vision == NULL) ||
        ((mode != VISION_UART_MODE_MATERIAL) &&
         (mode != VISION_UART_MODE_TARGET_RING)))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
#if VISION_FINE_ADJUST_BYPASS
    adjust->vision->current_mode = mode;
    return VISION_FINE_ADJUST_OK;
#else
    return (vision_uart_set_mode(adjust->vision, mode) ==
            VISION_UART_RESULT_OK) ?
           VISION_FINE_ADJUST_OK : VISION_FINE_ADJUST_UART_ERROR;
#endif
}

vision_fine_adjust_result_t vision_fine_adjust_start_preselected(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id)
{
    return vision_fine_adjust_start_internal(adjust, mode, target_id, 1U);
}

vision_fine_adjust_result_t vision_fine_adjust_step(
    vision_fine_adjust_t *adjust)
{
#if !VISION_FINE_ADJUST_BYPASS
    vision_uart_frame_t frame;
    vision_detection_t detection;
    int32_t body_x_pulse;
    int32_t body_y_pulse;
    int32_t heading_pulse;
    int32_t wheel_pulse[MECANUM_WHEEL_COUNT];
    uint32_t local_age_ms;
    vision_uart_result_t frame_result;
    mecanum_result_t chassis_result;
    uint8_t target_found;
    uint8_t visual_aligned;
    uint8_t frame_is_new;
    int16_t tolerance_x_pixel;
    int16_t tolerance_y_pixel;
    float position_pulses_per_pixel;
    float heading_tolerance_deg;
    float heading_kp_pulses_per_deg;
    uint8_t max_step_pulses;
    uint8_t heading_max_correction_pulses;
    uint16_t position_speed_rpm;
#endif

    if ((adjust == NULL) || (adjust->vision == NULL) ||
        (adjust->chassis == NULL))
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
#if VISION_FINE_ADJUST_BYPASS
    adjust->state = VISION_FINE_ADJUST_ALIGNED;
    return VISION_FINE_ADJUST_REACHED;
#else
    tolerance_x_pixel = adjust->config.tolerance_x_pixel;
    tolerance_y_pixel = adjust->config.tolerance_y_pixel;
    if (adjust->coarse_pick_enabled != 0U)
    {
        tolerance_x_pixel = VISION_FINE_ADJUST_COARSE_PICK_TOLERANCE_X_PIXEL;
        tolerance_y_pixel = VISION_FINE_ADJUST_COARSE_PICK_TOLERANCE_Y_PIXEL;
    }
    position_pulses_per_pixel = adjust->config.position_pulses_per_pixel;
    max_step_pulses = adjust->config.max_step_pulses;
    position_speed_rpm = adjust->config.position_speed_rpm;
    heading_tolerance_deg = adjust->config.heading_tolerance_deg;
    heading_kp_pulses_per_deg = adjust->config.heading_kp_pulses_per_deg;
    heading_max_correction_pulses =
        adjust->config.heading_max_correction_pulses;
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
    frame_is_new = 0U;
    if (frame_result == VISION_UART_RESULT_OK)
    {
        /* 新帧到达时更新缓存；后续控制周期可复用这帧继续走小步。 */
        adjust->last_frame = frame;
        adjust->last_frame_valid = 1U;
        frame_is_new = 1U;
    }
    else if ((frame_result == VISION_UART_RESULT_NOT_READY) &&
             (adjust->last_frame_valid != 0U))
    {
        frame = adjust->last_frame;
    }
    else
    {
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_NO_FRAME);
    }
    /* 以单片机实际收到帧的时间判断新鲜度，避免视觉端 age_ms 的时间基准
       或帧周期略大于150 ms时误丢弃有效目标。 */
    local_age_ms = (uint32_t)(HAL_GetTick() - frame.received_tick);
    if (frame.mode != adjust->mode)
    {
        if (frame_is_new != 0U)
        {
            adjust->last_frame_valid = 0U;
        }
        vision_fine_reset_stability(adjust);
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_MODE);
    }
    if (frame.valid == 0U)
    {
        if (frame_is_new != 0U)
        {
            adjust->last_frame_valid = 0U;
        }
        vision_fine_reset_stability(adjust);
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_INVALID);
    }
    if (frame.status != VISION_UART_STATUS_OK)
    {
        if (frame_is_new != 0U)
        {
            adjust->last_frame_valid = 0U;
        }
        vision_fine_reset_stability(adjust);
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_STATUS);
    }
    if (local_age_ms > adjust->config.max_age_ms)
    {
        adjust->last_frame_valid = 0U;
        vision_fine_reset_stability(adjust);
        return vision_fine_hold_or_wait(adjust,
                                        VISION_FINE_ADJUST_REJECT_AGE);
    }
    target_found = (vision_uart_find_detection(&frame,
                                               adjust->target_id,
                                               &detection) ==
                    VISION_UART_RESULT_OK) ? 1U : 0U;
    if (target_found == 0U)
    {
        if ((adjust->require_target_match == 0U) &&
            (vision_fine_find_any_detection(&frame, &detection) != 0U))
        {
            /* 兼容未启用严格匹配的其他流程。 */
        }
        else
        {
            /* 错误颜色不能驱动底盘，也不能作为后续控制周期的缓存目标。 */
            adjust->last_frame_valid = 0U;
            vision_fine_reset_stability(adjust);
            return vision_fine_hold_or_wait(adjust,
                                            VISION_FINE_ADJUST_REJECT_TARGET);
        }
    }

    if (vision_fine_update_stability(adjust,
                                     &frame,
                                     &detection,
                                     frame_is_new) == 0U)
    {
        /* 首个物料必须先收到连续3个误差接近的新帧，才能开始微调。 */
        adjust->state = VISION_FINE_ADJUST_WAITING_FRAME;
        return VISION_FINE_ADJUST_WAITING;
    }

    visual_aligned =
        ((vision_fine_abs((float)detection.dx) <=
          (float)tolerance_x_pixel) &&
         (vision_fine_abs((float)detection.dy) <=
          (float)tolerance_y_pixel) &&
         (target_found != 0U)) ? 1U : 0U;
    /* 位置进入容差后仍需确认偏航误差已经消除；否则继续发送纯旋转脉冲。 */
    /* 关闭航向补偿时只做平移；航向由上层旋转闭环统一修正。 */
    heading_pulse = (adjust->heading_adjust_enabled != 0U) ?
                    vision_fine_heading_pulse(
                        adjust,
                        heading_tolerance_deg,
                        heading_kp_pulses_per_deg,
                        heading_max_correction_pulses) : 0;
    if ((visual_aligned != 0U) && (heading_pulse == 0))
    {
        /* 位置和航向均进入容差后才结束视觉阶段。 */
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_ALIGNED;
        return VISION_FINE_ADJUST_REACHED;
    }
    if ((adjust->motion_enabled == 0U) && (heading_pulse == 0))
    {
        adjust->state = VISION_FINE_ADJUST_RUNNING;
        return VISION_FINE_ADJUST_WAITING;
    }
    if (heading_pulse != 0)
    {
        /* 先单独完成航向矫正，航向进入容差后下一帧才允许位置矫正。 */
        body_x_pulse = 0;
        body_y_pulse = 0;
    }
    else if (adjust->motion_enabled == 0U)
    {
        /* 后续物料禁止平移，但航向偏差仍通过纯旋转脉冲持续消除。 */
        body_x_pulse = 0;
        body_y_pulse = 0;
    }
    else
    {
        /* 图像 x 控制车体前后，图像 y 控制车体左右。 */
        body_x_pulse = vision_fine_error_to_pulse(
            detection.dx,
            adjust->config.image_x_to_body_x_sign,
            position_pulses_per_pixel,
            max_step_pulses);
        body_y_pulse = vision_fine_error_to_pulse(
            detection.dy,
            adjust->config.image_y_to_body_y_sign,
            position_pulses_per_pixel,
            max_step_pulses);
    }
    wheel_pulse[MECANUM_WHEEL_FL] = body_x_pulse - body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] = body_x_pulse + body_y_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] = body_x_pulse - body_y_pulse;
    /* 航向误差为正时补偿逆时针旋转：左侧轮反向、右侧轮正向。 */
    wheel_pulse[MECANUM_WHEEL_FL] -= heading_pulse;
    wheel_pulse[MECANUM_WHEEL_FR] += heading_pulse;
    wheel_pulse[MECANUM_WHEEL_RL] -= heading_pulse;
    wheel_pulse[MECANUM_WHEEL_RR] += heading_pulse;
    vision_fine_limit_pulses(wheel_pulse,
                             max_step_pulses);
    chassis_result = mecanum_chassis_set_position_increment(
        adjust->chassis,
        wheel_pulse,
        position_speed_rpm,
        adjust->config.position_acceleration);
    if ((chassis_result != MECANUM_OK) &&
        (chassis_result != MECANUM_CLAMPED))
    {
        vision_fine_stop(adjust);
        adjust->state = VISION_FINE_ADJUST_ERROR;
        return VISION_FINE_ADJUST_CHASSIS_ERROR;
    }
    for (uint8_t index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        adjust->accumulated_motion_pulses[index] += wheel_pulse[index];
    }
    adjust->command_active = 1U;
    adjust->last_command_tick = HAL_GetTick();
    adjust->last_reject_reason = VISION_FINE_ADJUST_REJECT_NONE;
    adjust->state = VISION_FINE_ADJUST_RUNNING;
    return VISION_FINE_ADJUST_OK;
#endif
}

void vision_fine_adjust_set_coarse_pick(
    vision_fine_adjust_t *adjust,
    uint8_t enabled)
{
    if (adjust != NULL)
    {
        adjust->coarse_pick_enabled = (enabled != 0U) ? 1U : 0U;
    }
}

void vision_fine_adjust_set_motion_enabled(
    vision_fine_adjust_t *adjust,
    uint8_t enabled)
{
    if (adjust != NULL)
    {
        adjust->motion_enabled = (enabled != 0U) ? 1U : 0U;
    }
}

void vision_fine_adjust_clear_motion_pulses(
    vision_fine_adjust_t *adjust)
{
    uint8_t index;

    if (adjust == NULL)
    {
        return;
    }
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        adjust->accumulated_motion_pulses[index] = 0;
    }
}

void vision_fine_adjust_take_motion_pulses(
    vision_fine_adjust_t *adjust,
    int32_t pulses[MECANUM_WHEEL_COUNT])
{
    uint8_t index;

    if ((adjust == NULL) || (pulses == NULL))
    {
        return;
    }
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        pulses[index] = adjust->accumulated_motion_pulses[index];
        adjust->accumulated_motion_pulses[index] = 0;
    }
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

vision_fine_adjust_result_t
vision_fine_adjust_prepare_platform_pick_and_start_preselected(
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
    return vision_fine_adjust_start_preselected(
        adjust,
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
    if (arm_controller_set_target_ring(arm, target_ring) !=
        ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    if (arm_controller_prepare_ground_place(arm) != ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    return vision_fine_adjust_start(adjust,
                                    VISION_UART_MODE_TARGET_RING,
                                    target_ring);
}

vision_fine_adjust_result_t
vision_fine_adjust_prepare_place_and_start_preselected(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t target_ring)
{
    if (arm == NULL)
    {
        return VISION_FINE_ADJUST_INVALID_PARAM;
    }
    if (arm_controller_set_target_ring(arm, target_ring) !=
        ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    if (arm_controller_prepare_ground_place(arm) != ARM_CONTROLLER_OK)
    {
        return VISION_FINE_ADJUST_ARM_ERROR;
    }
    return vision_fine_adjust_start_preselected(
        adjust,
        VISION_UART_MODE_TARGET_RING,
        target_ring);
}

void vision_fine_adjust_abort(vision_fine_adjust_t *adjust)
{
    if (adjust != NULL)
    {
        vision_fine_stop(adjust);
        adjust->last_frame_valid = 0U;
#if !VISION_FINE_ADJUST_BYPASS
        vision_fine_reset_stability(adjust);
#endif
        adjust->heading_target_valid = 0U;
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
