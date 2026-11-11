//
// 机械臂整体动作控制实现。
//

#include "arm_controller.h"

#include "usart.h"

#include <math.h>

#define ARM_CONTROLLER_POSITION_TIMEOUT_MS    30000U
#define ARM_CONTROLLER_HOME_TIMEOUT_MS        20000U
#define ARM_CONTROLLER_HOME_START_TIMEOUT_MS  3000U
#define ARM_CONTROLLER_POLL_INTERVAL_MS       20U
#define ARM_CONTROLLER_FEEDBACK_TIMEOUT_MS     200U
#define ARM_CONTROLLER_FEEDBACK_POLL_MS          1U

static arm_controller_result_t arm_controller_move_z_position(
    arm_controller_t *controller,
    uint16_t position);
static arm_controller_result_t arm_controller_move_z_position_only(
    arm_controller_t *controller,
    uint16_t position);
static arm_controller_result_t arm_controller_move_to_tray_direction(
    arm_controller_t *controller);
static uint16_t arm_controller_target_ring_z_position(
    const arm_controller_t *controller);
static float arm_controller_target_ring_horizontal_angle(
    const arm_controller_t *controller);

static arm_controller_result_t arm_controller_from_motor_result(
    motor_result_t result)
{
    switch (result)
    {
        case MOTOR_OK:
            return ARM_CONTROLLER_OK;
        case MOTOR_BUSY:
            return ARM_CONTROLLER_BUSY;
        case MOTOR_NOT_READY:
            return ARM_CONTROLLER_NOT_READY;
        default:
            return ARM_CONTROLLER_MOTOR_ERROR;
    }
}

static arm_controller_result_t arm_controller_from_gripper_result(
    gripper_servo_result_t result)
{
    return (result == GRIPPER_SERVO_OK) ?
           ARM_CONTROLLER_OK : ARM_CONTROLLER_MOTOR_ERROR;
}

static arm_controller_result_t arm_controller_open_gripper_at_angle(
    arm_controller_t *controller,
    uint16_t angle_deg)
{
    gripper_servo_result_t gripper_result;
    uint16_t default_open_angle;

    if ((controller == NULL) || (angle_deg > SERVO_MAX_ANGLE_DEG))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 仅本次动作临时使用指定张开角度，其他动作仍使用默认 95 度。 */
    default_open_angle = controller->gripper.config.open_angle_deg;
    controller->gripper.config.open_angle_deg = angle_deg;
    gripper_result = gripper_servo_open(&controller->gripper);
    controller->gripper.config.open_angle_deg = default_open_angle;
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 夹取前在最高点等待夹爪实际张开，再允许后续机构运动和下降。 */
    HAL_Delay(ARM_CONTROLLER_GRIPPER_OPEN_DELAY_MS);
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_open_tray_gripper(
    arm_controller_t *controller)
{
    return arm_controller_open_gripper_at_angle(
        controller,
        ARM_CONTROLLER_TRAY_PICK_OPEN_ANGLE_DEG);
}

static arm_controller_result_t arm_controller_release_gripper_delay(
    arm_controller_t *controller,
    uint32_t delay_ms)
{
    gripper_servo_result_t gripper_result;

    if (controller == NULL)
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 到达放置位置后先保持指定时间，再打开夹爪。 */
    HAL_Delay(delay_ms);
    gripper_result = gripper_servo_open(&controller->gripper);
    return arm_controller_from_gripper_result(gripper_result);
}

static arm_controller_result_t arm_controller_release_gripper(
    arm_controller_t *controller)
{
    return arm_controller_release_gripper_delay(
        controller,
        ARM_CONTROLLER_RELEASE_DELAY_MS);
}

static int64_t arm_controller_angle_to_pulse(float angle_deg)
{
    double pulse;

    pulse = (double)angle_deg *
            (double)ARM_CONTROLLER_COMMAND_PULSES_PER_REV / 360.0;
    return (pulse >= 0.0) ?
           (int64_t)(pulse + 0.5) :
           (int64_t)(pulse - 0.5);
}

static arm_controller_result_t arm_controller_from_z_servo_result(
    arm_z_servo_axis_result_t result)
{
    switch (result)
    {
        case ARM_Z_SERVO_AXIS_OK:
            return ARM_CONTROLLER_OK;
        case ARM_Z_SERVO_AXIS_INVALID_PARAM:
            return ARM_CONTROLLER_INVALID_PARAM;
        case ARM_Z_SERVO_AXIS_NOT_READY:
            return ARM_CONTROLLER_NOT_READY;
        default:
            return ARM_CONTROLLER_MOTOR_ERROR;
    }
}

static arm_controller_result_t arm_controller_read_motor_feedback_count(
    motor_t *motor,
    int64_t *feedback_count)
{
    uint32_t baseline_count;
    uint32_t before_count;
    uint32_t after_count;
    uint32_t start_ms;
    int64_t position_count;
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (feedback_count == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 先记住更新序号，只接受本次请求之后收到的 0x36 实时位置反馈。 */
    baseline_count = motor->position_update_count;
    result = motor_read_position(motor);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    start_ms = HAL_GetTick();
    while (motor->position_update_count == baseline_count)
    {
        if ((HAL_GetTick() - start_ms) >=
            ARM_CONTROLLER_FEEDBACK_TIMEOUT_MS)
        {
            return ARM_CONTROLLER_TIMEOUT;
        }
        HAL_Delay(ARM_CONTROLLER_FEEDBACK_POLL_MS);
    }

    /* Cortex-M4 不能原子读取 64 位值；用 count-value-count 防止中断撕裂。 */
    do
    {
        before_count = motor->position_update_count;
        position_count = motor->actual_position_pulse;
        after_count = motor->position_update_count;
    } while (before_count != after_count);

    *feedback_count = position_count;
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_capture_motor_feedback_zero(
    motor_t *motor)
{
    int64_t feedback_count;
    arm_controller_result_t result;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    motor->position_feedback_zero_valid = 0U;
    result = arm_controller_read_motor_feedback_count(motor, &feedback_count);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    motor->position_feedback_zero_count = feedback_count;
    motor->position_feedback_zero_valid = 1U;
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_read_motor_angle_deg(
    motor_t *motor,
    float *angle_deg)
{
    int64_t feedback_count;
    int64_t relative_count;
    arm_controller_result_t result;

    if ((motor == NULL) || (angle_deg == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    if (motor->position_feedback_zero_valid == 0U)
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_read_motor_feedback_count(motor, &feedback_count);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    relative_count = feedback_count - motor->position_feedback_zero_count;
    /* Emm 的 0x36 是 65536 counts/rev，不是命令使用的 3200 pulses/rev。 */
    *angle_deg = (float)((double)relative_count * 360.0 /
                         (double)MOTOR_EMM_FEEDBACK_COUNTS_PER_REV);
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_motor_at_angle(
    motor_t *motor,
    float target_angle_deg,
    uint8_t *at_target)
{
    float actual_angle_deg;
    arm_controller_result_t result;

    if ((at_target == NULL) || !isfinite(target_angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    result = arm_controller_read_motor_angle_deg(motor, &actual_angle_deg);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    *at_target = (fabsf(target_angle_deg - actual_angle_deg) <=
                  ARM_CONTROLLER_POSITION_TOLERANCE_DEG) ? 1U : 0U;
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_wait_position(motor_t *motor)
{
    uint32_t start_ms;
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 给驱动器留出启动位置命令的时间，避免读到上一条命令的旧到位状态。 */
    HAL_Delay(100U);
    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < ARM_CONTROLLER_POSITION_TIMEOUT_MS)
    {
        result = motor_read_state(motor);
        if (result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(result);
        }

        HAL_Delay(ARM_CONTROLLER_POLL_INTERVAL_MS);

        if ((motor->state_flags & MOTOR_STATE_STALL_PROTECTED) != 0U)
        {
            return ARM_CONTROLLER_MOTOR_ERROR;
        }

        if ((motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U)
        {
            return ARM_CONTROLLER_OK;
        }
    }

    return ARM_CONTROLLER_TIMEOUT;
}

static arm_controller_result_t arm_controller_wait_two_positions(
    motor_t *first_motor,
    motor_t *second_motor)
{
    uint32_t start_ms;
    motor_result_t result;
    uint8_t first_reached = 0U;
    uint8_t second_reached = 0U;

    if ((first_motor == NULL) || (second_motor == NULL) ||
        (first_motor->bus == NULL) ||
        (first_motor->bus != second_motor->bus))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 同步触发后给两个驱动器留出启动时间，再在同一超时窗口内轮询。 */
    HAL_Delay(100U);
    start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < ARM_CONTROLLER_POSITION_TIMEOUT_MS)
    {
        if (first_reached == 0U)
        {
            result = motor_read_state(first_motor);
            if (result != MOTOR_OK)
            {
                return arm_controller_from_motor_result(result);
            }
        }
        if (second_reached == 0U)
        {
            result = motor_read_state(second_motor);
            if (result != MOTOR_OK)
            {
                return arm_controller_from_motor_result(result);
            }
        }

        HAL_Delay(ARM_CONTROLLER_POLL_INTERVAL_MS);

        if (((first_motor->state_flags | second_motor->state_flags) &
             MOTOR_STATE_STALL_PROTECTED) != 0U)
        {
            return ARM_CONTROLLER_MOTOR_ERROR;
        }
        if ((first_motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U)
        {
            first_reached = 1U;
        }
        if ((second_motor->state_flags & MOTOR_STATE_POSITION_REACHED) != 0U)
        {
            second_reached = 1U;
        }
        if ((first_reached != 0U) && (second_reached != 0U))
        {
            return ARM_CONTROLLER_OK;
        }
    }

    return ARM_CONTROLLER_TIMEOUT;
}

static arm_controller_result_t
arm_controller_move_lift_horizontal_absolute_sync(
    arm_controller_t *controller,
    float lift_absolute_angle_deg,
    float horizontal_absolute_angle_deg)
{
    motor_result_t motor_result;
    arm_controller_result_t controller_result;
    int64_t lift_pulse;
    int64_t horizontal_pulse;
    uint8_t lift_at_target;
    uint8_t horizontal_at_target;
    uint8_t attempt;

    if ((controller == NULL) || (controller->bus == NULL) ||
        !isfinite(lift_absolute_angle_deg) ||
        !isfinite(horizontal_absolute_angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 先读两轴实时编码位置；都在目标容差内时不重复运动。 */
    controller_result = arm_controller_motor_at_angle(
        &controller->lift_motor,
        lift_absolute_angle_deg,
        &lift_at_target);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }
    controller_result = arm_controller_motor_at_angle(
        &controller->horizontal_motor,
        horizontal_absolute_angle_deg,
        &horizontal_at_target);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }
    if ((lift_at_target != 0U) && (horizontal_at_target != 0U))
    {
        return ARM_CONTROLLER_OK;
    }

    lift_pulse = arm_controller_angle_to_pulse(lift_absolute_angle_deg);
    horizontal_pulse = arm_controller_angle_to_pulse(
        horizontal_absolute_angle_deg);

    for (attempt = 0U;
         attempt <= ARM_CONTROLLER_POSITION_CORRECTION_RETRIES;
         attempt++)
    {
        controller->lift_motor.state_flags &=
            (uint8_t)~MOTOR_STATE_POSITION_REACHED;
        controller->horizontal_motor.state_flags &=
            (uint8_t)~MOTOR_STATE_POSITION_REACHED;

        motor_result = motor_set_position(
            &controller->lift_motor,
            lift_pulse,
            ARM_CONTROLLER_LIFT_SPEED_RPM,
            ARM_CONTROLLER_LIFT_ACCELERATION,
            MOTOR_POSITION_ABSOLUTE,
            MOTOR_EXECUTE_SYNC);
        if (motor_result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(motor_result);
        }

        motor_result = motor_set_position(
            &controller->horizontal_motor,
            horizontal_pulse,
            ARM_CONTROLLER_HORIZONTAL_SPEED_RPM,
            ARM_CONTROLLER_HORIZONTAL_ACCELERATION,
            MOTOR_POSITION_ABSOLUTE,
            MOTOR_EXECUTE_SYNC);
        if (motor_result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(motor_result);
        }

        motor_result = motor_trigger_sync(controller->bus);
        if (motor_result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(motor_result);
        }

        controller_result = arm_controller_wait_two_positions(
            &controller->lift_motor,
            &controller->horizontal_motor);
        if (controller_result != ARM_CONTROLLER_OK)
        {
            return controller_result;
        }

        controller_result = arm_controller_motor_at_angle(
            &controller->lift_motor,
            lift_absolute_angle_deg,
            &lift_at_target);
        if (controller_result != ARM_CONTROLLER_OK)
        {
            return controller_result;
        }
        controller_result = arm_controller_motor_at_angle(
            &controller->horizontal_motor,
            horizontal_absolute_angle_deg,
            &horizontal_at_target);
        if (controller_result != ARM_CONTROLLER_OK)
        {
            return controller_result;
        }
        if ((lift_at_target != 0U) && (horizontal_at_target != 0U))
        {
            return ARM_CONTROLLER_OK;
        }
        /* 驱动器报告到位但编码角度越界时，下一轮整体同步重发一次。 */
    }

    return ARM_CONTROLLER_TIMEOUT;
}

static arm_controller_result_t arm_controller_wait_home(motor_t *motor)
{
    uint32_t start_ms;
    uint32_t home_start_ms;
    motor_result_t result;
    uint8_t running_seen;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    start_ms = HAL_GetTick();
    home_start_ms = start_ms;
    running_seen = 0U;
    while ((HAL_GetTick() - start_ms) < ARM_CONTROLLER_HOME_TIMEOUT_MS)
    {
        result = motor_read_home_state(motor);
        if (result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(result);
        }

        HAL_Delay(ARM_CONTROLLER_POLL_INTERVAL_MS);

        /*
         * X42S 回零状态：bit2 表示“正在回零”，bit3 表示“回零失败”。
         * 回零成功时 bit2 和 bit3 都会清零，因此不能把 bit2 当成成功。
         * 必须先观察到运行状态，再等待运行状态清零，避免把命令发送前
         * 的空闲状态误判为本次回零已经完成。
         */
        if ((motor->home_flags & MOTOR_HOME_FAILED) != 0U)
        {
            return ARM_CONTROLLER_HOME_FAILED;
        }
        if ((motor->home_flags & MOTOR_HOME_RUNNING) != 0U)
        {
            running_seen = 1U;
        }
        else if (running_seen != 0U)
        {
            return ARM_CONTROLLER_OK;
        }
        else if ((HAL_GetTick() - home_start_ms) >=
                 ARM_CONTROLLER_HOME_START_TIMEOUT_MS)
        {
            /* 命令没有进入回零状态，禁止继续执行后续抓取动作。 */
            return ARM_CONTROLLER_TIMEOUT;
        }
    }

    return ARM_CONTROLLER_TIMEOUT;
}

static arm_controller_result_t arm_controller_move_motor_absolute_closed_loop(
    motor_t *motor,
    float target_angle_deg,
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    int64_t pulse;
    motor_result_t motor_result;
    arm_controller_result_t controller_result;
    uint8_t at_target;
    uint8_t attempt;

    if ((motor == NULL) || (motor->bus == NULL) ||
        !isfinite(target_angle_deg) || (speed_rpm == 0U) ||
        (speed_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    /* 任何伸缩或升降动作前，先按新鲜的 0x36 编码位置判断是否需要修正。 */
    controller_result = arm_controller_motor_at_angle(
        motor,
        target_angle_deg,
        &at_target);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }
    if (at_target != 0U)
    {
        return ARM_CONTROLLER_OK;
    }

    pulse = arm_controller_angle_to_pulse(target_angle_deg);
    for (attempt = 0U;
         attempt <= ARM_CONTROLLER_POSITION_CORRECTION_RETRIES;
         attempt++)
    {
        motor->state_flags &= (uint8_t)~MOTOR_STATE_POSITION_REACHED;
        motor_result = motor_set_position(motor,
                                          pulse,
                                          speed_rpm,
                                          acceleration,
                                          MOTOR_POSITION_ABSOLUTE,
                                          MOTOR_EXECUTE_NOW);
        if (motor_result != MOTOR_OK)
        {
            return arm_controller_from_motor_result(motor_result);
        }

        controller_result = arm_controller_wait_position(motor);
        if (controller_result != ARM_CONTROLLER_OK)
        {
            return controller_result;
        }

        controller_result = arm_controller_motor_at_angle(
            motor,
            target_angle_deg,
            &at_target);
        if (controller_result != ARM_CONTROLLER_OK)
        {
            return controller_result;
        }
        if (at_target != 0U)
        {
            return ARM_CONTROLLER_OK;
        }
        /* 到位标志与真实角度不一致时，按同一绝对目标有界重发。 */
    }

    return ARM_CONTROLLER_TIMEOUT;
}

static arm_controller_result_t arm_controller_move_lift_angle(
    motor_t *motor,
    float angle_deg,
    motor_position_mode_t mode)
{
    if ((motor == NULL) || (motor->bus == NULL) || !isfinite(angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    (void)mode;
    return arm_controller_move_motor_absolute_closed_loop(
        motor,
        angle_deg,
        ARM_CONTROLLER_LIFT_SPEED_RPM,
        ARM_CONTROLLER_LIFT_ACCELERATION);
}

static arm_controller_result_t arm_controller_move_horizontal_angle(
    arm_controller_t *controller,
    float angle_deg)
{
    if ((controller == NULL) || (controller->horizontal_motor.bus == NULL) ||
        !isfinite(angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    return arm_controller_move_motor_absolute_closed_loop(
        &controller->horizontal_motor,
        angle_deg,
        ARM_CONTROLLER_HORIZONTAL_SPEED_RPM,
        ARM_CONTROLLER_HORIZONTAL_ACCELERATION);
}

static arm_controller_result_t arm_controller_move_z_angle(
    arm_controller_t *controller,
    float angle_deg,
    motor_position_mode_t mode)
{
    float target_angle_deg;

    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    if ((controller->z_servo_axis.initialized == 0U) ||
        (controller->z_servo_axis.hardware_bound == 0U))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    if (!isfinite(angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    target_angle_deg = angle_deg;
    if (mode == MOTOR_POSITION_RELATIVE_CURRENT)
    {
        /* 兼容旧动作表：正向表示物料盘，负向表示当前靶环。 */
        return arm_controller_move_z_position(
            controller,
            (angle_deg >= 0.0f) ?
            ARM_CONTROLLER_Z_TRAY_POSITION :
            arm_controller_target_ring_z_position(controller));
    }
    else if (mode != MOTOR_POSITION_ABSOLUTE)
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    if ((target_angle_deg < 0.0f) || (target_angle_deg > 4095.0f))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    return arm_controller_move_z_position(controller,
                                          (uint16_t)target_angle_deg);
}

static arm_controller_result_t arm_controller_move_z_position(
    arm_controller_t *controller,
    uint16_t position)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_move_z_position_only(controller, position);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 水平轴在整个 Z 轴旋转时间内保持 0 度，旋转结束后才允许伸出。 */
    return arm_controller_move_horizontal_angle(
        controller,
        (position == ARM_CONTROLLER_Z_TRAY_POSITION) ?
        ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG :
        arm_controller_target_ring_horizontal_angle(controller));
}

static arm_controller_result_t arm_controller_move_z_position_only(
    arm_controller_t *controller,
    uint16_t position)
{
    arm_controller_result_t result;
    arm_z_servo_axis_result_t z_result;

    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    /* 每次旋转 Z 轴前都用实时编码位置确认水平轴已经收回最里侧。 */
    result = arm_controller_move_horizontal_angle(
        controller,
        ARM_CONTROLLER_HORIZONTAL_ZERO_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    z_result = arm_z_servo_axis_move_to_position(
        &controller->z_servo_axis,
        position);
    result = arm_controller_from_z_servo_result(z_result);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 总线舵机指令异步执行；等待配置的运动时间，期间水平轴保持 0 度。 */
    HAL_Delay(controller->z_servo_axis.config.run_time_ms);
    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_move_to_tray_direction(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_move_z_position_only(
        controller,
        ARM_CONTROLLER_Z_TRAY_POSITION);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* Z 轴运动时间结束后再保留短暂稳定时间，随后到物料盘绝对位置。 */
    HAL_Delay(ARM_CONTROLLER_TRAY_ROTATION_SETTLE_MS);
    return arm_controller_move_horizontal_angle(
        controller,
        ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG);
}

static uint16_t arm_controller_target_ring_z_position(
    const arm_controller_t *controller)
{
    if (controller == NULL)
    {
        return ARM_CONTROLLER_Z_RING_2_POSITION;
    }
    switch (controller->target_ring)
    {
        case 1U: return ARM_CONTROLLER_Z_RING_1_POSITION;
        case 3U: return ARM_CONTROLLER_Z_RING_3_POSITION;
        case 2U:
        default: return ARM_CONTROLLER_Z_RING_2_POSITION;
    }
}

static float arm_controller_target_ring_horizontal_angle(
    const arm_controller_t *controller)
{
    if (controller == NULL)
    {
        return ARM_CONTROLLER_HORIZONTAL_RING_2_ANGLE_DEG;
    }
    switch (controller->target_ring)
    {
        case 1U: return ARM_CONTROLLER_HORIZONTAL_RING_1_ANGLE_DEG;
        case 3U: return ARM_CONTROLLER_HORIZONTAL_RING_3_ANGLE_DEG;
        case 2U:
        default: return ARM_CONTROLLER_HORIZONTAL_RING_2_ANGLE_DEG;
    }
}

arm_controller_result_t arm_controller_init(arm_controller_t *controller,
                                            motor_bus_t *bus)
{
    motor_result_t result;
    arm_controller_result_t controller_result;
    arm_z_servo_axis_config_t z_servo_config;
    arm_z_servo_axis_result_t z_servo_result;

    if ((controller == NULL) || (bus == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    controller->bus = bus;
    controller->initialized = 0U;
    controller->target_ring = 2U;
    result = motor_init(&controller->lift_motor,
                        bus,
                        ARM_CONTROLLER_VERTICAL_MOTOR_ID);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    result = motor_init(&controller->horizontal_motor,
                        bus,
                        ARM_CONTROLLER_HORIZONTAL_MOTOR_ID);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    controller->initialized = 1U;

    arm_z_servo_axis_config_default(&z_servo_config);
    z_servo_result = arm_z_servo_axis_bind_usart2(
        &controller->z_servo_axis,
        &z_servo_config);
    if (z_servo_result != ARM_Z_SERVO_AXIS_OK)
    {
        return arm_controller_from_z_servo_result(z_servo_result);
    }
    z_servo_result = arm_z_servo_axis_start(&controller->z_servo_axis);
    if (z_servo_result != ARM_Z_SERVO_AXIS_OK)
    {
        return arm_controller_from_z_servo_result(z_servo_result);
    }

    /* 上电第一步先让夹爪回到闭合 0 度，避免后续机械臂动作时夹爪张开。 */
    {
        gripper_servo_config_t gripper_config;
        gripper_servo_config_default(&gripper_config);
        if (gripper_servo_init(&controller->gripper, &gripper_config) !=
            GRIPPER_SERVO_OK)
        {
            return ARM_CONTROLLER_MOTOR_ERROR;
        }
    }

    /* 上电执行电机 5 的碰撞回零；回零方向由上位机保存的 O_Dir 参数决定。 */
    controller->lift_motor.home_flags = 0U;
    result = motor_home(&controller->lift_motor,
                        ARM_CONTROLLER_LIFT_HOME_MODE,
                        MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    controller_result = arm_controller_wait_home(&controller->lift_motor);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }

    /* 碰撞回零完成后，将当前位置明确设置为 0 度。 */
    result = motor_clear_position(&controller->lift_motor);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }
    controller_result = arm_controller_capture_motor_feedback_zero(
        &controller->lift_motor);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }

    /* 水平轴不执行自动回零；上电前由人工置于最里侧 0 度。 */
    result = motor_clear_position(&controller->horizontal_motor);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }
    controller_result = arm_controller_capture_motor_feedback_zero(
        &controller->horizontal_motor);
    if (controller_result != ARM_CONTROLLER_OK)
    {
        return controller_result;
    }

    /* 两个电机都完成安全归零后，再转到车身物料盘方向。 */
    z_servo_result = arm_z_servo_axis_move_to_position(
        &controller->z_servo_axis,
        ARM_CONTROLLER_Z_TRAY_POSITION);
    if (z_servo_result != ARM_Z_SERVO_AXIS_OK)
    {
        return arm_controller_from_z_servo_result(z_servo_result);
    }

    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_bind_z_servo(
    arm_controller_t *controller,
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    const arm_z_servo_axis_config_t *config)
{
    arm_z_servo_axis_result_t result;

    if ((controller == NULL) || (config == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    result = arm_z_servo_axis_bind(&controller->z_servo_axis,
                                   huart,
                                   servo_id,
                                   config);
    if (result == ARM_Z_SERVO_AXIS_OK)
    {
        if ((huart != NULL) && (servo_id != 0U))
        {
            return arm_controller_from_z_servo_result(
                arm_z_servo_axis_start(&controller->z_servo_axis));
        }
        return ARM_CONTROLLER_OK;
    }
    if (result == ARM_Z_SERVO_AXIS_INVALID_PARAM)
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    if (result == ARM_Z_SERVO_AXIS_NOT_READY)
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    return ARM_CONTROLLER_MOTOR_ERROR;
}

arm_controller_result_t arm_controller_bind_z_servo_usart2(
    arm_controller_t *controller,
    const arm_z_servo_axis_config_t *config)
{
    if ((controller == NULL) || (config == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    return arm_controller_bind_z_servo(controller,
                                       &huart2,
                                       (config->servo_id != 0U) ?
                                       config->servo_id :
                                       ARM_Z_SERVO_AXIS_DEFAULT_SERVO_ID,
                                       config);
}

arm_controller_result_t arm_controller_init_horizontal_axis(
    arm_controller_t *controller,
    motor_t *motor,
    uint8_t motor_id,
    const arm_horizontal_axis_config_t *config)
{
    arm_horizontal_axis_result_t result;

    if ((controller == NULL) || (controller->bus == NULL) ||
        (motor == NULL) || (config == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    result = arm_horizontal_axis_init(&controller->horizontal_axis,
                                      controller->bus,
                                      motor,
                                      motor_id,
                                      config);
    switch (result)
    {
        case ARM_HORIZONTAL_AXIS_OK:
            return ARM_CONTROLLER_OK;
        case ARM_HORIZONTAL_AXIS_INVALID_PARAM:
            return ARM_CONTROLLER_INVALID_PARAM;
        case ARM_HORIZONTAL_AXIS_BUSY:
            return ARM_CONTROLLER_BUSY;
        case ARM_HORIZONTAL_AXIS_NOT_READY:
            return ARM_CONTROLLER_NOT_READY;
        default:
            return ARM_CONTROLLER_MOTOR_ERROR;
    }
}

arm_controller_result_t arm_controller_set_target_ring(
    arm_controller_t *controller,
    uint8_t target_ring)
{
    if ((controller == NULL) || (controller->initialized == 0U) ||
        (target_ring < 1U) || (target_ring > 3U))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }
    controller->target_ring = target_ring;
    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_face_target_ring(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_z_position(
        controller,
        arm_controller_target_ring_z_position(controller));
}

arm_controller_result_t arm_controller_hold_z_position(
    arm_controller_t *controller)
{
    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->z_servo_axis.initialized == 0U) ||
        (controller->z_servo_axis.hardware_bound == 0U))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_zero(arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 调零开始前先闭合夹爪。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 调零时重新确认升降轴最高点 0 度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_ZERO_LIFT_HOME_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 水平轴不主动运动；调用者应在上电前手动置于最里侧后清零。 */
    if (motor_clear_position(&controller->horizontal_motor) != MOTOR_OK)
    {
        return ARM_CONTROLLER_MOTOR_ERROR;
    }
    result = arm_controller_capture_motor_feedback_zero(
        &controller->horizontal_motor);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 垂直轴升高后只旋转 Z 轴；水平轴保持刚清零的最里侧位置。 */
    return arm_controller_move_z_position_only(
        controller,
        ARM_CONTROLLER_Z_TRAY_POSITION);
}

arm_controller_result_t arm_controller_prepare_travel_safe(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_ZERO_LIFT_HOME_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_horizontal_angle(
        controller,
        ARM_CONTROLLER_HORIZONTAL_ZERO_ANGLE_DEG);
}

arm_controller_result_t arm_controller_pick_and_place(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;
    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 调用前应已由平台夹取准备函数下降到取料高度，先等待机构稳定。 */
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 上升到最高点 0 度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 保持最高点，直接转到车身物料盘方向。 */
    result = arm_controller_move_z_position_only(
        controller,
        ARM_CONTROLLER_Z_TRAY_POSITION);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    /* Z 轴函数内部已等待配置的运动时间；再稳定 300 ms 后下降放料。 */
    HAL_Delay(ARM_CONTROLLER_PLATFORM_TRAY_WAIT_MS);

    /* 5 号下降与 6 号到物料盘绝对标定位置同步启动，到位后再松爪。 */
    result = arm_controller_move_lift_horizontal_absolute_sync(
        controller,
        ARM_CONTROLLER_PLATFORM_TRAY_PLACE_LIFT_ANGLE_DEG,
        ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_OPEN_SETTLE_MS);

    /* 物料脱离后，5 号上升与 6 号绝对收回 0 度同步执行。 */
    return arm_controller_move_lift_horizontal_absolute_sync(
        controller,
        0.0f,
        ARM_CONTROLLER_HORIZONTAL_ZERO_ANGLE_DEG);
}

arm_controller_result_t arm_controller_ground_pick_and_place(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_ground_pick(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_place_to_tray(controller);
}

arm_controller_result_t arm_controller_ground_pick(
    arm_controller_t *controller)
{
    gripper_servo_result_t gripper_result;


    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 调用前应已由地面夹取准备函数下降到取料高度，先等待机构稳定。 */
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 夹紧物料并保持 300 ms 后上升；到位后才向上层报告抓取成功。 */
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_place_to_tray(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 升高后再转到车身物料盘方向。 */
    result = arm_controller_move_to_tray_direction(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 延时后打开夹爪，放下物料。 */
    result = arm_controller_release_gripper_delay(
        controller,
        ARM_CONTROLLER_TRAY_RELEASE_DELAY_MS);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_place_to_ground(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_pick_from_tray(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_place_to_target_ring(controller);
}

arm_controller_result_t arm_controller_pick_from_tray(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 从小车物料盘取料时只打开 20 度，避免干涉相邻物料。 */
    result = arm_controller_open_tray_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 先在最高点确认物料盘方向和水平伸长位置，再下降。 */
    result = arm_controller_move_z_position(
        controller,
        ARM_CONTROLLER_Z_TRAY_POSITION);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    /* 下降到物料盘取料高度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_TRAY_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降到位后等待机构稳定，再夹取物料盘中的物料。 */
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 夹紧物料并保持 300 ms 后，再抬高到最高点。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return result;
}

arm_controller_result_t arm_controller_place_to_target_ring(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 先在最高点转到目标靶环方向，再下降。 */
    result = arm_controller_move_z_position(
        controller,
        arm_controller_target_ring_z_position(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_horizontal_angle(
        controller,
        arm_controller_target_ring_horizontal_angle(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降到靶环放料高度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 延时后打开夹爪，将物料释放到靶环。 */
    result = arm_controller_release_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 最后上升到最高点。 */
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}

static arm_controller_result_t arm_controller_place_ring_item_from_tray_internal(
    arm_controller_t *controller,
    uint8_t lift_after_place)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 视觉对准结束时机械臂在地面放置姿态，先抬升并回到物料盘方向。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_position(
        controller,
        ARM_CONTROLLER_Z_TRAY_POSITION);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_open_tray_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_TRAY_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_position(
        controller,
        arm_controller_target_ring_z_position(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_horizontal_angle(
        controller,
        arm_controller_target_ring_horizontal_angle(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_release_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    if (lift_after_place == 0U)
    {
        /* 松爪稳定后保持低位，供移车后直接识别下一靶环。 */
        HAL_Delay(ARM_CONTROLLER_GROUND_RELEASE_SETTLE_MS);
        return ARM_CONTROLLER_OK;
    }
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_place_ring_item_from_tray(
    arm_controller_t *controller)
{
    return arm_controller_place_ring_item_from_tray_internal(controller, 1U);
}

arm_controller_result_t arm_controller_place_ring_item_from_tray_keep_low(
    arm_controller_t *controller)
{
    return arm_controller_place_ring_item_from_tray_internal(controller, 0U);
}

arm_controller_result_t arm_controller_place_stack_item_from_tray(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 视觉对准完成时机械臂面向暂存区、位于地面放置高度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_open_tray_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_angle(
        controller,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降到暂存区第一批物料顶部的码垛高度后松开。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_release_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_finish_place(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* Z 轴舵机转到机械逆时针方向。 */
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 放置结束后关闭夹爪。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    return ARM_CONTROLLER_OK;
}

static arm_controller_result_t arm_controller_prepare_position(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* Z 轴舵机转到机械顺时针方向。 */
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_preposition_platform_z(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_prepare_travel_safe(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_move_z_position_only(
        controller,
        ARM_CONTROLLER_Z_PLATFORM_POSITION);
}

arm_controller_result_t arm_controller_prepare_platform_pick(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_prepare_travel_safe(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_open_gripper_at_angle(
        controller,
        GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_move_z_position_only(
        controller,
        ARM_CONTROLLER_Z_PLATFORM_POSITION);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_platform_pick_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_prepare_travel_safe(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_open_gripper_at_angle(
        controller,
        GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* Z 轴已由调用者预置到平台位置，平台取料阶段不再旋转或伸缩。 */
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_ground_pick(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 夹取动作先打开夹爪，再移动 Z 轴、水平轴和升降轴。 */
    result = arm_controller_open_gripper_at_angle(
        controller,
        GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_prepare_position(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_ground_place(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_prepare_position(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_ground_pick_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 在最高点打开夹爪并等待 100 ms，再移动水平轴和升降轴。 */
    result = arm_controller_open_gripper_at_angle(
        controller,
        GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_move_horizontal_angle(
        controller,
        arm_controller_target_ring_horizontal_angle(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    return result;
}

arm_controller_result_t arm_controller_ground_pick_to_tray_and_return(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 视觉细调完成后夹取地面物料，并抬升到最高点。 */
    result = arm_controller_ground_pick(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* Z 轴转到料盘方向，水平轴先回到料盘基准位置。 */
    result = arm_controller_move_to_tray_direction(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    /* Z轴和水平轴均已确认到位，仅保留短暂的机构稳定时间。 */
    HAL_Delay(ARM_CONTROLLER_GROUND_TRAY_WAIT_MS);

    /* 5 号下降，6 号保持物料盘绝对标定位置，两轴同步确认到位。 */
    result = arm_controller_move_lift_horizontal_absolute_sync(
        controller,
        ARM_CONTROLLER_PLATFORM_TRAY_PLACE_LIFT_ANGLE_DEG,
        ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_OPEN_SETTLE_MS);

    /* 松爪稳定后，5 号上升，6 号保持物料盘绝对标定位置。 */
    result = arm_controller_move_lift_horizontal_absolute_sync(
        controller,
        0.0f,
        ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 保持最高点，转回当前目标环方向。 */
    return arm_controller_move_z_angle(
        controller,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
}

arm_controller_result_t arm_controller_prepare_ground_place_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 放置动作保持夹爪闭合，完成旋转、伸缩和下降后由放置函数松爪。 */
    result = arm_controller_move_horizontal_angle(
        controller,
        arm_controller_target_ring_horizontal_angle(controller));
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    return result;
}

arm_controller_result_t arm_controller_prepare_target_ring_hold(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    /* Z 轴舵机转回靶环方向。 */
    result = arm_controller_move_z_angle(
        controller,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_stack_place(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_prepare_position(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_stack_pick(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 任务8结束时机械臂在物料盘最高点，先转到暂存区方向；
     * 对准后统一执行一次开爪、等待 100 ms 和下降。 */
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_prepare_stack_pick_aligned(controller);
}

arm_controller_result_t arm_controller_prepare_stack_pick_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    result = arm_controller_open_gripper_at_angle(
        controller,
        GRIPPER_SERVO_DEFAULT_OPEN_ANGLE_DEG);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

static arm_controller_result_t arm_controller_stack_item_from_tray_internal(
    arm_controller_t *controller,
    uint8_t lift_after_place)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 视觉对准完成时位于暂存区码垛高度，先抬升再回物料盘。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    result = arm_controller_open_tray_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_release_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    if (lift_after_place == 0U)
    {
        /* 已确认码垛高度可避开双层物料，松爪稳定后保持该姿态移车。 */
        HAL_Delay(ARM_CONTROLLER_STACK_RELEASE_SETTLE_MS);
        return ARM_CONTROLLER_OK;
    }
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_stack_item_from_tray(
    arm_controller_t *controller)
{
    return arm_controller_stack_item_from_tray_internal(controller, 1U);
}

arm_controller_result_t arm_controller_stack_item_from_tray_keep_low(
    arm_controller_t *controller)
{
    return arm_controller_stack_item_from_tray_internal(controller, 0U);
}

arm_controller_result_t arm_controller_return_to_tray_direction(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_place_to_stack(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 1. 上升到最高点 0 度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 2. Z 轴舵机转到物料盘方向。 */
    result = arm_controller_move_z_angle(
        controller,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降取料前只打开 20 度，避免夹爪碰到物料盘上的其他物料。 */
    result = arm_controller_open_tray_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 3. 下降到物料盘取料高度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 4. 下降到位后等待机构稳定，再闭合夹爪夹取物料盘物料。 */
    HAL_Delay(ARM_CONTROLLER_PICK_POSITION_SETTLE_MS);
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 5. 夹紧物料并保持 300 ms 后，上升到最高点 0 度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 6. Z 轴舵机转回地面方向。 */
    result = arm_controller_move_z_angle(
        controller,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 7. 下降到与码垛准备相同的高度。 */
    result = arm_controller_move_lift_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 8. 延时后打开夹爪，放下物料。 */
    result = arm_controller_release_gripper(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 9. 返回最高点 0 度。 */
    return arm_controller_move_lift_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}
