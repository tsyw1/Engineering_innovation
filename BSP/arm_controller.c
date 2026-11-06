//
// 机械臂整体动作控制实现。
//

#include "arm_controller.h"

#include <math.h>

#define ARM_CONTROLLER_POSITION_TIMEOUT_MS    30000U
#define ARM_CONTROLLER_HOME_TIMEOUT_MS        20000U
#define ARM_CONTROLLER_HOME_START_TIMEOUT_MS  3000U
#define ARM_CONTROLLER_POLL_INTERVAL_MS       20U

/* 当前工程只有一套机械臂；用它把位置命令和周期保持服务关联起来。 */
static arm_controller_t *s_arm_controller;

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

static arm_controller_result_t arm_controller_move_motor_angle(
    motor_t *motor,
    float angle_deg,
    motor_position_mode_t mode)
{
    int64_t pulse;
    int64_t hold_target_pulse;
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) || !isfinite(angle_deg))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    pulse = arm_controller_angle_to_pulse(angle_deg);
    if ((motor->id == ARM_CONTROLLER_Z_MOTOR_ID) &&
        (s_arm_controller != NULL))
    {
        /* 新的旋转动作开始后，等待本次动作重新到位再进入保持。 */
        s_arm_controller->z_last_hold_tick = 0U;
    }
    hold_target_pulse = pulse;
    if ((motor->id == ARM_CONTROLLER_Z_MOTOR_ID) &&
        (s_arm_controller != NULL))
    {
        if (mode == MOTOR_POSITION_ABSOLUTE)
        {
            hold_target_pulse = pulse;
        }
        else if (s_arm_controller->z_hold_valid != 0U)
        {
            hold_target_pulse = s_arm_controller->z_hold_target_pulse + pulse;
        }
        else if (motor->position_valid != 0U)
        {
            hold_target_pulse = motor->actual_position_pulse + pulse;
        }
    }
    /* 清除上一条位置命令的到位缓存，再发送新的位置命令。 */
    motor->state_flags &= (uint8_t)~MOTOR_STATE_POSITION_REACHED;
    result = motor_set_position(motor,
                                pulse,
                                (motor->id == ARM_CONTROLLER_LIFT_MOTOR_ID) ?
                                ARM_CONTROLLER_LIFT_SPEED_RPM :
                                ARM_CONTROLLER_Z_SPEED_RPM,
                                (motor->id == ARM_CONTROLLER_LIFT_MOTOR_ID) ?
                                ARM_CONTROLLER_LIFT_ACCELERATION :
                                ARM_CONTROLLER_Z_ACCELERATION,
                                mode,
                                MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }
    if ((motor->id == ARM_CONTROLLER_Z_MOTOR_ID) &&
        (s_arm_controller != NULL))
    {
        s_arm_controller->z_hold_target_pulse = hold_target_pulse;
        s_arm_controller->z_hold_valid = 1U;
    }
    return arm_controller_wait_position(motor);
}

arm_controller_result_t arm_controller_init(arm_controller_t *controller,
                                            motor_bus_t *bus)
{
    motor_result_t result;
    arm_controller_result_t controller_result;

    if ((controller == NULL) || (bus == NULL))
    {
        return ARM_CONTROLLER_INVALID_PARAM;
    }

    controller->bus = bus;
    controller->initialized = 0U;
    controller->z_hold_target_pulse = 0;
    controller->z_last_hold_tick = 0U;
    controller->z_hold_valid = 0U;
    s_arm_controller = controller;

    result = motor_init(&controller->lift_motor,
                        bus,
                        ARM_CONTROLLER_LIFT_MOTOR_ID);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    result = motor_init(&controller->z_motor,
                        bus,
                        ARM_CONTROLLER_Z_MOTOR_ID);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    controller->initialized = 1U;

    /* 初始化阶段只让升降轴回最高点并建立升降轴零点。 */
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

    /* 碰撞回零完成后，再明确发送一次当前位置清零。 */
    result = motor_clear_position(&controller->lift_motor);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }

    {
        gripper_servo_config_t gripper_config;
        gripper_servo_config_default(&gripper_config);
        if (gripper_servo_init(&controller->gripper, &gripper_config) !=
            GRIPPER_SERVO_OK)
        {
            return ARM_CONTROLLER_MOTOR_ERROR;
        }
    }

    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_hold_z_position(
    arm_controller_t *controller)
{
    uint32_t now;
    motor_result_t result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }
    if (controller->z_hold_valid == 0U)
    {
        return ARM_CONTROLLER_OK;
    }

    /* 位置动作尚未到位时不打断原有轨迹。 */
    if ((controller->z_motor.state_flags & MOTOR_STATE_POSITION_REACHED) == 0U &&
        (controller->z_last_hold_tick == 0U))
    {
        return ARM_CONTROLLER_OK;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - controller->z_last_hold_tick) <
        ARM_CONTROLLER_Z_HOLD_PERIOD_MS)
    {
        return ARM_CONTROLLER_OK;
    }

    /* 到位后用绝对目标轻量重发，驱动器会在负载扰动后回到原位置。 */
    result = motor_set_position(&controller->z_motor,
                                controller->z_hold_target_pulse,
                                ARM_CONTROLLER_Z_HOLD_SPEED_RPM,
                                ARM_CONTROLLER_Z_HOLD_ACCELERATION,
                                MOTOR_POSITION_ABSOLUTE,
                                MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(result);
    }
    controller->z_motor.state_flags &=
        (uint8_t)~MOTOR_STATE_POSITION_REACHED;
    controller->z_last_hold_tick = now;
    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_zero(arm_controller_t *controller)
{
    arm_controller_result_t result;
    motor_result_t motor_result;
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

    /* 机械顺时针 250 度：根据方向约定，电机执行 CCW 负脉冲。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_Z_PRE_HOME_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 升降电机负角度为 CCW，执行下降 -2948.9 度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_LIFT_DOWN_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* Z 轴顺时针碰撞回零：驱动器参数中的 CCW 对应机械顺时针。 */
    controller->z_motor.home_flags = 0U;
    motor_result = motor_home(&controller->z_motor,
                              ARM_CONTROLLER_Z_HOME_MODE,
                              MOTOR_EXECUTE_NOW);
    if (motor_result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(motor_result);
    }

    result = arm_controller_wait_home(&controller->z_motor);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* Z 轴碰撞回零成功后，清除旋转电机当前位置。 */
    motor_result = motor_clear_position(&controller->z_motor);
    if (motor_result != MOTOR_OK)
    {
        return arm_controller_from_motor_result(motor_result);
    }
    controller->z_hold_target_pulse = 0;
    controller->z_hold_valid = 1U;
    controller->z_last_hold_tick = HAL_GetTick();
    controller->z_motor.actual_position_pulse = 0;
    controller->z_motor.position_valid = 1U;

    /* 5 号电机回零完成后，6 号升降电机回到最高点 0 度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_ZERO_LIFT_HOME_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 升降轴到位后，5 号电机从零点机械逆时针旋转 270 度。 */
    return arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_ZERO_Z_AFTER_HOME_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
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

    /* 调用前应已由平台夹取准备函数下降到取料高度。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 上升到最高点 0 度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 机械逆时针旋转 180 度，电机方向为 CW。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
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

    /* 调用前应已由地面夹取准备函数下降到取料高度。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 上升到最高点 0 度；该动作完成后才向上层报告抓取成功。 */
    return arm_controller_move_motor_angle(
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

    /* 机械逆时针旋转 180 度，电机方向为 CW。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
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

    /* 调零结束后机械臂已在物料盘上方，先打开夹爪准备取料。 */
    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 下降到物料盘取料高度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 夹爪闭合，夹取物料盘中的物料。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 夹紧物料并保持 200 ms 后，再抬高到最高点。 */
    result = arm_controller_move_motor_angle(
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

    /* 机械顺时针旋转 180 度，返回靶环方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降到靶环放料高度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG,
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
    return arm_controller_move_motor_angle(
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
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG,
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
    return arm_controller_move_motor_angle(
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
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降到暂存区第一批物料顶部的码垛高度后松开。 */
    result = arm_controller_move_motor_angle(
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
    return arm_controller_move_motor_angle(
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

    /* 机械逆时针旋转 180 度，电机方向为 CW。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
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
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 从当前位置机械顺时针旋转 180 度，电机使用 CCW 负脉冲。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    return ARM_CONTROLLER_OK;
}

arm_controller_result_t arm_controller_prepare_platform_pick(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_prepare_position(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_platform_pick_aligned(
    arm_controller_t *controller)
{
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    return arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_ground_pick(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

    result = arm_controller_prepare_position(controller);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    return arm_controller_move_motor_angle(
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

    return arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
}

arm_controller_result_t arm_controller_prepare_ground_pick_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    /* 任务3结束时机械臂已经面向靶环，此处不重复旋转 Z 轴。 */
    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG,
        MOTOR_POSITION_ABSOLUTE);
    return result;
}

arm_controller_result_t arm_controller_ground_pick_to_tray_and_return(
    arm_controller_t *controller)
{
    arm_controller_result_t result;

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

    /* 机械逆时针旋转 180 度，转到物料盘方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 到达物料盘上方后等待 100 ms，再打开夹爪。 */
    result = arm_controller_release_gripper_delay(
        controller,
        ARM_CONTROLLER_TRAY_RELEASE_DELAY_MS);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_OPEN_SETTLE_MS);

    /* 夹爪打开并稳定后，顺时针旋转 180 度回到靶环方向。 */
    return arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
}

arm_controller_result_t arm_controller_prepare_ground_place_aligned(
    arm_controller_t *controller)
{
    arm_controller_result_t result;
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG,
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
    /* 机械顺时针旋转 180 度，回到靶环方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    return arm_controller_move_motor_angle(
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

    return arm_controller_move_motor_angle(
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

    /* 任务8结束时机械臂在物料盘最高点，顺时针转到暂存区方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
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
    gripper_servo_result_t gripper_result;

    if ((controller == NULL) || (controller->initialized == 0U) ||
        (controller->bus == NULL))
    {
        return ARM_CONTROLLER_NOT_READY;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    return arm_controller_move_motor_angle(
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
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }
    result = arm_controller_move_motor_angle(
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
    return arm_controller_move_motor_angle(
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
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
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
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 2. Z 轴机械逆时针旋转 180 度，转到物料盘方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 下降取料前先打开夹爪，避免夹爪碰到物料盘。 */
    gripper_result = gripper_servo_open(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }

    /* 3. 下降到物料盘取料高度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 4. 夹爪闭合，夹取物料盘物料。 */
    gripper_result = gripper_servo_close(&controller->gripper);
    if (gripper_result != GRIPPER_SERVO_OK)
    {
        return arm_controller_from_gripper_result(gripper_result);
    }
    HAL_Delay(ARM_CONTROLLER_TRAY_GRIP_DELAY_MS);

    /* 5. 夹紧物料并保持 200 ms 后，上升到最高点 0 度。 */
    result = arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 6. Z 轴机械顺时针旋转 180 度，转回地面方向。 */
    result = arm_controller_move_motor_angle(
        &controller->z_motor,
        -ARM_CONTROLLER_PLACE_Z_ANGLE_DEG,
        MOTOR_POSITION_RELATIVE_CURRENT);
    if (result != ARM_CONTROLLER_OK)
    {
        return result;
    }

    /* 7. 下降到与码垛准备相同的高度。 */
    result = arm_controller_move_motor_angle(
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
    return arm_controller_move_motor_angle(
        &controller->lift_motor,
        0.0f,
        MOTOR_POSITION_ABSOLUTE);
}
