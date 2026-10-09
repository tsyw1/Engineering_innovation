//
// 机械臂整体动作控制。
//

#ifndef ENGINEERING_INNOVATION_ARM_CONTROLLER_H
#define ENGINEERING_INNOVATION_ARM_CONTROLLER_H

#include "gripper_servo.h"
#include "motor_can.h"

#include <stdint.h>

#define ARM_CONTROLLER_LIFT_MOTOR_ID             6U
#define ARM_CONTROLLER_Z_MOTOR_ID                5U
#define ARM_CONTROLLER_COMMAND_PULSES_PER_REV   3200UL
#define ARM_CONTROLLER_LIFT_SPEED_RPM           5000U
#define ARM_CONTROLLER_Z_SPEED_RPM               5000U
#define ARM_CONTROLLER_LIFT_ACCELERATION        200U
#define ARM_CONTROLLER_Z_ACCELERATION             50U
#define ARM_CONTROLLER_Z_HOLD_SPEED_RPM          120U
#define ARM_CONTROLLER_Z_HOLD_ACCELERATION        20U
#define ARM_CONTROLLER_Z_HOLD_PERIOD_MS          100U
#define ARM_CONTROLLER_RELEASE_DELAY_MS         1000U
#define ARM_CONTROLLER_TRAY_RELEASE_DELAY_MS     100U
#define ARM_CONTROLLER_TRAY_OPEN_SETTLE_MS      100U
#define ARM_CONTROLLER_TRAY_GRIP_DELAY_MS        200U

/* 当前机械方向：升降电机 CCW 下降、CW 上升。 */
#define ARM_CONTROLLER_LIFT_HOME_MODE            2U

/* Z 轴旋转电机：CCW 为机械顺时针，CW 为机械逆时针。 */
#define ARM_CONTROLLER_Z_HOME_MODE               2U
#define ARM_CONTROLLER_Z_PRE_HOME_ANGLE_DEG      (-250.0f)
#define ARM_CONTROLLER_LIFT_DOWN_ANGLE_DEG       (-3518.7f)
#define ARM_CONTROLLER_ZERO_LIFT_HOME_ANGLE_DEG  0.0f
#define ARM_CONTROLLER_ZERO_Z_AFTER_HOME_DEG     270.0f

#define ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG       (-3864.0f)
#define ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG (-7665.9f)
#define ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG (-7500.0f)
#define ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG (-895.5f)
#define ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG (-4663.7f)
#define ARM_CONTROLLER_PLACE_Z_ANGLE_DEG         180.0f
#define ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG       (-180.0f)

typedef enum
{
    ARM_CONTROLLER_OK = 0,
    ARM_CONTROLLER_INVALID_PARAM,
    ARM_CONTROLLER_BUSY,
    ARM_CONTROLLER_TIMEOUT,
    ARM_CONTROLLER_MOTOR_ERROR,
    ARM_CONTROLLER_HOME_FAILED,
    ARM_CONTROLLER_NOT_READY
} arm_controller_result_t;

typedef struct
{
    motor_bus_t *bus;
    motor_t lift_motor;
    motor_t z_motor;
    gripper_servo_t gripper;
    /* Z轴最后一次成功位置命令的绝对目标，用于运动期间抗惯性保持。 */
    int64_t z_hold_target_pulse;
    uint32_t z_last_hold_tick;
    uint8_t z_hold_valid;
    uint8_t initialized;
} arm_controller_t;

/**
 * 初始化机械臂电机，并只执行升降轴回最高点/归零。
 * 不执行 Z 轴旋转电机回零。
 */
arm_controller_result_t arm_controller_init(arm_controller_t *controller,
                                            motor_bus_t *bus);

/**
 * 周期性保持Z轴最后目标位置。应在主循环中高频调用；函数内部限频，
 * 仅在电机已到位后重发绝对位置命令，不会打断正在执行的旋转动作。
 */
arm_controller_result_t arm_controller_hold_z_position(
    arm_controller_t *controller);

/**
 * 执行机械臂调零顺序：
 * 1. 夹爪闭合；
 * 2. Z 轴机械顺时针 250 度；
 * 3. 升降轴电机 CCW 下降 -3518.7 度；
 * 4. Z 轴按驱动器保存的顺时针方向执行碰撞回零。
 * 5. Z 轴回零成功后，升降轴回到 0 度；
 * 6. 升降轴到位后，Z 轴机械逆时针旋转 270 度。
 */
arm_controller_result_t arm_controller_zero(arm_controller_t *controller);

/**
 * 从已经准备好的平台取料并放到物料盘：
 * 1. 夹爪闭合取料；
 * 3. 升降轴回到 0 度；
 * 4. Z 轴机械逆时针 180 度；
 * 5. 夹爪打开下料。
 */
arm_controller_result_t arm_controller_pick_and_place(
    arm_controller_t *controller);

/**
 * 从已经准备好的地面位置取料并放到物料盘：
 * 1. 夹爪闭合取料；
 * 3. 升降轴回到 0 度；
 * 4. Z 轴机械逆时针 180 度；
 * 5. 夹爪打开下料。
 */
arm_controller_result_t arm_controller_ground_pick_and_place(
    arm_controller_t *controller);

/** 从已经准备好的地面位置闭合夹爪并抬回安全高度。 */
arm_controller_result_t arm_controller_ground_pick(
    arm_controller_t *controller);

/** 将已抓取的物料旋转到物料盘方向并松开夹爪。 */
arm_controller_result_t arm_controller_place_to_tray(
    arm_controller_t *controller);

/** 从物料盘取料并放到靶环。 */
arm_controller_result_t arm_controller_place_to_ground(
    arm_controller_t *controller);

/**
 * 调零结束后机械臂已位于物料盘上方，本函数不再旋转 Z 轴。
 * 打开夹爪、下降取料、闭合保持 200 ms，然后回到最高点。
 */
arm_controller_result_t arm_controller_pick_from_tray(
    arm_controller_t *controller);

/** 将已从物料盘取出的物料放到靶环；调用前应已完成取料。 */
arm_controller_result_t arm_controller_place_to_target_ring(
    arm_controller_t *controller);

/**
 * 视觉对准地面放置位后，从物料盘取出当前物料并完成一次靶环放置：
 * 抬升、逆时针旋转到物料盘、下降夹取、抬升、顺时针旋转到靶环、
 * 下降放料并再次抬升到最高点。
 */
arm_controller_result_t arm_controller_place_ring_item_from_tray(
    arm_controller_t *controller);

/** 从物料盘取料并放到暂存区已有物料上，完成一次码垛放置。 */
arm_controller_result_t arm_controller_place_stack_item_from_tray(
    arm_controller_t *controller);

/** 放置结束：机械逆时针旋转 180 度，并关闭夹爪。 */
arm_controller_result_t arm_controller_finish_place(
    arm_controller_t *controller);

/** 平台夹取准备：Z 轴顺时针 180 度、打开夹爪、下降到平台取料高度。 */
arm_controller_result_t arm_controller_prepare_platform_pick(
    arm_controller_t *controller);

/** 已经朝向平台时的取料准备：不旋转Z轴，只打开夹爪并下降。 */
arm_controller_result_t arm_controller_prepare_platform_pick_aligned(
    arm_controller_t *controller);

/** 地面夹取准备：Z 轴顺时针 180 度、打开夹爪、下降到地面取料高度。 */
arm_controller_result_t arm_controller_prepare_ground_pick(
    arm_controller_t *controller);

/** 已经面向靶环时的地面夹取准备：打开夹爪并下降，不旋转 Z 轴。 */
arm_controller_result_t arm_controller_prepare_ground_pick_aligned(
    arm_controller_t *controller);

/** 从已经准备好的地面位置取料，放入物料盘后再转回靶环方向。 */
arm_controller_result_t arm_controller_ground_pick_to_tray_and_return(
    arm_controller_t *controller);

/** 地面放置准备：Z 轴顺时针 180 度、打开夹爪、下降到地面放料高度。 */
arm_controller_result_t arm_controller_prepare_ground_place(
    arm_controller_t *controller);

/**
 * 已经朝向靶环时的地面放置准备：不旋转 Z 轴，只打开夹爪并下降。
 * 用于连续放置多个物料，避免在相邻物料之间重复旋转 180 度。
 */
arm_controller_result_t arm_controller_prepare_ground_place_aligned(
    arm_controller_t *controller);

/** 从物料盘取料后顺时针旋转 180 度到靶环方向并下降，保持夹爪闭合用于坐标标定。 */
arm_controller_result_t arm_controller_prepare_target_ring_hold(
    arm_controller_t *controller);

/**
 * 码垛放置准备：Z 轴顺时针 180 度、打开夹爪、下降到指定码垛高度。
 * 当前码垛高度为 ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG。
 */
arm_controller_result_t arm_controller_prepare_stack_place(
    arm_controller_t *controller);

/** 从物料盘最高点顺时针转到暂存区，打开夹爪并下降到码垛高度。 */
arm_controller_result_t arm_controller_prepare_stack_pick(
    arm_controller_t *controller);

/** 在暂存区视觉对准后，从物料盘取料并完成一次码垛放置。 */
arm_controller_result_t arm_controller_stack_item_from_tray(
    arm_controller_t *controller);

/** 码垛完成后逆时针旋转回物料盘方向，并保持夹爪闭合。 */
arm_controller_result_t arm_controller_return_to_tray_direction(
    arm_controller_t *controller);

/**
 * 将物料从物料盘取出并放到码垛高度。
 * 使用 ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG 作为码垛高度。
 */
arm_controller_result_t arm_controller_place_to_stack(
    arm_controller_t *controller);

#endif /* ENGINEERING_INNOVATION_ARM_CONTROLLER_H */
