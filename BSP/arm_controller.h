//
// 机械臂整体动作控制。
//

#ifndef ENGINEERING_INNOVATION_ARM_CONTROLLER_H
#define ENGINEERING_INNOVATION_ARM_CONTROLLER_H

#include "gripper_servo.h"
#include "arm_horizontal_axis.h"
#include "arm_z_servo_axis.h"
#include "motor_can.h"

#include <stdint.h>

#define ARM_CONTROLLER_VERTICAL_MOTOR_ID         5U
#define ARM_CONTROLLER_HORIZONTAL_MOTOR_ID       6U
#define ARM_CONTROLLER_LIFT_MOTOR_ID             ARM_CONTROLLER_VERTICAL_MOTOR_ID
#define ARM_CONTROLLER_COMMAND_PULSES_PER_REV   3200UL
#define ARM_CONTROLLER_LIFT_SPEED_RPM           5000U
#define ARM_CONTROLLER_LIFT_ACCELERATION          0U
/* 下降使用曲线加减速；速度保持与上升相同。 */
#define ARM_CONTROLLER_LIFT_DOWN_ACCELERATION     255U
#define ARM_CONTROLLER_HORIZONTAL_SPEED_RPM      70U
#define ARM_CONTROLLER_HORIZONTAL_ACCELERATION   50U
/* 5 号、6 号电机采用开环位置命令；命令发送后固定等待，不再等待到位标志。 */
#define ARM_CONTROLLER_LIFT_MOVE_WAIT_MS        150U
#define ARM_CONTROLLER_LIFT_DOWN_MOVE_WAIT_MS   500U
#define ARM_CONTROLLER_HORIZONTAL_MOVE_WAIT_MS  500U
#define ARM_CONTROLLER_RELEASE_DELAY_MS          300U
#define ARM_CONTROLLER_GROUND_RELEASE_SETTLE_MS  100U
#define ARM_CONTROLLER_STACK_RELEASE_SETTLE_MS   100U
#define ARM_CONTROLLER_TRAY_RELEASE_DELAY_MS     100U
#define ARM_CONTROLLER_TRAY_OPEN_SETTLE_MS      200U
#define ARM_CONTROLLER_GRIPPER_OPEN_DELAY_MS     100U
#define ARM_CONTROLLER_PLATFORM_PICK_OPEN_DELAY_MS 300U
#define ARM_CONTROLLER_TRAY_PICK_OPEN_DELAY_MS  150U
#define ARM_CONTROLLER_PICK_POSITION_SETTLE_MS  150U
#define ARM_CONTROLLER_PICK_LIFT_MOVE_WAIT_MS   ARM_CONTROLLER_LIFT_DOWN_MOVE_WAIT_MS
#define ARM_CONTROLLER_PLATFORM_GRIP_DELAY_MS    500U
#define ARM_CONTROLLER_TRAY_GRIP_DELAY_MS        500U
#define ARM_CONTROLLER_RING_HORIZONTAL_SETTLE_MS 300U
#define ARM_CONTROLLER_PLATFORM_ALIGN_SETTLE_MS  200U
#define ARM_CONTROLLER_PLATFORM_TRAY_WAIT_MS     300U
#define ARM_CONTROLLER_GROUND_TRAY_WAIT_MS       100U
#define ARM_CONTROLLER_TRAY_ROTATION_SETTLE_MS  100U
#define ARM_CONTROLLER_TRAY_PICK_OPEN_ANGLE_DEG   20U

/* 当前机械方向：升降电机 CCW 下降、CW 上升。
 * 2U 是 X42S 的“无限位碰撞回零”模式；回零方向不是此数值，
 * 而是电机驱动器内部的回零方向参数。 */
#define ARM_CONTROLLER_LIFT_HOME_MODE            2U

#define ARM_CONTROLLER_LIFT_DOWN_ANGLE_DEG       (-3660.6f)
#define ARM_CONTROLLER_ZERO_LIFT_HOME_ANGLE_DEG  0.0f
#define ARM_CONTROLLER_PLATFORM_LIFT_ANGLE_DEG   (-1253.5f)
/* 平台取料在原标定值基础上继续下降 50 度。 */
#define ARM_CONTROLLER_PLATFORM_PICK_LIFT_ANGLE_DEG (-1303.5f)
#define ARM_CONTROLLER_PLATFORM_TRAY_PLACE_LIFT_ANGLE_DEG (-180.0f)
#define ARM_CONTROLLER_TRAY_PICK_LIFT_ANGLE_DEG  (-700.4f)
#define ARM_CONTROLLER_GROUND_LIFT_ANGLE_DEG     (-3660.6f)
/* 地面取料在原标定值基础上继续下降 50 度。 */
#define ARM_CONTROLLER_GROUND_PICK_LIFT_ANGLE_DEG (-3600.6f)
#define ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG (-1700.6f)

#define ARM_CONTROLLER_Z_TRAY_POSITION           3830U
#define ARM_CONTROLLER_Z_PLATFORM_POSITION       1300U
#define ARM_CONTROLLER_Z_RING_1_POSITION         1887U
#define ARM_CONTROLLER_Z_RING_2_POSITION         1300U
#define ARM_CONTROLLER_Z_RING_3_POSITION         700U

#define ARM_CONTROLLER_HORIZONTAL_ZERO_ANGLE_DEG  0.0f
/* 物料盘水平轴使用新的绝对伸长位置。 */
#define ARM_CONTROLLER_HORIZONTAL_TRAY_ANGLE_DEG  (-24.6f)
#define ARM_CONTROLLER_HORIZONTAL_RING_1_ANGLE_DEG (-128.6f)
/* 2号靶环位于中心位置，放置时水平轴保持最里侧零位。 */
#define ARM_CONTROLLER_HORIZONTAL_RING_2_ANGLE_DEG (0.0f)
#define ARM_CONTROLLER_HORIZONTAL_RING_3_ANGLE_DEG (-118.4f)

/* 兼容旧任务接口；旧的“相对 180 度”命令现在映射为绝对协议位置。 */
#define ARM_CONTROLLER_ZERO_Z_ANGLE_DEG            0.0f
#define ARM_CONTROLLER_ZERO_Z_AFTER_HOME_DEG       3830.0f
#define ARM_CONTROLLER_PICK_LIFT_ANGLE_DEG         ARM_CONTROLLER_PLATFORM_PICK_LIFT_ANGLE_DEG
#define ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG ARM_CONTROLLER_GROUND_LIFT_ANGLE_DEG
#define ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG  ARM_CONTROLLER_TRAY_PICK_LIFT_ANGLE_DEG
#define ARM_CONTROLLER_PLACE_Z_ANGLE_DEG            180.0f
#define ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG          (-180.0f)

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
    motor_t horizontal_motor;
    arm_horizontal_axis_t horizontal_axis;
    arm_z_servo_axis_t z_servo_axis;
    gripper_servo_t gripper;
    uint8_t target_ring;
    uint8_t initialized;
} arm_controller_t;

/**
 * 初始化升降轴、Z 轴舵机和夹爪，并执行升降轴碰撞回零。
 */
arm_controller_result_t arm_controller_init(arm_controller_t *controller,
                                            motor_bus_t *bus);

/**
 * 绑定新的 Z 轴旋转舵机。串口和总线 ID 由硬件确认后传入；在绑定前，
 * z_servo_axis 仅作为未就绪的软件接口存在。
 */
arm_controller_result_t arm_controller_bind_z_servo(
    arm_controller_t *controller,
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    const arm_z_servo_axis_config_t *config);

/** 按当前硬件绑定 Z 轴舵机：USART2_TX / PA2。 */
arm_controller_result_t arm_controller_bind_z_servo_usart2(
    arm_controller_t *controller,
    const arm_z_servo_axis_config_t *config);

/** 初始化新的水平伸缩轴，使用绝对位置模式。motor_id 和行程参数待机械确认。 */
arm_controller_result_t arm_controller_init_horizontal_axis(
    arm_controller_t *controller,
    motor_t *motor,
    uint8_t motor_id,
    const arm_horizontal_axis_config_t *config);

/** 设置当前动作对应的靶环编号，用于选择 Z 舵机和水平轴标定位置。 */
arm_controller_result_t arm_controller_set_target_ring(
    arm_controller_t *controller,
    uint8_t target_ring);

/** 在最高点转到当前目标环方向，并将水平轴移动到该环的标定位置。 */
arm_controller_result_t arm_controller_face_target_ring(
    arm_controller_t *controller);

/**
 * 兼容旧任务层的 Z 轴保持接口。舵机由 PWM 持续保持目标位置，
 * 因此本接口只检查舵机是否已就绪，不再重发步进电机位置命令。
 */
arm_controller_result_t arm_controller_hold_z_position(
    arm_controller_t *controller);

/**
 * 执行机械臂调零顺序：夹爪闭合、升降轴回到最高点、清除水平轴当前位置，
 * 最后将 Z 轴舵机转到物料盘方向。
 */
arm_controller_result_t arm_controller_zero(arm_controller_t *controller);

/** 行车前抬升到最高点，并将水平轴绝对收回到最里侧 0 度。 */
arm_controller_result_t arm_controller_prepare_travel_safe(
    arm_controller_t *controller);

/**
 * 从已经准备好的平台取料并放到物料盘：闭合夹爪并保持 500 ms，
 * 升降轴回到 0 度，Z 轴等待配置的运动时间后再额外稳定 300 ms，
 * 先让 5 号升降轴下降到位，再让 6 号轴到绝对位置 -28.6 度，
 * 打开夹爪后，先让 5 号轴升高，再让 6 号轴绝对回到 0 度。
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
 * 先在最高点完成物料盘方向转向和水平伸长，再打开 20 度并等待 150 ms、下降取料，
 * 闭合保持 500 ms 后回到最高点。
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

/**
 * 连续地面放置：取盘料并放到靶环，松爪后等待 100 ms，保持放置低位、
 * 靶环朝向和夹爪打开。仅在已确认低位移车可避开物料时使用；
 * 移到下一靶环并完成视觉对准后，再调用取盘料放置动作。
 */
arm_controller_result_t arm_controller_place_ring_item_from_tray_keep_low(
    arm_controller_t *controller);

/** 从物料盘取料并放到暂存区已有物料上，完成一次码垛放置。 */
arm_controller_result_t arm_controller_place_stack_item_from_tray(
    arm_controller_t *controller);

/** 放置结束：机械逆时针旋转 180 度，并关闭夹爪。 */
arm_controller_result_t arm_controller_finish_place(
    arm_controller_t *controller);

/** 先确认水平轴收回到 0 度，再将 Z 轴转到平台位置，不下降。 */
arm_controller_result_t arm_controller_preposition_platform_z(
    arm_controller_t *controller);

/** 平台夹取准备：先完成收回、转向，再在最高点开爪等待 300 ms 后下降。 */
arm_controller_result_t arm_controller_prepare_platform_pick(
    arm_controller_t *controller);

/** 已朝向平台时的取料准备：水平轴收回 0 度，最高点开爪等待 300 ms 后下降。 */
arm_controller_result_t arm_controller_prepare_platform_pick_aligned(
    arm_controller_t *controller);

/** 地面夹取准备：Z 轴顺时针 180 度、打开夹爪、下降到地面取料高度。 */
arm_controller_result_t arm_controller_prepare_ground_pick(
    arm_controller_t *controller);

/** 已经面向靶环时的地面夹取准备：打开夹爪并下降，不旋转 Z 轴。 */
arm_controller_result_t arm_controller_prepare_ground_pick_aligned(
    arm_controller_t *controller);

/**
 * 从已经准备好的地面位置取料并放入物料盘：松爪稳定后升到最高点，
 * 将水平轴收回 0 度并结束，不再转回刚刚取料的靶环。
 * 仅供任务4使用，下一件由任务状态机设置新目标环。
 */
arm_controller_result_t arm_controller_ground_pick_to_tray(
    arm_controller_t *controller);

/**
 * 从已经准备好的地面位置取料，放入物料盘后再转回靶环方向：
 * Z 轴到达料盘方向后等待，先让 5 号轴下降到位，再让 6 号轴前伸，
 * 松爪稳定后先升高 5 号轴，再收回 6 号轴。
 */
arm_controller_result_t arm_controller_ground_pick_to_tray_and_return(
    arm_controller_t *controller);

/** 地面放置准备：水平轴保持 0 度，转到靶环方向后下降供视觉识别。 */
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

/** 已面向暂存区时，只打开夹爪并下降到码垛高度，不旋转 Z 轴。 */
arm_controller_result_t arm_controller_prepare_stack_pick_aligned(
    arm_controller_t *controller);

/** 在暂存区视觉对准后，从物料盘取料并码垛，放完抬到最高点。 */
arm_controller_result_t arm_controller_stack_item_from_tray(
    arm_controller_t *controller);

/**
 * 连续码垛：取盘料并码垛，松爪后等待 100 ms，保持码垛高度、
 * 暂存区朝向和夹爪打开。仅在已确认该高度可避开双层物料时使用；
 * 移车并完成下一件视觉对准后，再抬升取盘料。
 */
arm_controller_result_t arm_controller_stack_item_from_tray_keep_low(
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
