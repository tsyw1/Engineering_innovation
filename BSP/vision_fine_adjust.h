#ifndef ENGINEERING_INNOVATION_VISION_FINE_ADJUST_H
#define ENGINEERING_INNOVATION_VISION_FINE_ADJUST_H

#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "HWT101CT.h"
#include "task_plan.h"
#include "vision_uart.h"

#include <stdint.h>

/* 当前中心与视觉参考中心的默认轴向容差，单位：像素。 */
#define VISION_FINE_ADJUST_DEFAULT_TOLERANCE_X_PIXEL  8
#define VISION_FINE_ADJUST_DEFAULT_TOLERANCE_Y_PIXEL  8

/* 平台取料后续物料的粗略容差，单位：像素。只由任务2/6临时启用。 */
#define VISION_FINE_ADJUST_COARSE_PICK_TOLERANCE_X_PIXEL  8
#define VISION_FINE_ADJUST_COARSE_PICK_TOLERANCE_Y_PIXEL  8

/* 微调期间保持启动时航向，单位分别为度和位置脉冲/度。 */
#define VISION_FINE_ADJUST_DEFAULT_HEADING_TOLERANCE_DEG  0.30f
#define VISION_FINE_ADJUST_DEFAULT_HEADING_KP_PULSES_PER_DEG  0.80f
#define VISION_FINE_ADJUST_DEFAULT_HEADING_MAX_CORRECTION_PULSES  16U
#define VISION_FINE_ADJUST_DEFAULT_HEADING_MAX_AGE_MS  120U

/* 临时流程联调开关：1=跳过视觉模式发送、视觉帧等待和底盘细调，
 * 保留机械臂准备及任务状态机；恢复视觉时改为 0。 */
#define VISION_FINE_ADJUST_BYPASS                    0U

typedef enum
{
    VISION_FINE_ADJUST_IDLE = 0,
    VISION_FINE_ADJUST_WAITING_FRAME,
    VISION_FINE_ADJUST_RUNNING,
    VISION_FINE_ADJUST_ALIGNED,
    VISION_FINE_ADJUST_TIMEOUT,
    VISION_FINE_ADJUST_ERROR
} vision_fine_adjust_state_t;

typedef enum
{
    VISION_FINE_ADJUST_OK = 0,
    VISION_FINE_ADJUST_WAITING,
    VISION_FINE_ADJUST_REACHED,
    VISION_FINE_ADJUST_INVALID_PARAM,
    VISION_FINE_ADJUST_UART_ERROR,
    VISION_FINE_ADJUST_CHASSIS_ERROR,
    VISION_FINE_ADJUST_ARM_ERROR,
    VISION_FINE_ADJUST_TIMEOUT_RESULT
} vision_fine_adjust_result_t;

typedef enum
{
    VISION_FINE_ADJUST_REJECT_NONE = 0U,
    VISION_FINE_ADJUST_REJECT_NO_FRAME,
    VISION_FINE_ADJUST_REJECT_MODE,
    VISION_FINE_ADJUST_REJECT_INVALID,
    VISION_FINE_ADJUST_REJECT_STATUS,
    VISION_FINE_ADJUST_REJECT_AGE,
    VISION_FINE_ADJUST_REJECT_TARGET
} vision_fine_adjust_reject_reason_t;

typedef struct
{
    /* 像素误差到车体速度的比例，需结合相机安装和实车标定。 */
    float vx_per_pixel;
    float vy_per_pixel;
    /* 摄像头图像坐标到车体坐标的方向符号。 */
    int8_t image_x_to_body_x_sign;
    int8_t image_y_to_body_y_sign;
    float max_vx_mps;
    float max_vy_mps;
    int16_t tolerance_x_pixel;
    int16_t tolerance_y_pixel;
    uint32_t max_age_ms;
    uint32_t timeout_ms;
    /* 位置模式细调参数：每帧最多发送少量相对位置脉冲。 */
    float position_pulses_per_pixel;
    uint8_t max_step_pulses;
    uint16_t position_speed_rpm;
    uint8_t position_acceleration;
    /* 两次位置脉冲之间的最小间隔，避免新命令连续打断上一条动作。 */
    uint16_t position_settle_ms;
    float heading_tolerance_deg;
    float heading_kp_pulses_per_deg;
    uint8_t heading_max_correction_pulses;
    uint32_t heading_max_age_ms;
} vision_fine_adjust_config_t;

typedef struct
{
    vision_uart_t *vision;
    mecanum_chassis_t *chassis;
    hwt101ct_t *yaw_sensor;
    vision_fine_adjust_config_t config;
    vision_uart_mode_t mode;
    uint8_t target_id;
    uint32_t start_tick;
    uint8_t command_active;
    uint32_t last_command_tick;
    float heading_target_deg;
    uint8_t heading_target_valid;
    vision_fine_adjust_reject_reason_t last_reject_reason;
    /* 1=后续物料采用粗略取料窗口，0=使用默认精调窗口。 */
    uint8_t coarse_pick_enabled;
    vision_fine_adjust_state_t state;
} vision_fine_adjust_t;

void vision_fine_adjust_config_default(vision_fine_adjust_config_t *config);
vision_fine_adjust_result_t vision_fine_adjust_init(
    vision_fine_adjust_t *adjust,
    vision_uart_t *vision,
    mecanum_chassis_t *chassis,
    const vision_fine_adjust_config_t *config);

/* 设置微调期间使用的航向传感器；传入 NULL 可关闭航向矫正。 */
void vision_fine_adjust_set_heading_sensor(
    vision_fine_adjust_t *adjust,
    hwt101ct_t *yaw_sensor);

/* 发送模式命令并开始等待视觉帧；夹取阶段传 mode=1，放置阶段传 mode=2。 */
vision_fine_adjust_result_t vision_fine_adjust_start(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id);

/* 提前发送模式命令，用于在长距离行驶期间完成摄像头模式切换。 */
vision_fine_adjust_result_t vision_fine_adjust_preselect_mode(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode);

/* 使用已提前选择的模式启动微调；清空旧帧但不重复发送模式命令。 */
vision_fine_adjust_result_t vision_fine_adjust_start_preselected(
    vision_fine_adjust_t *adjust,
    vision_uart_mode_t mode,
    uint8_t target_id);

/* 在主循环周期调用；每个有效视觉帧只执行一次相对位置脉冲。
 * 目标 ID 暂不匹配时仍允许使用其他有效目标校准，但不会返回已对准。 */
vision_fine_adjust_result_t vision_fine_adjust_step(
    vision_fine_adjust_t *adjust);

/* 设置平台取料的粗略窗口；关闭后恢复默认精调容差。 */
void vision_fine_adjust_set_coarse_pick(
    vision_fine_adjust_t *adjust,
    uint8_t enabled);

/* 执行机械臂准备动作后立即发送对应摄像头模式命令。 */
vision_fine_adjust_result_t vision_fine_adjust_prepare_pick_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t material_color);

/* 执行平台取料准备动作后，启动物料视觉模式。 */
vision_fine_adjust_result_t vision_fine_adjust_prepare_platform_pick_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t material_color);

/* 使用已提前选择的物块模式完成平台取料准备；清空旧帧但不重复发送模式命令。 */
vision_fine_adjust_result_t
vision_fine_adjust_prepare_platform_pick_and_start_preselected(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t material_color);

/* 按任务码批次和顺序取出颜色，再进入物料视觉模式。 */
vision_fine_adjust_result_t vision_fine_adjust_prepare_pick_for_task(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    const task_execution_plan_t *plan,
    task_plan_batch_t batch,
    uint8_t sequence_index);

vision_fine_adjust_result_t vision_fine_adjust_prepare_place_and_start(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t target_ring);

/* 完成放置姿态准备后，使用已提前选择的靶环模式启动微调。 */
vision_fine_adjust_result_t
vision_fine_adjust_prepare_place_and_start_preselected(
    vision_fine_adjust_t *adjust,
    arm_controller_t *arm,
    uint8_t target_ring);

void vision_fine_adjust_abort(vision_fine_adjust_t *adjust);
vision_fine_adjust_state_t vision_fine_adjust_get_state(
    const vision_fine_adjust_t *adjust);
vision_fine_adjust_reject_reason_t vision_fine_adjust_get_last_reject_reason(
    const vision_fine_adjust_t *adjust);

#endif /* ENGINEERING_INNOVATION_VISION_FINE_ADJUST_H */
