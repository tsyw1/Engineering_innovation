#ifndef ENGINEERING_INNOVATION_CHASSIS_MOTION_CLOSED_LOOP_H
#define ENGINEERING_INNOVATION_CHASSIS_MOTION_CLOSED_LOOP_H

#include "HWT101CT.h"
#include "chassis_mecanum.h"

#include <stdint.h>

typedef enum
{
    CHASSIS_MOTION_OK = 0,
    CHASSIS_MOTION_INVALID_PARAM,
    CHASSIS_MOTION_CHASSIS_ERROR,
    CHASSIS_MOTION_SENSOR_ERROR,
    CHASSIS_MOTION_TIMEOUT
} chassis_motion_result_t;

/** 底盘阻塞运动期间周期调用的非阻塞后台服务。 */
typedef void (*chassis_motion_service_fn)(void *context, uint32_t now_ms);

/** 初始化完整任务共享的闭环底盘控制器。 */
chassis_motion_result_t chassis_motion_closed_loop_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *yaw_sensor);

/**
 * 注册或清除底盘运动期间的后台服务。callback 为 NULL 时清除服务。
 * 回调必须立即返回，不能包含 HAL_Delay 或其他阻塞等待。
 */
chassis_motion_result_t chassis_motion_set_service_callback(
    chassis_motion_service_fn callback,
    void *context);

/** 保持启动参考航向，不再把当前已经偏转的航向重新设为目标。 */
chassis_motion_result_t chassis_motion_lock_current_heading(void);

/** 捕获小车上电后的初始偏航角，作为整个流程的固定航向参考。 */
chassis_motion_result_t chassis_motion_capture_startup_heading(void);

/** 读取当前流程应保持的目标航向。 */
chassis_motion_result_t chassis_motion_get_target_heading(
    float *target_yaw_deg);

/** 将当前车体航向矫正到本段流程目标航向。 */
chassis_motion_result_t chassis_motion_correct_heading(void);

/** 切换机械臂扰动期间的增强航向保持参数。 */
chassis_motion_result_t chassis_motion_set_heading_disturbance_compensation(
    uint8_t enable);

/** 按世界坐标目标航向执行一段直线/横移，函数返回时车辆已稳定停车。 */
chassis_motion_result_t chassis_motion_move_linear_mm(
    mecanum_chassis_t *chassis,
    int16_t body_x_mm,
    int16_t body_y_mm);

/** 按目标角度执行原地旋转，正值逆时针，负值顺时针。 */
chassis_motion_result_t chassis_motion_rotate_deg(
    mecanum_chassis_t *chassis,
    int16_t yaw_deg);

#endif /* ENGINEERING_INNOVATION_CHASSIS_MOTION_CLOSED_LOOP_H */
