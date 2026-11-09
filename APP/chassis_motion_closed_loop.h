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

/** 初始化完整任务共享的闭环底盘控制器。 */
chassis_motion_result_t chassis_motion_closed_loop_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *yaw_sensor);

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
