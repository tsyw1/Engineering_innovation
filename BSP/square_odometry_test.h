#ifndef ENGINEERING_INNOVATION_SQUARE_ODOMETRY_TEST_H
#define ENGINEERING_INNOVATION_SQUARE_ODOMETRY_TEST_H

#include "HWT101CT.h"
#include "chassis_mecanum.h"

typedef enum
{
    SQUARE_ODOMETRY_TEST_OK = 0,
    SQUARE_ODOMETRY_TEST_INVALID_PARAM
} square_odometry_test_result_t;

/* 初始化并自动开始四边形轨迹：前进、左移、后退、右移，各 2.1 m。 */
square_odometry_test_result_t square_odometry_test_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *hwt101ct_sensor);

/* 主循环周期调用，建议不低于 100 Hz。 */
void square_odometry_test_task(void);

#endif /* ENGINEERING_INNOVATION_SQUARE_ODOMETRY_TEST_H */
