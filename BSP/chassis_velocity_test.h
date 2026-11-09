#ifndef ENGINEERING_INNOVATION_CHASSIS_VELOCITY_TEST_H
#define ENGINEERING_INNOVATION_CHASSIS_VELOCITY_TEST_H

#include "HWT101CT.h"
#include "HEADING_HOLD_CONTROLLER.h"
#include "JY901S_ACCEL_VELOCITY.h"
#include "VELOCITY_COMPLEMENTARY_FILTER.h"
#include "VELOCITY_ODOMETRY.h"
#include "chassis_mecanum.h"

#include <stdint.h>

typedef enum
{
    CHASSIS_VELOCITY_TEST_OK = 0,
    CHASSIS_VELOCITY_TEST_INVALID_PARAM
} chassis_velocity_test_result_t;

chassis_velocity_test_result_t chassis_velocity_test_init(
    mecanum_chassis_t *chassis,
    const hwt101ct_t *hwt101ct_sensor,
    const jy901s_t *jy901s_sensor,
    jy901s_accel_velocity_t *accel_velocity);

void chassis_velocity_test_task(void);

#endif /* ENGINEERING_INNOVATION_CHASSIS_VELOCITY_TEST_H */
