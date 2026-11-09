#ifndef ENGINEERING_INNOVATION_CHASSIS_FULL_FLOW_TEST_H
#define ENGINEERING_INNOVATION_CHASSIS_FULL_FLOW_TEST_H

#include "HEADING_HOLD_CONTROLLER.h"
#include "HWT101CT.h"
#include "VELOCITY_ODOMETRY.h"
#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "material_tray_servo.h"
#include "start_zone.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_FULL_FLOW_TEST_POINT_DWELL_MS 1000U

typedef void (*chassis_full_flow_test_status_fn)(uint8_t task_number,
                                                  uint16_t step_number,
                                                  const char *event);

typedef enum
{
    CHASSIS_FULL_FLOW_TEST_OK = 0,
    CHASSIS_FULL_FLOW_TEST_WAITING,
    CHASSIS_FULL_FLOW_TEST_DONE_RESULT,
    CHASSIS_FULL_FLOW_TEST_INVALID_PARAM,
    CHASSIS_FULL_FLOW_TEST_CHASSIS_ERROR,
    CHASSIS_FULL_FLOW_TEST_ARM_ERROR,
    CHASSIS_FULL_FLOW_TEST_SENSOR_ERROR,
    CHASSIS_FULL_FLOW_TEST_ODOMETRY_ERROR,
    CHASSIS_FULL_FLOW_TEST_TIMEOUT
} chassis_full_flow_test_result_t;

typedef enum
{
    CHASSIS_FULL_FLOW_TEST_WAIT_START = 0,
    CHASSIS_FULL_FLOW_TEST_WAIT_YAW,
    CHASSIS_FULL_FLOW_TEST_SETTLING,
    CHASSIS_FULL_FLOW_TEST_RUNNING,
    CHASSIS_FULL_FLOW_TEST_DONE,
    CHASSIS_FULL_FLOW_TEST_FAILED
} chassis_full_flow_test_state_t;

typedef struct
{
    mecanum_chassis_t *chassis;
    hwt101ct_t *yaw_sensor;
    arm_controller_t *arm;
    material_tray_servo_t *tray;
    chassis_full_flow_test_status_fn write_status;
    velocity_odometry_t odometry;
    heading_hold_controller_t heading_hold;
    uint32_t state_start_tick;
    uint32_t action_start_tick;
    uint32_t action_duration_ms;
    uint32_t stop_start_tick;
    uint32_t sensor_resync_start_tick;
    uint32_t last_control_tick;
    uint32_t last_encoder_ok_tick;
    uint32_t stop_stable_start_tick;
    float target_yaw_deg;
    float segment_start_x_m;
    float segment_start_y_m;
    float segment_target_m;
    float direction_body_x;
    float direction_body_y;
    float progress_m;
    float remaining_m;
    float cross_track_error_m;
    float cross_track_integral_m_s;
    float cross_correction_mps;
    uint16_t action_index;
    uint8_t action_active;
    uint8_t action_mode;
    uint8_t stop_stable_active;
    uint8_t sensor_resync_active;
    int8_t rotation_direction;
    start_zone_t start_zone;
    chassis_full_flow_test_state_t state;
} chassis_full_flow_test_t;

/** 初始化闭环底盘全流程测试，不会立即发出运动命令。 */
chassis_full_flow_test_result_t chassis_full_flow_test_init(
    chassis_full_flow_test_t *test,
    mecanum_chassis_t *chassis,
    hwt101ct_t *yaw_sensor,
    arm_controller_t *arm,
    material_tray_servo_t *tray,
    chassis_full_flow_test_status_fn write_status);

/** 接收串口屏选定的启停区并开始一次固定点位流程。 */
chassis_full_flow_test_result_t chassis_full_flow_test_request_start(
    chassis_full_flow_test_t *test,
    start_zone_t start_zone);

/** 主循环周期调用；内部控制周期与正方形测试一致，为 20 ms。 */
chassis_full_flow_test_result_t chassis_full_flow_test_task(
    chassis_full_flow_test_t *test);

chassis_full_flow_test_state_t chassis_full_flow_test_get_state(
    const chassis_full_flow_test_t *test);

uint16_t chassis_full_flow_test_get_step(
    const chassis_full_flow_test_t *test);

uint16_t chassis_full_flow_test_get_step_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_CHASSIS_FULL_FLOW_TEST_H */
