#ifndef ENGINEERING_INNOVATION_TASK10_RETURN_HOME_H
#define ENGINEERING_INNOVATION_TASK10_RETURN_HOME_H

#include "QR.h"
#include "chassis_mecanum.h"
#include "screen.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK10_RETURN_HOME_CENTER_RING 2U
#define TASK10_RETURN_HOME_LEFT_MM 1870U
#define TASK10_RETURN_HOME_BACKWARD_MM 1020U
/* 启停区2从暂存区返回起点时，以车体前向移动950 mm。 */
#define TASK10_RETURN_HOME_ZONE2_FORWARD_MM 950U

typedef void (*task10_return_home_status_fn)(uint8_t status,
                                             const char *event);

typedef enum
{
    TASK10_RETURN_HOME_WAIT_TASK = 0,
    TASK10_RETURN_HOME_MOVE_TO_CENTER,
    TASK10_RETURN_HOME_WAIT_CENTER_MOVE,
    TASK10_RETURN_HOME_LEFT,
    TASK10_RETURN_HOME_WAIT_LEFT,
    TASK10_RETURN_HOME_LONGITUDINAL,
    TASK10_RETURN_HOME_WAIT_LONGITUDINAL,
    TASK10_RETURN_HOME_DONE,
    TASK10_RETURN_HOME_FAILED,
    /* 保留旧状态名和数值，兼容已有调试代码。 */
    TASK10_RETURN_HOME_BACKWARD = TASK10_RETURN_HOME_LONGITUDINAL,
    TASK10_RETURN_HOME_WAIT_BACKWARD = TASK10_RETURN_HOME_WAIT_LONGITUDINAL
} task10_return_home_state_t;

typedef enum
{
    TASK10_RETURN_HOME_OK = 0,
    TASK10_RETURN_HOME_WAITING,
    TASK10_RETURN_HOME_INVALID_PARAM,
    TASK10_RETURN_HOME_CHASSIS_ERROR,
    TASK10_RETURN_HOME_SCREEN_ERROR
} task10_return_home_result_t;

typedef enum
{
    TASK10_RETURN_HOME_STATUS_INIT = 0U,
    TASK10_RETURN_HOME_STATUS_MOVE = 1U,
    TASK10_RETURN_HOME_STATUS_DONE = 2U,
    TASK10_RETURN_HOME_STATUS_ERROR = 255U
} task10_return_home_status_t;

typedef struct
{
    mecanum_chassis_t *chassis;
    screen_t *screen;
    task10_return_home_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t current_ring;
    uint8_t task_ready;
    start_zone_t start_zone;
    task10_return_home_state_t state;
} task10_return_home_t;

task10_return_home_result_t task10_return_home_init(
    task10_return_home_t *task,
    mecanum_chassis_t *chassis,
    screen_t *screen,
    task10_return_home_status_fn write_status);

/** 仅允许尚未提交任务时设置返回终点；初始化默认启停区1。 */
task10_return_home_result_t task10_return_home_set_start_zone(
    task10_return_home_t *task, start_zone_t start_zone);

task10_return_home_result_t task10_return_home_submit_qr(
    task10_return_home_t *task,
    const QR_TaskCode_t *qr_task);

task10_return_home_result_t task10_return_home_step(
    task10_return_home_t *task);

task10_return_home_state_t task10_return_home_get_state(
    const task10_return_home_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK10_RETURN_HOME_H */
