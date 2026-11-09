#ifndef ENGINEERING_INNOVATION_TASK1_START_QR_H
#define ENGINEERING_INNOVATION_TASK1_START_QR_H

#include "QR.h"
#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "screen.h"
#include "task_plan.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK1_START_QR_FORWARD_FIRST_MM 100U
#define TASK1_START_QR_LEFT_MM         1050U
#define TASK1_START_QR_RIGHT_MM        1050U
#define TASK1_START_QR_FORWARD_QR_MM   150U

typedef void (*task1_start_qr_status_fn)(uint8_t status,
                                          const char *event);

typedef enum
{
    TASK1_START_QR_WAIT_START = 0,
    TASK1_START_QR_FORWARD_FIRST,
    TASK1_START_QR_LATERAL_AND_ZERO,
    TASK1_START_QR_FORWARD_QR,
    TASK1_START_QR_WAIT_QR,
    TASK1_START_QR_PROCESS_QR,
    TASK1_START_QR_DONE,
    TASK1_START_QR_FAILED,
    /* 保留旧状态名，兼容现有调试代码。 */
    TASK1_START_QR_LEFT_AND_ZERO = TASK1_START_QR_LATERAL_AND_ZERO
} task1_start_qr_state_t;

typedef enum
{
    TASK1_START_QR_STATUS_INIT = 0U,
    TASK1_START_QR_STATUS_WAIT_START,
    TASK1_START_QR_STATUS_MOVE,
    TASK1_START_QR_STATUS_ARM_ZERO,
    TASK1_START_QR_STATUS_QR,
    TASK1_START_QR_STATUS_TASK_READY,
    TASK1_START_QR_STATUS_ERROR = 255U
} task1_start_qr_status_t;

typedef enum
{
    TASK1_START_QR_OK = 0,
    TASK1_START_QR_WAITING,
    TASK1_START_QR_INVALID_PARAM,
    TASK1_START_QR_CHASSIS_ERROR,
    TASK1_START_QR_ARM_ERROR,
    TASK1_START_QR_SCREEN_ERROR,
    TASK1_START_QR_QR_ERROR,
    TASK1_START_QR_PLAN_ERROR
} task1_start_qr_result_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    screen_t *screen;
    task1_start_qr_status_fn write_status;
    QR_TaskCode_t qr_task;
    task_execution_plan_t plan;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t start_requested;
    uint8_t task_ready;
    start_zone_t start_zone;
    task1_start_qr_state_t state;
} task1_start_qr_t;

task1_start_qr_result_t task1_start_qr_init(
    task1_start_qr_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    screen_t *screen,
    task1_start_qr_status_fn write_status);

void task1_start_qr_request_start(task1_start_qr_t *task);

/** 仅允许尚未启动时设置起点；初始化默认启停区1。 */
task1_start_qr_result_t task1_start_qr_set_start_zone(
    task1_start_qr_t *task, start_zone_t start_zone);

task1_start_qr_result_t task1_start_qr_step(task1_start_qr_t *task);

task1_start_qr_state_t task1_start_qr_get_state(
    const task1_start_qr_t *task);

uint8_t task1_start_qr_get_task(
    const task1_start_qr_t *task,
    QR_TaskCode_t *qr_task);

uint8_t task1_start_qr_get_plan(
    const task1_start_qr_t *task,
    task_execution_plan_t *plan);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK1_START_QR_H */
