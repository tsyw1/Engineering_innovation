#ifndef ENGINEERING_INNOVATION_TASK4_RING_RETURN_H
#define ENGINEERING_INNOVATION_TASK4_RING_RETURN_H

#include "QR.h"
#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "material_tray_servo.h"
#include "screen.h"
#include "vision_fine_adjust.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK4_RING_RETURN_ITEM_COUNT 3U
#define TASK4_RING_RETURN_SPACING_MM 150U
#define TASK4_RING_RETURN_TRAY_SETTLE_MS 400U

typedef void (*task4_ring_return_status_fn)(uint8_t status,
                                            const char *event);

typedef enum
{
    TASK4_RING_RETURN_WAIT_TASK = 0,
    TASK4_RING_RETURN_MOVE_TO_ITEM,
    TASK4_RING_RETURN_WAIT_MOVE,
    TASK4_RING_RETURN_PREPARE_ITEM,
    TASK4_RING_RETURN_ALIGN,
    TASK4_RING_RETURN_PICK_AND_RETURN,
    TASK4_RING_RETURN_DONE,
    TASK4_RING_RETURN_FAILED
} task4_ring_return_state_t;

typedef enum
{
    TASK4_RING_RETURN_OK = 0,
    TASK4_RING_RETURN_WAITING,
    TASK4_RING_RETURN_INVALID_PARAM,
    TASK4_RING_RETURN_CHASSIS_ERROR,
    TASK4_RING_RETURN_ARM_ERROR,
    TASK4_RING_RETURN_VISION_ERROR,
    TASK4_RING_RETURN_TRAY_ERROR,
    TASK4_RING_RETURN_SCREEN_ERROR
} task4_ring_return_result_t;

typedef enum
{
    TASK4_RING_RETURN_STATUS_INIT = 0U,
    TASK4_RING_RETURN_STATUS_MOVE = 1U,
    TASK4_RING_RETURN_STATUS_PREPARE = 2U,
    TASK4_RING_RETURN_STATUS_FINE_ADJUST = 3U,
    TASK4_RING_RETURN_STATUS_PICK = 4U,
    TASK4_RING_RETURN_STATUS_PLACE = 5U,
    TASK4_RING_RETURN_STATUS_DONE = 6U,
    TASK4_RING_RETURN_STATUS_ERROR = 255U
} task4_ring_return_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task4_ring_return_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t item_index;
    uint8_t current_ring;
    uint8_t target_ring;
    uint8_t task_ready;
    /* 任务4按第一批放置顺序取回；任务8复用时切换为第二批目标顺序。 */
    uint8_t use_placement_order;
    task4_ring_return_state_t state;
} task4_ring_return_t;

task4_ring_return_result_t task4_ring_return_init(
    task4_ring_return_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task4_ring_return_status_fn write_status);

task4_ring_return_result_t task4_ring_return_submit_qr(
    task4_ring_return_t *task,
    const QR_TaskCode_t *qr_task);

task4_ring_return_result_t task4_ring_return_step(
    task4_ring_return_t *task);

task4_ring_return_state_t task4_ring_return_get_state(
    const task4_ring_return_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK4_RING_RETURN_H */
