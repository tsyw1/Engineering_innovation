#ifndef ENGINEERING_INNOVATION_TASK5_TEMPORARY_RING_H
#define ENGINEERING_INNOVATION_TASK5_TEMPORARY_RING_H

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

#define TASK5_TEMPORARY_RING_ITEM_COUNT 3U
#define TASK5_TEMPORARY_RING_ROTATE_DEG 90U
#define TASK5_TEMPORARY_RING_BACKWARD_TO_AREA_MM 850U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_1_MM 1000U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_2_MM 860U
#define TASK5_TEMPORARY_RING_BACKWARD_RING_3_MM 700U
#define TASK5_TEMPORARY_RING_CENTER 2U
#define TASK5_TEMPORARY_RING_TRAY_SETTLE_MS 400U

typedef void (*task5_temporary_ring_status_fn)(uint8_t status,
                                                const char *event);

typedef enum
{
    TASK5_TEMPORARY_RING_WAIT_TASK = 0,
    TASK5_TEMPORARY_RING_BACKWARD_FROM_RING,
    TASK5_TEMPORARY_RING_WAIT_BACKWARD_FROM_RING,
    TASK5_TEMPORARY_RING_ROTATE,
    TASK5_TEMPORARY_RING_WAIT_ROTATE,
    TASK5_TEMPORARY_RING_BACKWARD_TO_AREA,
    TASK5_TEMPORARY_RING_WAIT_BACKWARD_TO_AREA,
    TASK5_TEMPORARY_RING_PREPARE_ITEM,
    TASK5_TEMPORARY_RING_ALIGN,
    TASK5_TEMPORARY_RING_PLACE_ITEM,
    TASK5_TEMPORARY_RING_DONE,
    TASK5_TEMPORARY_RING_FAILED
} task5_temporary_ring_state_t;

typedef enum
{
    TASK5_TEMPORARY_RING_OK = 0,
    TASK5_TEMPORARY_RING_WAITING,
    TASK5_TEMPORARY_RING_INVALID_PARAM,
    TASK5_TEMPORARY_RING_CHASSIS_ERROR,
    TASK5_TEMPORARY_RING_ARM_ERROR,
    TASK5_TEMPORARY_RING_VISION_ERROR,
    TASK5_TEMPORARY_RING_TRAY_ERROR,
    TASK5_TEMPORARY_RING_SCREEN_ERROR
} task5_temporary_ring_result_t;

typedef enum
{
    TASK5_TEMPORARY_RING_STATUS_INIT = 0U,
    TASK5_TEMPORARY_RING_STATUS_MOVE = 1U,
    TASK5_TEMPORARY_RING_STATUS_ROTATE = 2U,
    TASK5_TEMPORARY_RING_STATUS_PREPARE = 3U,
    TASK5_TEMPORARY_RING_STATUS_FINE_ADJUST = 4U,
    TASK5_TEMPORARY_RING_STATUS_PLACE = 5U,
    TASK5_TEMPORARY_RING_STATUS_DONE = 6U,
    TASK5_TEMPORARY_RING_STATUS_ERROR = 255U
} task5_temporary_ring_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task5_temporary_ring_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t item_index;
    uint8_t current_ring;
    uint8_t target_ring;
    uint8_t task_ready;
    task5_temporary_ring_state_t state;
} task5_temporary_ring_t;

task5_temporary_ring_result_t task5_temporary_ring_init(
    task5_temporary_ring_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task5_temporary_ring_status_fn write_status);

task5_temporary_ring_result_t task5_temporary_ring_submit_qr(
    task5_temporary_ring_t *task,
    const QR_TaskCode_t *qr_task);

task5_temporary_ring_result_t task5_temporary_ring_step(
    task5_temporary_ring_t *task);

task5_temporary_ring_state_t task5_temporary_ring_get_state(
    const task5_temporary_ring_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK5_TEMPORARY_RING_H */
