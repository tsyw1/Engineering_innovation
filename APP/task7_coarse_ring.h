#ifndef ENGINEERING_INNOVATION_TASK7_COARSE_RING_H
#define ENGINEERING_INNOVATION_TASK7_COARSE_RING_H

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

#define TASK7_COARSE_RING_LEFT_MM       100U
#define TASK7_COARSE_RING_ROTATE_DEG    180U
#define TASK7_COARSE_RING_RIGHT_MM     1750U
#define TASK7_COARSE_RING_SPACING_MM    150U
#define TASK7_COARSE_RING_CENTER          2U
#define TASK7_COARSE_RING_ITEM_COUNT      3U
#define TASK7_COARSE_RING_TRAY_SETTLE_MS 500U

typedef void (*task7_coarse_ring_status_fn)(uint8_t status,
                                            const char *event);

typedef enum
{
    TASK7_COARSE_RING_WAIT_TASK = 0,
    TASK7_COARSE_RING_LEFT,
    TASK7_COARSE_RING_ROTATE,
    TASK7_COARSE_RING_RIGHT,
    TASK7_COARSE_RING_PREPARE_ITEM,
    TASK7_COARSE_RING_ALIGN,
    TASK7_COARSE_RING_PLACE_ITEM,
    TASK7_COARSE_RING_FINISH_ITEM,
    TASK7_COARSE_RING_DONE,
    TASK7_COARSE_RING_FAILED
} task7_coarse_ring_state_t;

typedef enum
{
    TASK7_COARSE_RING_OK = 0,
    TASK7_COARSE_RING_WAITING,
    TASK7_COARSE_RING_INVALID_PARAM,
    TASK7_COARSE_RING_CHASSIS_ERROR,
    TASK7_COARSE_RING_ARM_ERROR,
    TASK7_COARSE_RING_VISION_ERROR,
    TASK7_COARSE_RING_TRAY_ERROR,
    TASK7_COARSE_RING_SCREEN_ERROR
} task7_coarse_ring_result_t;

typedef enum
{
    TASK7_COARSE_RING_STATUS_INIT = 0U,
    TASK7_COARSE_RING_STATUS_MOVE = 1U,
    TASK7_COARSE_RING_STATUS_ROTATE = 2U,
    TASK7_COARSE_RING_STATUS_PREPARE = 3U,
    TASK7_COARSE_RING_STATUS_FINE_ADJUST = 4U,
    TASK7_COARSE_RING_STATUS_PICK = 5U,
    TASK7_COARSE_RING_STATUS_PLACE = 6U,
    TASK7_COARSE_RING_STATUS_DONE = 7U,
    TASK7_COARSE_RING_STATUS_ERROR = 255U
} task7_coarse_ring_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task7_coarse_ring_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t item_index;
    uint8_t current_ring;
    uint8_t target_ring;
    uint8_t task_ready;
    task7_coarse_ring_state_t state;
} task7_coarse_ring_t;

task7_coarse_ring_result_t task7_coarse_ring_init(
    task7_coarse_ring_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task7_coarse_ring_status_fn write_status);

task7_coarse_ring_result_t task7_coarse_ring_submit_qr(
    task7_coarse_ring_t *task,
    const QR_TaskCode_t *qr_task);

task7_coarse_ring_result_t task7_coarse_ring_step(
    task7_coarse_ring_t *task);

task7_coarse_ring_state_t task7_coarse_ring_get_state(
    const task7_coarse_ring_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK7_COARSE_RING_H */
