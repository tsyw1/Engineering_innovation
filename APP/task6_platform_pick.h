#ifndef ENGINEERING_INNOVATION_TASK6_PLATFORM_PICK_H
#define ENGINEERING_INNOVATION_TASK6_PLATFORM_PICK_H

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

#define TASK6_PLATFORM_PICK_ITEM_COUNT 3U
#define TASK6_PLATFORM_PICK_ROTATE_DEG 90U
#define TASK6_PLATFORM_PICK_BACKWARD_TO_AREA_MM 850U
#define TASK6_PLATFORM_PICK_BACKWARD_RING_1_MM 1160U
#define TASK6_PLATFORM_PICK_BACKWARD_RING_2_MM 1010U
#define TASK6_PLATFORM_PICK_BACKWARD_RING_3_MM 860U
#define TASK6_PLATFORM_PICK_TRAY_SETTLE_MS 500U
#define TASK6_PLATFORM_PICK_TRAY_OPEN_SETTLE_MS 100U

typedef void (*task6_platform_pick_status_fn)(uint8_t status,
                                               const char *event);

typedef enum
{
    TASK6_PLATFORM_PICK_WAIT_TASK = 0,
    TASK6_PLATFORM_PICK_BACKWARD_FROM_RING,
    TASK6_PLATFORM_PICK_WAIT_BACKWARD_FROM_RING,
    TASK6_PLATFORM_PICK_ROTATE,
    TASK6_PLATFORM_PICK_WAIT_ROTATE,
    TASK6_PLATFORM_PICK_BACKWARD_TO_AREA,
    TASK6_PLATFORM_PICK_WAIT_BACKWARD_TO_AREA,
    TASK6_PLATFORM_PICK_PREPARE_ITEM,
    TASK6_PLATFORM_PICK_ALIGN,
    TASK6_PLATFORM_PICK_PICK_ITEM,
    TASK6_PLATFORM_PICK_DONE,
    TASK6_PLATFORM_PICK_FAILED
} task6_platform_pick_state_t;

typedef enum
{
    TASK6_PLATFORM_PICK_OK = 0,
    TASK6_PLATFORM_PICK_WAITING,
    TASK6_PLATFORM_PICK_INVALID_PARAM,
    TASK6_PLATFORM_PICK_CHASSIS_ERROR,
    TASK6_PLATFORM_PICK_ARM_ERROR,
    TASK6_PLATFORM_PICK_VISION_ERROR,
    TASK6_PLATFORM_PICK_TRAY_ERROR,
    TASK6_PLATFORM_PICK_SCREEN_ERROR
} task6_platform_pick_result_t;

typedef enum
{
    TASK6_PLATFORM_PICK_STATUS_INIT = 0U,
    TASK6_PLATFORM_PICK_STATUS_MOVE = 1U,
    TASK6_PLATFORM_PICK_STATUS_ROTATE = 2U,
    TASK6_PLATFORM_PICK_STATUS_PREPARE = 3U,
    TASK6_PLATFORM_PICK_STATUS_FINE_ADJUST = 4U,
    TASK6_PLATFORM_PICK_STATUS_PICK = 5U,
    TASK6_PLATFORM_PICK_STATUS_PLACE = 6U,
    TASK6_PLATFORM_PICK_STATUS_DONE = 7U,
    TASK6_PLATFORM_PICK_STATUS_ERROR = 255U
} task6_platform_pick_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task6_platform_pick_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t item_index;
    uint8_t current_ring;
    uint8_t task_ready;
    task6_platform_pick_state_t state;
} task6_platform_pick_t;

task6_platform_pick_result_t task6_platform_pick_init(
    task6_platform_pick_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task6_platform_pick_status_fn write_status);

task6_platform_pick_result_t task6_platform_pick_submit_qr(
    task6_platform_pick_t *task,
    const QR_TaskCode_t *qr_task);

task6_platform_pick_result_t task6_platform_pick_step(
    task6_platform_pick_t *task);

task6_platform_pick_state_t task6_platform_pick_get_state(
    const task6_platform_pick_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK6_PLATFORM_PICK_H */
