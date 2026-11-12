#ifndef ENGINEERING_INNOVATION_TASK2_PLATFORM_PICK_H
#define ENGINEERING_INNOVATION_TASK2_PLATFORM_PICK_H

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

#define TASK2_PLATFORM_PICK_FORWARD_MM 800U
#define TASK2_PLATFORM_PICK_RIGHT_MM   920U
#define TASK2_PLATFORM_PICK_ITEM_COUNT   3U
#define TASK2_PLATFORM_PICK_TRAY_SETTLE_MS 100U

typedef void (*task2_platform_pick_status_fn)(uint8_t status,
                                              const char *event);

typedef enum
{
    TASK2_PLATFORM_PICK_WAIT_TASK = 0,
    TASK2_PLATFORM_PICK_FORWARD,
    TASK2_PLATFORM_PICK_RIGHT,
    TASK2_PLATFORM_PICK_ITEMS,
    TASK2_PLATFORM_PICK_DONE,
    TASK2_PLATFORM_PICK_FAILED
} task2_platform_pick_state_t;

typedef enum
{
    TASK2_PLATFORM_PICK_OK = 0,
    TASK2_PLATFORM_PICK_WAITING,
    TASK2_PLATFORM_PICK_INVALID_PARAM,
    TASK2_PLATFORM_PICK_CHASSIS_ERROR,
    TASK2_PLATFORM_PICK_ARM_ERROR,
    TASK2_PLATFORM_PICK_VISION_ERROR,
    TASK2_PLATFORM_PICK_TRAY_ERROR,
    TASK2_PLATFORM_PICK_SCREEN_ERROR
} task2_platform_pick_result_t;

typedef enum
{
    TASK2_PLATFORM_PICK_ITEMS_WAIT = 0,
    TASK2_PLATFORM_PICK_ITEMS_ALIGN,
    TASK2_PLATFORM_PICK_ITEMS_PICK,
    TASK2_PLATFORM_PICK_ITEMS_DONE,
    TASK2_PLATFORM_PICK_ITEMS_FAILED
} task2_platform_pick_items_state_t;

typedef enum
{
    TASK2_PLATFORM_PICK_STATUS_INIT = 0U,
    TASK2_PLATFORM_PICK_STATUS_ZERO = 1U,
    TASK2_PLATFORM_PICK_STATUS_PREPARE_PICK = 2U,
    TASK2_PLATFORM_PICK_STATUS_FINE_ADJUST = 3U,
    TASK2_PLATFORM_PICK_STATUS_TRAY_POSITION = 4U,
    TASK2_PLATFORM_PICK_STATUS_PICK = 5U,
    TASK2_PLATFORM_PICK_STATUS_PLACE = 6U,
    TASK2_PLATFORM_PICK_STATUS_DONE = 7U,
    TASK2_PLATFORM_PICK_STATUS_TASK_CODE = 8U,
    TASK2_PLATFORM_PICK_STATUS_ERROR = 255U
} task2_platform_pick_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task2_platform_pick_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint8_t target_ids[TASK2_PLATFORM_PICK_ITEM_COUNT];
    uint8_t item_index;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    uint8_t task_ready;
    task2_platform_pick_items_state_t item_state;
    task2_platform_pick_state_t state;
} task2_platform_pick_t;

task2_platform_pick_result_t task2_platform_pick_init(
    task2_platform_pick_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task2_platform_pick_status_fn write_status);

task2_platform_pick_result_t task2_platform_pick_submit_qr(
    task2_platform_pick_t *task,
    const QR_TaskCode_t *qr_task);

task2_platform_pick_result_t task2_platform_pick_step(
    task2_platform_pick_t *task);

task2_platform_pick_state_t task2_platform_pick_get_state(
    const task2_platform_pick_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK2_PLATFORM_PICK_H */
