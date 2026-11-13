#ifndef ENGINEERING_INNOVATION_TASK9_TEMPORARY_STACK_H
#define ENGINEERING_INNOVATION_TASK9_TEMPORARY_STACK_H

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

#define TASK9_TEMPORARY_STACK_ITEM_COUNT 3U
#define TASK9_TEMPORARY_STACK_ROTATE_DEG 90U
#define TASK9_TEMPORARY_STACK_BACKWARD_TO_AREA_MM 860U
#define TASK9_TEMPORARY_STACK_BACKWARD_RING_1_MM 960U
#define TASK9_TEMPORARY_STACK_BACKWARD_RING_2_MM 860U
#define TASK9_TEMPORARY_STACK_BACKWARD_RING_3_MM 660U
#define TASK9_TEMPORARY_STACK_CENTER 2U

typedef void (*task9_temporary_stack_status_fn)(uint8_t status,
                                                 const char *event);

typedef enum
{
    TASK9_TEMPORARY_STACK_WAIT_TASK = 0,
    TASK9_TEMPORARY_STACK_BACKWARD_FROM_RING,
    TASK9_TEMPORARY_STACK_WAIT_BACKWARD_FROM_RING,
    TASK9_TEMPORARY_STACK_ROTATE,
    TASK9_TEMPORARY_STACK_WAIT_ROTATE,
    TASK9_TEMPORARY_STACK_BACKWARD_TO_AREA,
    TASK9_TEMPORARY_STACK_WAIT_BACKWARD_TO_AREA,
    TASK9_TEMPORARY_STACK_PREPARE_ITEM,
    TASK9_TEMPORARY_STACK_ALIGN,
    TASK9_TEMPORARY_STACK_PLACE_ITEM,
    TASK9_TEMPORARY_STACK_RETURN_TRAY,
    TASK9_TEMPORARY_STACK_DONE,
    TASK9_TEMPORARY_STACK_FAILED
} task9_temporary_stack_state_t;

typedef enum
{
    TASK9_TEMPORARY_STACK_OK = 0,
    TASK9_TEMPORARY_STACK_WAITING,
    TASK9_TEMPORARY_STACK_INVALID_PARAM,
    TASK9_TEMPORARY_STACK_CHASSIS_ERROR,
    TASK9_TEMPORARY_STACK_ARM_ERROR,
    TASK9_TEMPORARY_STACK_VISION_ERROR,
    TASK9_TEMPORARY_STACK_TRAY_ERROR,
    TASK9_TEMPORARY_STACK_SCREEN_ERROR
} task9_temporary_stack_result_t;

typedef enum
{
    TASK9_TEMPORARY_STACK_STATUS_INIT = 0U,
    TASK9_TEMPORARY_STACK_STATUS_MOVE = 1U,
    TASK9_TEMPORARY_STACK_STATUS_ROTATE = 2U,
    TASK9_TEMPORARY_STACK_STATUS_PREPARE = 3U,
    TASK9_TEMPORARY_STACK_STATUS_FINE_ADJUST = 4U,
    TASK9_TEMPORARY_STACK_STATUS_PLACE = 5U,
    TASK9_TEMPORARY_STACK_STATUS_DONE = 6U,
    TASK9_TEMPORARY_STACK_STATUS_ERROR = 255U
} task9_temporary_stack_status_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    task9_temporary_stack_status_fn write_status;
    QR_TaskCode_t qr_task;
    uint32_t move_start_tick;
    uint32_t move_duration_ms;
    int16_t saved_tolerance_x_pixel;
    int16_t saved_tolerance_y_pixel;
    uint16_t saved_position_settle_ms;
    uint8_t item_index;
    uint8_t current_ring;
    uint8_t target_ring;
    /* bit0/bit1/bit2 分别表示1/2/3号环已经完成码垛。 */
    uint8_t stacked_ring_mask;
    uint8_t task_ready;
    uint8_t fine_adjust_config_saved;
    task9_temporary_stack_state_t state;
} task9_temporary_stack_t;

task9_temporary_stack_result_t task9_temporary_stack_init(
    task9_temporary_stack_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task9_temporary_stack_status_fn write_status);

task9_temporary_stack_result_t task9_temporary_stack_submit_qr(
    task9_temporary_stack_t *task,
    const QR_TaskCode_t *qr_task);

task9_temporary_stack_result_t task9_temporary_stack_step(
    task9_temporary_stack_t *task);

task9_temporary_stack_state_t task9_temporary_stack_get_state(
    const task9_temporary_stack_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK9_TEMPORARY_STACK_H */
