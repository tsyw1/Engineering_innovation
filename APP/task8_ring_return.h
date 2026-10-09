#ifndef ENGINEERING_INNOVATION_TASK8_RING_RETURN_H
#define ENGINEERING_INNOVATION_TASK8_RING_RETURN_H

#include "task4_ring_return.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TASK8_RING_RETURN_WAIT_TASK = 0,
    TASK8_RING_RETURN_RUNNING,
    TASK8_RING_RETURN_DONE,
    TASK8_RING_RETURN_FAILED
} task8_ring_return_state_t;

typedef enum
{
    TASK8_RING_RETURN_OK = 0,
    TASK8_RING_RETURN_WAITING,
    TASK8_RING_RETURN_INVALID_PARAM,
    TASK8_RING_RETURN_CHASSIS_ERROR,
    TASK8_RING_RETURN_ARM_ERROR,
    TASK8_RING_RETURN_VISION_ERROR,
    TASK8_RING_RETURN_TRAY_ERROR,
    TASK8_RING_RETURN_SCREEN_ERROR
} task8_ring_return_result_t;

typedef struct
{
    task4_ring_return_t implementation;
    task8_ring_return_state_t state;
} task8_ring_return_t;

task8_ring_return_result_t task8_ring_return_init(
    task8_ring_return_t *task,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    task4_ring_return_status_fn write_status);

task8_ring_return_result_t task8_ring_return_submit_qr(
    task8_ring_return_t *task,
    const QR_TaskCode_t *qr_task);

task8_ring_return_result_t task8_ring_return_step(
    task8_ring_return_t *task);

task8_ring_return_state_t task8_ring_return_get_state(
    const task8_ring_return_t *task);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_TASK8_RING_RETURN_H */
