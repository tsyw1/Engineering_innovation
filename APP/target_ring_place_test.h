#ifndef ENGINEERING_INNOVATION_TARGET_RING_PLACE_TEST_H
#define ENGINEERING_INNOVATION_TARGET_RING_PLACE_TEST_H

#include "arm_controller.h"
#include "chassis_mecanum.h"
#include "material_tray_servo.h"
#include "screen.h"
#include "vision_fine_adjust.h"

#include <stdint.h>

#define TARGET_RING_PLACE_TEST_ITEM_COUNT 3U

typedef void (*target_ring_place_test_status_fn)(uint8_t status,
                                                 const char *event);

typedef enum
{
    TARGET_RING_PLACE_TEST_STATUS_INIT = 0U,
    TARGET_RING_PLACE_TEST_STATUS_COARSE_MOVE = 1U,
    TARGET_RING_PLACE_TEST_STATUS_TRAY = 2U,
    TARGET_RING_PLACE_TEST_STATUS_ALIGN = 3U,
    TARGET_RING_PLACE_TEST_STATUS_PLACE = 4U,
    TARGET_RING_PLACE_TEST_STATUS_DONE = 5U,
    TARGET_RING_PLACE_TEST_STATUS_ERROR = 255U
} target_ring_place_test_status_t;

typedef enum
{
    TARGET_RING_PLACE_TEST_IDLE = 0,
    TARGET_RING_PLACE_TEST_COARSE_MOVE,
    TARGET_RING_PLACE_TEST_SELECT_TRAY,
    TARGET_RING_PLACE_TEST_START_ALIGN,
    TARGET_RING_PLACE_TEST_ALIGN,
    TARGET_RING_PLACE_TEST_PLACE,
    TARGET_RING_PLACE_TEST_FINISH_PLACE,
    TARGET_RING_PLACE_TEST_DONE,
    TARGET_RING_PLACE_TEST_FAILED
} target_ring_place_test_state_t;

typedef enum
{
    TARGET_RING_PLACE_TEST_OK = 0,
    TARGET_RING_PLACE_TEST_WAITING,
    TARGET_RING_PLACE_TEST_INVALID_PARAM,
    TARGET_RING_PLACE_TEST_CHASSIS_ERROR,
    TARGET_RING_PLACE_TEST_TRAY_ERROR,
    TARGET_RING_PLACE_TEST_VISION_ERROR,
    TARGET_RING_PLACE_TEST_ARM_ERROR,
    TARGET_RING_PLACE_TEST_SCREEN_ERROR
} target_ring_place_test_result_t;

typedef struct
{
    arm_controller_t *arm;
    mecanum_chassis_t *chassis;
    vision_fine_adjust_t *fine_adjust;
    material_tray_servo_t *tray;
    screen_t *screen;
    target_ring_place_test_status_fn write_status;
    uint8_t item_index;
    target_ring_place_test_state_t state;
} target_ring_place_test_t;

target_ring_place_test_result_t target_ring_place_test_init(
    target_ring_place_test_t *test,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    target_ring_place_test_status_fn write_status);

target_ring_place_test_result_t target_ring_place_test_start(
    target_ring_place_test_t *test);

target_ring_place_test_result_t target_ring_place_test_step(
    target_ring_place_test_t *test);

target_ring_place_test_state_t target_ring_place_test_get_state(
    const target_ring_place_test_t *test);

#endif /* ENGINEERING_INNOVATION_TARGET_RING_PLACE_TEST_H */
