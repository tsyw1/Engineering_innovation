#include "target_ring_place_test.h"

#include <stdio.h>
#include <string.h>

#define TARGET_RING_PLACE_TEST_RING_SPACING_MM       150
#define TARGET_RING_PLACE_TEST_COARSE_SPEED_MPS      0.08f
#define TARGET_RING_PLACE_TEST_COARSE_SETTLE_MS      300U
#define TARGET_RING_PLACE_TEST_TRAY_SETTLE_MS        400U

typedef struct
{
    material_tray_position_t tray_position;
    uint8_t target_ring;
    int16_t nominal_x_mm;
    const char *color_name;
} target_ring_place_test_item_t;

static const target_ring_place_test_item_t s_test_items[
    TARGET_RING_PLACE_TEST_ITEM_COUNT] =
{
    {
        MATERIAL_TRAY_POSITION_1,
        1U,
        TARGET_RING_PLACE_TEST_RING_SPACING_MM,
        "red"
    },
    {
        MATERIAL_TRAY_POSITION_2,
        2U,
        0,
        "yellow"
    },
    {
        MATERIAL_TRAY_POSITION_3,
        3U,
        -TARGET_RING_PLACE_TEST_RING_SPACING_MM,
        "blue"
    }
};

static void target_ring_place_test_status(
    target_ring_place_test_t *test,
    uint8_t status,
    const char *event)
{
    if ((test != NULL) && (test->write_status != NULL))
    {
        test->write_status(status, event);
    }
}

static target_ring_place_test_result_t target_ring_place_test_fail(
    target_ring_place_test_t *test,
    target_ring_place_test_result_t result,
    const char *event)
{
    if (test != NULL)
    {
        vision_fine_adjust_abort(test->fine_adjust);
        (void)mecanum_chassis_stop(test->chassis);
        (void)mecanum_chassis_enable(test->chassis, false);
        test->state = TARGET_RING_PLACE_TEST_FAILED;
        target_ring_place_test_status(
            test,
            TARGET_RING_PLACE_TEST_STATUS_ERROR,
            event);
    }
    return result;
}

static void target_ring_place_test_write_item_status(
    target_ring_place_test_t *test,
    uint8_t status,
    const char *action)
{
    char event[96];
    int length;
    const target_ring_place_test_item_t *item;

    if ((test == NULL) ||
        (test->item_index >= TARGET_RING_PLACE_TEST_ITEM_COUNT) ||
        (action == NULL))
    {
        return;
    }

    item = &s_test_items[test->item_index];
    length = snprintf(event,
                      sizeof(event),
                      "%s_to_ring_%u_%s",
                      item->color_name,
                      (unsigned int)item->target_ring,
                      action);
    if ((length > 0) && ((size_t)length < sizeof(event)))
    {
        target_ring_place_test_status(test, status, event);
    }
}

static target_ring_place_test_result_t target_ring_place_test_coarse_move(
    target_ring_place_test_t *test,
    int16_t target_x_mm)
{
    int16_t delta_x_mm;
    uint16_t distance_mm;
    uint32_t move_time_ms;
    float vx_mps;
    mecanum_result_t chassis_result;

    delta_x_mm = (int16_t)(target_x_mm - test->nominal_x_mm);
    if (delta_x_mm == 0)
    {
        return TARGET_RING_PLACE_TEST_OK;
    }

    distance_mm = (uint16_t)((delta_x_mm < 0) ?
                             -delta_x_mm : delta_x_mm);
    vx_mps = (delta_x_mm > 0) ?
             TARGET_RING_PLACE_TEST_COARSE_SPEED_MPS :
             -TARGET_RING_PLACE_TEST_COARSE_SPEED_MPS;
    move_time_ms = (uint32_t)((float)distance_mm /
                              TARGET_RING_PLACE_TEST_COARSE_SPEED_MPS);

    chassis_result = mecanum_chassis_set_velocity(test->chassis,
                                                  vx_mps,
                                                  0.0f,
                                                  0.0f);
    if ((chassis_result != MECANUM_OK) &&
        (chassis_result != MECANUM_CLAMPED))
    {
        return TARGET_RING_PLACE_TEST_CHASSIS_ERROR;
    }

    HAL_Delay(move_time_ms);
    if (mecanum_chassis_stop(test->chassis) != MECANUM_OK)
    {
        return TARGET_RING_PLACE_TEST_CHASSIS_ERROR;
    }
    HAL_Delay(TARGET_RING_PLACE_TEST_COARSE_SETTLE_MS);
    test->nominal_x_mm = target_x_mm;
    return TARGET_RING_PLACE_TEST_OK;
}

target_ring_place_test_result_t target_ring_place_test_init(
    target_ring_place_test_t *test,
    arm_controller_t *arm,
    mecanum_chassis_t *chassis,
    vision_fine_adjust_t *fine_adjust,
    material_tray_servo_t *tray,
    screen_t *screen,
    target_ring_place_test_status_fn write_status)
{
    if ((test == NULL) || (arm == NULL) || (chassis == NULL) ||
        (fine_adjust == NULL) || (tray == NULL) || (screen == NULL))
    {
        return TARGET_RING_PLACE_TEST_INVALID_PARAM;
    }

    memset(test, 0, sizeof(*test));
    test->arm = arm;
    test->chassis = chassis;
    test->fine_adjust = fine_adjust;
    test->tray = tray;
    test->screen = screen;
    test->write_status = write_status;
    /* 小车侧面对着靶环，初始位于 2 号环；车头朝 1 号环方向。 */
    test->nominal_x_mm = 0;
    test->state = TARGET_RING_PLACE_TEST_IDLE;
    return TARGET_RING_PLACE_TEST_OK;
}

target_ring_place_test_result_t target_ring_place_test_start(
    target_ring_place_test_t *test)
{
    if ((test == NULL) || (test->arm == NULL) ||
        (test->chassis == NULL) || (test->fine_adjust == NULL) ||
        (test->tray == NULL) || (test->screen == NULL))
    {
        return TARGET_RING_PLACE_TEST_INVALID_PARAM;
    }
    if (test->state != TARGET_RING_PLACE_TEST_IDLE)
    {
        return TARGET_RING_PLACE_TEST_INVALID_PARAM;
    }

    test->item_index = 0U;
    test->nominal_x_mm = 0;
    test->state = TARGET_RING_PLACE_TEST_COARSE_MOVE;
    target_ring_place_test_status(test,
                                  TARGET_RING_PLACE_TEST_STATUS_INIT,
                                  "target_ring_place_test_start");
    return TARGET_RING_PLACE_TEST_OK;
}

target_ring_place_test_result_t target_ring_place_test_step(
    target_ring_place_test_t *test)
{
    const target_ring_place_test_item_t *item;
    target_ring_place_test_result_t test_result;
    vision_fine_adjust_result_t adjust_result;

    if (test == NULL)
    {
        return TARGET_RING_PLACE_TEST_INVALID_PARAM;
    }
    if (test->state == TARGET_RING_PLACE_TEST_DONE)
    {
        return TARGET_RING_PLACE_TEST_OK;
    }
    if (test->state == TARGET_RING_PLACE_TEST_IDLE)
    {
        return TARGET_RING_PLACE_TEST_WAITING;
    }
    if (test->state == TARGET_RING_PLACE_TEST_FAILED)
    {
        return TARGET_RING_PLACE_TEST_VISION_ERROR;
    }
    if (test->item_index >= TARGET_RING_PLACE_TEST_ITEM_COUNT)
    {
        return TARGET_RING_PLACE_TEST_INVALID_PARAM;
    }
    item = &s_test_items[test->item_index];

    switch (test->state)
    {
        case TARGET_RING_PLACE_TEST_COARSE_MOVE:
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_COARSE_MOVE,
                "coarse_move_start");
            test_result = target_ring_place_test_coarse_move(
                test,
                item->nominal_x_mm);
            if (test_result != TARGET_RING_PLACE_TEST_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    test_result,
                    "coarse_move_failed");
            }
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_COARSE_MOVE,
                "coarse_move_done");
            test->state = TARGET_RING_PLACE_TEST_SELECT_TRAY;
            return TARGET_RING_PLACE_TEST_OK;

        case TARGET_RING_PLACE_TEST_SELECT_TRAY:
            if (material_tray_servo_set_position(
                    test->tray,
                    item->tray_position) != MATERIAL_TRAY_SERVO_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    TARGET_RING_PLACE_TEST_TRAY_ERROR,
                    "tray_position_failed");
            }
            HAL_Delay(TARGET_RING_PLACE_TEST_TRAY_SETTLE_MS);
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_TRAY,
                "tray_ready");
            test->state = TARGET_RING_PLACE_TEST_START_ALIGN;
            return TARGET_RING_PLACE_TEST_OK;

        case TARGET_RING_PLACE_TEST_START_ALIGN:
            /* 机械臂保持在物料盘方向的最高点，先完成底盘视觉细调。 */
            adjust_result = vision_fine_adjust_start(
                test->fine_adjust,
                VISION_UART_MODE_TARGET_RING,
                item->target_ring);
            if (adjust_result != VISION_FINE_ADJUST_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    TARGET_RING_PLACE_TEST_VISION_ERROR,
                    "target_ring_align_start_failed");
            }
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_ALIGN,
                "fine_adjust_start");
            test->state = TARGET_RING_PLACE_TEST_ALIGN;
            return TARGET_RING_PLACE_TEST_OK;

        case TARGET_RING_PLACE_TEST_ALIGN:
            adjust_result = vision_fine_adjust_step(test->fine_adjust);
            if (adjust_result == VISION_FINE_ADJUST_REACHED)
            {
                target_ring_place_test_write_item_status(
                    test,
                    TARGET_RING_PLACE_TEST_STATUS_ALIGN,
                    "fine_adjust_done");
                test->state = TARGET_RING_PLACE_TEST_PLACE;
                return TARGET_RING_PLACE_TEST_OK;
            }
            if ((adjust_result == VISION_FINE_ADJUST_OK) ||
                (adjust_result == VISION_FINE_ADJUST_WAITING))
            {
                return TARGET_RING_PLACE_TEST_WAITING;
            }
            return target_ring_place_test_fail(
                test,
                TARGET_RING_PLACE_TEST_VISION_ERROR,
                "target_ring_align_failed");

        case TARGET_RING_PLACE_TEST_PLACE:
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_PLACE,
                "place_start");
            if (arm_controller_place_to_ground(test->arm) !=
                ARM_CONTROLLER_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    TARGET_RING_PLACE_TEST_ARM_ERROR,
                    "place_to_target_ring_failed");
            }
            if (screen_record_successful_place(test->screen) != SCREEN_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    TARGET_RING_PLACE_TEST_SCREEN_ERROR,
                    "place_count_display_failed");
            }
            target_ring_place_test_write_item_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_PLACE,
                "place_done");
            if (test->item_index + 1U >=
                TARGET_RING_PLACE_TEST_ITEM_COUNT)
            {
                test->state = TARGET_RING_PLACE_TEST_FINISH_PLACE;
            }
            else
            {
                /* 下一件物料需要从物料盘取料，先转回物料盘方向。 */
                if (arm_controller_finish_place(test->arm) !=
                    ARM_CONTROLLER_OK)
                {
                    return target_ring_place_test_fail(
                        test,
                        TARGET_RING_PLACE_TEST_ARM_ERROR,
                        "return_to_tray_failed");
                }
                test->item_index++;
                test->state = TARGET_RING_PLACE_TEST_COARSE_MOVE;
            }
            return TARGET_RING_PLACE_TEST_OK;

        case TARGET_RING_PLACE_TEST_FINISH_PLACE:
            if (arm_controller_finish_place(test->arm) != ARM_CONTROLLER_OK)
            {
                return target_ring_place_test_fail(
                    test,
                    TARGET_RING_PLACE_TEST_ARM_ERROR,
                    "finish_place_failed");
            }
            (void)mecanum_chassis_stop(test->chassis);
            (void)mecanum_chassis_enable(test->chassis, false);
            test->item_index++;
            test->state = TARGET_RING_PLACE_TEST_DONE;
            target_ring_place_test_status(
                test,
                TARGET_RING_PLACE_TEST_STATUS_DONE,
                "red_yellow_blue_place_done");
            return TARGET_RING_PLACE_TEST_OK;

        default:
            return TARGET_RING_PLACE_TEST_VISION_ERROR;
    }
}

target_ring_place_test_state_t target_ring_place_test_get_state(
    const target_ring_place_test_t *test)
{
    return (test == NULL) ?
           TARGET_RING_PLACE_TEST_FAILED : test->state;
}
