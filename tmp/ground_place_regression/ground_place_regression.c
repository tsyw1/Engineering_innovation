#include "task3_coarse_ring.h"
#include "task5_temporary_ring.h"
#include "task7_coarse_ring.h"
#include "task9_temporary_stack.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum fault_kind { NO_FAULT, SECOND_VISION_START, SECOND_VISION_TIMEOUT,
                  SECOND_ARM_MOVE, FIRST_RELEASE, SECOND_STACK_PICK_LIFT,
                  SECOND_STACK_RING_MOVE, FINAL_STACK_LIFT,
                  FINAL_STACK_RETURN, FINAL_STACK_CLOSE, FINAL_STACK_TRAY };

static arm_controller_t arm;
static motor_bus_t bus;
static mecanum_chassis_t chassis;
static vision_fine_adjust_t adjust;
static vision_uart_t vision;
static material_tray_servo_t tray;
static screen_t screen;
static uint32_t tick_ms, last_release_tick, ground_reached_tick, tray_tick, last_delay_ms;
static unsigned releases, pickups, displays, vision_starts, frame_step;
static unsigned arm_commands, z_commands, lift_commands, inter_ring_moves;
static unsigned motion_commands;
static unsigned post_align_moves, final_closes, tray_returns;
static int stopped, enabled, task_mode, stack_mode, tray_wait_pending;
static int allow_early_mode, alignment_started;
static enum fault_kind fault;
static uint8_t ring_order[3];
static uint8_t color_order[3];

static int64_t pulses(float angle)
{
    double value = (double)angle * ARM_CONTROLLER_COMMAND_PULSES_PER_REV / 360.0;
    return (int64_t)(value >= 0.0 ? value + 0.5 : value - 0.5);
}

static int64_t placement_pulses(void)
{
    return pulses(stack_mode ? ARM_CONTROLLER_STACK_PLACE_LIFT_ANGLE_DEG :
                              ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG);
}

static void assert_low_open(void)
{
    assert(arm.lift_motor.actual_position_pulse == placement_pulses());
    assert(arm.z_motor.actual_position_pulse ==
           pulses(ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG));
    assert(arm.gripper.state == GRIPPER_SERVO_STATE_OPEN);
}

uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_Delay(uint32_t delay_ms)
{
    if (tray_wait_pending)
    {
        assert(delay_ms == 400U);
        tray_wait_pending = 0;
    }
    last_delay_ms = delay_ms;
    tick_ms += delay_ms;
}

/* Link-only guards for APIs outside this regression's initialized task scope. */
motor_result_t motor_init(motor_t *value, motor_bus_t *value_bus, uint8_t id)
{
    (void)value; (void)value_bus; (void)id;
    assert(0 && "unexpected motor initialization");
    return MOTOR_ERROR;
}

motor_result_t motor_home(motor_t *value, uint8_t mode, motor_sync_t sync)
{
    (void)value; (void)mode; (void)sync;
    assert(0 && "unexpected homing");
    return MOTOR_ERROR;
}

motor_result_t motor_read_home_state(motor_t *value)
{
    (void)value;
    assert(0 && "unexpected homing feedback");
    return MOTOR_ERROR;
}

motor_result_t motor_clear_position(motor_t *value)
{
    (void)value;
    assert(0 && "unexpected position reset");
    return MOTOR_ERROR;
}

void gripper_servo_config_default(gripper_servo_config_t *config)
{
    (void)config;
    assert(0 && "unexpected gripper initialization");
}

gripper_servo_result_t gripper_servo_init(gripper_servo_t *value,
                                         const gripper_servo_config_t *config)
{
    (void)value; (void)config;
    assert(0 && "unexpected gripper initialization");
    return GRIPPER_SERVO_ERROR;
}

const task_plan_item_t *task_plan_get_item(const task_execution_plan_t *plan,
                                          task_plan_batch_t batch, uint8_t index)
{
    (void)plan; (void)batch; (void)index;
    assert(0 && "unexpected task-plan lookup");
    return NULL;
}

motor_result_t motor_set_position(motor_t *motor, int64_t value,
                                  uint16_t rpm, uint8_t acceleration,
                                  motor_position_mode_t mode, motor_sync_t sync)
{
    (void)rpm;
    (void)acceleration;
    assert(sync == MOTOR_EXECUTE_NOW);
    assert(stopped);
    arm_commands++;
    if (motor->id == ARM_CONTROLLER_Z_MOTOR_ID)
    {
        /* Every rotation must happen at the highest lift position. */
        assert(arm.lift_motor.actual_position_pulse == 0);
        z_commands++;
        if (fault == FINAL_STACK_RETURN && releases == 3U)
        {
            return MOTOR_ERROR;
        }
    }
    else
    {
        assert(motor->id == ARM_CONTROLLER_LIFT_MOTOR_ID);
        assert(mode == MOTOR_POSITION_ABSOLUTE);
        lift_commands++;
        if (value == pulses(ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG))
        {
            assert(arm.z_motor.actual_position_pulse == 0);
            assert(arm.gripper.state == GRIPPER_SERVO_STATE_OPEN);
        }
        if (task_mode && value == 0 &&
            motor->actual_position_pulse == placement_pulses())
        {
            /* The camera must finish alignment before leaving its low pose. */
            assert(adjust.state == VISION_FINE_ADJUST_ALIGNED);
            if (releases > 0U && releases < 3U &&
                arm.gripper.state == GRIPPER_SERVO_STATE_OPEN)
            {
                assert(vision_starts == releases + 1U);
                if (stack_mode) assert(post_align_moves == vision_starts);
            }
        }
        if (fault == SECOND_ARM_MOVE && releases == 1U &&
            value == placement_pulses() &&
            arm.gripper.state == GRIPPER_SERVO_STATE_CLOSED)
        {
            return MOTOR_ERROR;
        }
        if (fault == SECOND_STACK_PICK_LIFT && releases == 1U && value == 0 &&
            motor->actual_position_pulse == placement_pulses() &&
            arm.gripper.state == GRIPPER_SERVO_STATE_OPEN)
        {
            return MOTOR_ERROR;
        }
        if (fault == FINAL_STACK_LIFT && releases == 3U && value == 0 &&
            motor->actual_position_pulse == placement_pulses() &&
            arm.gripper.state == GRIPPER_SERVO_STATE_OPEN)
        {
            return MOTOR_ERROR;
        }
    }
    motor->actual_position_pulse = (mode == MOTOR_POSITION_ABSOLUTE) ?
                                  value : motor->actual_position_pulse + value;
    return MOTOR_OK;
}

motor_result_t motor_read_state(motor_t *motor)
{
    motor->state_flags = MOTOR_STATE_POSITION_REACHED;
    if (motor->id == ARM_CONTROLLER_LIFT_MOTOR_ID &&
        motor->actual_position_pulse == placement_pulses())
    {
        ground_reached_tick = tick_ms;
    }
    return MOTOR_OK;
}

gripper_servo_result_t gripper_servo_open(gripper_servo_t *gripper)
{
    assert(stopped);
    if (arm.lift_motor.actual_position_pulse == placement_pulses() &&
        gripper->state == GRIPPER_SERVO_STATE_CLOSED)
    {
        assert(last_delay_ms == 500U);
        assert(tick_ms - ground_reached_tick >= ARM_CONTROLLER_RELEASE_DELAY_MS);
        if (fault == FIRST_RELEASE && releases == 0U)
        {
            return GRIPPER_SERVO_ERROR;
        }
        assert(arm.z_motor.actual_position_pulse ==
               pulses(ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG));
        releases++;
        last_release_tick = tick_ms;
    }
    gripper->state = GRIPPER_SERVO_STATE_OPEN;
    return GRIPPER_SERVO_OK;
}

gripper_servo_result_t gripper_servo_close(gripper_servo_t *gripper)
{
    assert(stopped);
    /* Tray pickup requires the selected slot to have settled. */
    assert(arm.z_motor.actual_position_pulse == 0);
    if (stack_mode && arm.lift_motor.actual_position_pulse == 0)
    {
        assert(releases == 3U);
        if (fault == FINAL_STACK_CLOSE) return GRIPPER_SERVO_ERROR;
        final_closes++;
        gripper->state = GRIPPER_SERVO_STATE_CLOSED;
        return GRIPPER_SERVO_OK;
    }
    assert(arm.lift_motor.actual_position_pulse ==
           pulses(ARM_CONTROLLER_GROUND_PLACE_PICK_LIFT_DEG));
    assert(tray.current_position == (material_tray_position_t)releases);
    assert(tick_ms - tray_tick >= 400U);
    pickups++;
    gripper->state = GRIPPER_SERVO_STATE_CLOSED;
    return GRIPPER_SERVO_OK;
}

mecanum_result_t mecanum_chassis_enable(mecanum_chassis_t *value, bool enable)
{
    (void)value;
    enabled = enable;
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *value)
{
    (void)value;
    stopped = 1;
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_position_increment(
    mecanum_chassis_t *value, const int32_t wheel_pulse[MECANUM_WHEEL_COUNT],
    uint16_t rpm, uint8_t acceleration)
{
    int32_t max_pulse = 0;
    (void)value;
    (void)rpm;
    (void)acceleration;
    assert(enabled);
    motion_commands++;
    for (unsigned i = 0; i < MECANUM_WHEEL_COUNT; ++i)
    {
        if (abs(wheel_pulse[i]) > max_pulse) max_pulse = abs(wheel_pulse[i]);
    }
    if (alignment_started != 0)
    {
        /* All movement after the first alignment begins in the open, low pose. */
        assert_low_open();
        if (max_pulse > 96)
        {
            if (stack_mode && wheel_pulse[MECANUM_WHEEL_FL] !=
                              wheel_pulse[MECANUM_WHEEL_FR])
            {
                /* Preserve task9's rightward 15 mm correction after alignment. */
                const float circumference_mm =
                    2.0f * 3.14159265359f * chassis.config.wheel_radius_m * 1000.0f;
                int32_t expected_pulse =
                    (int32_t)(MECANUM_POSITION_PULSES_PER_REV / circumference_mm + 0.5f) * 15;
                assert(adjust.state == VISION_FINE_ADJUST_ALIGNED);
                assert(wheel_pulse[MECANUM_WHEEL_FL] == expected_pulse);
                assert(wheel_pulse[MECANUM_WHEEL_FR] == -expected_pulse);
                assert(wheel_pulse[MECANUM_WHEEL_RL] == -expected_pulse);
                assert(wheel_pulse[MECANUM_WHEEL_RR] == expected_pulse);
                post_align_moves++;
            }
            else
            {
                assert(releases > 0U && releases < 3U);
                assert(tick_ms - last_release_tick >=
                       (stack_mode ? ARM_CONTROLLER_STACK_RELEASE_SETTLE_MS :
                                     ARM_CONTROLLER_GROUND_RELEASE_SETTLE_MS));
                inter_ring_moves++;
                if (fault == SECOND_STACK_RING_MOVE && releases == 1U)
                {
                    return MECANUM_MOTOR_ERROR;
                }
            }
        }
    }
    stopped = 0;
    return MECANUM_OK;
}

vision_uart_t *vision_uart_get_default(void) { return &vision; }

vision_uart_result_t vision_uart_set_mode(vision_uart_t *value,
                                         vision_uart_mode_t mode)
{
    assert(stopped);
    assert(mode == (stack_mode ? VISION_UART_MODE_MATERIAL : VISION_UART_MODE_TARGET_RING));
    if ((allow_early_mode != 0) && (vision_starts == 0U))
    {
        assert(motion_commands == 2U);
        alignment_started = 0;
    }
    else
    {
        assert_low_open();
        alignment_started = 1;
    }
    if (fault == SECOND_VISION_START && vision_starts == 1U)
    {
        return VISION_UART_RESULT_UART_ERROR;
    }
    assert(vision_starts < 3U);
    vision_starts++;
    frame_step = 0;
    value->current_mode = mode;
    return VISION_UART_RESULT_OK;
}

void vision_uart_clear_frame(vision_uart_t *value)
{
    assert(value != NULL);
    assert(value->current_mode == VISION_UART_MODE_TARGET_RING);
    assert_low_open();
    value->frame_ready = 0U;
    value->pending_ready = 0U;
    value->rx_length = 0U;
    alignment_started = 1;
    frame_step = 0U;
}

vision_uart_result_t vision_uart_take_frame(vision_uart_t *value,
                                           vision_uart_frame_t *frame)
{
    assert_low_open();
    assert(adjust.target_id == (stack_mode ? color_order[vision_starts - 1U] :
                                          ring_order[vision_starts - 1U]));
    memset(frame, 0, sizeof(*frame));
    frame_step++;
    /* No frame, wrong ring, a correction, then an aligned target. */
    if (frame_step == 1U || (fault == SECOND_VISION_TIMEOUT && vision_starts == 2U))
    {
        return VISION_UART_RESULT_NO_TARGET;
    }
    frame->mode = value->current_mode;
    frame->status = VISION_UART_STATUS_OK;
    frame->received_tick = tick_ms;
    frame->valid = 1;
    frame->detection_count = 1;
    frame->detections[0].detected = 1;
    frame->detections[0].valid = 1;
    frame->detections[0].confidence = 1.0f;
    frame->detections[0].id = (frame_step == 2U) ?
        (uint8_t)(adjust.target_id % 3U + 1U) : adjust.target_id;
    if (frame_step == 3U)
    {
        frame->detections[0].dx = 20.0f;
        frame->detections[0].dy = -10.0f;
    }
    return VISION_UART_RESULT_OK;
}

vision_uart_result_t vision_uart_find_detection(
    const vision_uart_frame_t *frame, uint8_t id, vision_detection_t *detection)
{
    if (frame->detections[0].id != id) return VISION_UART_RESULT_NO_TARGET;
    *detection = frame->detections[0];
    return VISION_UART_RESULT_OK;
}

material_tray_servo_result_t material_tray_servo_set_position(
    material_tray_servo_t *value, material_tray_position_t position)
{
    assert(stopped);
    assert(adjust.state == VISION_FINE_ADJUST_ALIGNED);
    if (stack_mode && releases == 3U)
    {
        assert(position == MATERIAL_TRAY_POSITION_2);
        assert(arm.lift_motor.actual_position_pulse == 0);
        assert(arm.z_motor.actual_position_pulse == 0);
        assert(arm.gripper.state == GRIPPER_SERVO_STATE_CLOSED);
        if (fault == FINAL_STACK_TRAY) return MATERIAL_TRAY_SERVO_ERROR;
        tray_returns++;
    }
    else assert(position == (material_tray_position_t)releases);
    value->current_position = position;
    tray_tick = tick_ms;
    tray_wait_pending = 1;
    return MATERIAL_TRAY_SERVO_OK;
}

screen_result_t screen_record_successful_place(screen_t *value)
{
    (void)value;
    assert(releases == displays + 1U);
    if (releases < 3U)
    {
        assert_low_open();
        assert(tick_ms - last_release_tick >=
               (stack_mode ? ARM_CONTROLLER_STACK_RELEASE_SETTLE_MS :
                             ARM_CONTROLLER_GROUND_RELEASE_SETTLE_MS));
    }
    else
    {
        assert(arm.lift_motor.actual_position_pulse == 0);
        assert(arm.z_motor.actual_position_pulse ==
               pulses(ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG));
    }
    displays++;
    return SCREEN_OK;
}

static void reset_case(const uint8_t order[3], enum fault_kind requested_fault,
                       int aligned_at_entry, int early_mode)
{
    memset(&arm, 0, sizeof(arm));
    memset(&chassis, 0, sizeof(chassis));
    memset(&adjust, 0, sizeof(adjust));
    memset(&tray, 0, sizeof(tray));
    tick_ms = last_release_tick = ground_reached_tick = tray_tick = last_delay_ms = 0;
    releases = pickups = displays = vision_starts = frame_step = 0;
    arm_commands = z_commands = lift_commands = inter_ring_moves = motion_commands = 0;
    post_align_moves = final_closes = tray_returns = 0;
    stack_mode = tray_wait_pending = alignment_started = 0;
    allow_early_mode = early_mode;
    stopped = enabled = task_mode = 1;
    fault = requested_fault;
    memcpy(ring_order, order, sizeof(ring_order));
    arm.bus = &bus;
    arm.initialized = 1;
    arm.lift_motor.bus = arm.z_motor.bus = &bus;
    arm.lift_motor.id = ARM_CONTROLLER_LIFT_MOTOR_ID;
    arm.z_motor.id = ARM_CONTROLLER_Z_MOTOR_ID;
    arm.z_motor.actual_position_pulse = aligned_at_entry ?
        pulses(ARM_CONTROLLER_PREPARE_Z_ANGLE_DEG) : 0;
    arm.gripper.state = GRIPPER_SERVO_STATE_OPEN;
    chassis.config.wheel_radius_m = 0.05f;
    chassis.config.half_length_m = chassis.config.half_width_m = 0.1f;
    chassis.config.pulses_per_rev = 65536;
    /* Task3 reuses the fine-adjust object initialized by the main program. */
    vision_fine_adjust_config_t config;
    vision_fine_adjust_config_default(&config);
    assert(vision_fine_adjust_init(&adjust, &vision, &chassis, &config) ==
           VISION_FINE_ADJUST_OK);
}

static void report_status(uint8_t status, const char *event)
{
    if (status == 255U && fault == NO_FAULT)
    {
        fprintf(stderr, "Unexpected failure: %s, releases=%u, starts=%u, arm_commands=%u\n",
                event, releases, vision_starts, arm_commands);
    }
}

#define DEFINE_TASK_CASE(name, NAME, batch, entry_aligned, early_mode)           \
static void run_##name(const uint8_t order[3], enum fault_kind requested_fault)  \
{                                                                             \
    name##_t task;                                                            \
    QR_TaskCode_t code = { .valid = 1 };                                       \
    reset_case(order, requested_fault, entry_aligned, early_mode);              \
    for (unsigned i = 0; i < 3; ++i)                                          \
    {                                                                         \
        code.first_batch_color[i] = (uint8_t)(i + 1);                          \
        code.second_batch_color[i] = (uint8_t)(6 - i);                         \
        code.first_batch_position[i] = (uint8_t)(3 - i);                        \
        code.second_batch_position[i] = (uint8_t)(i + 1);                      \
        code.batch[i] = order[i];                                             \
    }                                                                         \
    assert(name##_init(&task, &arm, &chassis, &adjust, &tray, &screen, report_status) == \
           NAME##_OK);                                                        \
    assert(name##_submit_qr(&task, &code) == NAME##_OK);                        \
    if (requested_fault == SECOND_VISION_TIMEOUT) adjust.config.timeout_ms = 100; \
    unsigned steps = 0;                                                       \
    while (task.state != NAME##_DONE && task.state != NAME##_FAILED)            \
    {                                                                         \
        assert(++steps < 100);                                                \
        int result = name##_step(&task);                                      \
        if (task.state == NAME##_FAILED)                                      \
        {                                                                     \
            assert(result == ((requested_fault == SECOND_VISION_START ||     \
                               requested_fault == SECOND_VISION_TIMEOUT) ?   \
                              NAME##_VISION_ERROR : NAME##_ARM_ERROR));       \
        }                                                                     \
        else assert(result == NAME##_OK || result == NAME##_WAITING);          \
        HAL_Delay(20);                                                        \
    }                                                                         \
    if (requested_fault == NO_FAULT)                                          \
    {                                                                         \
        assert(task.state == NAME##_DONE);                                    \
        assert(task.current_ring == order[2]);                                \
        assert(task.item_index == 3 && releases == 3 && displays == 3);         \
        assert(pickups == 3 && vision_starts == 3 && inter_ring_moves == 2);     \
        assert(z_commands == (entry_aligned ? 6U : 7U));                       \
        assert(lift_commands == 14);                                          \
        assert(arm.lift_motor.actual_position_pulse == 0);                     \
    }                                                                         \
    else                                                                      \
    {                                                                         \
        assert(task.state == NAME##_FAILED && stopped && !enabled);           \
        assert(releases == (requested_fault == FIRST_RELEASE ? 0U : 1U));      \
        assert(displays == releases);                                         \
    }                                                                         \
    unsigned previous_arm = arm_commands, previous_motion = motion_commands;  \
    (void)name##_step(&task);                                                  \
    assert(arm_commands == previous_arm && motion_commands == previous_motion); \
}

DEFINE_TASK_CASE(task3_coarse_ring, TASK3_COARSE_RING, first_batch_position, 0, 1)
DEFINE_TASK_CASE(task5_temporary_ring, TASK5_TEMPORARY_RING, first_batch_position, 1, 0)
DEFINE_TASK_CASE(task7_coarse_ring, TASK7_COARSE_RING, second_batch_position, 0, 0)

static void run_task9_temporary_stack(const uint8_t positions[3],
                                      const uint8_t pickup_order[3],
                                      enum fault_kind requested_fault)
{
    task9_temporary_stack_t task;
    QR_TaskCode_t code = { .valid = 1 };
    static const uint8_t colors[3] = {1, 3, 5};
    uint8_t expected_rings[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        expected_rings[i] = positions[pickup_order[i] - 1U];
        code.first_batch_color[i] = colors[i];
        code.first_batch_position[i] = positions[i];
        code.second_batch_color[i] = colors[pickup_order[i] - 1U];
        /* Coarse-area ring positions must not be used as stacking targets. */
        code.second_batch_position[i] = (uint8_t)(3U - i);
    }
    reset_case(expected_rings, requested_fault, 0, 0);
    stack_mode = 1;
    arm.gripper.state = GRIPPER_SERVO_STATE_CLOSED;
    memcpy(color_order, code.second_batch_color, sizeof(color_order));
    assert(task9_temporary_stack_init(&task, &arm, &chassis, &adjust, &tray,
                                     &screen, report_status) == TASK9_TEMPORARY_STACK_OK);
    assert(task9_temporary_stack_submit_qr(&task, &code) == TASK9_TEMPORARY_STACK_OK);
    if (requested_fault == SECOND_VISION_TIMEOUT) adjust.config.timeout_ms = 100;
    unsigned steps = 0;
    while (task.state != TASK9_TEMPORARY_STACK_DONE &&
           task.state != TASK9_TEMPORARY_STACK_FAILED)
    {
        assert(++steps < 100);
        task9_temporary_stack_result_t result = task9_temporary_stack_step(&task);
        if (task.state == TASK9_TEMPORARY_STACK_FAILED)
        {
            task9_temporary_stack_result_t expected_error =
                requested_fault == SECOND_VISION_START || requested_fault == SECOND_VISION_TIMEOUT ?
                TASK9_TEMPORARY_STACK_VISION_ERROR :
                requested_fault == SECOND_STACK_RING_MOVE ? TASK9_TEMPORARY_STACK_CHASSIS_ERROR :
                requested_fault == FINAL_STACK_TRAY ? TASK9_TEMPORARY_STACK_TRAY_ERROR :
                                                     TASK9_TEMPORARY_STACK_ARM_ERROR;
            assert(result == expected_error);
        }
        else assert(result == TASK9_TEMPORARY_STACK_OK ||
                    result == TASK9_TEMPORARY_STACK_WAITING);
        if (vision_starts > 0U && task.state != TASK9_TEMPORARY_STACK_FAILED)
        {
            unsigned index = vision_starts - 1U;
            assert(task.target_color == code.second_batch_color[index]);
            assert(task.target_ring == expected_rings[index]);
        }
        HAL_Delay(20);
    }
    if (requested_fault == NO_FAULT)
    {
        assert(task.state == TASK9_TEMPORARY_STACK_DONE);
        assert(task.current_ring == expected_rings[2]);
        assert(task.item_index == 3 && releases == 3 && displays == 3);
        assert(pickups == 3 && vision_starts == 3 && inter_ring_moves == 2);
        assert(post_align_moves == 3 && final_closes == 1 && tray_returns == 1);
        assert(z_commands == 8 && lift_commands == 14);
        assert(arm.lift_motor.actual_position_pulse == 0);
        assert(arm.z_motor.actual_position_pulse == 0);
        assert(arm.gripper.state == GRIPPER_SERVO_STATE_CLOSED);
        assert(tray.current_position == MATERIAL_TRAY_POSITION_2);
    }
    else
    {
        assert(task.state == TASK9_TEMPORARY_STACK_FAILED && stopped && !enabled);
        assert(releases == (requested_fault == FIRST_RELEASE ? 0U :
                            requested_fault >= FINAL_STACK_LIFT ? 3U : 1U));
        assert(displays == (requested_fault == FINAL_STACK_LIFT ? 2U : releases));
    }
    unsigned previous_arm = arm_commands, previous_motion = motion_commands;
    (void)task9_temporary_stack_step(&task);
    assert(arm_commands == previous_arm && motion_commands == previous_motion);
}

static void check_arm_api(void)
{
    const uint8_t order[3] = {1, 2, 3};
    for (unsigned keep_low = 0; keep_low < 2; ++keep_low)
    {
        reset_case(order, NO_FAULT, 1, 0);
        task_mode = 0;
        arm.lift_motor.actual_position_pulse =
            pulses(ARM_CONTROLLER_GROUND_PLACE_LIFT_ANGLE_DEG);
        tray.current_position = MATERIAL_TRAY_POSITION_1;
        HAL_Delay(400);
        assert((keep_low ? arm_controller_place_ring_item_from_tray_keep_low(&arm) :
                          arm_controller_place_ring_item_from_tray(&arm)) == ARM_CONTROLLER_OK);
        assert(releases == 1 && pickups == 1 && z_commands == 2);
        if (keep_low)
        {
            assert_low_open();
            assert(tick_ms - last_release_tick == ARM_CONTROLLER_GROUND_RELEASE_SETTLE_MS);
        }
        else assert(arm.lift_motor.actual_position_pulse == 0);
    }
    assert(arm_controller_place_ring_item_from_tray_keep_low(NULL) == ARM_CONTROLLER_NOT_READY);
    arm.initialized = 0;
    assert(arm_controller_place_ring_item_from_tray_keep_low(&arm) == ARM_CONTROLLER_NOT_READY);

    for (unsigned keep_low = 0; keep_low < 2; ++keep_low)
    {
        reset_case(order, NO_FAULT, 1, 0);
        stack_mode = 1;
        task_mode = 0;
        arm.lift_motor.actual_position_pulse = placement_pulses();
        tray.current_position = MATERIAL_TRAY_POSITION_1;
        HAL_Delay(400);
        assert((keep_low ? arm_controller_stack_item_from_tray_keep_low(&arm) :
                          arm_controller_stack_item_from_tray(&arm)) == ARM_CONTROLLER_OK);
        assert(releases == 1 && pickups == 1 && z_commands == 2);
        assert(lift_commands == (keep_low ? 4U : 5U));
        if (keep_low)
        {
            assert_low_open();
            assert(tick_ms - last_release_tick == ARM_CONTROLLER_STACK_RELEASE_SETTLE_MS);
        }
        else assert(arm.lift_motor.actual_position_pulse == 0);
    }
    assert(arm_controller_stack_item_from_tray_keep_low(NULL) == ARM_CONTROLLER_NOT_READY);
    arm.initialized = 0;
    assert(arm_controller_stack_item_from_tray_keep_low(&arm) == ARM_CONTROLLER_NOT_READY);
    arm.initialized = 1;
    arm.bus = NULL;
    assert(arm_controller_stack_item_from_tray_keep_low(&arm) == ARM_CONTROLLER_NOT_READY);

    reset_case(order, NO_FAULT, 1, 0);
    stack_mode = 1;
    assert(arm_controller_prepare_stack_pick_aligned(&arm) == ARM_CONTROLLER_OK);
    assert_low_open();
    assert(z_commands == 0 && lift_commands == 1);
    assert(arm_controller_prepare_stack_pick_aligned(NULL) == ARM_CONTROLLER_NOT_READY);
    arm.initialized = 0;
    assert(arm_controller_prepare_stack_pick_aligned(&arm) == ARM_CONTROLLER_NOT_READY);
    arm.initialized = 1;
    arm.bus = NULL;
    assert(arm_controller_prepare_stack_pick_aligned(&arm) == ARM_CONTROLLER_NOT_READY);
}

int main(void)
{
    const uint8_t permutations[6][3] = {
        {1, 2, 3}, {1, 3, 2}, {2, 1, 3}, {2, 3, 1}, {3, 1, 2}, {3, 2, 1}
    };
    for (unsigned i = 0; i < 6; ++i)
    {
        run_task3_coarse_ring(permutations[i], NO_FAULT);
        run_task5_temporary_ring(permutations[i], NO_FAULT);
        run_task7_coarse_ring(permutations[i], NO_FAULT);
        for (unsigned j = 0; j < 6; ++j)
        {
            run_task9_temporary_stack(permutations[i], permutations[j], NO_FAULT);
        }
    }
    for (enum fault_kind kind = SECOND_VISION_START; kind <= FIRST_RELEASE; ++kind)
    {
        run_task3_coarse_ring(permutations[0], kind);
        run_task5_temporary_ring(permutations[0], kind);
        run_task7_coarse_ring(permutations[0], kind);
    }
    for (enum fault_kind kind = SECOND_VISION_START; kind <= FINAL_STACK_TRAY; ++kind)
    {
        run_task9_temporary_stack(permutations[0], permutations[1], kind);
    }
    check_arm_api();
    puts("PASS: 18 ground + 36 stacking sequences, 22 failure cases; timing, pose and API checks");
    return 0;
}
