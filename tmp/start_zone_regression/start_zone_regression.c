#include "task1_start_qr.h"
#include "task10_return_home.h"
#include "usart.h"
#include "material_tray_uart_control.h"
#include "motor_TTL.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5, huart6;
static screen_t screen;
static mecanum_chassis_t chassis;
static arm_controller_t arm;
static vision_uart_t vision;
static uint32_t tick_ms, primask;
static unsigned rx_calls, tx_calls, zero_calls, stops, moves;
static unsigned cleared_uart_errors;
static unsigned fail_move, status_count;
static uint8_t fail_zero, fail_tx;
static HAL_StatusTypeDef next_rx_status;
static int enabled;
static int32_t motion[4][MECANUM_WHEEL_COUNT];
static char last_event[80];
static uint8_t tx_bytes[4096];
static size_t tx_length;

uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_Delay(uint32_t delay_ms) { tick_ms += delay_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __set_PRIMASK(uint32_t value) { primask = value; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void mock_clear_uart_errors(UART_HandleTypeDef *huart)
{ assert(huart == &huart5); cleared_uart_errors++; }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef result = next_rx_status;
    next_rx_status = HAL_OK;
    rx_calls++;
    assert(data != NULL && length == 1U);
    if (result != HAL_OK) { return result; }
    if (huart->RxState != HAL_UART_STATE_READY) { return HAL_BUSY; }
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->rx_buffer = data;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *data,
                                  uint16_t length, uint32_t timeout_ms)
{
    assert(huart == &huart5 && data != NULL && length > 0U && timeout_ms == 100U);
    tx_calls++;
    if (fail_tx) { return HAL_ERROR; }
    assert(tx_length + length <= sizeof(tx_bytes));
    memcpy(tx_bytes + tx_length, data, length);
    tx_length += length;
    return HAL_OK;
}

/* Existing UART callbacks must still dispatch alongside UART5. */
void jy901s_uart_on_rx_complete(UART_HandleTypeDef *huart) { (void)huart; }
void jy901s_uart_on_error(UART_HandleTypeDef *huart) { (void)huart; }
void motor_ttl_on_rx_complete(UART_HandleTypeDef *huart) { (void)huart; }
void motor_ttl_on_error(UART_HandleTypeDef *huart) { (void)huart; }
void material_tray_uart_control_on_rx_complete(UART_HandleTypeDef *huart) { (void)huart; }
void material_tray_uart_control_on_error(UART_HandleTypeDef *huart) { (void)huart; }
vision_uart_t *vision_uart_get_default(void) { return &vision; }
void vision_uart_process_byte(vision_uart_t *value, uint8_t byte) { (void)value; (void)byte; }
vision_uart_result_t vision_uart_start_receive(vision_uart_t *value)
{ (void)value; return VISION_UART_RESULT_OK; }
void vision_uart_on_error(vision_uart_t *value) { (void)value; }

mecanum_result_t mecanum_chassis_set_position_increment(
    mecanum_chassis_t *value, const int32_t pulse[MECANUM_WHEEL_COUNT],
    uint16_t speed_rpm, uint8_t acceleration)
{
    assert(value == &chassis && enabled && moves < 4U);
    assert(speed_rpm == 60U && acceleration == 240U);
    memcpy(motion[moves++], pulse, sizeof(motion[0]));
    return (fail_move == moves) ? MECANUM_MOTOR_ERROR : MECANUM_OK;
}
mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *value)
{ assert(value == &chassis); stops++; return MECANUM_OK; }
mecanum_result_t mecanum_chassis_enable(mecanum_chassis_t *value, bool enable)
{ assert(value == &chassis); enabled = enable; return MECANUM_OK; }
arm_controller_result_t arm_controller_zero(arm_controller_t *value)
{
    assert(value == &arm && moves == 2U);
    zero_calls++;
    HAL_Delay(600U);
    return fail_zero ? ARM_CONTROLLER_HOME_FAILED : ARM_CONTROLLER_OK;
}
static void status(uint8_t code, const char *event)
{
    (void)code;
    assert(strlen(event) < sizeof(last_event));
    strcpy(last_event, event);
    status_count++;
}

static void reset_case(void)
{
    memset(&huart5, 0, sizeof(huart5));
    memset(&chassis, 0, sizeof(chassis));
    chassis.config.wheel_radius_m = MECANUM_DEFAULT_WHEEL_RADIUS_M;
    chassis.config.pulses_per_rev = MECANUM_DEFAULT_ENCODER_COUNTS_PER_REV;
    tick_ms = primask = 0U;
    rx_calls = tx_calls = zero_calls = stops = moves = status_count = 0U;
    cleared_uart_errors = 0U;
    fail_move = fail_zero = fail_tx = 0U;
    enabled = 1;
    tx_length = 0U;
    next_rx_status = HAL_OK;
    last_event[0] = '\0';
    assert(screen_init(&screen, &huart5) == SCREEN_OK);
    assert(screen_start_receive(&screen) == SCREEN_OK);
}

static void receive(UART_HandleTypeDef *huart, uint8_t byte, uint32_t error)
{
    assert(primask == 0U && huart->RxState == HAL_UART_STATE_BUSY_RX);
    *huart->rx_buffer = byte;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ErrorCode = error;
    HAL_UART_RxCpltCallback(huart);
    if (error != HAL_UART_ERROR_NONE)
    {
        /* HAL invokes RxCplt before Error; the RX callback must preserve ErrorCode. */
        assert(huart->ErrorCode == error);
        HAL_UART_ErrorCallback(huart);
    }
}

static void check_motion(unsigned index, int x_mm, int y_mm)
{
    const int32_t p = 13; /* Calibrated wheel radius: 3200 / circumference rounds to 13. */
    assert(index < moves);
    assert(motion[index][0] == p * (x_mm - y_mm));
    assert(motion[index][1] == p * (x_mm + y_mm));
    assert(motion[index][2] == p * (x_mm + y_mm));
    assert(motion[index][3] == p * (x_mm - y_mm));
}

static void test_protocol(void)
{
    start_zone_t zone = START_ZONE_NONE;
    screen_t other;
    reset_case();
    assert(screen_init(NULL, &huart5) == SCREEN_INVALID_PARAM);
    assert(screen_start_receive(NULL) == SCREEN_INVALID_PARAM);
    assert(screen_init(&other, &huart6) == SCREEN_OK);
    assert(screen_start_receive(&other) == SCREEN_INVALID_PARAM);
    assert(screen_take_start_zone(NULL, &zone) == 0U);
    screen_rx_task(NULL);
    HAL_UART_RxCpltCallback(&huart6);
    HAL_UART_ErrorCallback(&huart6);
    assert(rx_calls == 1U);
    for (unsigned byte = 0U; byte <= 255U; byte++)
    {
        if (byte == 0x31U || byte == 0x32U) { continue; }
        receive(&huart5, (uint8_t)byte, HAL_UART_ERROR_NONE);
        assert(screen_take_start_zone(&screen, &zone) == 0U);
    }
    assert(rx_calls == 255U && tx_calls == 0U && moves == 0U);
    assert(screen_start_receive(&screen) == SCREEN_OK && rx_calls == 255U);
    for (unsigned command = 0x31U; command <= 0x32U; command++)
    {
        reset_case();
        receive(&huart5, (uint8_t)command, HAL_UART_ERROR_NONE);
        receive(&huart5, (uint8_t)(command == 0x31U ? 0x32U : 0x31U), HAL_UART_ERROR_NONE);
        assert(screen_reset_statistics(&screen) == SCREEN_OK);
        assert(screen_take_start_zone(&screen, NULL) == 0U);
        primask = 1U;
        assert(screen_take_start_zone(&screen, &zone) == 1U && primask == 1U);
        assert(zone == (command == 0x31U ? START_ZONE_1 : START_ZONE_2));
        assert(screen_take_start_zone(&screen, &zone) == 0U && primask == 1U);
        primask = 0U;
        receive(&huart5, (uint8_t)command, HAL_UART_ERROR_NONE);
        assert(screen_take_start_zone(&screen, &zone) == 0U);
        assert(screen.start_locked == 1U);
    }
    /* ORE/FE bytes that look like valid start commands must be discarded. */
    for (unsigned error = HAL_UART_ERROR_FE; error <= HAL_UART_ERROR_ORE; error += 4U)
    {
        reset_case();
        receive(&huart5, 0x31U, error);
        assert(cleared_uart_errors == 1U);
        assert(screen_take_start_zone(&screen, &zone) == 0U);
        receive(&huart5, 0x32U, HAL_UART_ERROR_NONE);
        assert(screen_take_start_zone(&screen, &zone) == 1U && zone == START_ZONE_2);
    }
    reset_case();
    huart5.ErrorCode = HAL_UART_ERROR_FE;
    HAL_UART_ErrorCallback(&huart5); /* A nonblocking error without RX completion. */
    assert(cleared_uart_errors == 1U);
    assert(huart5.RxState == HAL_UART_STATE_BUSY_RX && rx_calls == 1U);
    huart5.ErrorCode = HAL_UART_ERROR_NONE;
    next_rx_status = HAL_BUSY;
    receive(&huart5, 0U, HAL_UART_ERROR_NONE);
    assert(screen.rx_restart_needed == 1U);
    primask = 1U;
    next_rx_status = HAL_ERROR;
    screen_rx_task(&screen);
    assert(screen.rx_restart_needed == 1U && primask == 1U);
    screen_rx_task(&screen);
    assert(screen.rx_restart_needed == 0U && primask == 1U);
    primask = 0U;
    receive(&huart5, 0x31U, HAL_UART_ERROR_NONE);
    assert(screen_take_start_zone(&screen, &zone) == 1U && zone == START_ZONE_1);
    reset_case();
    huart5.RxState = HAL_UART_STATE_READY;
    next_rx_status = HAL_ERROR;
    HAL_UART_ErrorCallback(&huart5);
    assert(screen.rx_restart_needed == 1U);
    screen_rx_task(&screen);
    receive(&huart5, 0x32U, HAL_UART_ERROR_NONE);
    assert(screen_take_start_zone(&screen, &zone) == 1U && zone == START_ZONE_2);
}

static void test_task1(start_zone_t zone, unsigned failure)
{
    task1_start_qr_t task;
    QR_TaskCode_t qr_task;
    task_execution_plan_t plan;
    const char *code = "135+123+513+312";
    reset_case();
    if (zone == START_ZONE_2) { tick_ms = UINT32_MAX - 200U; }
    fail_move = (failure == 1U) ? 2U : 0U;
    fail_zero = (failure == 2U);
    assert(task1_start_qr_init(&task, &arm, &chassis, &screen, status) == TASK1_START_QR_OK);
    assert(task.start_zone == START_ZONE_1);
    assert(task1_start_qr_set_start_zone(&task, START_ZONE_NONE) == TASK1_START_QR_INVALID_PARAM);
    for (unsigned i = 0U; i < 3U; i++)
    {
        HAL_Delay(10000U);
        assert(task1_start_qr_step(&task) == TASK1_START_QR_WAITING);
    }
    assert(moves == 0U && zero_calls == 0U);
    receive(&huart5, (zone == START_ZONE_2) ? 0x32U : 0x31U, HAL_UART_ERROR_NONE);
    assert(screen_take_start_zone(&screen, &zone) == 1U);
    assert(task1_start_qr_set_start_zone(&task, zone) == TASK1_START_QR_OK);
    task1_start_qr_request_start(&task);
    assert(task1_start_qr_set_start_zone(&task, START_ZONE_1) == TASK1_START_QR_INVALID_PARAM);
    for (unsigned i = 0U; i < 10U && task.state != TASK1_START_QR_WAIT_QR &&
         task.state != TASK1_START_QR_FAILED; i++) { (void)task1_start_qr_step(&task); }
    check_motion(0U, 100, 0);
    check_motion(1U, 0, (zone == START_ZONE_2) ? -1150 : 1050);
    if (failure != 0U)
    {
        assert(task.state == TASK1_START_QR_FAILED && !enabled && stops >= 2U);
        assert(moves == 2U && zero_calls == (failure == 2U ? 1U : 0U));
        return;
    }
    assert(task.state == TASK1_START_QR_WAIT_QR && moves == 3U && zero_calls == 1U);
    check_motion(2U, 150, 0);
    assert(task1_start_qr_step(&task) == TASK1_START_QR_WAITING);
    for (size_t i = 0U; i < strlen(code); i++) { QR_ProcessByte((uint8_t)code[i]); }
    QR_ProcessByte('\r');
    assert(task1_start_qr_step(&task) == TASK1_START_QR_OK);
    assert(task1_start_qr_step(&task) == TASK1_START_QR_OK);
    assert(task.state == TASK1_START_QR_DONE);
    assert(task1_start_qr_get_task(&task, &qr_task) == 1U && strcmp(qr_task.raw_code, code) == 0);
    assert(task1_start_qr_get_plan(&task, &plan) == 1U && plan.valid);
    assert(tx_length >= 3U && tx_bytes[tx_length - 1U] == 0xFFU &&
           tx_bytes[tx_length - 2U] == 0xFFU && tx_bytes[tx_length - 3U] == 0xFFU);
    receive(&huart5, (zone == START_ZONE_2) ? 0x31U : 0x32U, HAL_UART_ERROR_NONE);
    assert(screen_take_start_zone(&screen, &zone) == 0U);
}

static void test_task10(start_zone_t zone, const char *code, unsigned failure)
{
    task10_return_home_t task;
    QR_TaskCode_t qr_task;
    task_execution_plan_t plan;
    unsigned index, last_ring;
    reset_case();
    assert(QR_ParseTaskCode(code, &qr_task) == QR_STATUS_OK);
    assert(task_plan_build(&qr_task, &plan) == 1U);
    last_ring = plan.second_batch[2].temporary_ring;
    assert(last_ring >= 1U && last_ring <= 3U);
    assert(task10_return_home_init(&task, &chassis, &screen, status) == TASK10_RETURN_HOME_OK);
    assert(task.start_zone == START_ZONE_1);
    assert(task10_return_home_set_start_zone(&task, START_ZONE_NONE) == TASK10_RETURN_HOME_INVALID_PARAM);
    assert(task10_return_home_set_start_zone(&task, zone) == TASK10_RETURN_HOME_OK);
    assert(task10_return_home_step(&task) == TASK10_RETURN_HOME_WAITING && moves == 0U);
    assert(task10_return_home_submit_qr(&task, &qr_task) == TASK10_RETURN_HOME_OK);
    assert(task.current_ring == last_ring);
    assert(task10_return_home_set_start_zone(&task, START_ZONE_1) == TASK10_RETURN_HOME_INVALID_PARAM);
    fail_move = failure == 1U ? (last_ring == 2U ? 2U : 3U) : 0U;
    fail_tx = (failure == 2U);
    for (unsigned step = 0U; step < 15U && task.state != TASK10_RETURN_HOME_DONE &&
         task.state != TASK10_RETURN_HOME_FAILED; step++) { (void)task10_return_home_step(&task); }
    index = 0U;
    if (last_ring != 2U) { check_motion(index++, last_ring == 1U ? -150 : 150, 0); }
    check_motion(index++, 0, 1900);
    check_motion(index++, zone == START_ZONE_2 ? 980 : -1050, 0);
    assert(moves == index && !enabled);
    if (failure != 0U) { assert(task.state == TASK10_RETURN_HOME_FAILED); return; }
    assert(task.state == TASK10_RETURN_HOME_DONE && task.current_ring == 2U);
    assert(strcmp(last_event, "task10_return_home_done") == 0);
    assert(tx_calls > 0U);
}

int main(void)
{
    const char *colors[] = {"123", "132", "213", "231", "312", "321"};
    char code[16];
    test_protocol();
    for (start_zone_t zone = START_ZONE_1; zone <= START_ZONE_2; zone++)
    {
        for (unsigned failure = 0U; failure < 3U; failure++) { test_task1(zone, failure); }
        for (unsigned first = 0U; first < 6U; first++)
        {
            for (unsigned second = 0U; second < 6U; second++)
            {
                for (unsigned positions = 0U; positions < 6U; positions++)
                {
                    int n = snprintf(code, sizeof(code), "%s+%s+%s+312",
                                     colors[first], colors[positions], colors[second]);
                    assert(n == 15);
                    test_task10(zone, code, 0U);
                }
            }
        }
        test_task10(zone, "312+231+123+132", 0U);
        test_task10(zone, "135+123+513+312", 0U);
        test_task10(zone, "135+123+513+312", 1U);
        test_task10(zone, "135+123+513+312", 2U);
    }
    puts("PASS: UART5 commands, callback dispatch, error recovery, PRIMASK; 6 task1 cases, 440 task10 cases");
    return 0;
}
