#include "heading_hold_test.h"

#include "HEADING_HOLD_CONTROLLER.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEADING_HOLD_TEST_CONTROL_PERIOD_MS       20U
#define HEADING_HOLD_TEST_OUTPUT_PERIOD_MS       100U
#define HEADING_HOLD_TEST_INITIAL_SETTLE_MS     1000U
#define HEADING_HOLD_TEST_MAX_YAW_AGE_MS         200U
#define HEADING_HOLD_TEST_TX_TIMEOUT_MS          100U
#define HEADING_HOLD_TEST_RX_QUEUE_SIZE           64U
#define HEADING_HOLD_TEST_LINE_SIZE               48U
#define HEADING_HOLD_TEST_IDLE_TIMEOUT_MS         100U
#define HEADING_HOLD_TEST_KP_MIN                  0.0f
#define HEADING_HOLD_TEST_KP_MAX                  5.0f
#define HEADING_HOLD_TEST_MAX_CORRECTION_RAD_S    3.50f

typedef enum
{
    HEADING_HOLD_TEST_WAIT_SENSOR = 0,
    HEADING_HOLD_TEST_SETTLING,
    HEADING_HOLD_TEST_RUNNING,
    HEADING_HOLD_TEST_STOPPED
} heading_hold_test_state_t;

static mecanum_chassis_t *s_chassis;
static hwt101ct_t *s_hwt101ct_sensor;
static UART_HandleTypeDef *s_command_uart;
static heading_hold_controller_t s_controller;
static heading_hold_test_state_t s_state;
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_queue[HEADING_HOLD_TEST_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_uart_error_pending;
static volatile uint32_t s_last_rx_tick;
static char s_line[HEADING_HOLD_TEST_LINE_SIZE];
static uint8_t s_line_length;
static uint8_t s_run_requested;
static uint8_t s_initialized;
static uint8_t s_chassis_stopped;
static uint8_t s_yaw_zero_initialized;
static uint32_t s_settle_start_ms;
static uint32_t s_last_control_ms;
static uint32_t s_last_output_ms;

static void heading_hold_test_send_text(const char *text)
{
    if ((text == NULL) || (s_command_uart == NULL))
    {
        return;
    }

    (void)HAL_UART_Transmit(s_command_uart,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            HEADING_HOLD_TEST_TX_TIMEOUT_MS);
}

static const char *heading_hold_test_state_name(void)
{
    switch (s_state)
    {
        case HEADING_HOLD_TEST_SETTLING:
            return "SETTLING";

        case HEADING_HOLD_TEST_RUNNING:
            return "RUN";

        case HEADING_HOLD_TEST_STOPPED:
            return "STOP";

        case HEADING_HOLD_TEST_WAIT_SENSOR:
        default:
            return "WAIT_YAW";
    }
}

static void heading_hold_test_stop_chassis(void)
{
    if ((s_chassis != NULL) && (s_chassis_stopped == 0U))
    {
        (void)mecanum_chassis_stop(s_chassis);
        s_chassis_stopped = 1U;
    }
}

static uint8_t heading_hold_test_yaw_is_fresh(
    const hwt101ct_data_t *data,
    uint32_t now_ms)
{
    if ((data == NULL) || (data->yaw_valid == 0U))
    {
        return 0U;
    }

    return ((uint32_t)(now_ms - data->yaw_update_ms) <=
            HEADING_HOLD_TEST_MAX_YAW_AGE_MS) ? 1U : 0U;
}

static uint8_t heading_hold_test_get_fresh_yaw(
    hwt101ct_data_t *data,
    uint32_t now_ms)
{
    if ((data == NULL) ||
        (hwt101ct_get_data(s_hwt101ct_sensor, data) != HWT101CT_OK))
    {
        return 0U;
    }

    return heading_hold_test_yaw_is_fresh(data, now_ms);
}

static void heading_hold_test_output_status(uint32_t now_ms)
{
    hwt101ct_data_t data;
    char text[180];
    float target_yaw_deg = 0.0f;
    float error_deg = 0.0f;
    float correction_rad_s = 0.0f;
    uint32_t yaw_age_ms = 0U;
    uint8_t sensor_valid;
    int length;

    memset(&data, 0, sizeof(data));
    sensor_valid = heading_hold_test_get_fresh_yaw(&data, now_ms);
    if (data.yaw_valid != 0U)
    {
        yaw_age_ms = (uint32_t)(now_ms - data.yaw_update_ms);
    }
    (void)heading_hold_controller_get_status(&s_controller,
                                             &target_yaw_deg,
                                             &error_deg,
                                             &correction_rad_s);

    length = snprintf(text,
                      sizeof(text),
                      "HEADING STATE=%s ENABLE=%u SENSOR=%u "
                      "ZERO=%u YAW=%.2f TARGET=%.2f ERR=%+.2f "
                      "KP=%.3f SIGN=%+.0f WZ=%+.3f AGE=%lums\r\n",
                      heading_hold_test_state_name(),
                      (unsigned int)s_run_requested,
                      (unsigned int)sensor_valid,
                      (unsigned int)s_yaw_zero_initialized,
                      (double)data.yaw_deg,
                      (double)target_yaw_deg,
                      (double)error_deg,
                      (double)s_controller.config.kp_rad_s_per_rad,
                      (double)s_controller.config.control_sign,
                      (double)correction_rad_s,
                      (unsigned long)yaw_age_ms);
    if ((length > 0) && ((size_t)length < sizeof(text)))
    {
        heading_hold_test_send_text(text);
    }
}

static uint8_t heading_hold_test_queue_pop(uint8_t *data)
{
    uint32_t primask;

    if ((data == NULL) || (s_rx_head == s_rx_tail))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *data = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) %
                          HEADING_HOLD_TEST_RX_QUEUE_SIZE);
    __set_PRIMASK(primask);
    return 1U;
}

static char *heading_hold_test_trim_line(void)
{
    char *command = s_line;

    s_line[s_line_length] = '\0';
    while ((*command == ' ') || (*command == '\t'))
    {
        command++;
    }
    while ((s_line_length > 0U) &&
           ((s_line[s_line_length - 1U] == ' ') ||
            (s_line[s_line_length - 1U] == '\t')))
    {
        s_line_length--;
        s_line[s_line_length] = '\0';
    }
    return command;
}

static uint8_t heading_hold_test_parse_float(const char *text,
                                              float *value)
{
    char *end;
    float parsed_value;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    while ((*text == ' ') || (*text == '\t'))
    {
        text++;
    }
    parsed_value = strtof(text, &end);
    if (end == text)
    {
        return 0U;
    }
    while ((*end == ' ') || (*end == '\t'))
    {
        end++;
    }
    if ((*end != '\0') || !isfinite(parsed_value))
    {
        return 0U;
    }

    *value = parsed_value;
    return 1U;
}

static void heading_hold_test_start_now(uint32_t now_ms)
{
    hwt101ct_data_t data;
    float correction_rad_s;

    s_run_requested = 1U;
    s_settle_start_ms = 0U;
    if (heading_hold_test_get_fresh_yaw(&data, now_ms) == 0U)
    {
        heading_hold_controller_disable(&s_controller);
        s_state = HEADING_HOLD_TEST_WAIT_SENSOR;
        heading_hold_test_stop_chassis();
        heading_hold_test_send_text("OK START WAIT_YAW\r\n");
        return;
    }

    heading_hold_controller_enable(&s_controller);
    (void)heading_hold_controller_update(&s_controller,
                                         data.yaw_deg,
                                         &correction_rad_s);
    s_state = HEADING_HOLD_TEST_RUNNING;
    s_last_control_ms = now_ms;
    s_chassis_stopped = 0U;
    (void)mecanum_chassis_set_velocity(s_chassis, 0.0f, 0.0f, 0.0f);
    heading_hold_test_send_text("OK START TARGET_LOCKED\r\n");
}

static void heading_hold_test_zero_now(uint32_t now_ms)
{
    hwt101ct_data_t data;
    float correction_rad_s;

    if ((heading_hold_test_get_fresh_yaw(&data, now_ms) == 0U) ||
        (hwt101ct_set_current_yaw_zero(s_hwt101ct_sensor) != HWT101CT_OK) ||
        (heading_hold_test_get_fresh_yaw(&data, now_ms) == 0U))
    {
        heading_hold_test_send_text("ERR ZERO WAIT_YAW\r\n");
        return;
    }

    s_yaw_zero_initialized = 1U;
    s_run_requested = 1U;
    s_settle_start_ms = 0U;
    heading_hold_controller_enable(&s_controller);
    (void)heading_hold_controller_update(&s_controller,
                                         data.yaw_deg,
                                         &correction_rad_s);
    s_state = HEADING_HOLD_TEST_RUNNING;
    s_last_control_ms = now_ms;
    s_chassis_stopped = 0U;
    (void)mecanum_chassis_set_velocity(s_chassis, 0.0f, 0.0f, 0.0f);
    heading_hold_test_send_text("OK ZERO YAW=0 TARGET=0\r\n");
}

static void heading_hold_test_process_line(uint32_t now_ms)
{
    char response[80];
    char *command;
    const char *value_text = NULL;
    float value;
    uint8_t index;
    int length;

    command = heading_hold_test_trim_line();
    for (index = 0U; command[index] != '\0'; index++)
    {
        if ((command[index] >= 'a') && (command[index] <= 'z'))
        {
            command[index] = (char)(command[index] - 'a' + 'A');
        }
    }

    if ((strncmp(command, "KP=", 3U) == 0) ||
        (strncmp(command, "KP ", 3U) == 0))
    {
        value_text = &command[3];
        if ((heading_hold_test_parse_float(value_text, &value) == 0U) ||
            (value < HEADING_HOLD_TEST_KP_MIN) ||
            (value > HEADING_HOLD_TEST_KP_MAX))
        {
            heading_hold_test_send_text("ERR KP RANGE 0.0-5.0\r\n");
        }
        else
        {
            s_controller.config.kp_rad_s_per_rad = value;
            length = snprintf(response,
                              sizeof(response),
                              "OK KP=%.3f\r\n",
                              (double)value);
            if ((length > 0) && ((size_t)length < sizeof(response)))
            {
                heading_hold_test_send_text(response);
            }
        }
    }
    else if (strcmp(command, "START") == 0)
    {
        heading_hold_test_start_now(now_ms);
    }
    else if (strcmp(command, "ZERO") == 0)
    {
        heading_hold_test_zero_now(now_ms);
    }
    else if (strcmp(command, "SIGN=1") == 0)
    {
        s_controller.config.control_sign = 1.0f;
        heading_hold_test_send_text("OK SIGN=1\r\n");
    }
    else if (strcmp(command, "SIGN=-1") == 0)
    {
        s_controller.config.control_sign = -1.0f;
        heading_hold_test_send_text("OK SIGN=-1\r\n");
    }
    else if (strcmp(command, "STOP") == 0)
    {
        s_run_requested = 0U;
        s_state = HEADING_HOLD_TEST_STOPPED;
        heading_hold_controller_disable(&s_controller);
        heading_hold_test_stop_chassis();
        heading_hold_test_send_text("OK STOP\r\n");
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        heading_hold_test_output_status(now_ms);
    }
    else if (strcmp(command, "HELP") == 0)
    {
        heading_hold_test_send_text(
            "CMD: KP=0.0..5.0, SIGN=1/-1, START, STOP, ZERO, STATUS, HELP\r\n");
    }
    else if (*command != '\0')
    {
        heading_hold_test_send_text("ERR COMMAND, SEND HELP\r\n");
    }

    s_line_length = 0U;
}

static void heading_hold_test_process_uart(uint32_t now_ms)
{
    uint8_t data;

    if (s_uart_error_pending != 0U)
    {
        s_uart_error_pending = 0U;
        heading_hold_test_send_text("ERR USART6 RX\r\n");
    }

    while (heading_hold_test_queue_pop(&data) != 0U)
    {
        if ((data == '\r') || (data == '\n'))
        {
            if (s_line_length != 0U)
            {
                heading_hold_test_process_line(now_ms);
            }
            continue;
        }

        if (s_line_length >= (HEADING_HOLD_TEST_LINE_SIZE - 1U))
        {
            s_line_length = 0U;
            heading_hold_test_send_text("ERR COMMAND TOO LONG\r\n");
            continue;
        }

        s_line[s_line_length] = (char)data;
        s_line_length++;
    }

    if ((s_line_length != 0U) &&
        ((uint32_t)(now_ms - s_last_rx_tick) >=
         HEADING_HOLD_TEST_IDLE_TIMEOUT_MS))
    {
        heading_hold_test_process_line(now_ms);
    }
}

static void heading_hold_test_update_control(uint32_t now_ms)
{
    hwt101ct_data_t data;
    float correction_rad_s = 0.0f;

    if (s_run_requested == 0U)
    {
        heading_hold_test_stop_chassis();
        return;
    }

    if (heading_hold_test_get_fresh_yaw(&data, now_ms) == 0U)
    {
        heading_hold_controller_disable(&s_controller);
        s_state = HEADING_HOLD_TEST_WAIT_SENSOR;
        s_settle_start_ms = 0U;
        heading_hold_test_stop_chassis();
        return;
    }

    if (s_state == HEADING_HOLD_TEST_WAIT_SENSOR)
    {
        s_state = HEADING_HOLD_TEST_SETTLING;
        s_settle_start_ms = now_ms;
        heading_hold_test_stop_chassis();
        return;
    }

    if (s_state == HEADING_HOLD_TEST_SETTLING)
    {
        if ((uint32_t)(now_ms - s_settle_start_ms) <
            HEADING_HOLD_TEST_INITIAL_SETTLE_MS)
        {
            heading_hold_test_stop_chassis();
            return;
        }

        if (s_yaw_zero_initialized == 0U)
        {
            if ((hwt101ct_set_current_yaw_zero(s_hwt101ct_sensor) !=
                 HWT101CT_OK) ||
                (heading_hold_test_get_fresh_yaw(&data, now_ms) == 0U))
            {
                s_state = HEADING_HOLD_TEST_WAIT_SENSOR;
                s_settle_start_ms = 0U;
                heading_hold_test_stop_chassis();
                return;
            }
            s_yaw_zero_initialized = 1U;
            heading_hold_test_send_text("HEADING_TEST YAW_ZEROED\r\n");
        }

        heading_hold_controller_enable(&s_controller);
        (void)heading_hold_controller_update(&s_controller,
                                             data.yaw_deg,
                                             &correction_rad_s);
        s_state = HEADING_HOLD_TEST_RUNNING;
        s_last_control_ms = now_ms;
        s_chassis_stopped = 0U;
        (void)mecanum_chassis_set_velocity(s_chassis, 0.0f, 0.0f, 0.0f);
        heading_hold_test_send_text("HEADING_TEST_READY TARGET_LOCKED\r\n");
        return;
    }

    if ((s_state != HEADING_HOLD_TEST_RUNNING) ||
        ((uint32_t)(now_ms - s_last_control_ms) <
         HEADING_HOLD_TEST_CONTROL_PERIOD_MS))
    {
        return;
    }

    s_last_control_ms = now_ms;
    if (heading_hold_controller_update(&s_controller,
                                       data.yaw_deg,
                                       &correction_rad_s) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        heading_hold_test_stop_chassis();
        return;
    }

    s_chassis_stopped = 0U;
    if (mecanum_chassis_set_velocity(s_chassis,
                                     0.0f,
                                     0.0f,
                                     correction_rad_s) ==
        MECANUM_MOTOR_ERROR)
    {
        s_run_requested = 0U;
        s_state = HEADING_HOLD_TEST_STOPPED;
        heading_hold_controller_disable(&s_controller);
        heading_hold_test_stop_chassis();
        heading_hold_test_send_text("ERR CHASSIS COMMAND\r\n");
    }
}

heading_hold_test_result_t heading_hold_test_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *hwt101ct_sensor,
    UART_HandleTypeDef *command_uart)
{
    uint32_t now_ms;

    if ((chassis == NULL) || (hwt101ct_sensor == NULL) ||
        (command_uart == NULL))
    {
        return HEADING_HOLD_TEST_INVALID_PARAM;
    }
    if (heading_hold_controller_init(&s_controller) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return HEADING_HOLD_TEST_CONTROLLER_ERROR;
    }

    s_chassis = chassis;
    s_hwt101ct_sensor = hwt101ct_sensor;
    s_command_uart = command_uart;
    s_controller.config.max_correction_rad_s =
        HEADING_HOLD_TEST_MAX_CORRECTION_RAD_S;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_uart_error_pending = 0U;
    s_line_length = 0U;
    s_run_requested = 1U;
    s_initialized = 0U;
    s_chassis_stopped = 0U;
    s_yaw_zero_initialized = 0U;
    s_settle_start_ms = 0U;
    s_state = HEADING_HOLD_TEST_WAIT_SENSOR;
    now_ms = HAL_GetTick();
    s_last_rx_tick = now_ms;
    s_last_control_ms = now_ms;
    s_last_output_ms = now_ms;

    heading_hold_test_stop_chassis();
    s_initialized = 1U;
    if (HAL_UART_Receive_IT(s_command_uart, &s_rx_byte, 1U) != HAL_OK)
    {
        s_initialized = 0U;
        return HEADING_HOLD_TEST_UART_ERROR;
    }

    heading_hold_test_send_text(
        "HEADING_TEST BOOT, KEEP STILL 1s, SEND HELP FOR COMMANDS\r\n");
    return HEADING_HOLD_TEST_OK;
}

void heading_hold_test_task(void)
{
    uint32_t now_ms;

    if (s_initialized == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    heading_hold_test_process_uart(now_ms);
    heading_hold_test_update_control(now_ms);

    if ((uint32_t)(now_ms - s_last_output_ms) >=
        HEADING_HOLD_TEST_OUTPUT_PERIOD_MS)
    {
        s_last_output_ms = now_ms;
        heading_hold_test_output_status(now_ms);
    }
}

void heading_hold_test_on_rx_complete(UART_HandleTypeDef *huart)
{
    uint8_t next_head;

    if ((s_initialized == 0U) || (huart != s_command_uart))
    {
        return;
    }

    next_head = (uint8_t)((s_rx_head + 1U) %
                          HEADING_HOLD_TEST_RX_QUEUE_SIZE);
    if (next_head != s_rx_tail)
    {
        s_rx_queue[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
        s_last_rx_tick = HAL_GetTick();
    }
    else
    {
        s_uart_error_pending = 1U;
    }

    if (HAL_UART_Receive_IT(s_command_uart, &s_rx_byte, 1U) != HAL_OK)
    {
        s_uart_error_pending = 1U;
    }
}

void heading_hold_test_on_error(UART_HandleTypeDef *huart)
{
    if ((s_initialized == 0U) || (huart != s_command_uart))
    {
        return;
    }

    s_uart_error_pending = 1U;
    (void)HAL_UART_Receive_IT(s_command_uart, &s_rx_byte, 1U);
}
