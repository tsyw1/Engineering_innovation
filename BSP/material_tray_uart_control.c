#include "material_tray_uart_control.h"

#include "material_tray_servo.h"
#include "servo.h"
#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define MATERIAL_TRAY_UART_MIN_ANGLE_DEG       0U
#define MATERIAL_TRAY_UART_MAX_ANGLE_DEG       MATERIAL_TRAY_SERVO_MAX_ANGLE_DEG
#define MATERIAL_TRAY_UART_RX_QUEUE_SIZE      16U
#define MATERIAL_TRAY_UART_LINE_SIZE          6U
#define MATERIAL_TRAY_UART_TX_TIMEOUT_MS      100U

static servo_t s_material_tray_servo;
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_queue[MATERIAL_TRAY_UART_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_uart_error_pending;
static char s_line[MATERIAL_TRAY_UART_LINE_SIZE];
static uint8_t s_line_length;
static uint8_t s_pwm_started;
static uint8_t s_initialized;

static void material_tray_uart_send_text(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart6,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            MATERIAL_TRAY_UART_TX_TIMEOUT_MS);
}

static uint8_t material_tray_uart_queue_pop(uint8_t *data)
{
    uint32_t primask;

    if ((data == NULL) || (s_rx_head == s_rx_tail))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *data = s_rx_queue[s_rx_tail];
    s_rx_tail =
        (uint8_t)((s_rx_tail + 1U) % MATERIAL_TRAY_UART_RX_QUEUE_SIZE);
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

static uint8_t material_tray_uart_parse_angle(uint16_t *angle_deg)
{
    uint32_t value = 0U;
    uint8_t index;

    if ((angle_deg == NULL) || (s_line_length == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < s_line_length; index++)
    {
        if ((s_line[index] < '0') || (s_line[index] > '9'))
        {
            return 0U;
        }

        value = value * 10U + (uint32_t)(s_line[index] - '0');
    }

    if (value > MATERIAL_TRAY_UART_MAX_ANGLE_DEG)
    {
        return 0U;
    }

    *angle_deg = (uint16_t)value;
    return 1U;
}

static void material_tray_uart_process_line(void)
{
    char response[32];
    int length;
    uint16_t angle_deg;

    if (material_tray_uart_parse_angle(&angle_deg) == 0U)
    {
        material_tray_uart_send_text("ERR ANGLE 0-270deg\r\n");
        s_line_length = 0U;
        return;
    }

    if (servo_set_angle_with_range(&s_material_tray_servo,
                                   angle_deg,
                                   MATERIAL_TRAY_UART_MAX_ANGLE_DEG) !=
        SERVO_OK)
    {
        material_tray_uart_send_text("ERR SERVO\r\n");
        s_line_length = 0U;
        return;
    }

    if (s_pwm_started == 0U)
    {
        if (servo_start(&s_material_tray_servo) != SERVO_OK)
        {
            material_tray_uart_send_text("ERR SERVO\r\n");
            s_line_length = 0U;
            return;
        }
        s_pwm_started = 1U;
    }

    length = snprintf(response,
                      sizeof(response),
                       "OK TRAY ANGLE %udeg\r\n",
                       (unsigned int)angle_deg);
    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        material_tray_uart_send_text(response);
    }

    s_line_length = 0U;
}

material_tray_uart_control_result_t material_tray_uart_control_init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_uart_error_pending = 0U;
    s_line_length = 0U;
    s_pwm_started = 0U;
    s_initialized = 0U;

    if (servo_init(&s_material_tray_servo, &htim1, TIM_CHANNEL_2) !=
        SERVO_OK)
    {
        return MATERIAL_TRAY_UART_CONTROL_SERVO_ERROR;
    }

    if (servo_set_pulse_range(&s_material_tray_servo,
                              MATERIAL_TRAY_SERVO_MIN_PULSE_US,
                              MATERIAL_TRAY_SERVO_MAX_PULSE_US) != SERVO_OK)
    {
        return MATERIAL_TRAY_UART_CONTROL_SERVO_ERROR;
    }

    if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) != HAL_OK)
    {
        return MATERIAL_TRAY_UART_CONTROL_UART_ERROR;
    }

    s_initialized = 1U;
    material_tray_uart_send_text(
        "material tray angle test, send 0-270deg plus CR/LF\r\n");
    return MATERIAL_TRAY_UART_CONTROL_OK;
}

void material_tray_uart_control_task(void)
{
    uint8_t data;

    if (s_initialized == 0U)
    {
        return;
    }

    if (s_uart_error_pending != 0U)
    {
        s_uart_error_pending = 0U;
        material_tray_uart_send_text("ERR UART6\r\n");
    }

    while (material_tray_uart_queue_pop(&data) != 0U)
    {
        if ((data == '\r') || (data == '\n'))
        {
            if (s_line_length != 0U)
            {
                material_tray_uart_process_line();
            }
            continue;
        }

        if ((data < '0') || (data > '9'))
        {
            s_line_length = 0U;
            material_tray_uart_send_text("ERR FORMAT\r\n");
            continue;
        }

        if (s_line_length >= (MATERIAL_TRAY_UART_LINE_SIZE - 1U))
        {
            s_line_length = 0U;
            material_tray_uart_send_text("ERR FORMAT\r\n");
            continue;
        }

        s_line[s_line_length] = (char)data;
        s_line_length++;
    }
}

void material_tray_uart_control_on_rx_complete(UART_HandleTypeDef *huart)
{
    uint8_t next_head;

    if (huart != &huart6)
    {
        return;
    }

    next_head =
        (uint8_t)((s_rx_head + 1U) % MATERIAL_TRAY_UART_RX_QUEUE_SIZE);
    if (next_head != s_rx_tail)
    {
        s_rx_queue[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
    }
    else
    {
        s_uart_error_pending = 1U;
    }

    if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) != HAL_OK)
    {
        s_uart_error_pending = 1U;
    }
}

void material_tray_uart_control_on_error(UART_HandleTypeDef *huart)
{
    if (huart != &huart6)
    {
        return;
    }

    s_uart_error_pending = 1U;
    (void)HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U);
}
