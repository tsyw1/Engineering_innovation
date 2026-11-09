#include "servo_uart_test.h"

#include "joho_bus_servo.h"
#include "servo.h"
#include "tim.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define SERVO_UART_TEST_PWM_SERVO_COUNT    2U
#define SERVO_UART_TEST_PWM_MAX_ANGLE    180U
#define SERVO_UART_TEST_S3_MAX_POSITION 4095U
#define SERVO_UART_TEST_S3_ID              1U
#define SERVO_UART_TEST_S3_RUN_TIME_MS   200U
#define SERVO_UART_TEST_MIN_PULSE_US     500U
#define SERVO_UART_TEST_MAX_PULSE_US    2500U
#define SERVO_UART_TEST_RX_QUEUE_SIZE     32U
#define SERVO_UART_TEST_LINE_SIZE         16U
#define SERVO_UART_TEST_TX_TIMEOUT_MS    100U
#define SERVO_UART_TEST_IDLE_TIMEOUT_MS  100U

static servo_t s_pwm_servos[SERVO_UART_TEST_PWM_SERVO_COUNT];
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_queue[SERVO_UART_TEST_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_uart_error_pending;
static volatile uint32_t s_last_rx_tick;
static char s_line[SERVO_UART_TEST_LINE_SIZE];
static uint8_t s_line_length;
static uint8_t s_initialized;

static void servo_uart_test_send_text(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart6,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            SERVO_UART_TEST_TX_TIMEOUT_MS);
}

static uint8_t servo_uart_test_queue_pop(uint8_t *data)
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
                          SERVO_UART_TEST_RX_QUEUE_SIZE);
    __set_PRIMASK(primask);
    return 1U;
}

static uint8_t servo_uart_test_parse_line(uint8_t *servo_number,
                                          uint16_t *target)
{
    uint32_t value = 0U;
    uint8_t index;

    if ((servo_number == NULL) || (target == NULL) ||
        (s_line_length < 4U) || (s_line[0] != 'S') ||
        (s_line[1] < '1') || (s_line[1] > '3') ||
        (s_line[2] != ' '))
    {
        return 0U;
    }

    for (index = 3U; index < s_line_length; index++)
    {
        if ((s_line[index] < '0') || (s_line[index] > '9'))
        {
            return 0U;
        }

        value = value * 10U + (uint32_t)(s_line[index] - '0');
        if (value > SERVO_UART_TEST_S3_MAX_POSITION)
        {
            return 0U;
        }
    }

    *servo_number = (uint8_t)(s_line[1] - '0');
    *target = (uint16_t)value;
    if ((*servo_number <= SERVO_UART_TEST_PWM_SERVO_COUNT) &&
        (*target > SERVO_UART_TEST_PWM_MAX_ANGLE))
    {
        return 0U;
    }

    return 1U;
}

static void servo_uart_test_process_line(void)
{
    char response[40];
    int length;
    uint8_t servo_number;
    uint16_t target;
    servo_result_t servo_result;

    if (servo_uart_test_parse_line(&servo_number, &target) == 0U)
    {
        servo_uart_test_send_text(
            "ERR USE S1/S2 0-180 OR S3 0-4095\r\n");
        s_line_length = 0U;
        return;
    }

    if (servo_number <= SERVO_UART_TEST_PWM_SERVO_COUNT)
    {
        servo_result = servo_set_angle(
            &s_pwm_servos[servo_number - 1U], target);
        if (servo_result == SERVO_OK)
        {
            servo_result = servo_start(
                &s_pwm_servos[servo_number - 1U]);
        }
        if (servo_result != SERVO_OK)
        {
            servo_uart_test_send_text("ERR SERVO\r\n");
            s_line_length = 0U;
            return;
        }

        length = snprintf(response,
                          sizeof(response),
                          "OK S%u %udeg\r\n",
                          (unsigned int)servo_number,
                          (unsigned int)target);
    }
    else
    {
        if (joho_bus_servo_set_position(&huart2,
                                        SERVO_UART_TEST_S3_ID,
                                        target,
                                        SERVO_UART_TEST_S3_RUN_TIME_MS) !=
            JOHO_BUS_SERVO_OK)
        {
            servo_uart_test_send_text("ERR SERVO\r\n");
            s_line_length = 0U;
            return;
        }

        length = snprintf(response,
                          sizeof(response),
                          "OK S3 %upos\r\n",
                          (unsigned int)target);
    }

    if ((length > 0) && ((size_t)length < sizeof(response)))
    {
        servo_uart_test_send_text(response);
    }
    s_line_length = 0U;
}

servo_uart_test_result_t servo_uart_test_init(void)
{
    uint8_t index;

    memset(s_pwm_servos, 0, sizeof(s_pwm_servos));
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_uart_error_pending = 0U;
    s_last_rx_tick = HAL_GetTick();
    s_line_length = 0U;
    s_initialized = 0U;

    if ((servo_init(&s_pwm_servos[0], &htim1, TIM_CHANNEL_1) != SERVO_OK) ||
        (servo_init(&s_pwm_servos[1], &htim1, TIM_CHANNEL_2) != SERVO_OK))
    {
        return SERVO_UART_TEST_SERVO_ERROR;
    }

    for (index = 0U; index < SERVO_UART_TEST_PWM_SERVO_COUNT; index++)
    {
        if (servo_set_pulse_range(&s_pwm_servos[index],
                                  SERVO_UART_TEST_MIN_PULSE_US,
                                  SERVO_UART_TEST_MAX_PULSE_US) != SERVO_OK)
        {
            return SERVO_UART_TEST_SERVO_ERROR;
        }
    }

    if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) != HAL_OK)
    {
        return SERVO_UART_TEST_UART_ERROR;
    }

    s_initialized = 1U;
    servo_uart_test_send_text(
        "Servo test ready: S1/S2 0-180deg, S3 position 0-4095\r\n");
    return SERVO_UART_TEST_OK;
}

void servo_uart_test_task(void)
{
    uint8_t data;

    if (s_initialized == 0U)
    {
        return;
    }

    if (s_uart_error_pending != 0U)
    {
        s_uart_error_pending = 0U;
        servo_uart_test_send_text("ERR USART6\r\n");
    }

    while (servo_uart_test_queue_pop(&data) != 0U)
    {
        if ((data == '\r') || (data == '\n'))
        {
            if (s_line_length != 0U)
            {
                servo_uart_test_process_line();
            }
            continue;
        }

        if (s_line_length >= (SERVO_UART_TEST_LINE_SIZE - 1U))
        {
            s_line_length = 0U;
            servo_uart_test_send_text("ERR FORMAT\r\n");
            continue;
        }

        s_line[s_line_length] = (char)data;
        s_line_length++;
    }

    /* 串口助手未附加回车换行时，在一帧空闲后自动执行。 */
    if ((s_line_length != 0U) &&
        ((uint32_t)(HAL_GetTick() - s_last_rx_tick) >=
         SERVO_UART_TEST_IDLE_TIMEOUT_MS))
    {
        servo_uart_test_process_line();
    }
}

void servo_uart_test_on_rx_complete(UART_HandleTypeDef *huart)
{
    uint8_t next_head;

    if ((s_initialized == 0U) || (huart != &huart6))
    {
        return;
    }

    next_head = (uint8_t)((s_rx_head + 1U) %
                          SERVO_UART_TEST_RX_QUEUE_SIZE);
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

    if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) != HAL_OK)
    {
        s_uart_error_pending = 1U;
    }
}

void servo_uart_test_on_error(UART_HandleTypeDef *huart)
{
    if ((s_initialized == 0U) || (huart != &huart6))
    {
        return;
    }

    s_uart_error_pending = 1U;
    (void)HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U);
}
