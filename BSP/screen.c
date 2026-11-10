#include "screen.h"

#include "QR.h"

#include <stdio.h>
#include <string.h>

static const uint8_t s_screen_end_bytes[3] = {0xFFU, 0xFFU, 0xFFU};
static screen_t * volatile s_rx_screen;

static screen_result_t screen_rearm_receive(screen_t *screen)
{
    /* HAL 同时开启 RXNE/错误中断；等价寄存器位为 CR1.RXNEIE、CR3.EIE。 */
    if (HAL_UART_Receive_IT(screen->huart, &screen->rx_byte, 1U) != HAL_OK)
    {
        screen->rx_restart_needed = 1U;
        return SCREEN_ERROR;
    }
    screen->rx_restart_needed = 0U;
    return SCREEN_OK;
}

static screen_result_t screen_send_data(screen_t *screen,
                                        const uint8_t *data,
                                        uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((screen == NULL) || (screen->huart == NULL) ||
        (data == NULL) || (length == 0U))
    {
        return SCREEN_INVALID_PARAM;
    }

    status = HAL_UART_Transmit(screen->huart,
                               (uint8_t *)data,
                               length,
                               screen->timeout_ms);
    if (status != HAL_OK)
    {
        return SCREEN_ERROR;
    }

    return SCREEN_OK;
}

static uint8_t screen_text_is_valid(const char *text)
{
    if (text == NULL)
    {
        return 0U;
    }

    while (*text != '\0')
    {
        /* 双引号会提前结束串口屏的文本参数。 */
        if (*text == '"')
        {
            return 0U;
        }
        text++;
    }

    return 1U;
}

static screen_result_t screen_show_uint32(screen_t *screen,
                                          const char *control_name,
                                          uint32_t value)
{
    char text[11];
    int length;

    length = snprintf(text, sizeof(text), "%lu", (unsigned long)value);
    if ((length < 0) || ((size_t)length >= sizeof(text)))
    {
        return SCREEN_ERROR;
    }

    return screen_set_text(screen, control_name, text);
}

screen_result_t screen_init(screen_t *screen, UART_HandleTypeDef *huart)
{
    if ((screen == NULL) || (huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    screen->huart = huart;
    screen->timeout_ms = SCREEN_DEFAULT_TIMEOUT_MS;
    screen->successful_pick_count = 0U;
    screen->successful_place_count = 0U;
    screen->rx_byte = 0U;
    screen->pending_start_zone = START_ZONE_NONE;
    screen->start_locked = 0U;
    screen->rx_restart_needed = 0U;

    return SCREEN_OK;
}

screen_result_t screen_start_receive(screen_t *screen)
{
    uint32_t primask;
    screen_result_t result;

    if ((screen == NULL) || (screen->huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_rx_screen != NULL) && (s_rx_screen != screen))
    {
        result = SCREEN_INVALID_PARAM;
    }
    else if ((s_rx_screen == screen) &&
             (screen->huart->RxState == HAL_UART_STATE_BUSY_RX))
    {
        result = SCREEN_OK;
    }
    else
    {
        s_rx_screen = screen;
        result = screen_rearm_receive(screen);
    }
    __set_PRIMASK(primask);
    return result;
}

void screen_rx_task(screen_t *screen)
{
    uint32_t primask;

    if ((screen == NULL) || (screen->huart == NULL) ||
        (screen != s_rx_screen))
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if ((screen->rx_restart_needed != 0U) &&
        (screen->huart->RxState == HAL_UART_STATE_READY))
    {
        (void)screen_rearm_receive(screen);
    }
    __set_PRIMASK(primask);
}

uint8_t screen_take_start_zone(screen_t *screen, start_zone_t *start_zone)
{
    uint32_t primask;
    uint8_t ready = 0U;

    if ((screen == NULL) || (start_zone == NULL))
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (screen->pending_start_zone != START_ZONE_NONE)
    {
        *start_zone = screen->pending_start_zone;
        screen->pending_start_zone = START_ZONE_NONE;
        ready = 1U;
    }
    __set_PRIMASK(primask);
    return ready;
}

void screen_on_rx_complete(UART_HandleTypeDef *huart)
{
    screen_t *screen = s_rx_screen;
    start_zone_t start_zone = START_ZONE_NONE;

    if ((screen == NULL) || (huart != screen->huart))
    {
        return;
    }
    /* HAL 可能先回调收到字节、再回调错误；错误字节不能触发启动。
     * 此处也不重挂接收，以免清除 ErrorCode 干扰 HAL 的 ORE 处理。 */
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        screen->rx_restart_needed = 1U;
        return;
    }
    if (screen->rx_byte == SCREEN_START_ZONE_1_COMMAND)
    {
        start_zone = START_ZONE_1;
    }
    else if (screen->rx_byte == SCREEN_START_ZONE_2_COMMAND)
    {
        start_zone = START_ZONE_2;
    }
    if ((screen->start_locked == 0U) && (start_zone != START_ZONE_NONE))
    {
        /* 第一条有效命令锁定本轮路线，连点或运行中切区均不会覆盖。 */
        screen->start_locked = 1U;
        screen->pending_start_zone = start_zone;
    }
    (void)screen_rearm_receive(screen);
}

void screen_on_error(UART_HandleTypeDef *huart)
{
    screen_t *screen = s_rx_screen;

    if ((screen == NULL) || (huart != screen->huart))
    {
        return;
    }
    /* SR 后读 DR 可清除 PE/FE/NE/ORE；无 RXNE 的错误也须清除，避免重复中断。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        /* 非阻塞错误可能未结束接收，保留 HAL 当前的接收请求。 */
        screen->rx_restart_needed = 0U;
    }
    else
    {
        (void)screen_rearm_receive(screen);
    }
}

screen_result_t screen_send_command(screen_t *screen, const char *command)
{
    screen_result_t result;
    size_t command_length;

    if ((screen == NULL) || (screen->huart == NULL) || (command == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    command_length = strlen(command);
    if ((command_length == 0U) || (command_length > UINT16_MAX))
    {
        return SCREEN_INVALID_PARAM;
    }

    result = screen_send_data(screen,
                              (const uint8_t *)command,
                              (uint16_t)command_length);
    if (result != SCREEN_OK)
    {
        return result;
    }

    return screen_send_data(screen,
                            s_screen_end_bytes,
                            (uint16_t)sizeof(s_screen_end_bytes));
}

screen_result_t screen_set_text(screen_t *screen,
                                const char *control_name,
                                const char *text)
{
    static const uint8_t text_prefix[] = ".txt=\"";
    static const uint8_t text_suffix[] = "\"";
    screen_result_t result;
    size_t control_length;
    size_t text_length;

    if ((screen == NULL) || (screen->huart == NULL) ||
        (control_name == NULL) || (screen_text_is_valid(text) == 0U))
    {
        return SCREEN_INVALID_PARAM;
    }

    control_length = strlen(control_name);
    text_length = strlen(text);
    if ((control_length == 0U) ||
        (control_length > UINT16_MAX) ||
        (text_length > UINT16_MAX))
    {
        return SCREEN_INVALID_PARAM;
    }

    result = screen_send_data(screen,
                              (const uint8_t *)control_name,
                              (uint16_t)control_length);
    if (result != SCREEN_OK)
    {
        return result;
    }

    result = screen_send_data(screen,
                              text_prefix,
                              (uint16_t)(sizeof(text_prefix) - 1U));
    if (result != SCREEN_OK)
    {
        return result;
    }

    if (text_length > 0U)
    {
        result = screen_send_data(screen,
                                  (const uint8_t *)text,
                                  (uint16_t)text_length);
        if (result != SCREEN_OK)
        {
            return result;
        }
    }

    result = screen_send_data(screen,
                              text_suffix,
                              (uint16_t)(sizeof(text_suffix) - 1U));
    if (result != SCREEN_OK)
    {
        return result;
    }

    return screen_send_data(screen,
                            s_screen_end_bytes,
                            (uint16_t)sizeof(s_screen_end_bytes));
}

screen_result_t screen_show_task_code(screen_t *screen,
                                      const char *task_code)
{
    char first_half[(QR_TASK_TEXT_LENGTH / 2U) + 2U];
    char second_half[(QR_TASK_TEXT_LENGTH / 2U) + 1U];
    const size_t half_length = QR_TASK_TEXT_LENGTH / 2U;
    screen_result_t result;

    if ((task_code == NULL) ||
        (strlen(task_code) != QR_TASK_TEXT_LENGTH) ||
        (task_code[half_length] != '+'))
    {
        return SCREEN_INVALID_PARAM;
    }

    memcpy(first_half, task_code, half_length);
    first_half[half_length] = task_code[half_length];
    first_half[half_length + 1U] = '\0';
    memcpy(second_half,
           &task_code[half_length + 1U],
           half_length);
    second_half[half_length] = '\0';

    result = screen_set_text(screen,
                             SCREEN_TASK_FIRST_HALF_CONTROL,
                             first_half);
    if (result != SCREEN_OK)
    {
        return result;
    }

    return screen_set_text(screen,
                           SCREEN_TASK_SECOND_HALF_CONTROL,
                           second_half);
}

screen_result_t screen_show_pick_count(screen_t *screen,
                                       uint32_t count)
{
    return screen_show_uint32(screen, SCREEN_PICK_COUNT_CONTROL, count);
}

screen_result_t screen_show_place_count(screen_t *screen,
                                        uint32_t count)
{
    return screen_show_uint32(screen, SCREEN_PLACE_COUNT_CONTROL, count);
}

screen_result_t screen_reset_statistics(screen_t *screen)
{
    screen_result_t result;

    if ((screen == NULL) || (screen->huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    screen->successful_pick_count = 0U;
    screen->successful_place_count = 0U;

    result = screen_show_pick_count(screen, screen->successful_pick_count);
    if (result != SCREEN_OK)
    {
        return result;
    }

    return screen_show_place_count(screen, screen->successful_place_count);
}

screen_result_t screen_record_successful_pick(screen_t *screen)
{
    if ((screen == NULL) || (screen->huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    /* 饱和计数，避免达到 UINT32_MAX 后回绕为 0。 */
    if (screen->successful_pick_count != UINT32_MAX)
    {
        screen->successful_pick_count++;
    }

    return screen_show_pick_count(screen, screen->successful_pick_count);
}

screen_result_t screen_record_successful_place(screen_t *screen)
{
    if ((screen == NULL) || (screen->huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    /* 饱和计数，避免达到 UINT32_MAX 后回绕为 0。 */
    if (screen->successful_place_count != UINT32_MAX)
    {
        screen->successful_place_count++;
    }

    return screen_show_place_count(screen, screen->successful_place_count);
}
