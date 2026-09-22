#include "screen.h"

#include <string.h>

static const uint8_t s_screen_end_bytes[3] = {0xFFU, 0xFFU, 0xFFU};

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

screen_result_t screen_init(screen_t *screen, UART_HandleTypeDef *huart)
{
    if ((screen == NULL) || (huart == NULL))
    {
        return SCREEN_INVALID_PARAM;
    }

    screen->huart = huart;
    screen->timeout_ms = SCREEN_DEFAULT_TIMEOUT_MS;

    return SCREEN_OK;
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
    return screen_set_text(screen, SCREEN_TASK_TEXT_CONTROL, task_code);
}
