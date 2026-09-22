#ifndef ENGINEERING_INNOVATION_SCREEN_H
#define ENGINEERING_INNOVATION_SCREEN_H

#include "main.h"

#include <stdint.h>

#define SCREEN_DEFAULT_TIMEOUT_MS    100U
#define SCREEN_TASK_TEXT_CONTROL     "t0"

typedef enum
{
    SCREEN_OK = 0,
    SCREEN_ERROR,
    SCREEN_INVALID_PARAM
} screen_result_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint32_t timeout_ms;
} screen_t;

/** 初始化串口屏对象，UART 外设需要先由 CubeMX 完成初始化。 */
screen_result_t screen_init(screen_t *screen, UART_HandleTypeDef *huart);

/** 发送一条串口屏指令，并自动追加 0xFF 0xFF 0xFF。 */
screen_result_t screen_send_command(screen_t *screen, const char *command);

/** 给文本控件赋值，例如 screen_set_text(&screen, "t0", "任务码字符串")。 */
screen_result_t screen_set_text(screen_t *screen,
                                const char *control_name,
                                const char *text);

/** 将二维码任务码显示到 t0 文本控件。 */
screen_result_t screen_show_task_code(screen_t *screen,
                                      const char *task_code);

#endif /* ENGINEERING_INNOVATION_SCREEN_H */
