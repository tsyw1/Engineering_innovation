#ifndef ENGINEERING_INNOVATION_SCREEN_H
#define ENGINEERING_INNOVATION_SCREEN_H

#include "main.h"

#include <stdint.h>

#define SCREEN_DEFAULT_TIMEOUT_MS    100U
#define SCREEN_TASK_TEXT_CONTROL     "t0"
#define SCREEN_PICK_COUNT_CONTROL    "t3"
#define SCREEN_PLACE_COUNT_CONTROL   "t5"

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
    uint32_t successful_pick_count;
    uint32_t successful_place_count;
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

/** 将成功抓取次数显示到 t3 文本控件。 */
screen_result_t screen_show_pick_count(screen_t *screen,
                                       uint32_t count);

/** 将放到正确靶环或已有物料上的成功次数显示到 t5 文本控件。 */
screen_result_t screen_show_place_count(screen_t *screen,
                                        uint32_t count);

/** 清零抓取/放置统计，并同步刷新 t3、t5。 */
screen_result_t screen_reset_statistics(screen_t *screen);

/** 在一次抓取动作确认成功后调用，计数加 1 并刷新 t3。 */
screen_result_t screen_record_successful_pick(screen_t *screen);

/** 在一次靶环/已有物料正确放置动作确认成功后调用，计数加 1 并刷新 t5。 */
screen_result_t screen_record_successful_place(screen_t *screen);

#endif /* ENGINEERING_INNOVATION_SCREEN_H */
