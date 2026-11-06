#ifndef ENGINEERING_INNOVATION_SCREEN_H
#define ENGINEERING_INNOVATION_SCREEN_H

#include "main.h"
#include "start_zone.h"

#include <stdint.h>

#define SCREEN_DEFAULT_TIMEOUT_MS    100U
#define SCREEN_TASK_TEXT_CONTROL     "t0"
#define SCREEN_PICK_COUNT_CONTROL    "t3"
#define SCREEN_PLACE_COUNT_CONTROL   "t5"
#define SCREEN_START_ZONE_1_COMMAND   0x31U
#define SCREEN_START_ZONE_2_COMMAND   0x32U

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
    uint8_t rx_byte;
    volatile start_zone_t pending_start_zone;
    volatile uint8_t start_locked;
    volatile uint8_t rx_restart_needed;
} screen_t;

/** 初始化串口屏对象，UART 外设需要先由 CubeMX 完成初始化。 */
screen_result_t screen_init(screen_t *screen, UART_HandleTypeDef *huart);

/** 启用单字节中断接收。仅支持一个活动串口屏，不等待结束符。 */
screen_result_t screen_start_receive(screen_t *screen);

/** 主循环调用，重试因 UART 错误或接收挂接失败而中断的接收。 */
void screen_rx_task(screen_t *screen);

/** 原子取走首次有效启动命令；本次上电期间后续启动命令均忽略。 */
uint8_t screen_take_start_zone(screen_t *screen, start_zone_t *start_zone);

/** 由工程唯一的 HAL UART 回调分发调用；中断内不执行运动或发送。 */
void screen_on_rx_complete(UART_HandleTypeDef *huart);
void screen_on_error(UART_HandleTypeDef *huart);

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
