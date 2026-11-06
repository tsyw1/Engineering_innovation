#ifndef ENGINEERING_INNOVATION_VISION_UART_H
#define ENGINEERING_INNOVATION_VISION_UART_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_UART_LINE_SIZE       1024U
#define VISION_UART_MAX_DETECTIONS  6U
#define VISION_FLAG_CONFIRMED      (1U << 0U)

typedef enum
{
    VISION_UART_MODE_NONE = 0,
    VISION_UART_MODE_MATERIAL = 1,
    VISION_UART_MODE_TARGET_RING = 2
} vision_uart_mode_t;

typedef enum
{
    VISION_UART_STATUS_UNKNOWN = 0,
    VISION_UART_STATUS_OK,
    VISION_UART_STATUS_UNCONFIRMED,
    VISION_UART_STATUS_NO_TARGET,
    VISION_UART_STATUS_ERROR,
    VISION_UART_STATUS_OVERFLOW,
    VISION_UART_STATUS_STALE,
    VISION_UART_STATUS_IDLE
} vision_uart_status_t;

typedef enum
{
    VISION_UART_RESULT_OK = 0,
    VISION_UART_RESULT_NOT_READY,
    VISION_UART_RESULT_INVALID_PARAM,
    VISION_UART_RESULT_UART_ERROR,
    VISION_UART_RESULT_PARSE_ERROR,
    VISION_UART_RESULT_NO_TARGET
} vision_uart_result_t;

typedef struct
{
    uint8_t id;
    uint8_t detected;
    uint8_t valid;
    uint8_t flags;
    uint16_t x;
    uint16_t y;
    /* 新协议中的像素偏差允许带小数，不能用整数接收。 */
    float dx;
    float dy;
    float confidence;
} vision_detection_t;

typedef struct
{
    vision_uart_mode_t mode;
    vision_uart_status_t status;
    uint32_t seq;
    uint32_t frame_id;
    uint32_t age_ms;
    uint16_t image_width;
    uint16_t image_height;
    uint8_t sequence_continuous;
    uint8_t valid;
    uint8_t detection_count;
    vision_detection_t detections[VISION_UART_MAX_DETECTIONS];
    uint32_t received_tick;
} vision_uart_frame_t;

typedef struct
{
    uint32_t rx_byte_count;
    uint32_t line_count;
    uint32_t frame_count;
    vision_uart_mode_t last_mode;
    vision_uart_status_t last_status;
    uint8_t last_valid;
    uint8_t last_detection_count;
    uint32_t last_age_ms;
    uint8_t first_id;
    uint8_t first_detected;
    uint8_t first_valid;
    int16_t first_dx;
    int16_t first_dy;
} vision_uart_diagnostics_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    char rx_line[VISION_UART_LINE_SIZE];
    uint16_t rx_length;
    uint8_t rx_overflow;
    char pending_line[VISION_UART_LINE_SIZE];
    uint16_t pending_length;
    volatile uint8_t pending_ready;
    volatile uint8_t pending_overflow;
    volatile vision_uart_frame_t latest_frame;
    volatile uint8_t frame_ready;
    volatile uint32_t rx_byte_count;
    volatile uint32_t line_count;
    volatile uint32_t frame_count;
    volatile vision_uart_mode_t last_mode;
    volatile vision_uart_status_t last_status;
    volatile uint8_t last_valid;
    volatile uint8_t last_detection_count;
    volatile uint32_t last_age_ms;
    volatile uint8_t first_id;
    volatile uint8_t first_detected;
    volatile uint8_t first_valid;
    volatile int16_t first_dx;
    volatile int16_t first_dy;
    uint32_t last_seq;
    uint8_t have_last_seq;
    uint8_t initialized;
    /* 视觉结果包不再携带 mode，使用最近一次发送的命令确定当前模式。 */
    vision_uart_mode_t current_mode;
} vision_uart_t;

vision_uart_result_t vision_uart_init(vision_uart_t *vision,
                                      UART_HandleTypeDef *huart);
/** 启动视觉串口单字节中断接收。 */
vision_uart_result_t vision_uart_start_receive(vision_uart_t *vision);
vision_uart_t *vision_uart_get_default(void);
vision_uart_result_t vision_uart_set_mode(vision_uart_t *vision,
                                          vision_uart_mode_t mode);
void vision_uart_process_byte(vision_uart_t *vision, uint8_t byte);
void vision_uart_task(vision_uart_t *vision);
vision_uart_result_t vision_uart_take_frame(vision_uart_t *vision,
                                            vision_uart_frame_t *frame);
/** 丢弃当前半帧、待解析帧及已发布帧，不发送模式命令。 */
void vision_uart_clear_frame(vision_uart_t *vision);
void vision_uart_get_diagnostics(const vision_uart_t *vision,
                                 vision_uart_diagnostics_t *diagnostics);
/* 按 ID 筛选有效目标；同 ID 有多个时返回置信度最高者。 */
vision_uart_result_t vision_uart_find_detection(
    const vision_uart_frame_t *frame,
    uint8_t id,
    vision_detection_t *detection);
void vision_uart_on_error(vision_uart_t *vision);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_VISION_UART_H */
