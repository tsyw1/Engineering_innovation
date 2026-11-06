#include "vision_uart.h"

#include <stdlib.h>
#include <string.h>

static vision_uart_t s_default_vision;

static const char *vision_skip_space(const char *text)
{
    while ((text != NULL) && ((*text == ' ') || (*text == '\t') ||
                               (*text == '\r') || (*text == '\n')))
    {
        text++;
    }
    return text;
}

static const char *vision_find_key(const char *begin,
                                   const char *end,
                                   const char *key)
{
    size_t key_length;
    const char *cursor;

    if ((begin == NULL) || (end == NULL) || (key == NULL) || (begin >= end))
    {
        return NULL;
    }

    key_length = strlen(key);
    cursor = begin;
    while (cursor < end)
    {
        if ((*cursor == '"') &&
            ((size_t)(end - cursor) >= key_length + 2U) &&
            (memcmp(cursor + 1, key, key_length) == 0) &&
            (cursor[1U + key_length] == '"'))
        {
            return cursor;
        }
        cursor++;
    }
    return NULL;
}

static const char *vision_find_colon(const char *begin, const char *end)
{
    const char *cursor = begin;

    while ((cursor != NULL) && (cursor < end))
    {
        if (*cursor == ':')
        {
            return cursor;
        }
        cursor++;
    }
    return NULL;
}

static uint8_t vision_parse_u32(const char *begin,
                                const char *end,
                                const char *key,
                                uint32_t *value)
{
    const char *key_start;
    const char *cursor;
    char *parse_end;
    unsigned long parsed;

    if ((value == NULL) || (begin == NULL) || (end == NULL))
    {
        return 0U;
    }
    key_start = vision_find_key(begin, end, key);
    if (key_start == NULL)
    {
        return 0U;
    }
    cursor = vision_find_colon(key_start, end);
    if (cursor == NULL)
    {
        return 0U;
    }
    cursor = vision_skip_space(cursor + 1);
    if (*cursor == '-')
    {
        return 0U;
    }
    parsed = strtoul(cursor, &parse_end, 10);
    if ((parse_end == cursor) || (parse_end > end))
    {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

static uint8_t vision_parse_u32_value(const char **cursor,
                                      const char *end,
                                      uint32_t *value)
{
    char *parse_end;
    unsigned long parsed;

    if ((cursor == NULL) || (*cursor == NULL) || (end == NULL) ||
        (value == NULL))
    {
        return 0U;
    }
    *cursor = vision_skip_space(*cursor);
    if (**cursor == '-')
    {
        return 0U;
    }
    parsed = strtoul(*cursor, &parse_end, 10);
    if ((parse_end == *cursor) || (parse_end > end))
    {
        return 0U;
    }
    *value = (uint32_t)parsed;
    *cursor = parse_end;
    return 1U;
}

static uint8_t vision_parse_float_value(const char **cursor,
                                        const char *end,
                                        float *value)
{
    char *parse_end;

    if ((cursor == NULL) || (*cursor == NULL) || (end == NULL) ||
        (value == NULL))
    {
        return 0U;
    }
    *cursor = vision_skip_space(*cursor);
    *value = strtof(*cursor, &parse_end);
    if ((parse_end == *cursor) || (parse_end > end))
    {
        return 0U;
    }
    *cursor = parse_end;
    return 1U;
}

static uint8_t vision_expect_char(const char **cursor,
                                  const char *end,
                                  char expected)
{
    if ((cursor == NULL) || (*cursor == NULL) || (end == NULL))
    {
        return 0U;
    }
    *cursor = vision_skip_space(*cursor);
    if ((*cursor >= end) || (**cursor != expected))
    {
        return 0U;
    }
    (*cursor)++;
    return 1U;
}

/* 解析 items:[[id,dx,dy], ...]，dx/dy 必须保留视觉端发送的小数。 */
static uint8_t vision_parse_items(const char *line,
                                  vision_uart_frame_t *frame)
{
    const char *line_end;
    const char *key_start;
    const char *cursor;
    const char *array_end;
    uint32_t id;
    float dx;
    float dy;

    if ((line == NULL) || (frame == NULL))
    {
        return 0U;
    }
    line_end = line + strlen(line);
    key_start = vision_find_key(line, line_end, "items");
    if (key_start == NULL)
    {
        return 0U;
    }
    cursor = vision_find_colon(key_start, line_end);
    if (cursor == NULL)
    {
        return 0U;
    }
    cursor++;
    cursor = vision_skip_space(cursor);
    if (*cursor != '[')
    {
        return 0U;
    }
    cursor++;
    array_end = line_end;

    cursor = vision_skip_space(cursor);
    if (*cursor == ']')
    {
        cursor++;
        cursor = vision_skip_space(cursor);
        return (*cursor == '}') ? 1U : 0U;
    }

    while (cursor < array_end)
    {
        if (frame->detection_count >= VISION_UART_MAX_DETECTIONS)
        {
            return 0U;
        }
        if (vision_expect_char(&cursor, array_end, '[') == 0U ||
            vision_parse_u32_value(&cursor, array_end, &id) == 0U ||
            vision_expect_char(&cursor, array_end, ',') == 0U ||
            vision_parse_float_value(&cursor, array_end, &dx) == 0U ||
            vision_expect_char(&cursor, array_end, ',') == 0U ||
            vision_parse_float_value(&cursor, array_end, &dy) == 0U ||
            vision_expect_char(&cursor, array_end, ']') == 0U)
        {
            return 0U;
        }
        if ((id == 0U) ||
            ((frame->mode == VISION_UART_MODE_MATERIAL) && (id > 6U)) ||
            ((frame->mode == VISION_UART_MODE_TARGET_RING) && (id > 3U)))
        {
            return 0U;
        }

        frame->detections[frame->detection_count].id = (uint8_t)id;
        frame->detections[frame->detection_count].detected = 1U;
        frame->detections[frame->detection_count].valid = 1U;
        frame->detections[frame->detection_count].flags = VISION_FLAG_CONFIRMED;
        frame->detections[frame->detection_count].dx = dx;
        frame->detections[frame->detection_count].dy = dy;
        frame->detections[frame->detection_count].confidence = 1.0f;
        frame->detection_count++;

        cursor = vision_skip_space(cursor);
        if (*cursor == ']')
        {
            cursor++;
            cursor = vision_skip_space(cursor);
            return (*cursor == '}') ? 1U : 0U;
        }
        if (*cursor != ',')
        {
            return 0U;
        }
        cursor++;
    }
    return 0U;
}

static void vision_parse_line(vision_uart_t *vision,
                              const char *line,
                              uint8_t overflow)
{
    vision_uart_frame_t frame;
    const char *line_end;

    memset(&frame, 0, sizeof(frame));
    frame.received_tick = HAL_GetTick();
    frame.mode = vision->current_mode;
    if ((line == NULL) || (*line != '{') || (overflow != 0U) ||
        (vision->current_mode == VISION_UART_MODE_NONE))
    {
        return;
    }
    line_end = line + strlen(line);
    if ((vision_parse_u32(line, line_end, "seq", &frame.seq) == 0U) ||
        (vision_parse_items(line, &frame) == 0U))
    {
        return;
    }
    /* 新协议没有视觉端 age_ms，接收时间由本机记录。 */
    frame.age_ms = 0U;
    frame.sequence_continuous = (vision->have_last_seq == 0U) ||
                                (frame.seq == (vision->last_seq + 1U));
    vision->last_seq = frame.seq;
    vision->have_last_seq = 1U;
    frame.valid = 1U;
    frame.status = (frame.detection_count > 0U) ?
                   VISION_UART_STATUS_OK : VISION_UART_STATUS_NO_TARGET;

    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy((void *)&vision->latest_frame, &frame, sizeof(frame));
        vision->frame_ready = 1U;
        vision->frame_count++;
        vision->last_mode = frame.mode;
        vision->last_status = frame.status;
        vision->last_valid = frame.valid;
        vision->last_detection_count = frame.detection_count;
        vision->last_age_ms = frame.age_ms;
        if (frame.detection_count > 0U)
        {
            vision->first_id = frame.detections[0].id;
            vision->first_detected = frame.detections[0].detected;
            vision->first_valid = frame.detections[0].valid;
            vision->first_dx = frame.detections[0].dx;
            vision->first_dy = frame.detections[0].dy;
        }
        else
        {
            vision->first_id = 0U;
            vision->first_detected = 0U;
            vision->first_valid = 0U;
            vision->first_dx = 0;
            vision->first_dy = 0;
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
}

vision_uart_result_t vision_uart_init(vision_uart_t *vision,
                                      UART_HandleTypeDef *huart)
{
    if ((vision == NULL) || (huart == NULL))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }
    memset(vision, 0, sizeof(*vision));
    vision->huart = huart;
    vision->current_mode = VISION_UART_MODE_NONE;
    vision->initialized = 1U;
    return VISION_UART_RESULT_OK;
}

vision_uart_result_t vision_uart_start_receive(vision_uart_t *vision)
{
    if ((vision == NULL) || (vision->initialized == 0U) ||
        (vision->huart == NULL))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }

    vision->rx_byte = 0U;
    return (HAL_UART_Receive_IT(vision->huart,
                                &vision->rx_byte,
                                1U) == HAL_OK) ?
           VISION_UART_RESULT_OK : VISION_UART_RESULT_UART_ERROR;
}

vision_uart_t *vision_uart_get_default(void)
{
    return &s_default_vision;
}

static void vision_uart_discard_input(vision_uart_t *vision)
{
    uint32_t primask;

    if (vision == NULL)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    vision->frame_ready = 0U;
    vision->pending_ready = 0U;
    vision->pending_length = 0U;
    vision->pending_overflow = 0U;
    vision->rx_length = 0U;
    vision->rx_overflow = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

vision_uart_result_t vision_uart_set_mode(vision_uart_t *vision,
                                          vision_uart_mode_t mode)
{
    uint8_t command[2];

    if ((vision == NULL) || (vision->initialized == 0U) ||
        (vision->huart == NULL))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }
    if ((mode != VISION_UART_MODE_MATERIAL) &&
        (mode != VISION_UART_MODE_TARGET_RING))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }

    /* 切换阶段时丢弃旧模式的半帧、待解析帧和已发布帧。 */
    vision_uart_discard_input(vision);

    command[0] = (mode == VISION_UART_MODE_MATERIAL) ? '1' : '2';
    command[1] = '\n';
    if (HAL_UART_Transmit(vision->huart, command, sizeof(command), 100U) != HAL_OK)
    {
        return VISION_UART_RESULT_UART_ERROR;
    }
    vision->current_mode = mode;
    return VISION_UART_RESULT_OK;
}

void vision_uart_process_byte(vision_uart_t *vision, uint8_t byte)
{
    uint32_t primask;

    if ((vision == NULL) || (vision->initialized == 0U))
    {
        return;
    }
    vision->rx_byte_count++;
    /* 兼容视觉端发送 LF、CR 或 CRLF 作为一帧结束符。 */
    if ((byte == '\r') || (byte == '\n'))
    {
        if ((vision->rx_length == 0U) && (vision->rx_overflow == 0U))
        {
            return;
        }
        vision->line_count++;
        primask = __get_PRIMASK();
        __disable_irq();
        if (vision->pending_ready == 0U)
        {
            memcpy(vision->pending_line, vision->rx_line, vision->rx_length);
            vision->pending_line[vision->rx_length] = '\0';
            vision->pending_length = vision->rx_length;
            vision->pending_overflow = vision->rx_overflow;
            vision->pending_ready = 1U;
        }
        vision->rx_length = 0U;
        vision->rx_overflow = 0U;
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    if (vision->rx_length < (VISION_UART_LINE_SIZE - 1U))
    {
        vision->rx_line[vision->rx_length++] = (char)byte;
    }
    else
    {
        vision->rx_overflow = 1U;
    }
}

void vision_uart_task(vision_uart_t *vision)
{
    char line[VISION_UART_LINE_SIZE];
    uint16_t length;
    uint8_t overflow;
    uint32_t primask;

    if ((vision == NULL) || (vision->initialized == 0U))
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (vision->pending_ready == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    length = vision->pending_length;
    overflow = vision->pending_overflow;
    memcpy(line, vision->pending_line, length + 1U);
    vision->pending_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    vision_parse_line(vision, line, overflow);
}

vision_uart_result_t vision_uart_take_frame(vision_uart_t *vision,
                                            vision_uart_frame_t *frame)
{
    uint32_t primask;

    if ((vision == NULL) || (frame == NULL) ||
        (vision->initialized == 0U))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (vision->frame_ready == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return VISION_UART_RESULT_NOT_READY;
    }
    memcpy(frame, (const void *)&vision->latest_frame, sizeof(*frame));
    vision->frame_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return VISION_UART_RESULT_OK;
}

void vision_uart_clear_frame(vision_uart_t *vision)
{
    vision_uart_discard_input(vision);
}

void vision_uart_get_diagnostics(const vision_uart_t *vision,
                                 vision_uart_diagnostics_t *diagnostics)
{
    uint32_t primask;

    if ((vision == NULL) || (diagnostics == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    diagnostics->rx_byte_count = vision->rx_byte_count;
    diagnostics->line_count = vision->line_count;
    diagnostics->frame_count = vision->frame_count;
    diagnostics->last_mode = vision->last_mode;
    diagnostics->last_status = vision->last_status;
    diagnostics->last_valid = vision->last_valid;
    diagnostics->last_detection_count = vision->last_detection_count;
    diagnostics->last_age_ms = vision->last_age_ms;
    diagnostics->first_id = vision->first_id;
    diagnostics->first_detected = vision->first_detected;
    diagnostics->first_valid = vision->first_valid;
    diagnostics->first_dx = vision->first_dx;
    diagnostics->first_dy = vision->first_dy;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

vision_uart_result_t vision_uart_find_detection(
    const vision_uart_frame_t *frame,
    uint8_t id,
    vision_detection_t *detection)
{
    uint8_t index;
    uint8_t found = 0U;
    float best_confidence = 0.0f;

    if ((frame == NULL) || (detection == NULL) || (id == 0U))
    {
        return VISION_UART_RESULT_INVALID_PARAM;
    }
    for (index = 0U; index < frame->detection_count; index++)
    {
        if ((frame->detections[index].id == id) &&
            (frame->detections[index].valid != 0U))
        {
            /* 同一颜色出现多个候选时，优先使用置信度最高的目标。 */
            if ((found == 0U) ||
                (frame->detections[index].confidence > best_confidence))
            {
                *detection = frame->detections[index];
                best_confidence = frame->detections[index].confidence;
                found = 1U;
            }
        }
    }
    return (found != 0U) ?
           VISION_UART_RESULT_OK : VISION_UART_RESULT_NO_TARGET;
}

void vision_uart_on_error(vision_uart_t *vision)
{
    if (vision != NULL)
    {
        vision->rx_length = 0U;
        vision->rx_overflow = 0U;
        vision->pending_ready = 0U;
    }
}
