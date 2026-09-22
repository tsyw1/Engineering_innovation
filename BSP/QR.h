#ifndef ENGINEERING_INNOVATION_QR_H
#define ENGINEERING_INNOVATION_QR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 任务码固定为：XXX+XXX+XXX+XXX，共 15 个有效字符。 */
#define QR_TASK_TEXT_LENGTH     15U
#define QR_BATCH_ITEM_COUNT     3U

typedef enum
{
    QR_STATUS_OK = 0,
    QR_STATUS_NOT_READY,
    QR_STATUS_NULL_POINTER,
    QR_STATUS_FORMAT_ERROR,
    QR_STATUS_COLOR_ERROR,
    QR_STATUS_POSITION_ERROR,
    QR_STATUS_HAL_ERROR
} QR_Status_t;

typedef struct
{
    /* 第一批物料：颜色和搬运顺序。 */
    uint8_t first_batch_color[QR_BATCH_ITEM_COUNT];
    /* 第一批物料在粗加工区/暂存区对应的圆环编号。 */
    uint8_t first_batch_position[QR_BATCH_ITEM_COUNT];

    /* 第二批物料：颜色和搬运顺序。 */
    uint8_t second_batch_color[QR_BATCH_ITEM_COUNT];
    /* 第二批物料在粗加工区对应的圆环编号。 */
    uint8_t second_batch_position[QR_BATCH_ITEM_COUNT];

    /* 最近一次有效任务码的原始字符串。 */
    char raw_code[QR_TASK_TEXT_LENGTH + 1U];
    uint8_t valid;
} QR_TaskCode_t;

/* 在 MX_USART6_UART_Init() 之后调用，启动二维码串口的单字节中断接收。 */
HAL_StatusTypeDef QR_Init(void);
HAL_StatusTypeDef QR_StartReceive(void);

/* 供串口接收回调输入一个字节。 */
void QR_ProcessByte(uint8_t data);

/* 解析一个完整任务码，例如：XXX+XXX+XXX+XXX。 */
QR_Status_t QR_ParseTaskCode(const char *code, QR_TaskCode_t *task);

/* 读取最近一次有效任务码；没有新任务时返回 QR_STATUS_NOT_READY。 */
QR_Status_t QR_GetTask(QR_TaskCode_t *task);
uint8_t QR_IsTaskReady(void);
void QR_ClearTask(void);

/* 非法任务码会生成一次错误事件，处理完成后调用 QR_ClearError()。 */
uint8_t QR_IsErrorReady(void);
QR_Status_t QR_GetError(void);
void QR_ClearError(void);

QR_Status_t QR_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* ENGINEERING_INNOVATION_QR_H */
