#include "QR.h"
#include "IMU_UART.h"
#include "material_tray_uart_control.h"
#include "motor_TTL.h"
#include "vision_uart.h"
#include "screen.h"
#include "usart.h"

#include <string.h>

static uint8_t s_rx_byte;
static char s_rx_buffer[QR_TASK_TEXT_LENGTH + 1U];
static uint8_t s_rx_length;
static uint8_t s_rx_overflow;

static volatile QR_TaskCode_t s_task;
static volatile uint8_t s_task_ready;
static volatile uint8_t s_error_ready;
static volatile QR_Status_t s_error_status = QR_STATUS_NOT_READY;
static volatile QR_Status_t s_last_status = QR_STATUS_NOT_READY;
static volatile QR_Diagnostics_t s_diagnostics;

static uint8_t qr_is_digit(char value)
{
    return (value >= '0') && (value <= '9');
}

static uint8_t qr_is_color(uint8_t value)
{
    return (value >= 1U) && (value <= 6U);
}

static uint8_t qr_is_position(uint8_t value)
{
    return (value >= 1U) && (value <= 3U);
}

static void qr_copy_task(volatile QR_TaskCode_t *destination,
                         const QR_TaskCode_t *source)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    memcpy((void *)destination, source, sizeof(QR_TaskCode_t));
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void qr_finish_receive(void)
{
    QR_TaskCode_t task;
    QR_Status_t status;

    s_rx_buffer[s_rx_length] = '\0';
    status = QR_ParseTaskCode(s_rx_buffer, &task);
    s_last_status = status;

    if (status == QR_STATUS_OK)
    {
        s_diagnostics.valid_task_count++;
        qr_copy_task(&s_task, &task);
        s_task_ready = 1U;
        s_error_ready = 0U;
        s_error_status = QR_STATUS_NOT_READY;
    }
    else
    {
        s_diagnostics.parse_error_count++;
        s_error_status = status;
        s_error_ready = 1U;
    }

    s_rx_length = 0U;
    s_rx_buffer[0] = '\0';
}

HAL_StatusTypeDef QR_Init(void)
{
    QR_ClearTask();
    QR_ClearError();
    s_rx_length = 0U;
    s_rx_overflow = 0U;
    s_last_status = QR_STATUS_NOT_READY;
    memset((void *)&s_diagnostics, 0, sizeof(s_diagnostics));

    return QR_StartReceive();
}

HAL_StatusTypeDef QR_StartReceive(void)
{
    HAL_StatusTypeDef status;

    s_rx_byte = 0U;
    status = HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
    if (status != HAL_OK)
    {
        s_diagnostics.uart_error_count++;
        s_last_status = QR_STATUS_HAL_ERROR;
        s_error_status = QR_STATUS_HAL_ERROR;
        s_error_ready = 1U;
    }

    return status;
}

void QR_ProcessByte(uint8_t data)
{
    s_diagnostics.rx_byte_count++;
    s_diagnostics.last_byte = data;

    /* MaixCAM 使用 CR/LF 结束一帧，收到结束符后再校验完整任务码。 */
    if ((data == '\r') || (data == '\n'))
    {
        if (s_rx_overflow != 0U)
        {
            s_last_status = QR_STATUS_FORMAT_ERROR;
            s_error_status = QR_STATUS_FORMAT_ERROR;
            s_error_ready = 1U;
            s_diagnostics.parse_error_count++;
            s_rx_length = 0U;
            s_rx_overflow = 0U;
            s_rx_buffer[0] = '\0';
        }
        else if (s_rx_length > 0U)
        {
            qr_finish_receive();
        }
        s_diagnostics.current_length = s_rx_length;
        return;
    }

    if (s_rx_overflow != 0U)
    {
        return;
    }

    if (s_rx_length >= QR_TASK_TEXT_LENGTH)
    {
        /* 超长帧一直丢弃到行结束，避免误接收合法的 15 字符前缀。 */
        s_rx_overflow = 1U;
        return;
    }

    s_rx_buffer[s_rx_length] = (char)data;
    s_rx_length++;
    s_diagnostics.current_length = s_rx_length;

}

QR_Status_t QR_ParseTaskCode(const char *code, QR_TaskCode_t *task)
{
    uint8_t index;
    uint8_t value;
    size_t length;

    if ((code == NULL) || (task == NULL))
    {
        return QR_STATUS_NULL_POINTER;
    }

    length = strlen(code);
    while ((length > 0U) &&
           ((code[length - 1U] == '\r') ||
            (code[length - 1U] == '\n') ||
            (code[length - 1U] == ' ') ||
            (code[length - 1U] == '\t')))
    {
        length--;
    }

    if (length != QR_TASK_TEXT_LENGTH)
    {
        return QR_STATUS_FORMAT_ERROR;
    }

    if ((code[3] != '+') || (code[7] != '+') || (code[11] != '+'))
    {
        return QR_STATUS_FORMAT_ERROR;
    }

    memset(task, 0, sizeof(QR_TaskCode_t));

    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        if ((!qr_is_digit(code[index])) ||
            (!qr_is_digit(code[index + 4U])) ||
            (!qr_is_digit(code[index + 8U])) ||
            (!qr_is_digit(code[index + 12U])))
        {
            return QR_STATUS_FORMAT_ERROR;
        }

        task->first_batch_color[index] = (uint8_t)(code[index] - '0');
        task->first_batch_position[index] = (uint8_t)(code[index + 4U] - '0');
        task->second_batch_color[index] = (uint8_t)(code[index + 8U] - '0');
        task->second_batch_position[index] = (uint8_t)(code[index + 12U] - '0');
    }

    for (index = 0U; index < QR_BATCH_ITEM_COUNT; index++)
    {
        value = task->first_batch_color[index];
        if (!qr_is_color(value))
        {
            return QR_STATUS_COLOR_ERROR;
        }

        value = task->second_batch_color[index];
        if (!qr_is_color(value))
        {
            return QR_STATUS_COLOR_ERROR;
        }

        value = task->first_batch_position[index];
        if (!qr_is_position(value))
        {
            return QR_STATUS_POSITION_ERROR;
        }

        value = task->second_batch_position[index];
        if (!qr_is_position(value))
        {
            return QR_STATUS_POSITION_ERROR;
        }
    }

    memcpy(task->raw_code, code, QR_TASK_TEXT_LENGTH);
    task->raw_code[QR_TASK_TEXT_LENGTH] = '\0';
    task->valid = 1U;

    return QR_STATUS_OK;
}

QR_Status_t QR_GetTask(QR_TaskCode_t *task)
{
    if (task == NULL)
    {
        return QR_STATUS_NULL_POINTER;
    }

    if (s_task_ready == 0U)
    {
        return QR_STATUS_NOT_READY;
    }

    qr_copy_task((volatile QR_TaskCode_t *)task, (const QR_TaskCode_t *)&s_task);
    return QR_STATUS_OK;
}

QR_Status_t QR_TakeTask(QR_TaskCode_t *task)
{
    uint32_t primask;

    if (task == NULL)
    {
        return QR_STATUS_NULL_POINTER;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_task_ready == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return QR_STATUS_NOT_READY;
    }

    memcpy(task, (const void *)&s_task, sizeof(QR_TaskCode_t));
    s_task_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return QR_STATUS_OK;
}

uint8_t QR_IsTaskReady(void)
{
    return s_task_ready;
}

void QR_ClearTask(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    memset((void *)&s_task, 0, sizeof(s_task));
    s_task_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t QR_IsErrorReady(void)
{
    return s_error_ready;
}

QR_Status_t QR_GetError(void)
{
    if (s_error_ready == 0U)
    {
        return QR_STATUS_NOT_READY;
    }

    return s_error_status;
}

void QR_ClearError(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_error_status = QR_STATUS_NOT_READY;
    s_error_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

QR_Status_t QR_GetLastStatus(void)
{
    return s_last_status;
}

vision_uart_t *QR_GetVisionUart(void)
{
    return vision_uart_get_default();
}

void QR_GetDiagnostics(QR_Diagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(diagnostics,
           (const void *)&s_diagnostics,
           sizeof(QR_Diagnostics_t));
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* UART4 接收二维码，USART3 接收视觉 JSON，UART5 接收屏幕启动命令。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    vision_uart_t *vision;

    jy901s_uart_on_rx_complete(huart);
    motor_ttl_on_rx_complete(huart);
    material_tray_uart_control_on_rx_complete(huart);
    screen_on_rx_complete(huart);

    if (huart == &huart4)
    {
        QR_ProcessByte(s_rx_byte);
        if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) != HAL_OK)
        {
            s_diagnostics.uart_error_count++;
            s_last_status = QR_STATUS_HAL_ERROR;
            s_error_status = QR_STATUS_HAL_ERROR;
            s_error_ready = 1U;
        }
    }
    else if (huart == &huart3)
    {
        vision = vision_uart_get_default();
        vision_uart_process_byte(vision, vision->rx_byte);
        if (vision_uart_start_receive(vision) != VISION_UART_RESULT_OK)
        {
            vision_uart_on_error(vision);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    jy901s_uart_on_error(huart);
    motor_ttl_on_error(huart);
    material_tray_uart_control_on_error(huart);
    screen_on_error(huart);

    if (huart == &huart4)
    {
        s_diagnostics.uart_error_count++;
        s_last_status = QR_STATUS_HAL_ERROR;
        s_error_status = QR_STATUS_HAL_ERROR;
        s_error_ready = 1U;
        s_rx_length = 0U;
        s_rx_overflow = 0U;
        s_diagnostics.current_length = 0U;
        (void)QR_StartReceive();
    }
    else if (huart == &huart3)
    {
        vision_uart_on_error(vision_uart_get_default());
        (void)vision_uart_start_receive(vision_uart_get_default());
    }
}
