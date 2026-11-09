#include "HWT101CT.h"

#include <string.h>

static hwt101ct_t *s_hwt101ct;

static float hwt101ct_wrap_yaw_deg(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg <= -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static int16_t hwt101ct_read_i16_le(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static uint8_t hwt101ct_checksum(const uint8_t *frame)
{
    uint8_t index;
    uint8_t checksum = 0U;

    for (index = 0U; index < (HWT101CT_UART_FRAME_SIZE - 1U); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    return checksum;
}

static uint8_t hwt101ct_is_supported_type(uint8_t type)
{
    return (type == HWT101CT_UART_TYPE_ANGLE);
}

static void hwt101ct_parse_frame(hwt101ct_t *sensor)
{
    int16_t yaw_raw;
    float yaw_raw_deg;
    uint8_t type = sensor->frame[1];
    uint32_t now = HAL_GetTick();
    uint32_t update_mask = 0U;

    if (type == HWT101CT_UART_TYPE_ANGLE)
    {
        /* 0x53 帧的字节 6、7 是传感器内部解算的 Yaw(Z) 值。 */
        yaw_raw = hwt101ct_read_i16_le(&sensor->frame[6]);
        yaw_raw_deg = (float)yaw_raw * HWT101CT_ANGLE_SCALE_DEG;
        sensor->yaw_raw_deg = yaw_raw_deg;
        if (sensor->yaw_zero_valid != 0U)
        {
            sensor->data.yaw_deg = hwt101ct_wrap_yaw_deg(
                yaw_raw_deg - sensor->yaw_zero_offset_deg);
        }
        else
        {
            sensor->data.yaw_deg = yaw_raw_deg;
        }
        sensor->data.yaw_valid = 1U;
        sensor->data.yaw_update_ms = now;
        sensor->yaw_frame_count++;
        update_mask = HWT101CT_UPDATE_ANGLE;
    }
    else
    {
        return;
    }

    sensor->data.last_update_ms = now;
    sensor->data_ready = 1U;
    sensor->pending_update_mask |= update_mask;
    sensor->frame_count++;
}

static hwt101ct_result_t hwt101ct_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return HWT101CT_OK;

        case HAL_BUSY:
            return HWT101CT_BUSY;

        case HAL_TIMEOUT:
            return HWT101CT_TIMEOUT;

        case HAL_ERROR:
        default:
            return HWT101CT_ERROR;
    }
}

static uint8_t hwt101ct_is_valid(const hwt101ct_t *sensor)
{
    return (sensor != NULL) && (sensor->huart != NULL);
}

hwt101ct_result_t hwt101ct_init(hwt101ct_t *sensor,
                                UART_HandleTypeDef *huart)
{
    if ((sensor == NULL) || (huart == NULL))
    {
        return HWT101CT_INVALID_PARAM;
    }

    if ((s_hwt101ct != NULL) && (s_hwt101ct != sensor))
    {
        return HWT101CT_BUSY;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->huart = huart;
    sensor->last_rx_byte_ms = HAL_GetTick();
    s_hwt101ct = sensor;

    return hwt101ct_start_receive(sensor);
}

hwt101ct_result_t hwt101ct_start_receive(hwt101ct_t *sensor)
{
    HAL_StatusTypeDef status;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    status = HAL_UART_Receive_IT(sensor->huart, &sensor->rx_byte, 1U);
    return hwt101ct_from_hal_status(status);
}

hwt101ct_result_t hwt101ct_restart_receive(hwt101ct_t *sensor)
{
    HAL_StatusTypeDef status;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    status = HAL_UART_AbortReceive(sensor->huart);
    if (status != HAL_OK)
    {
        return hwt101ct_from_hal_status(status);
    }

    sensor->frame_index = 0U;
    sensor->last_rx_byte_ms = HAL_GetTick();
    sensor->rx_restart_count++;
    return hwt101ct_start_receive(sensor);
}

void hwt101ct_receive_watchdog(hwt101ct_t *sensor,
                               uint32_t now_ms,
                               uint32_t stale_ms)
{
    int32_t elapsed_ms;

    if ((hwt101ct_is_valid(sensor) == 0U) || (stale_ms == 0U))
    {
        return;
    }

    /* 只依据最近一次校验通过的角度帧判断传感器是否正常。
     * 收到乱码/校验错误字节不能阻止接收看门狗重启串口。 */
    elapsed_ms = (int32_t)(now_ms - sensor->data.yaw_update_ms);
    if ((sensor->data.yaw_update_ms == 0U) || (elapsed_ms < 0) ||
        ((uint32_t)elapsed_ms < stale_ms))
    {
        return;
    }

    (void)hwt101ct_restart_receive(sensor);
}

void hwt101ct_process_byte(hwt101ct_t *sensor, uint8_t byte)
{
    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return;
    }

    sensor->rx_byte_count++;
    sensor->last_rx_byte_ms = HAL_GetTick();

    if (sensor->frame_index == 0U)
    {
        if (byte == HWT101CT_UART_FRAME_HEADER)
        {
            sensor->frame[0] = byte;
            sensor->frame_index = 1U;
        }
        return;
    }

    if ((sensor->frame_index == 1U) &&
        (hwt101ct_is_supported_type(byte) == 0U))
    {
        sensor->unknown_frame_count++;
        sensor->frame_index =
            (byte == HWT101CT_UART_FRAME_HEADER) ? 1U : 0U;
        if (sensor->frame_index != 0U)
        {
            sensor->frame[0] = byte;
        }
        return;
    }

    sensor->frame[sensor->frame_index] = byte;
    sensor->frame_index++;

    if (sensor->frame_index < HWT101CT_UART_FRAME_SIZE)
    {
        return;
    }

    if (hwt101ct_checksum(sensor->frame) == sensor->frame[10])
    {
        hwt101ct_parse_frame(sensor);
    }
    else
    {
        sensor->checksum_error_count++;
    }

    sensor->frame_index = 0U;
}

hwt101ct_result_t hwt101ct_get_data(const hwt101ct_t *sensor,
                                    hwt101ct_data_t *data)
{
    uint32_t primask;

    if ((hwt101ct_is_valid(sensor) == 0U) || (data == NULL))
    {
        return HWT101CT_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (sensor->frame_count == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HWT101CT_NOT_READY;
    }

    memcpy(data, (const void *)&sensor->data, sizeof(*data));
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HWT101CT_OK;
}

hwt101ct_result_t hwt101ct_get_yaw_deg(const hwt101ct_t *sensor,
                                       float *yaw_deg)
{
    uint32_t primask;

    if ((hwt101ct_is_valid(sensor) == 0U) || (yaw_deg == NULL))
    {
        return HWT101CT_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((sensor->frame_count == 0U) || (sensor->data.yaw_valid == 0U))
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HWT101CT_NOT_READY;
    }

    *yaw_deg = sensor->data.yaw_deg;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HWT101CT_OK;
}

hwt101ct_result_t hwt101ct_set_current_yaw_zero(hwt101ct_t *sensor)
{
    uint32_t primask;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((sensor->frame_count == 0U) || (sensor->data.yaw_valid == 0U))
    {
        __set_PRIMASK(primask);
        return HWT101CT_NOT_READY;
    }

    sensor->yaw_zero_offset_deg = sensor->yaw_raw_deg;
    sensor->yaw_zero_valid = 1U;
    sensor->data.yaw_deg = 0.0f;
    __set_PRIMASK(primask);
    return HWT101CT_OK;
}

hwt101ct_result_t hwt101ct_clear_yaw_zero(hwt101ct_t *sensor)
{
    uint32_t primask;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    sensor->yaw_zero_offset_deg = 0.0f;
    sensor->yaw_zero_valid = 0U;
    if ((sensor->frame_count != 0U) && (sensor->data.yaw_valid != 0U))
    {
        sensor->data.yaw_deg = sensor->yaw_raw_deg;
    }
    __set_PRIMASK(primask);
    return HWT101CT_OK;
}

hwt101ct_result_t hwt101ct_take_data(hwt101ct_t *sensor,
                                     hwt101ct_data_t *data,
                                     uint32_t *updated_mask)
{
    uint32_t primask;

    if ((hwt101ct_is_valid(sensor) == 0U) || (data == NULL) ||
        (updated_mask == NULL))
    {
        return HWT101CT_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (sensor->pending_update_mask == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HWT101CT_NOT_READY;
    }

    memcpy(data, (const void *)&sensor->data, sizeof(*data));
    *updated_mask = sensor->pending_update_mask;
    sensor->pending_update_mask = 0U;
    sensor->data_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HWT101CT_OK;
}

void hwt101ct_clear_data(hwt101ct_t *sensor)
{
    uint32_t primask;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    sensor->pending_update_mask = 0U;
    sensor->data_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t hwt101ct_is_data_ready(const hwt101ct_t *sensor)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    return sensor->data_ready;
}

hwt101ct_result_t hwt101ct_write_register(hwt101ct_t *sensor,
                                          uint8_t address,
                                          uint16_t value)
{
    uint8_t command[HWT101CT_COMMAND_SIZE];
    HAL_StatusTypeDef status;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    command[0] = 0xFFU;
    command[1] = 0xAAU;
    command[2] = address;
    command[3] = (uint8_t)(value & 0x00FFU);
    command[4] = (uint8_t)(value >> 8U);

    status = HAL_UART_Transmit(sensor->huart,
                               command,
                               HWT101CT_COMMAND_SIZE,
                               HWT101CT_TX_TIMEOUT_MS);
    return hwt101ct_from_hal_status(status);
}

hwt101ct_result_t hwt101ct_configure_angle_output(hwt101ct_t *sensor,
                                                  uint16_t output_rate)
{
    hwt101ct_result_t result;

    if (hwt101ct_is_valid(sensor) == 0U)
    {
        return HWT101CT_INVALID_PARAM;
    }

    result = hwt101ct_write_register(sensor,
                                      HWT101CT_REGISTER_UNLOCK,
                                      HWT101CT_UNLOCK_VALUE);
    if (result != HWT101CT_OK)
    {
        return result;
    }
    HAL_Delay(HWT101CT_COMMAND_DELAY_MS);

    result = hwt101ct_write_register(sensor,
                                      HWT101CT_REGISTER_OUTPUT,
                                      HWT101CT_OUTPUT_ANGLE);
    if (result != HWT101CT_OK)
    {
        return result;
    }
    HAL_Delay(HWT101CT_COMMAND_DELAY_MS);

    result = hwt101ct_write_register(sensor,
                                      HWT101CT_REGISTER_RATE,
                                      output_rate);
    if (result != HWT101CT_OK)
    {
        return result;
    }
    HAL_Delay(HWT101CT_COMMAND_DELAY_MS);

    return HWT101CT_OK;
}

void hwt101ct_on_rx_complete(UART_HandleTypeDef *huart)
{
    if ((s_hwt101ct == NULL) || (huart != s_hwt101ct->huart))
    {
        return;
    }

    hwt101ct_process_byte(s_hwt101ct, s_hwt101ct->rx_byte);
    (void)hwt101ct_start_receive(s_hwt101ct);
}

void hwt101ct_on_error(UART_HandleTypeDef *huart)
{
    if ((s_hwt101ct == NULL) || (huart != s_hwt101ct->huart))
    {
        return;
    }

    s_hwt101ct->uart_error_count++;
    s_hwt101ct->last_uart_error = HAL_UART_GetError(huart);
    s_hwt101ct->frame_index = 0U;
    s_hwt101ct->last_rx_byte_ms = HAL_GetTick();
    s_hwt101ct->rx_restart_count++;
    (void)hwt101ct_start_receive(s_hwt101ct);
}
