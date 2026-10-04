#include "IMU_UART.h"

#include <string.h>

#define JY901S_UART_TYPE_TIME            0x50U
#define JY901S_UART_TYPE_ACCELERATION    0x51U
#define JY901S_UART_TYPE_GYRO            0x52U
#define JY901S_UART_TYPE_ANGLE           0x53U
#define JY901S_UART_TYPE_MAGNETIC        0x54U
#define JY901S_UART_TYPE_QUATERNION      0x59U

#define JY901S_UART_REGISTER_OUTPUT      0x02U
#define JY901S_UART_REGISTER_RATE        0x03U
#define JY901S_UART_REGISTER_UNLOCK      0x69U
#define JY901S_UART_UNLOCK_VALUE         0xB588U
#define JY901S_UART_COMMAND_SIZE         5U
#define JY901S_UART_COMMAND_DELAY_MS     100U
#define JY901S_UART_TX_TIMEOUT_MS        100U

#define JY901S_UART_ACCELERATION_SCALE   (16.0f / 32768.0f)
#define JY901S_UART_GYRO_SCALE           (2000.0f / 32768.0f)
#define JY901S_UART_ANGLE_SCALE          (180.0f / 32768.0f)
#define JY901S_UART_QUATERNION_SCALE     (1.0f / 32768.0f)
#define JY901S_UART_TEMPERATURE_SCALE    (1.0f / 100.0f)

static jy901s_uart_t *s_jy901s_uart;

static jy901s_uart_result_t jy901s_uart_write_register(
    jy901s_uart_t *sensor,
    uint8_t address,
    uint16_t value)
{
    uint8_t command[JY901S_UART_COMMAND_SIZE];
    HAL_StatusTypeDef status;

    command[0] = 0xFFU;
    command[1] = 0xAAU;
    command[2] = address;
    command[3] = (uint8_t)(value & 0x00FFU);
    command[4] = (uint8_t)(value >> 8U);

    status = HAL_UART_Transmit(sensor->huart,
                               command,
                               JY901S_UART_COMMAND_SIZE,
                               JY901S_UART_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        return JY901S_UART_OK;
    }
    if (status == HAL_BUSY)
    {
        return JY901S_UART_BUSY;
    }
    if (status == HAL_TIMEOUT)
    {
        return JY901S_UART_TIMEOUT;
    }
    return JY901S_UART_ERROR;
}

static uint32_t jy901s_uart_type_to_update_mask(uint8_t type)
{
    switch (type)
    {
        case JY901S_UART_TYPE_ACCELERATION:
            return JY901S_UART_UPDATE_ACCELERATION;

        case JY901S_UART_TYPE_GYRO:
            return JY901S_UART_UPDATE_GYRO;

        case JY901S_UART_TYPE_ANGLE:
            return JY901S_UART_UPDATE_ANGLE;

        case JY901S_UART_TYPE_MAGNETIC:
            return JY901S_UART_UPDATE_MAGNETIC;

        case JY901S_UART_TYPE_QUATERNION:
            return JY901S_UART_UPDATE_QUATERNION;

        default:
            return 0U;
    }
}

static int16_t jy901s_uart_read_i16_le(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static uint8_t jy901s_uart_is_supported_type(uint8_t type)
{
    return (type == JY901S_UART_TYPE_TIME) ||
           (type == JY901S_UART_TYPE_ACCELERATION) ||
           (type == JY901S_UART_TYPE_GYRO) ||
           (type == JY901S_UART_TYPE_ANGLE) ||
           (type == JY901S_UART_TYPE_MAGNETIC) ||
           (type == JY901S_UART_TYPE_QUATERNION);
}

static void jy901s_uart_parse_frame(jy901s_uart_t *sensor)
{
    uint8_t type = sensor->frame[1];
    uint8_t axis;
    uint32_t now;
    uint32_t update_mask;

    if (jy901s_uart_is_supported_type(type) == 0U)
    {
        return;
    }

    switch (type)
    {
        case JY901S_UART_TYPE_ACCELERATION:
            for (axis = 0U; axis < 3U; axis++)
            {
                sensor->acceleration_raw[axis] =
                    jy901s_uart_read_i16_le(&sensor->frame[2U + axis * 2U]);
                sensor->acceleration_g[axis] =
                    (float)sensor->acceleration_raw[axis] * JY901S_UART_ACCELERATION_SCALE;
                sensor->acceleration_mps2[axis] =
                    sensor->acceleration_g[axis] * JY901S_UART_GRAVITY_MPS2;
            }

            sensor->temperature_raw = jy901s_uart_read_i16_le(&sensor->frame[8]);
            sensor->temperature_c =
                (float)sensor->temperature_raw * JY901S_UART_TEMPERATURE_SCALE;
            break;

        case JY901S_UART_TYPE_GYRO:
            for (axis = 0U; axis < 3U; axis++)
            {
                sensor->angular_velocity_raw[axis] =
                    jy901s_uart_read_i16_le(&sensor->frame[2U + axis * 2U]);
                sensor->angular_velocity_dps[axis] =
                    (float)sensor->angular_velocity_raw[axis] * JY901S_UART_GYRO_SCALE;
                sensor->angular_velocity_rads[axis] =
                    sensor->angular_velocity_dps[axis] * JY901S_UART_DEG_TO_RAD;
            }
            break;

        case JY901S_UART_TYPE_ANGLE:
            for (axis = 0U; axis < 3U; axis++)
            {
                sensor->angle_raw[axis] =
                    jy901s_uart_read_i16_le(&sensor->frame[2U + axis * 2U]);
                sensor->angle_deg[axis] =
                    (float)sensor->angle_raw[axis] * JY901S_UART_ANGLE_SCALE;
            }
            break;

        case JY901S_UART_TYPE_MAGNETIC:
            for (axis = 0U; axis < 3U; axis++)
            {
                sensor->magnetic_raw[axis] =
                    jy901s_uart_read_i16_le(&sensor->frame[2U + axis * 2U]);
            }
            break;

        case JY901S_UART_TYPE_QUATERNION:
            for (axis = 0U; axis < 4U; axis++)
            {
                sensor->quaternion_raw[axis] =
                    jy901s_uart_read_i16_le(&sensor->frame[2U + axis * 2U]);
                sensor->quaternion[axis] =
                    (float)sensor->quaternion_raw[axis] * JY901S_UART_QUATERNION_SCALE;
            }
            break;

        case JY901S_UART_TYPE_TIME:
        default:
            return;
    }

    now = HAL_GetTick();
    update_mask = jy901s_uart_type_to_update_mask(type);
    if ((sensor->pending_update_mask & update_mask) != 0U)
    {
        sensor->overwritten_update_count++;
    }

    switch (type)
    {
        case JY901S_UART_TYPE_ACCELERATION:
            sensor->acceleration_frame_count++;
            sensor->acceleration_update_ms = now;
            break;

        case JY901S_UART_TYPE_GYRO:
            sensor->angular_velocity_frame_count++;
            sensor->angular_velocity_update_ms = now;
            break;

        case JY901S_UART_TYPE_ANGLE:
            sensor->angle_frame_count++;
            sensor->angle_update_ms = now;
            break;

        case JY901S_UART_TYPE_MAGNETIC:
            sensor->magnetic_frame_count++;
            sensor->magnetic_update_ms = now;
            break;

        case JY901S_UART_TYPE_QUATERNION:
            sensor->quaternion_frame_count++;
            sensor->quaternion_update_ms = now;
            break;

        default:
            return;
    }

    sensor->last_frame_type = type;
    sensor->last_update_ms = now;
    sensor->frame_count++;
    sensor->pending_update_mask |= update_mask;
    sensor->data_ready = 1U;
}

void jy901s_uart_process_byte(jy901s_uart_t *sensor, uint8_t byte)
{
    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return;
    }

    if (sensor->frame_index == 0U)
    {
        if (byte != 0x55U)
        {
            return;
        }

        sensor->frame[0] = byte;
        sensor->checksum = byte;
        sensor->frame_index = 1U;
        return;
    }

    sensor->frame[sensor->frame_index] = byte;

    if (sensor->frame_index < (JY901S_UART_FRAME_SIZE - 1U))
    {
        sensor->checksum = (uint8_t)(sensor->checksum + byte);
        sensor->frame_index++;
        return;
    }

    sensor->frame_index = 0U;
    if (sensor->checksum != byte)
    {
        sensor->checksum_error_count++;
        if (byte == 0x55U)
        {
            sensor->frame[0] = byte;
            sensor->checksum = byte;
            sensor->frame_index = 1U;
        }
        return;
    }

    jy901s_uart_parse_frame(sensor);
}

jy901s_uart_result_t jy901s_uart_init(jy901s_uart_t *sensor,
                                      UART_HandleTypeDef *huart)
{
    if ((sensor == NULL) || (huart == NULL))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    if ((s_jy901s_uart != NULL) && (s_jy901s_uart != sensor))
    {
        return JY901S_UART_BUSY;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->huart = huart;
    s_jy901s_uart = sensor;

    return jy901s_uart_start_receive(sensor);
}

jy901s_uart_result_t jy901s_uart_start_receive(jy901s_uart_t *sensor)
{
    HAL_StatusTypeDef status;

    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    status = HAL_UART_Receive_IT(sensor->huart, &sensor->rx_byte, 1U);
    if (status == HAL_OK)
    {
        return JY901S_UART_OK;
    }

    if (status == HAL_BUSY)
    {
        return JY901S_UART_BUSY;
    }

    if (status == HAL_TIMEOUT)
    {
        return JY901S_UART_TIMEOUT;
    }

    return JY901S_UART_ERROR;
}

jy901s_uart_result_t jy901s_uart_configure_output(jy901s_uart_t *sensor,
                                                  uint16_t output_mask,
                                                  uint16_t output_rate)
{
    jy901s_uart_result_t result;

    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    result = jy901s_uart_write_register(sensor,
                                        JY901S_UART_REGISTER_UNLOCK,
                                        JY901S_UART_UNLOCK_VALUE);
    if (result != JY901S_UART_OK)
    {
        return result;
    }
    HAL_Delay(JY901S_UART_COMMAND_DELAY_MS);

    result = jy901s_uart_write_register(sensor,
                                        JY901S_UART_REGISTER_OUTPUT,
                                        output_mask);
    if (result != JY901S_UART_OK)
    {
        return result;
    }
    HAL_Delay(JY901S_UART_COMMAND_DELAY_MS);

    result = jy901s_uart_write_register(sensor,
                                        JY901S_UART_REGISTER_RATE,
                                        output_rate);
    if (result != JY901S_UART_OK)
    {
        return result;
    }
    HAL_Delay(JY901S_UART_COMMAND_DELAY_MS);

    return JY901S_UART_OK;
}

jy901s_uart_result_t jy901s_uart_get_data(const jy901s_uart_t *sensor,
                                          jy901s_uart_t *data)
{
    uint32_t primask;

    if ((sensor == NULL) || (data == NULL) || (sensor->huart == NULL))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (sensor->frame_count == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return JY901S_UART_NOT_READY;
    }

    memcpy(data, sensor, sizeof(*data));
    if (primask == 0U)
    {
        __enable_irq();
    }

    return JY901S_UART_OK;
}

jy901s_uart_result_t jy901s_uart_take_data(jy901s_uart_t *sensor,
                                           jy901s_uart_t *data,
                                           uint32_t *updated_mask)
{
    uint32_t primask;

    if ((sensor == NULL) || (data == NULL) || (updated_mask == NULL) ||
        (sensor->huart == NULL))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (sensor->pending_update_mask == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return JY901S_UART_NOT_READY;
    }

    memcpy(data, sensor, sizeof(*data));
    *updated_mask = sensor->pending_update_mask;
    sensor->pending_update_mask = 0U;
    sensor->data_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return JY901S_UART_OK;
}

void jy901s_uart_clear_data(jy901s_uart_t *sensor)
{
    uint32_t primask;

    if (sensor != NULL)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        sensor->pending_update_mask = 0U;
        sensor->data_ready = 0U;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
}

uint8_t jy901s_uart_is_data_ready(const jy901s_uart_t *sensor)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    return sensor->data_ready;
}

void jy901s_uart_on_rx_complete(UART_HandleTypeDef *huart)
{
    if ((s_jy901s_uart == NULL) || (huart != s_jy901s_uart->huart))
    {
        return;
    }

    s_jy901s_uart->rx_byte_count++;
    jy901s_uart_process_byte(s_jy901s_uart, s_jy901s_uart->rx_byte);
    (void)jy901s_uart_start_receive(s_jy901s_uart);
}

void jy901s_uart_on_error(UART_HandleTypeDef *huart)
{
    if ((s_jy901s_uart == NULL) || (huart != s_jy901s_uart->huart))
    {
        return;
    }

    s_jy901s_uart->uart_error_count++;
    s_jy901s_uart->last_uart_error = HAL_UART_GetError(huart);
    s_jy901s_uart->frame_index = 0U;
    s_jy901s_uart->checksum = 0U;
    (void)jy901s_uart_start_receive(s_jy901s_uart);
}
