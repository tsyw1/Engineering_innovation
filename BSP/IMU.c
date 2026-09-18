#include "IMU.h"

#include <string.h>

// 连续运动数据寄存器，范围为 0x34-0x40，共 13 个 16 位寄存器。
#define JY901S_REG_ACCELERATION_X       0x34U
#define JY901S_REG_QUATERNION_0         0x51U

#define JY901S_MOTION_REGISTER_COUNT    13U
#define JY901S_MOTION_BYTE_COUNT        (JY901S_MOTION_REGISTER_COUNT * 2U)
#define JY901S_QUATERNION_COUNT         4U
#define JY901S_QUATERNION_BYTE_COUNT    (JY901S_QUATERNION_COUNT * 2U)

#define JY901S_ACCELERATION_SCALE       (16.0f / 32768.0f)
#define JY901S_ANGULAR_VELOCITY_SCALE   (2000.0f / 32768.0f)
#define JY901S_ANGLE_SCALE              (180.0f / 32768.0f)
#define JY901S_QUATERNION_SCALE         (1.0f / 32768.0f)
#define JY901S_TEMPERATURE_SCALE        (1.0f / 100.0f)

/** 将 7 位地址转换为 STM32 HAL 要求的左对齐设备地址。 */
static uint16_t jy901s_hal_address(const jy901s_t *sensor)
{
    return (uint16_t)((uint16_t)sensor->address_7bit << 1U);
}

/** 从 JY901S 的小端字节流中读取一个有符号 16 位数。 */
static int16_t jy901s_read_i16_le(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

/** 将 STM32 HAL 状态统一转换为驱动结果。 */
static jy901s_result_t jy901s_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return JY901S_OK;

        case HAL_BUSY:
            return JY901S_BUSY;

        case HAL_TIMEOUT:
            return JY901S_TIMEOUT;

        case HAL_ERROR:
        default:
            return JY901S_ERROR;
    }
}

/** 检查对象和 I2C 句柄是否已经初始化。 */
static uint8_t jy901s_sensor_is_valid(const jy901s_t *sensor)
{
    if ((sensor == NULL) || (sensor->hi2c == NULL) ||
        (sensor->address_7bit == 0U) || (sensor->address_7bit > 0x7FU) ||
        (sensor->timeout_ms == 0U))
    {
        return 0U;
    }

    return 1U;
}

/** 读取一段连续的原始字节数据。 */
static jy901s_result_t jy901s_read_bytes(const jy901s_t *sensor,
                                         uint8_t start_register,
                                         uint8_t *buffer,
                                         uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((jy901s_sensor_is_valid(sensor) == 0U) ||
        (buffer == NULL) || (length == 0U))
    {
        return JY901S_INVALID_PARAM;
    }

    status = HAL_I2C_Mem_Read(sensor->hi2c,
                              jy901s_hal_address(sensor),
                              start_register,
                              I2C_MEMADD_SIZE_8BIT,
                              buffer,
                              length,
                              sensor->timeout_ms);

    return jy901s_from_hal_status(status);
}

/** 将 0x34-0x40 的连续寄存器转换为惯导数据。 */
static void jy901s_parse_motion(const uint8_t *buffer,
                                jy901s_data_t *data)
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++)
    {
        data->acceleration_raw[axis] = jy901s_read_i16_le(&buffer[axis * 2U]);
        data->angular_velocity_raw[axis] = jy901s_read_i16_le(&buffer[6U + axis * 2U]);
        data->magnetic_raw[axis] = jy901s_read_i16_le(&buffer[12U + axis * 2U]);
        data->angle_raw[axis] = jy901s_read_i16_le(&buffer[18U + axis * 2U]);

        data->acceleration_g[axis] =
            (float)data->acceleration_raw[axis] * JY901S_ACCELERATION_SCALE;
        data->acceleration_mps2[axis] =
            data->acceleration_g[axis] * JY901S_GRAVITY_MPS2;

        data->angular_velocity_dps[axis] =
            (float)data->angular_velocity_raw[axis] * JY901S_ANGULAR_VELOCITY_SCALE;
        data->angular_velocity_rads[axis] =
            data->angular_velocity_dps[axis] * JY901S_DEG_TO_RAD;

        data->angle_deg[axis] =
            (float)data->angle_raw[axis] * JY901S_ANGLE_SCALE;
    }

    data->temperature_raw = jy901s_read_i16_le(&buffer[24U]);
    data->temperature_c =
        (float)data->temperature_raw * JY901S_TEMPERATURE_SCALE;
}

/** 将 0x51-0x54 的连续寄存器转换为 w、x、y、z 四元数。 */
static void jy901s_parse_quaternion(const uint8_t *buffer,
                                    jy901s_data_t *data)
{
    uint8_t index;

    for (index = 0U; index < JY901S_QUATERNION_COUNT; index++)
    {
        data->quaternion_raw[index] = jy901s_read_i16_le(&buffer[index * 2U]);
        data->quaternion[index] =
            (float)data->quaternion_raw[index] * JY901S_QUATERNION_SCALE;
    }
}

jy901s_result_t jy901s_init(jy901s_t *sensor,
                            I2C_HandleTypeDef *hi2c,
                            uint8_t address_7bit)
{
    if ((sensor == NULL) || (hi2c == NULL) ||
        (address_7bit == 0U) || (address_7bit > 0x7FU))
    {
        return JY901S_INVALID_PARAM;
    }

    sensor->hi2c = hi2c;
    sensor->address_7bit = address_7bit;
    sensor->timeout_ms = JY901S_DEFAULT_TIMEOUT_MS;

    return jy901s_is_ready(sensor);
}

jy901s_result_t jy901s_is_ready(const jy901s_t *sensor)
{
    HAL_StatusTypeDef status;

    if (jy901s_sensor_is_valid(sensor) == 0U)
    {
        return JY901S_INVALID_PARAM;
    }

    status = HAL_I2C_IsDeviceReady(sensor->hi2c,
                                   jy901s_hal_address(sensor),
                                   3U,
                                   sensor->timeout_ms);
    if (status == HAL_ERROR)
    {
        return JY901S_NOT_READY;
    }

    return jy901s_from_hal_status(status);
}

jy901s_result_t jy901s_set_timeout(jy901s_t *sensor, uint32_t timeout_ms)
{
    if ((sensor == NULL) || (timeout_ms == 0U))
    {
        return JY901S_INVALID_PARAM;
    }

    sensor->timeout_ms = timeout_ms;
    return JY901S_OK;
}

jy901s_result_t jy901s_read_registers(const jy901s_t *sensor,
                                      uint8_t start_register,
                                      int16_t *values,
                                      uint8_t register_count)
{
    uint8_t buffer[32];
    uint8_t index;
    uint16_t byte_count;
    jy901s_result_t result;

    if ((values == NULL) || (register_count == 0U) ||
        (register_count > (sizeof(buffer) / 2U)))
    {
        return JY901S_INVALID_PARAM;
    }

    byte_count = (uint16_t)register_count * 2U;
    result = jy901s_read_bytes(sensor, start_register, buffer, byte_count);
    if (result != JY901S_OK)
    {
        return result;
    }

    for (index = 0U; index < register_count; index++)
    {
        values[index] = jy901s_read_i16_le(&buffer[index * 2U]);
    }

    return JY901S_OK;
}

jy901s_result_t jy901s_read_motion(const jy901s_t *sensor,
                                   jy901s_data_t *data)
{
    uint8_t buffer[JY901S_MOTION_BYTE_COUNT];
    jy901s_result_t result;

    if (data == NULL)
    {
        return JY901S_INVALID_PARAM;
    }

    result = jy901s_read_bytes(sensor,
                               JY901S_REG_ACCELERATION_X,
                               buffer,
                               sizeof(buffer));
    if (result != JY901S_OK)
    {
        return result;
    }

    jy901s_parse_motion(buffer, data);
    data->last_update_ms = HAL_GetTick();

    return JY901S_OK;
}

jy901s_result_t jy901s_read_quaternion(const jy901s_t *sensor,
                                       jy901s_data_t *data)
{
    uint8_t buffer[JY901S_QUATERNION_BYTE_COUNT];
    jy901s_result_t result;

    if (data == NULL)
    {
        return JY901S_INVALID_PARAM;
    }

    result = jy901s_read_bytes(sensor,
                               JY901S_REG_QUATERNION_0,
                               buffer,
                               sizeof(buffer));
    if (result != JY901S_OK)
    {
        return result;
    }

    jy901s_parse_quaternion(buffer, data);
    data->last_update_ms = HAL_GetTick();

    return JY901S_OK;
}

jy901s_result_t jy901s_read_all(const jy901s_t *sensor,
                                jy901s_data_t *data)
{
    uint8_t motion_buffer[JY901S_MOTION_BYTE_COUNT];
    uint8_t quaternion_buffer[JY901S_QUATERNION_BYTE_COUNT];
    jy901s_data_t new_data;
    jy901s_result_t result;

    if (data == NULL)
    {
        return JY901S_INVALID_PARAM;
    }

    result = jy901s_read_bytes(sensor,
                               JY901S_REG_ACCELERATION_X,
                               motion_buffer,
                               sizeof(motion_buffer));
    if (result != JY901S_OK)
    {
        return result;
    }

    result = jy901s_read_bytes(sensor,
                               JY901S_REG_QUATERNION_0,
                               quaternion_buffer,
                               sizeof(quaternion_buffer));
    if (result != JY901S_OK)
    {
        return result;
    }

    // 两次 I2C 事务都成功后再覆盖旧数据，避免留下半更新状态。
    memset(&new_data, 0, sizeof(new_data));
    jy901s_parse_motion(motion_buffer, &new_data);
    jy901s_parse_quaternion(quaternion_buffer, &new_data);
    new_data.last_update_ms = HAL_GetTick();
    memcpy(data, &new_data, sizeof(*data));

    return JY901S_OK;
}
