#include "IMU_DATA_LOGGER.h"

#include "MECANUM_INERTIAL_NAV.h"

#include <stdio.h>

/* JY901S 回传寄存器：加速度、角速度、欧拉角。 */
#define IMU_LOGGER_OUTPUT_MASK       0x000EU
#define IMU_LOGGER_OUTPUT_RATE       JY901S_UART_RATE_100HZ
#define IMU_LOGGER_TX_TIMEOUT_MS     20U
#define IMU_LOGGER_LINE_BUFFER_SIZE  128U

jy901s_uart_result_t imu_data_logger_init(imu_data_logger_t *logger,
                                          jy901s_uart_t *sensor,
                                          UART_HandleTypeDef *imu_uart,
                                          UART_HandleTypeDef *output_uart,
                                          uint32_t period_ms)
{
    jy901s_uart_result_t result;

    if ((logger == NULL) || (sensor == NULL) || (imu_uart == NULL) ||
        (output_uart == NULL) || (period_ms == 0U))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    logger->sensor = sensor;
    logger->output_uart = output_uart;
    logger->period_ms = period_ms;
    logger->next_output_ms = HAL_GetTick();
    logger->last_angle_frame_count = 0U;
    logger->last_gyro_frame_count = 0U;
    logger->last_gyro_update_ms = 0U;
    logger->gyro_z_bias_dps = 0.0f;
    logger->compensated_yaw_deg = 0.0f;
    logger->compensated_yaw_initialized = 0U;
    logger->navigation = NULL;
    logger->initialized = 0U;

    result = jy901s_uart_init(sensor, imu_uart);
    if (result != JY901S_UART_OK)
    {
        return result;
    }

    result = jy901s_uart_configure_output(sensor,
                                          IMU_LOGGER_OUTPUT_MASK,
                                          IMU_LOGGER_OUTPUT_RATE);
    if (result != JY901S_UART_OK)
    {
        return result;
    }

    logger->initialized = 1U;
    return JY901S_UART_OK;
}

jy901s_uart_result_t imu_data_logger_init_default(
    imu_data_logger_t *logger,
    jy901s_uart_t *sensor,
    UART_HandleTypeDef *imu_uart,
    UART_HandleTypeDef *output_uart)
{
    return imu_data_logger_init(logger,
                                sensor,
                                imu_uart,
                                output_uart,
                                IMU_DATA_LOGGER_DEFAULT_PERIOD_MS);
}

void imu_data_logger_set_gyro_z_bias(imu_data_logger_t *logger,
                                     float gyro_z_bias_dps)
{
    if (logger == NULL)
    {
        return;
    }

    logger->gyro_z_bias_dps = gyro_z_bias_dps;
}

void imu_data_logger_set_navigation(imu_data_logger_t *logger,
                                    mecanum_inertial_nav_t *navigation)
{
    if (logger != NULL)
    {
        logger->navigation = navigation;
    }
}

static void imu_data_logger_update_compensated_yaw(
    imu_data_logger_t *logger,
    const jy901s_uart_t *data)
{
    uint32_t dt_ms;
    float corrected_gyro_z_dps;

    if ((logger == NULL) || (data == NULL))
    {
        return;
    }

    if (data->angular_velocity_frame_count == logger->last_gyro_frame_count)
    {
        return;
    }

    logger->last_gyro_frame_count = data->angular_velocity_frame_count;

    /* 以校准完成后第一次有效 IMU 偏航角作为初始航向。 */
    if (logger->compensated_yaw_initialized == 0U)
    {
        logger->compensated_yaw_deg = data->angle_deg[2];
        logger->last_gyro_update_ms = data->angular_velocity_update_ms;
        logger->compensated_yaw_initialized = 1U;
        return;
    }

    dt_ms = data->angular_velocity_update_ms - logger->last_gyro_update_ms;
    logger->last_gyro_update_ms = data->angular_velocity_update_ms;

    /* 时间戳异常时只更新时间基准，避免一次错误积分造成跳变。 */
    if ((dt_ms == 0U) || (dt_ms > 1000U))
    {
        return;
    }

    corrected_gyro_z_dps = data->angular_velocity_dps[2] -
                           logger->gyro_z_bias_dps;
    logger->compensated_yaw_deg +=
        corrected_gyro_z_dps * ((float)dt_ms * 0.001f);
}

void imu_data_logger_task(imu_data_logger_t *logger)
{
    jy901s_uart_t data;
    uint32_t now;
    int length;
    char line[IMU_LOGGER_LINE_BUFFER_SIZE];

    if ((logger == NULL) || (logger->initialized == 0U) ||
        (logger->sensor == NULL) || (logger->output_uart == NULL))
    {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - logger->next_output_ms) < 0)
    {
        return;
    }
    logger->next_output_ms = now + logger->period_ms;

    if (jy901s_uart_get_data(logger->sensor, &data) != JY901S_UART_OK)
    {
        return;
    }

    imu_data_logger_update_compensated_yaw(logger, &data);

    /* 只在新角度帧到达后发送，避免同一组 IMU 数据重复输出。 */
    if (data.angle_frame_count == logger->last_angle_frame_count)
    {
        return;
    }
    logger->last_angle_frame_count = data.angle_frame_count;

    if ((logger->navigation != NULL) &&
        (logger->navigation->initialized != 0U))
    {
        /* FireWater：时间、位置、相对航向角、速度、角速度和在线零偏。 */
        length = snprintf(line,
                          sizeof(line),
                          "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                          (unsigned long)now,
                          (double)logger->navigation->x_m,
                          (double)logger->navigation->y_m,
                          (double)(logger->navigation->yaw_rad *
                                   57.29577951308232f),
                          (double)logger->navigation->vx_mps,
                          (double)logger->navigation->vy_mps,
                          (double)logger->navigation->fused_wz_rad_s,
                          (double)logger->navigation->gyro_z_bias_dps);
    }
    else
    {
        /* 兼容导航尚未初始化时的 IMU 校准测试输出。 */
        length = snprintf(line,
                          sizeof(line),
                          "%lu,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                          (unsigned long)now,
                          (double)(data.angular_velocity_dps[2] -
                                   logger->gyro_z_bias_dps),
                          (double)data.acceleration_g[0],
                          (double)data.acceleration_g[1],
                          (double)data.acceleration_g[2],
                          (double)logger->compensated_yaw_deg);
    }
    if ((length <= 0) || ((size_t)length >= sizeof(line)))
    {
        return;
    }

    (void)HAL_UART_Transmit(logger->output_uart,
                            (uint8_t *)line,
                            (uint16_t)length,
                            IMU_LOGGER_TX_TIMEOUT_MS);
}

float imu_data_logger_get_compensated_yaw_deg(
    const imu_data_logger_t *logger)
{
    if ((logger == NULL) || (logger->initialized == 0U) ||
        (logger->compensated_yaw_initialized == 0U))
    {
        return 0.0f;
    }

    return logger->compensated_yaw_deg;
}
