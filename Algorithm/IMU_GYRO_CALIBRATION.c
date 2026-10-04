#include "IMU_GYRO_CALIBRATION.h"

#include <math.h>

#define IMU_GYRO_Z_BIAS_WEIGHT           1.03f
#define IMU_GYRO_CALIBRATION_TIMEOUT_MS  15000U
#define IMU_GYRO_ACCEL_NORM_TOLERANCE    0.05f
#define IMU_GYRO_STATIC_XY_LIMIT         1.5f
#define IMU_GYRO_STATIC_Z_MIN_DPS        0.40f
#define IMU_GYRO_STATIC_Z_MAX_DPS        0.58f

jy901s_uart_result_t imu_gyro_calibration_run(
    imu_gyro_calibration_t *calibration,
    const jy901s_uart_t *sensor,
    uint32_t settle_ms,
    uint32_t sample_count)
{
    jy901s_uart_t data;
    uint32_t start_ms;
    uint32_t settle_end_ms;
    uint32_t last_frame_count;
    uint32_t valid_samples;
    uint32_t now;
    float sum;
    float accel_norm;
    float gyro_z_dps;

    if ((calibration == NULL) || (sensor == NULL) ||
        (sensor->huart == NULL) || (sample_count == 0U))
    {
        return JY901S_UART_INVALID_PARAM;
    }

    calibration->gyro_z_bias_dps = 0.0f;
    calibration->gyro_z_bias_weight = IMU_GYRO_Z_BIAS_WEIGHT;
    calibration->calibrated = 0U;

    start_ms = HAL_GetTick();
    settle_end_ms = start_ms + settle_ms;
    last_frame_count = 0U;
    valid_samples = 0U;
    sum = 0.0f;

    /* 上电后先等待传感器和机械结构稳定，期间设备必须保持静止。 */
    while ((int32_t)(HAL_GetTick() - settle_end_ms) < 0)
    {
        HAL_Delay(1U);
    }

    while (valid_samples < sample_count)
    {
        now = HAL_GetTick();
        if ((uint32_t)(now - start_ms) >= IMU_GYRO_CALIBRATION_TIMEOUT_MS)
        {
            return JY901S_UART_TIMEOUT;
        }

        if (jy901s_uart_get_data(sensor, &data) != JY901S_UART_OK)
        {
            HAL_Delay(1U);
            continue;
        }

        if (data.angular_velocity_frame_count == last_frame_count)
        {
            HAL_Delay(1U);
            continue;
        }
        last_frame_count = data.angular_velocity_frame_count;

        accel_norm = sqrtf(data.acceleration_g[0] * data.acceleration_g[0] +
                           data.acceleration_g[1] * data.acceleration_g[1] +
                           data.acceleration_g[2] * data.acceleration_g[2]);
        gyro_z_dps = data.angular_velocity_dps[2];

        /* 只使用静止数据，防止把真实运动计入陀螺仪零偏。 */
        if ((fabsf(accel_norm - 1.0f) > IMU_GYRO_ACCEL_NORM_TOLERANCE) ||
            (fabsf(data.angular_velocity_dps[0]) > IMU_GYRO_STATIC_XY_LIMIT) ||
            (fabsf(data.angular_velocity_dps[1]) > IMU_GYRO_STATIC_XY_LIMIT) ||
            (gyro_z_dps < IMU_GYRO_STATIC_Z_MIN_DPS) ||
            (gyro_z_dps > IMU_GYRO_STATIC_Z_MAX_DPS))
        {
            /* 丢弃当前异常样本，保留之前已经采集的有效样本。 */
            continue;
        }

        sum += data.angular_velocity_dps[2];
        valid_samples++;
    }

    calibration->gyro_z_bias_dps =
        (sum / (float)valid_samples) * IMU_GYRO_Z_BIAS_WEIGHT;
    calibration->calibrated = 1U;

    return JY901S_UART_OK;
}

jy901s_uart_result_t imu_gyro_calibration_run_default(
    imu_gyro_calibration_t *calibration,
    const jy901s_uart_t *sensor)
{
    return imu_gyro_calibration_run(
        calibration,
        sensor,
        IMU_GYRO_CALIBRATION_DEFAULT_SETTLE_MS,
        IMU_GYRO_CALIBRATION_DEFAULT_SAMPLE_COUNT);
}

float imu_gyro_calibration_correct_z(
    const imu_gyro_calibration_t *calibration,
    float gyro_z_dps)
{
    if ((calibration == NULL) || (calibration->calibrated == 0U))
    {
        return gyro_z_dps;
    }

    return gyro_z_dps - calibration->gyro_z_bias_dps;
}

float imu_gyro_calibration_get_bias(
    const imu_gyro_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return 0.0f;
    }

    return calibration->gyro_z_bias_dps;
}
