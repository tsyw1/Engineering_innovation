#ifndef ENGINEERING_INNOVATION_IMU_GYRO_CALIBRATION_H
#define ENGINEERING_INNOVATION_IMU_GYRO_CALIBRATION_H

#include "IMU_UART.h"

#include <stdint.h>

typedef struct
{
    float gyro_z_bias_dps;
    float gyro_z_bias_weight;
    uint8_t calibrated;
} imu_gyro_calibration_t;

#define IMU_GYRO_CALIBRATION_DEFAULT_SETTLE_MS  2000U
#define IMU_GYRO_CALIBRATION_DEFAULT_SAMPLE_COUNT 50U

/**
 * 上电静止校准 Z 轴陀螺仪零偏。
 * 最终零偏 = 静止采样平均值 × gyro_z_bias_weight。
 */
jy901s_uart_result_t imu_gyro_calibration_run(
    imu_gyro_calibration_t *calibration,
    const jy901s_uart_t *sensor,
    uint32_t settle_ms,
    uint32_t sample_count);

/** 使用当前工程默认等待时间和采样数量执行上电静止校准。 */
jy901s_uart_result_t imu_gyro_calibration_run_default(
    imu_gyro_calibration_t *calibration,
    const jy901s_uart_t *sensor);

/** 使用已校准的零偏补偿 Z 轴角速度。 */
float imu_gyro_calibration_correct_z(
    const imu_gyro_calibration_t *calibration,
    float gyro_z_dps);

/** 查询当前校准得到的零偏。 */
float imu_gyro_calibration_get_bias(
    const imu_gyro_calibration_t *calibration);

#endif // ENGINEERING_INNOVATION_IMU_GYRO_CALIBRATION_H
