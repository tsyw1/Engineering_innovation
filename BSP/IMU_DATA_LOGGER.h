#ifndef ENGINEERING_INNOVATION_IMU_DATA_LOGGER_H
#define ENGINEERING_INNOVATION_IMU_DATA_LOGGER_H

#include "IMU_UART.h"

#include <stdint.h>

struct mecanum_inertial_nav;
typedef struct mecanum_inertial_nav mecanum_inertial_nav_t;

typedef struct
{
    jy901s_uart_t *sensor;
    UART_HandleTypeDef *output_uart;
    uint32_t period_ms;
    uint32_t next_output_ms;
    uint32_t last_angle_frame_count;
    uint32_t last_gyro_frame_count;
    uint32_t last_gyro_update_ms;
    float gyro_z_bias_dps;
    float compensated_yaw_deg;
    uint8_t compensated_yaw_initialized;
    mecanum_inertial_nav_t *navigation;
    uint8_t initialized;
} imu_data_logger_t;

#define IMU_DATA_LOGGER_DEFAULT_PERIOD_MS 10U

/** 初始化 IMU 接收、配置回传内容，并准备 VOFA FireWater 输出。 */
jy901s_uart_result_t imu_data_logger_init(imu_data_logger_t *logger,
                                          jy901s_uart_t *sensor,
                                          UART_HandleTypeDef *imu_uart,
                                          UART_HandleTypeDef *output_uart,
                                          uint32_t period_ms);

/** 使用默认 VOFA 输出周期初始化 IMU 日志模块。 */
jy901s_uart_result_t imu_data_logger_init_default(
    imu_data_logger_t *logger,
    jy901s_uart_t *sensor,
    UART_HandleTypeDef *imu_uart,
    UART_HandleTypeDef *output_uart);

/** 设置算法层计算得到的 Z 轴陀螺仪零偏。 */
void imu_data_logger_set_gyro_z_bias(imu_data_logger_t *logger,
                                     float gyro_z_bias_dps);

/** 设置可选的二维导航状态输出；传入 NULL 时恢复 IMU 原始输出。 */
void imu_data_logger_set_navigation(imu_data_logger_t *logger,
                                    mecanum_inertial_nav_t *navigation);

/** 在主循环中周期调用；每个新的角度帧输出一行 CSV 数据。 */
void imu_data_logger_task(imu_data_logger_t *logger);

/** 读取最近一次计算得到的零偏补偿后偏航角，单位为度。 */
float imu_data_logger_get_compensated_yaw_deg(
    const imu_data_logger_t *logger);

#endif // ENGINEERING_INNOVATION_IMU_DATA_LOGGER_H
