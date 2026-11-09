#ifndef ENGINEERING_INNOVATION_JY901S_ACCEL_VELOCITY_H
#define ENGINEERING_INNOVATION_JY901S_ACCEL_VELOCITY_H

#include "IMU_IIC.h"

#include <stdint.h>

#define JY901S_ACCEL_VELOCITY_DEFAULT_GRAVITY_MPS2       9.80665f
#define JY901S_ACCEL_VELOCITY_DEFAULT_CALIBRATION_SAMPLES 200U
#define JY901S_ACCEL_VELOCITY_DEFAULT_DEADBAND_MPS2     0.03f
#define JY901S_ACCEL_VELOCITY_DEFAULT_STATIONARY_ACCEL_MPS2 0.08f
#define JY901S_ACCEL_VELOCITY_DEFAULT_STATIONARY_GYRO_RAD_S 0.05f
#define JY901S_ACCEL_VELOCITY_DEFAULT_ACCEL_FILTER_ALPHA 0.35f

typedef enum
{
    JY901S_ACCEL_VELOCITY_OK = 0,
    JY901S_ACCEL_VELOCITY_INVALID_PARAM,
    JY901S_ACCEL_VELOCITY_NOT_READY,
    JY901S_ACCEL_VELOCITY_CALIBRATING,
    JY901S_ACCEL_VELOCITY_TIME_ERROR
} jy901s_accel_velocity_result_t;

typedef struct
{
    float gravity_mps2;
    uint32_t calibration_samples;
    float deadband_mps2;
    float stationary_accel_mps2;
    float stationary_gyro_rad_s;
    float acceleration_filter_alpha;

    /* 车体水平 X/Y = matrix * 传感器 X/Y/Z。默认值为同轴安装。 */
    float vehicle_from_sensor[2][3];
} jy901s_accel_velocity_config_t;

typedef struct
{
    jy901s_accel_velocity_config_t config;
    const jy901s_t *sensor;

    /* 启动静止标定得到的传感器三轴线性加速度零偏。 */
    float acceleration_bias_sensor_mps2[3];
    /* 根据当前 roll/pitch 计算出的传感器坐标系重力分量。 */
    float gravity_sensor_mps2[3];
    /* 去零偏、去重力后的传感器坐标系线性加速度。 */
    float acceleration_sensor_mps2[3];
    /* 坐标变换到车体 X(前)、Y(左)后的线性加速度。 */
    float acceleration_vehicle_mps2[2];
    float filtered_acceleration_vehicle_mps2[2];
    float velocity_vehicle_mps[2];

    uint32_t calibration_count;
    uint32_t last_update_ms;
    uint8_t calibrated;
    uint8_t acceleration_filter_initialized;
    uint8_t stationary;
    uint8_t valid;
} jy901s_accel_velocity_t;

/** 初始化速度解算器，并默认假设 JY901S X/Y 轴与车体 X/Y 轴同向。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_init(
    jy901s_accel_velocity_t *solver,
    const jy901s_t *sensor);

/** 修改速度解算参数；调用前应保持传感器静止。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_set_config(
    jy901s_accel_velocity_t *solver,
    const jy901s_accel_velocity_config_t *config);

/** 清除速度、零偏和时间状态，并重新进入静止标定。 */
void jy901s_accel_velocity_reset(jy901s_accel_velocity_t *solver);

/** 在外部确认车辆静止时执行零速校正，不重新进行零偏标定。 */
void jy901s_accel_velocity_zero_velocity_update(
    jy901s_accel_velocity_t *solver);

/** 使用一帧 JY901S 数据更新零偏、去重力结果和预测速度。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_update(
    jy901s_accel_velocity_t *solver,
    const jy901s_data_t *data,
    uint32_t now_ms);

/** 读取最近一次速度解算结果。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_get(
    const jy901s_accel_velocity_t *solver,
    float *vx_mps,
    float *vy_mps);

/** 读取最近一次坐标变换后的车体线性加速度。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_get_acceleration(
    const jy901s_accel_velocity_t *solver,
    float *ax_mps2,
    float *ay_mps2);

/** 读取用于积分和互补滤波的低通后车体线性加速度。 */
jy901s_accel_velocity_result_t jy901s_accel_velocity_get_filtered_acceleration(
    const jy901s_accel_velocity_t *solver,
    float *ax_mps2,
    float *ay_mps2);

/** 查询当前是否已经完成静止标定。 */
uint8_t jy901s_accel_velocity_is_calibrated(
    const jy901s_accel_velocity_t *solver);

#endif /* ENGINEERING_INNOVATION_JY901S_ACCEL_VELOCITY_H */
