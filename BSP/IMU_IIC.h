#ifndef ENGINEERING_INNOVATION_IMU_IIC_H
#define ENGINEERING_INNOVATION_IMU_IIC_H

#include "main.h"

#include <stdint.h>

// JY901S 使用 7 位 I2C 地址，传给 HAL 时由驱动内部左移一位。
#define JY901S_DEFAULT_I2C_ADDRESS      0x50U
#define JY901S_DEFAULT_TIMEOUT_MS       20U

#define JY901S_GRAVITY_MPS2             9.80665f
#define JY901S_DEG_TO_RAD               0.01745329251994329577f

typedef enum
{
    JY901S_OK = 0,
    JY901S_ERROR,
    JY901S_BUSY,
    JY901S_TIMEOUT,
    JY901S_INVALID_PARAM,
    JY901S_NOT_READY
} jy901s_result_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;
    uint32_t timeout_ms;
} jy901s_t;

typedef struct
{
    int16_t acceleration_raw[3];
    int16_t angular_velocity_raw[3];
    int16_t magnetic_raw[3];
    int16_t angle_raw[3];
    int16_t quaternion_raw[4];
    int16_t temperature_raw;

    float acceleration_g[3];
    float acceleration_mps2[3];
    float angular_velocity_dps[3];
    float angular_velocity_rads[3];
    float angle_deg[3];
    float quaternion[4];
    float temperature_c;

    uint32_t last_update_ms;
} jy901s_data_t;

/**
 * 初始化 JY901S 对象并检测设备是否应答。
 * address_7bit 一般传 JY901S_DEFAULT_I2C_ADDRESS。
 */
jy901s_result_t jy901s_init(jy901s_t *sensor,
                            I2C_HandleTypeDef *hi2c,
                            uint8_t address_7bit);

/** 检查传感器是否在 I2C 总线上应答。 */
jy901s_result_t jy901s_is_ready(const jy901s_t *sensor);

/** 修改单次 HAL I2C 操作的超时时间。 */
jy901s_result_t jy901s_set_timeout(jy901s_t *sensor, uint32_t timeout_ms);

/**
 * 从指定寄存器开始连续读取多个 16 位寄存器。
 * JY901S 每个寄存器低字节在前，函数会转换为有符号 int16_t。
 */
jy901s_result_t jy901s_read_registers(const jy901s_t *sensor,
                                      uint8_t start_register,
                                      int16_t *values,
                                      uint8_t register_count);

/**
 * 一次连续读取加速度、角速度、磁场、欧拉角和温度。
 * 该函数只进行一次 26 字节 I2C 事务，适合固定周期采样。
 */
jy901s_result_t jy901s_read_motion(const jy901s_t *sensor,
                                   jy901s_data_t *data);

/** 单独读取 q0、q1、q2、q3 四元数。 */
jy901s_result_t jy901s_read_quaternion(const jy901s_t *sensor,
                                       jy901s_data_t *data);

/** 读取惯导核心数据和四元数，全部成功后才更新 data。 */
jy901s_result_t jy901s_read_all(const jy901s_t *sensor,
                                jy901s_data_t *data);

#endif // ENGINEERING_INNOVATION_IMU_IIC_H
