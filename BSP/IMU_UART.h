#ifndef ENGINEERING_INNOVATION_IMU_UART_H
#define ENGINEERING_INNOVATION_IMU_UART_H

#include "main.h"

#include <stdint.h>

#define JY901S_UART_DEFAULT_BAUDRATE    115200U
#define JY901S_UART_FRAME_SIZE          11U
#define JY901S_UART_GRAVITY_MPS2        9.80665f
#define JY901S_UART_DEG_TO_RAD          0.01745329251994329577f

#define JY901S_UART_UPDATE_ACCELERATION (1UL << 0U)
#define JY901S_UART_UPDATE_GYRO         (1UL << 1U)
#define JY901S_UART_UPDATE_ANGLE        (1UL << 2U)
#define JY901S_UART_UPDATE_MAGNETIC     (1UL << 3U)
#define JY901S_UART_UPDATE_QUATERNION   (1UL << 4U)

#define JY901S_UART_OUTPUT_ACCELERATION 0x0002U
#define JY901S_UART_OUTPUT_GYRO         0x0004U
#define JY901S_UART_OUTPUT_ANGLE        0x0008U
#define JY901S_UART_OUTPUT_MAGNETIC     0x0010U
#define JY901S_UART_OUTPUT_GYRO_ACCEL   (JY901S_UART_OUTPUT_ACCELERATION | \
                                         JY901S_UART_OUTPUT_GYRO)
#define JY901S_UART_RATE_100HZ          0x0009U

typedef enum
{
    JY901S_UART_OK = 0,
    JY901S_UART_ERROR,
    JY901S_UART_BUSY,
    JY901S_UART_TIMEOUT,
    JY901S_UART_INVALID_PARAM,
    JY901S_UART_NOT_READY
} jy901s_uart_result_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t frame[JY901S_UART_FRAME_SIZE];
    uint8_t frame_index;
    uint8_t checksum;
    uint8_t last_frame_type;

    volatile uint8_t data_ready;
    volatile uint32_t rx_byte_count;
    volatile uint32_t frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t last_uart_error;
    volatile uint32_t pending_update_mask;
    volatile uint32_t overwritten_update_count;

    uint32_t acceleration_frame_count;
    uint32_t angular_velocity_frame_count;
    uint32_t angle_frame_count;
    uint32_t magnetic_frame_count;
    uint32_t quaternion_frame_count;

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
    uint32_t acceleration_update_ms;
    uint32_t angular_velocity_update_ms;
    uint32_t angle_update_ms;
    uint32_t magnetic_update_ms;
    uint32_t quaternion_update_ms;
} jy901s_uart_t;

/** 初始化 USART1 接收，并启动单字节中断。 */
jy901s_uart_result_t jy901s_uart_init(jy901s_uart_t *sensor,
                                      UART_HandleTypeDef *huart);

/** 重新启动单字节中断接收。 */
jy901s_uart_result_t jy901s_uart_start_receive(jy901s_uart_t *sensor);

/** 临时设置回传内容和回传速率，不写入传感器 Flash。 */
jy901s_uart_result_t jy901s_uart_configure_output(jy901s_uart_t *sensor,
                                                  uint16_t output_mask,
                                                  uint16_t output_rate);

/** 原子复制最近一次已经解析出的数据，不清除新数据标志。 */
jy901s_uart_result_t jy901s_uart_get_data(const jy901s_uart_t *sensor,
                                          jy901s_uart_t *data);

/** 原子取出所有尚未消费的更新类型，并清除对应的新数据标志。 */
jy901s_uart_result_t jy901s_uart_take_data(jy901s_uart_t *sensor,
                                           jy901s_uart_t *data,
                                           uint32_t *updated_mask);

/** 清除“已有新数据”标志。 */
void jy901s_uart_clear_data(jy901s_uart_t *sensor);

/** 查询是否已经收到至少一帧有效数据。 */
uint8_t jy901s_uart_is_data_ready(const jy901s_uart_t *sensor);

/** 供统一 HAL 串口回调输入一个字节。 */
void jy901s_uart_process_byte(jy901s_uart_t *sensor, uint8_t byte);

/** 由 HAL_UART_RxCpltCallback 分发 USART1 接收完成事件。 */
void jy901s_uart_on_rx_complete(UART_HandleTypeDef *huart);

/** 由 HAL_UART_ErrorCallback 分发 USART1 错误事件。 */
void jy901s_uart_on_error(UART_HandleTypeDef *huart);

#endif // ENGINEERING_INNOVATION_IMU_UART_H
