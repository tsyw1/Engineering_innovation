#ifndef ENGINEERING_INNOVATION_HWT101CT_H
#define ENGINEERING_INNOVATION_HWT101CT_H

#include "main.h"

#include <stdint.h>

#define HWT101CT_UART_FRAME_SIZE          11U
#define HWT101CT_UART_FRAME_HEADER        0x55U
#define HWT101CT_UART_TYPE_ANGLE          0x53U

#define HWT101CT_ANGLE_SCALE_DEG          (180.0f / 32768.0f)

#define HWT101CT_UPDATE_ANGLE            (1UL << 0U)

#define HWT101CT_OUTPUT_ANGLE            0x0008U

#define HWT101CT_REGISTER_OUTPUT         0x02U
#define HWT101CT_REGISTER_RATE           0x03U
#define HWT101CT_REGISTER_UNLOCK         0x69U
#define HWT101CT_UNLOCK_VALUE            0xB588U
#define HWT101CT_COMMAND_SIZE             5U
#define HWT101CT_COMMAND_DELAY_MS         100U
#define HWT101CT_TX_TIMEOUT_MS            100U

typedef enum
{
    HWT101CT_OK = 0,
    HWT101CT_ERROR,
    HWT101CT_BUSY,
    HWT101CT_TIMEOUT,
    HWT101CT_INVALID_PARAM,
    HWT101CT_NOT_READY
} hwt101ct_result_t;

typedef struct
{
    float yaw_deg;
    uint8_t yaw_valid;
    uint32_t last_update_ms;
    uint32_t yaw_update_ms;
} hwt101ct_data_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    uint8_t frame[HWT101CT_UART_FRAME_SIZE];
    uint8_t frame_index;

    volatile uint8_t data_ready;
    volatile uint32_t rx_byte_count;
    volatile uint32_t frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t unknown_frame_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t last_uart_error;
    volatile uint32_t rx_restart_count;
    volatile uint32_t last_rx_byte_ms;
    volatile uint32_t pending_update_mask;

    uint32_t yaw_frame_count;
    float yaw_raw_deg;
    float yaw_zero_offset_deg;
    uint8_t yaw_zero_valid;
    hwt101ct_data_t data;
} hwt101ct_t;

/** 初始化 HWT101CT 对象，并启动 USART1 单字节中断接收。 */
hwt101ct_result_t hwt101ct_init(hwt101ct_t *sensor,
                                UART_HandleTypeDef *huart);

/** 重新启动单字节中断接收。 */
hwt101ct_result_t hwt101ct_start_receive(hwt101ct_t *sensor);

/** 中止可能卡住的接收并重新启动单字节中断接收。 */
hwt101ct_result_t hwt101ct_restart_receive(hwt101ct_t *sensor);

/** 航向帧超时前主动检查并恢复串口接收。 */
void hwt101ct_receive_watchdog(hwt101ct_t *sensor,
                               uint32_t now_ms,
                               uint32_t stale_ms);

/** 直接处理一个接收到的字节，便于测试或非 HAL 接收方式使用。 */
void hwt101ct_process_byte(hwt101ct_t *sensor, uint8_t byte);

/** 原子复制最近一次解析出的 HWT101CT 数据。 */
hwt101ct_result_t hwt101ct_get_data(const hwt101ct_t *sensor,
                                    hwt101ct_data_t *data);

/** 直接读取最近一次传感器内部解算出的 Z 轴偏航角，单位 degree。 */
hwt101ct_result_t hwt101ct_get_yaw_deg(const hwt101ct_t *sensor,
                                       float *yaw_deg);

/** 将当前朝向设为软件零度，后续 yaw_deg 统一输出相对航向。 */
hwt101ct_result_t hwt101ct_set_current_yaw_zero(hwt101ct_t *sensor);

/** 清除软件零点，恢复传感器原始偏航角输出。 */
hwt101ct_result_t hwt101ct_clear_yaw_zero(hwt101ct_t *sensor);

/** 原子取出尚未消费的数据更新类型。 */
hwt101ct_result_t hwt101ct_take_data(hwt101ct_t *sensor,
                                     hwt101ct_data_t *data,
                                     uint32_t *updated_mask);

/** 清除“已有新数据”标志。 */
void hwt101ct_clear_data(hwt101ct_t *sensor);

/** 查询是否已经收到至少一帧有效数据。 */
uint8_t hwt101ct_is_data_ready(const hwt101ct_t *sensor);

/** 写入 WIT 标准协议寄存器，命令格式为 FF AA addr value_low value_high。 */
hwt101ct_result_t hwt101ct_write_register(hwt101ct_t *sensor,
                                          uint8_t address,
                                          uint16_t value);

/** 临时设置为仅输出角度帧，并设置输出速率，不保存到传感器 Flash。 */
hwt101ct_result_t hwt101ct_configure_angle_output(hwt101ct_t *sensor,
                                                  uint16_t output_rate);

/** 在统一 HAL_UART_RxCpltCallback 中分发 USART1 接收事件。 */
void hwt101ct_on_rx_complete(UART_HandleTypeDef *huart);

/** 在统一 HAL_UART_ErrorCallback 中分发 USART1 错误事件。 */
void hwt101ct_on_error(UART_HandleTypeDef *huart);

#endif /* ENGINEERING_INNOVATION_HWT101CT_H */
