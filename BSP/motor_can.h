//
// Y42_V2.0 / X42S CAN + Emm5.0 电机驱动。
//
// 本文件使用 STM32F4 的原生 bxCAN 外设，不使用 USART TTL 模拟 CAN。
// CAN 帧采用电机手册规定的 29 位扩展帧：
//     扩展帧 ID = (电机地址 << 8) | 分包序号
//     数据区   = 功能码 + 参数 + 0x6B
//

#ifndef ENGINEERING_INNOVATION_MOTOR_CAN_H
#define ENGINEERING_INNOVATION_MOTOR_CAN_H

#include "can.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_MAX_COUNT                 6U
#define MOTOR_CAN_FRAME_DATA_SIZE       8U
#define MOTOR_CAN_COMMAND_MAX_SIZE      64U
#define MOTOR_EMM_MAX_SPEED_RPM         5000U
#define MOTOR_FIXED_CHECKSUM            0x6BU
#define MOTOR_BROADCAST_ADDRESS         0U
/* Emm 固件 0x36 实时位置反馈的单位；不同于位置命令的 3200 脉冲/圈。 */
#define MOTOR_EMM_FEEDBACK_COUNTS_PER_REV 65536UL

#define MOTOR_STATE_ENABLED             (1U << 0)
#define MOTOR_STATE_POSITION_REACHED    (1U << 1)
#define MOTOR_STATE_STALL               (1U << 2)
#define MOTOR_STATE_STALL_PROTECTED     (1U << 3)
#define MOTOR_STATE_LEFT_LIMIT          (1U << 4)
#define MOTOR_STATE_RIGHT_LIMIT         (1U << 5)
#define MOTOR_STATE_POWER_LOSS          (1U << 7)

#define MOTOR_HOME_ENCODER_READY        (1U << 0)
#define MOTOR_HOME_CALIBRATION_READY    (1U << 1)
#define MOTOR_HOME_RUNNING              (1U << 2)
#define MOTOR_HOME_FAILED               (1U << 3)
#define MOTOR_HOME_OVER_TEMPERATURE     (1U << 4)
#define MOTOR_HOME_OVER_CURRENT         (1U << 5)

/* 回零/校准模式由电机参数配置决定，0 表示使用默认模式。 */
#define MOTOR_HOME_MODE_DEFAULT         0U

typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERROR,
    MOTOR_BUSY,
    MOTOR_INVALID_PARAM,
    MOTOR_NOT_FOUND,
    MOTOR_NOT_READY
} motor_result_t;

typedef enum
{
    MOTOR_DIRECTION_CW = 0,
    MOTOR_DIRECTION_CCW = 1
} motor_direction_t;

typedef enum
{
    MOTOR_EXECUTE_NOW = 0,
    MOTOR_EXECUTE_SYNC = 1
} motor_sync_t;

typedef enum
{
    MOTOR_POSITION_RELATIVE_LAST = 0,
    MOTOR_POSITION_ABSOLUTE = 1,
    MOTOR_POSITION_RELATIVE_CURRENT = 2
} motor_position_mode_t;

typedef enum
{
    MOTOR_REPLY_NONE = 0,
    MOTOR_REPLY_OK = 0x02,
    MOTOR_REPLY_LOW_VOLTAGE = 0x12,
    MOTOR_REPLY_PROTECTED = 0x22,
    MOTOR_REPLY_REACHED = 0x9F,
    MOTOR_REPLY_PARAM_ERROR = 0xE2,
    MOTOR_REPLY_FORMAT_ERROR = 0xEE
} motor_reply_t;

typedef enum
{
    MOTOR_INFO_BUS_VOLTAGE = 0x24,
    MOTOR_INFO_BUS_CURRENT = 0x26,
    MOTOR_INFO_PHASE_CURRENT = 0x27,
    MOTOR_INFO_SPEED = 0x35,
    MOTOR_INFO_POSITION = 0x36,
    MOTOR_INFO_POSITION_ERROR = 0x37,
    MOTOR_INFO_BATTERY_VOLTAGE = 0x38,
    MOTOR_INFO_TEMPERATURE = 0x39,
    MOTOR_INFO_STATE = 0x3A,
    MOTOR_INFO_HOME_STATE = 0x3B,
    MOTOR_INFO_ALL_STATE = 0x3C
} motor_information_t;

struct motor_bus;
typedef struct motor_bus motor_bus_t;

/** 原生 CAN 帧发送回调。extended_id 为 29 位扩展帧 ID。 */
typedef HAL_StatusTypeDef (*motor_can_send_callback_t)(
    CAN_HandleTypeDef *hcan,
    uint32_t extended_id,
    const uint8_t *data,
    uint8_t length,
    uint32_t timeout_ms);

/** 使用 STM32 HAL_CAN_AddTxMessage() 发送一帧扩展 CAN 数据。 */
HAL_StatusTypeDef motor_can_hal_send_frame(CAN_HandleTypeDef *hcan,
                                           uint32_t extended_id,
                                           const uint8_t *data,
                                           uint8_t length,
                                           uint32_t timeout_ms);

typedef struct
{
    motor_bus_t *bus;
    uint8_t id;

    volatile int16_t target_speed_rpm;
    volatile int16_t actual_speed_rpm;
    /* 0x31 返回的单圈线性化编码器值，以及相邻采样的变化量。 */
    volatile uint16_t encoder_raw;
    volatile int16_t encoder_delta;
    volatile uint8_t encoder_valid;
    /* 保留旧字段名；其数值是 0x36 原始反馈计数，Emm 固件为 65536 计数/圈。 */
    volatile int64_t actual_position_pulse;
    volatile uint8_t position_valid;
    volatile uint32_t position_update_count;
    /* 清零后采集的 0x36 原始值，用于兼容不同固件的实时位置零点语义。 */
    int64_t position_feedback_zero_count;
    uint8_t position_feedback_zero_valid;
    volatile int64_t position_error_pulse;
    volatile uint16_t bus_voltage_mv;
    volatile uint16_t bus_current_ma;
    volatile uint16_t phase_current_ma;
    volatile uint16_t battery_voltage_mv;
    volatile int8_t temperature_c;
    volatile uint8_t state_flags;
    volatile uint8_t home_flags;
    volatile uint8_t last_function;
    volatile motor_reply_t last_reply;
    volatile uint32_t last_update_ms;
    volatile uint32_t speed_update_ms;
    volatile uint32_t encoder_update_ms;
    volatile uint32_t encoder_update_count;
    volatile uint32_t position_update_ms;
} motor_t;

struct motor_bus
{
    CAN_HandleTypeDef *hcan;
    motor_can_send_callback_t send_can_frame;
    motor_t *motors[MOTOR_MAX_COUNT];
    uint32_t tx_timeout_ms;
    uint8_t motor_count;
    volatile uint8_t tx_busy;

    /* 广播读取地址的结果；一次只接一台电机时使用。 */
    volatile uint8_t discovered_id;
    volatile uint8_t discovered_id_valid;
    volatile uint32_t rx_frame_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t last_can_error;
};

/** 初始化 CAN 电机总线对象，不会启动 CAN 外设。 */
motor_result_t motor_bus_init(motor_bus_t *bus,
                              CAN_HandleTypeDef *hcan,
                              motor_can_send_callback_t send_can_frame,
                              uint32_t tx_timeout_ms);

/** 配置全接收过滤器、启动 CAN 并开启 FIFO0 接收中断。 */
motor_result_t motor_bus_start(motor_bus_t *bus);

/** 停止 CAN 外设。 */
motor_result_t motor_bus_stop(motor_bus_t *bus);

/** 发送 00 15 6B 的 CAN 广播读地址命令。 */
motor_result_t motor_broadcast_read_id(motor_bus_t *bus);

/** 读取最近一次广播读地址结果。 */
bool motor_bus_get_discovered_id(const motor_bus_t *bus, uint8_t *id);

/**
 * 注册一台已知地址的电机。
 *
 * 调用前必须先执行 motor_bus_start()；注册时会自动完成一次
 * “使能-失能-再使能”的上电初始化流程。
 */
motor_result_t motor_init(motor_t *motor, motor_bus_t *bus, uint8_t id);

motor_result_t motor_enable(motor_t *motor, bool enable, motor_sync_t sync);
motor_result_t motor_set_speed(motor_t *motor,
                               int16_t speed_rpm,
                               uint8_t acceleration,
                               motor_sync_t sync);
motor_result_t motor_set_speeds_sync(motor_t *const motors[],
                                     const int16_t speed_rpm[],
                                     uint8_t motor_count,
                                     uint8_t acceleration);
motor_result_t motor_set_position(motor_t *motor,
                                  int64_t pulse,
                                  uint16_t speed_rpm,
                                  uint8_t acceleration,
                                  motor_position_mode_t mode,
                                  motor_sync_t sync);
motor_result_t motor_stop(motor_t *motor, motor_sync_t sync);
motor_result_t motor_stop_all(motor_bus_t *bus);
motor_result_t motor_trigger_sync(motor_bus_t *bus);

motor_result_t motor_clear_position(motor_t *motor);
motor_result_t motor_clear_protection(motor_t *motor);
/** 修改电机 CAN 地址；new_address 范围为 1～255，save_to_flash 为 true 时掉电保存。 */
motor_result_t motor_set_address(motor_t *motor,
                                  uint8_t new_address,
                                  bool save_to_flash);
/** 启动电机回零/校准，home_mode 的有效范围为 0～5。 */
motor_result_t motor_home(motor_t *motor,
                          uint8_t home_mode,
                          motor_sync_t sync);
/** 中止电机正在执行的回零/校准。 */
motor_result_t motor_home_abort(motor_t *motor);
motor_result_t motor_set_heartbeat(motor_t *motor,
                                   uint32_t heartbeat_ms,
                                   bool save_to_flash);
motor_result_t motor_set_periodic_report(motor_t *motor,
                                          motor_information_t information,
                                          uint16_t period_ms);

motor_result_t motor_read_speed(motor_t *motor);
motor_result_t motor_read_encoder(motor_t *motor);
motor_result_t motor_read_position(motor_t *motor);
motor_result_t motor_read_position_error(motor_t *motor);
motor_result_t motor_read_state(motor_t *motor);
motor_result_t motor_read_home_state(motor_t *motor);
motor_result_t motor_read_bus_voltage(motor_t *motor);
motor_result_t motor_read_bus_current(motor_t *motor);
motor_result_t motor_read_phase_current(motor_t *motor);
motor_result_t motor_read_temperature(motor_t *motor);
motor_result_t motor_read_battery_voltage(motor_t *motor);

/** 由 CAN FIFO0 回调调用，解析一帧电机返回数据。 */
motor_result_t motor_bus_process_can_frame(motor_bus_t *bus,
                                           uint32_t extended_id,
                                           const uint8_t *data,
                                           uint8_t length);

/** HAL CAN FIFO0 中断回调，由本驱动统一接收并解析。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#endif /* ENGINEERING_INNOVATION_MOTOR_CAN_H */
