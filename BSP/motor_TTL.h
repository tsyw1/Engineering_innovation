//
// X42_V1.3 + Emm5.0 TTL 电机驱动。
//
// TTL 协议帧格式为：
//     电机地址 + 功能码 + 参数 + 0x6B
// 本模块使用一条 USART2 总线管理最多六台电机，不依赖 motor_can.c/.h。
//

#ifndef ENGINEERING_INNOVATION_MOTOR_TTL_H
#define ENGINEERING_INNOVATION_MOTOR_TTL_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_TTL_MAX_COUNT             6U
#define MOTOR_TTL_MAX_COMMAND_LENGTH    64U
#define MOTOR_TTL_MAX_RX_LENGTH         16U
#define MOTOR_TTL_FIXED_CHECKSUM        0x6BU
#define MOTOR_TTL_MAX_SPEED_RPM         3000U
/* 手册要求多机连续命令之间间隔几毫秒，避免 TTL 模块发生粘包。 */
#define MOTOR_TTL_INTER_COMMAND_DELAY_MS 3U

/* Emm 方向字节定义。 */
#define MOTOR_TTL_DIRECTION_CW          0U
#define MOTOR_TTL_DIRECTION_CCW         1U

/* 电机状态字和回零状态字的常用位定义。 */
#define MOTOR_TTL_STATE_ENABLED         (1U << 0)
#define MOTOR_TTL_STATE_POSITION_REACHED (1U << 1)
#define MOTOR_TTL_STATE_STALL           (1U << 2)
#define MOTOR_TTL_STATE_STALL_PROTECTED (1U << 3)
#define MOTOR_TTL_STATE_LEFT_LIMIT      (1U << 4)
#define MOTOR_TTL_STATE_RIGHT_LIMIT     (1U << 5)
#define MOTOR_TTL_STATE_POWER_LOSS      (1U << 7)

#define MOTOR_TTL_HOME_ENCODER_READY    (1U << 0)
#define MOTOR_TTL_HOME_CALIBRATION_READY (1U << 1)
#define MOTOR_TTL_HOME_RUNNING          (1U << 2)
#define MOTOR_TTL_HOME_FAILED           (1U << 3)
#define MOTOR_TTL_HOME_OVER_TEMPERATURE (1U << 4)
#define MOTOR_TTL_HOME_OVER_CURRENT     (1U << 5)

typedef enum
{
    MOTOR_TTL_OK = 0,
    MOTOR_TTL_ERROR,
    MOTOR_TTL_BUSY,
    MOTOR_TTL_INVALID_PARAM,
    MOTOR_TTL_NOT_FOUND
} motor_ttl_result_t;

typedef enum
{
    MOTOR_TTL_EXECUTE_NOW = 0,
    MOTOR_TTL_EXECUTE_SYNC = 1
} motor_ttl_sync_t;

typedef enum
{
    MOTOR_TTL_POSITION_RELATIVE_LAST = 0,
    MOTOR_TTL_POSITION_ABSOLUTE = 1,
    MOTOR_TTL_POSITION_RELATIVE_CURRENT = 2
} motor_ttl_position_mode_t;

typedef enum
{
    MOTOR_TTL_REPLY_NONE = 0,
    MOTOR_TTL_REPLY_OK = 0x02,
    MOTOR_TTL_REPLY_LOW_VOLTAGE = 0x12,
    MOTOR_TTL_REPLY_PROTECTED = 0x22,
    MOTOR_TTL_REPLY_REACHED = 0x9F,
    MOTOR_TTL_REPLY_PARAM_ERROR = 0xE2,
    MOTOR_TTL_REPLY_FORMAT_ERROR = 0xEE
} motor_ttl_reply_t;

/* 这些功能码既可用于一次读取，也可用于配置周期回传。 */
typedef enum
{
    MOTOR_TTL_INFO_BUS_VOLTAGE = 0x24,
    MOTOR_TTL_INFO_BUS_CURRENT = 0x26,
    MOTOR_TTL_INFO_PHASE_CURRENT = 0x27,
    MOTOR_TTL_INFO_INPUT_PULSES = 0x32,
    MOTOR_TTL_INFO_SPEED = 0x35,
    MOTOR_TTL_INFO_POSITION = 0x36,
    MOTOR_TTL_INFO_POSITION_ERROR = 0x37,
    MOTOR_TTL_INFO_TEMPERATURE = 0x39,
    MOTOR_TTL_INFO_STATE = 0x3A,
    MOTOR_TTL_INFO_HOME_STATE = 0x3B,
    MOTOR_TTL_INFO_ALL_STATE = 0x3C
} motor_ttl_information_t;

struct motor_ttl_bus;
typedef struct motor_ttl_bus motor_ttl_bus_t;

typedef struct
{
    motor_ttl_bus_t *bus;
    uint8_t id;

    volatile int16_t target_speed_rpm;
    volatile int16_t actual_speed_rpm;

    /* 32 功能码返回的输入脉冲可直接用于麦轮底盘里程计。 */
    volatile int32_t actual_input_pulses;
    volatile uint8_t actual_input_pulses_valid;

    /* 36/37 的原始位置和误差，同时换算成 0.01 度供机械臂状态监控使用。 */
    volatile int32_t actual_position_raw;
    volatile int32_t actual_position_angle_cdeg;
    volatile uint8_t actual_position_valid;
    volatile int32_t position_error_raw;
    volatile int32_t position_error_angle_cdeg;

    volatile uint16_t bus_voltage_mv;
    volatile uint16_t bus_current_ma;
    volatile uint16_t phase_current_ma;
    volatile int8_t temperature_c;
    volatile uint16_t encoder_raw;
    volatile uint8_t state_flags;
    volatile uint8_t home_flags;

    volatile uint8_t last_function;
    volatile motor_ttl_reply_t last_reply;
    volatile uint32_t last_update_ms;
} motor_ttl_motor_t;

struct motor_ttl_bus
{
    UART_HandleTypeDef *huart;
    motor_ttl_motor_t *motors[MOTOR_TTL_MAX_COUNT];
    uint32_t tx_timeout_ms;
    uint8_t motor_count;
    volatile uint8_t tx_busy;

    /* USART2 按字节接收时使用的解析状态。 */
    uint8_t rx_byte;
    uint8_t rx_buffer[MOTOR_TTL_MAX_RX_LENGTH];
    uint8_t rx_length;
    uint8_t rx_expected_length;
};

/** 初始化共享 TTL 总线。默认串口参数应为 115200-8-N-1。 */
motor_ttl_result_t motor_ttl_init(motor_ttl_bus_t *bus,
                                  UART_HandleTypeDef *huart,
                                  uint32_t tx_timeout_ms);

/** 将一台电机注册到共享总线；同一条总线最多注册六台且 ID 不得重复。 */
motor_ttl_result_t motor_ttl_motor_init(motor_ttl_motor_t *motor,
                                        motor_ttl_bus_t *bus,
                                        uint8_t id);

/** 获取当前默认 TTL 总线及已经注册的电机对象，便于上层控制模块复用。 */
motor_ttl_bus_t *motor_ttl_get_bus(void);
motor_ttl_motor_t *motor_ttl_get_motor(uint8_t id);

/** 启动或重新启动 USART2 单字节接收中断。 */
motor_ttl_result_t motor_ttl_start_receive(motor_ttl_bus_t *bus);

/** 在 HAL_UART 回调中转发 USART2 接收完成和错误事件。 */
void motor_ttl_on_rx_complete(UART_HandleTypeDef *huart);
void motor_ttl_on_error(UART_HandleTypeDef *huart);

/** 控制命令。 */
motor_ttl_result_t motor_ttl_enable(motor_ttl_motor_t *motor,
                                    bool enable,
                                    motor_ttl_sync_t sync);
motor_ttl_result_t motor_ttl_set_speed(motor_ttl_motor_t *motor,
                                       int16_t speed_rpm,
                                       uint8_t acceleration,
                                       motor_ttl_sync_t sync);
motor_ttl_result_t motor_ttl_set_speeds_sync(motor_ttl_motor_t *const motors[],
                                             const int16_t speed_rpm[],
                                             uint8_t motor_count,
                                             uint8_t acceleration);
motor_ttl_result_t motor_ttl_set_position(motor_ttl_motor_t *motor,
                                          int64_t pulse,
                                          uint16_t speed_rpm,
                                          uint8_t acceleration,
                                          motor_ttl_position_mode_t mode,
                                          motor_ttl_sync_t sync);
motor_ttl_result_t motor_ttl_stop(motor_ttl_motor_t *motor,
                                  motor_ttl_sync_t sync);
motor_ttl_result_t motor_ttl_stop_all(motor_ttl_bus_t *bus);
motor_ttl_result_t motor_ttl_trigger_sync(motor_ttl_bus_t *bus);

/** 位置、保护、回零和安全配置。 */
motor_ttl_result_t motor_ttl_clear_position(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_clear_protection(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_home(motor_ttl_motor_t *motor,
                                  uint8_t home_mode,
                                  motor_ttl_sync_t sync);
motor_ttl_result_t motor_ttl_home_abort(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_set_heartbeat(motor_ttl_motor_t *motor,
                                           uint32_t heartbeat_ms,
                                           bool save_to_flash);
motor_ttl_result_t motor_ttl_set_periodic_report(motor_ttl_motor_t *motor,
                                                 motor_ttl_information_t information,
                                                 uint16_t period_ms);

/** 一次性读取反馈数据。 */
motor_ttl_result_t motor_ttl_read_input_pulses(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_speed(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_position(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_position_error(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_state(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_home_state(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_bus_voltage(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_bus_current(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_phase_current(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_encoder(motor_ttl_motor_t *motor);
motor_ttl_result_t motor_ttl_read_temperature(motor_ttl_motor_t *motor);

/** 供测试或其他接收路径提交一条完整 TTL 返回帧。 */
motor_ttl_result_t motor_ttl_process_frame(motor_ttl_bus_t *bus,
                                           const uint8_t *data,
                                           uint8_t length);

#endif /* ENGINEERING_INNOVATION_MOTOR_TTL_H */
