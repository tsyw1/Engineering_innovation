//
// Created by m1816 on 2026/9/14.
//
// Y42_V2.0 + Emm5.0 电机驱动公共接口。
//
// 本模块只负责 Y42 电机协议，不绑定某一种 CAN 转 TTL 模块协议：
// 1. 一个 motor_bus_t 对应一个串口和一个 CAN 转 TTL 模块；
// 2. 最多六个 motor_t 共享同一个 motor_bus_t；
// 3. CAN 转 TTL 模块的串口封装由 motor_can_send_callback_t 回调完成；
// 4. 转换模块收到 CAN 帧后，应调用 motor_bus_process_can_frame() 更新电机状态。
//

#ifndef ENGINEERING_INNOVATION_MOTOR_H
#define ENGINEERING_INNOVATION_MOTOR_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

// 当前项目一条 CAN 总线上最多挂载六台电机。
#define MOTOR_MAX_COUNT                 6U

// 经典 CAN 单帧最多携带 8 字节数据。
#define MOTOR_CAN_FRAME_DATA_SIZE       8U

// Emm 速度模式命令允许的速度范围为 0-3000 RPM。
#define MOTOR_EMM_MAX_SPEED_RPM         3000U

// 电机保持出厂默认的固定 0x6B 校验方式。
#define MOTOR_FIXED_CHECKSUM            0x6BU

// 电机状态字各位定义，对应手册“读取电机状态标志”。
#define MOTOR_STATE_ENABLED             (1U << 0)
#define MOTOR_STATE_POSITION_REACHED    (1U << 1)
#define MOTOR_STATE_STALL               (1U << 2)
#define MOTOR_STATE_STALL_PROTECTED     (1U << 3)
#define MOTOR_STATE_LEFT_LIMIT          (1U << 4)
#define MOTOR_STATE_RIGHT_LIMIT         (1U << 5)
#define MOTOR_STATE_POWER_LOSS          (1U << 7)

// 回零状态字各位定义，对应手册“读取回零状态标志”。
#define MOTOR_HOME_ENCODER_READY        (1U << 0)
#define MOTOR_HOME_CALIBRATION_READY    (1U << 1)
#define MOTOR_HOME_RUNNING              (1U << 2)
#define MOTOR_HOME_FAILED               (1U << 3)
#define MOTOR_HOME_OVER_TEMPERATURE     (1U << 4)
#define MOTOR_HOME_OVER_CURRENT         (1U << 5)

typedef enum
{
    MOTOR_OK = 0,            // 操作成功或命令已成功交给转换模块
    MOTOR_ERROR,             // 发送失败、校验失败或返回帧格式错误
    MOTOR_BUSY,              // 共享串口总线正在发送上一条命令
    MOTOR_INVALID_PARAM,     // 指针、速度、地址或模式参数不合法
    MOTOR_NOT_FOUND          // 收到的电机 ID 未在当前总线上注册
} motor_result_t;

typedef enum
{
    MOTOR_DIRECTION_CW = 0,      // 顺时针方向，对应协议方向字节 0x00
    MOTOR_DIRECTION_CCW = 1      // 逆时针方向，对应协议方向字节 0x01
} motor_direction_t;

typedef enum
{
    MOTOR_EXECUTE_NOW = 0,       // 电机收到命令后立即执行
    MOTOR_EXECUTE_SYNC = 1       // 先缓存命令，等待广播同步触发命令
} motor_sync_t;

typedef enum
{
    MOTOR_POSITION_RELATIVE_LAST = 0,       // 相对上一次输入的目标位置运动
    MOTOR_POSITION_ABSOLUTE = 1,            // 相对坐标零点进行绝对位置运动
    MOTOR_POSITION_RELATIVE_CURRENT = 2     // 相对当前实际位置运动
} motor_position_mode_t;

typedef enum
{
    MOTOR_REPLY_NONE = 0,               // 尚未收到控制命令应答
    MOTOR_REPLY_OK = 0x02,              // 命令接收正确
    MOTOR_REPLY_LOW_VOLTAGE = 0x12,     // 低压等运行条件不满足
    MOTOR_REPLY_PROTECTED = 0x22,       // 电机处于堵转、过流或过热保护状态
    MOTOR_REPLY_REACHED = 0x9F,         // 位置、回零或夹紧动作已经完成
    MOTOR_REPLY_PARAM_ERROR = 0xE2,     // 命令参数超出允许范围
    MOTOR_REPLY_FORMAT_ERROR = 0xEE     // 命令格式错误
} motor_reply_t;

// 定时返回命令支持的信息功能码，同时也对应各读取函数的功能码。
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

/**
 * @brief CAN 转 TTL 模块发送回调。
 *
 * 电机驱动已经生成好 29 位 CAN 扩展帧 ID、数据和长度。该回调只需要按照
 * 实际 CAN 转 TTL 模块的串口协议进行封装，再通过 huart 发送出去。
 *
 * @param huart         与 CAN 转 TTL 模块相连的串口句柄，本项目通常为 huart2。
 * @param extended_id   29 位 CAN 扩展帧 ID，Y42 格式为 (电机地址 << 8) | 分包号。
 * @param data          CAN 数据区，不包含 CAN ID 和 DLC。
 * @param length        CAN 数据长度，范围 1-8 字节。
 * @param timeout_ms    串口发送超时时间。
 * @return HAL_OK 表示转换模块已成功接收待发送帧；其他值会转换为驱动错误码。
 */
typedef HAL_StatusTypeDef (*motor_can_send_callback_t)(UART_HandleTypeDef *huart,
                                                        uint32_t extended_id,
                                                        const uint8_t *data,
                                                        uint8_t length,
                                                        uint32_t timeout_ms);

/**
 * @brief ZDT CAN 转 TTL 模块的串口命令发送回调。
 *
 * 该模块的 TTL 侧使用标准 ZDT 串口命令格式：
 *     电机地址 + 功能码 + 命令参数 + 0x6B
 *
 * 与原生 CAN 回调不同，这里传入的是完整命令，不需要拆成 8 字节 CAN 帧。
 */
typedef HAL_StatusTypeDef (*motor_ttl_send_callback_t)(UART_HandleTypeDef *huart,
                                                        uint8_t address,
                                                        const uint8_t *command,
                                                        uint8_t length,
                                                        uint32_t timeout_ms);

/**
 * @brief 通过张大头 ZDT V4.0 CAN 转 TTL 模块发送完整串口命令。
 *
 * TTL 侧帧格式为：电机地址 + 功能码 + 参数 + 0x6B。
 * 该函数内部使用 USART2（或调用者传入的 UART 句柄）发送，不需要额外帧头。
 */
HAL_StatusTypeDef motor_can_ttl_send_command(UART_HandleTypeDef *huart,
                                             uint8_t address,
                                             const uint8_t *command,
                                             uint8_t length,
                                             uint32_t timeout_ms);

typedef struct
{
    motor_bus_t *bus;                         // 所属共享总线
    uint8_t id;                               // 电机地址，合法范围 1-255，0 为广播地址

    volatile int16_t target_speed_rpm;        // 最近一次成功发送的目标转速，带正负方向
    volatile int16_t actual_speed_rpm;        // 电机返回的实时转速，Emm 固件单位为 RPM
    volatile int64_t actual_position_pulse;   // 实时累计位置，Emm 固件单位为脉冲
    volatile int64_t position_error_pulse;    // 目标位置与实际位置的误差，单位为脉冲
    volatile uint16_t bus_voltage_mv;         // 驱动输入母线电压，单位 mV
    volatile uint16_t bus_current_ma;         // 估算的母线电流，单位 mA，仅供趋势参考
    volatile uint16_t phase_current_ma;       // 电机实时相电流，单位 mA
    volatile uint16_t battery_voltage_mv;     // Y42 掉电记录电池电压，单位 mV
    volatile int8_t temperature_c;            // 驱动板温度，单位摄氏度
    volatile uint8_t state_flags;             // MOTOR_STATE_xxx 状态位组合
    volatile uint8_t home_flags;              // MOTOR_HOME_xxx 状态位组合
    volatile uint8_t last_function;           // 最近一次收到的功能码
    volatile motor_reply_t last_reply;        // 最近一次控制命令应答
    volatile uint32_t last_update_ms;          // 最近一次收到有效 CAN 帧的 HAL 系统时间
} motor_t;

struct motor_bus
{
    UART_HandleTypeDef *huart;                 // 六台电机共用的转换模块串口
    motor_can_send_callback_t send_can_frame; // 转换模块发送适配函数
    motor_ttl_send_callback_t send_ttl_command; // ZDT CAN 转 TTL 标准串口命令适配函数
    motor_t *motors[MOTOR_MAX_COUNT];          // 已注册电机指针表
    uint32_t tx_timeout_ms;                    // 单个 CAN 帧的串口发送超时
    uint8_t motor_count;                       // 当前已注册电机数量
    volatile uint8_t tx_busy;                  // 防止主循环和中断重复占用同一串口
};

/** 初始化共享电机总线，一条总线只需要调用一次。 */
motor_result_t motor_bus_init(motor_bus_t *bus,
                              UART_HandleTypeDef *huart,
                              motor_can_send_callback_t send_can_frame,
                              uint32_t tx_timeout_ms);

/** 初始化 ZDT CAN 转 TTL 模块总线，发送完整的地址+命令串口帧。 */
motor_result_t motor_bus_init_ttl(motor_bus_t *bus,
                                  UART_HandleTypeDef *huart,
                                  motor_ttl_send_callback_t send_ttl_command,
                                  uint32_t tx_timeout_ms);

/** 将一台电机及其 ID 注册到共享总线，六台电机分别调用一次。 */
motor_result_t motor_init(motor_t *motor, motor_bus_t *bus, uint8_t id);

/** 使能或失能电机；失能后电机松轴。 */
motor_result_t motor_enable(motor_t *motor, bool enable, motor_sync_t sync);

/** 设置单台电机速度，正数为 CW，负数为 CCW，范围 -3000 至 3000 RPM。 */
motor_result_t motor_set_speed(motor_t *motor, int16_t speed_rpm, uint8_t acceleration, motor_sync_t sync);

/**
 * 使用 Y42 多电机命令一次设置多台电机速度。
 * 所有 motor 必须属于同一个 bus，命令完整接收后各电机同时开始运动。
 */
motor_result_t motor_set_speeds_sync(motor_t *const motors[],
                                     const int16_t speed_rpm[],
                                     uint8_t motor_count,
                                     uint8_t acceleration);

/**
 * 设置 Emm 位置模式。
 * pulse 正负号表示方向；默认 1.8 度电机、16 细分时 3200 脉冲为一圈。
 */
motor_result_t motor_set_position(motor_t *motor,
                                  int64_t pulse,
                                  uint16_t speed_rpm,
                                  uint8_t acceleration,
                                  motor_position_mode_t mode,
                                  motor_sync_t sync);

/** 立即停止或缓存停止单台电机，停止时保持使能并刹车。 */
motor_result_t motor_stop(motor_t *motor, motor_sync_t sync);

/** 使用广播地址 0 立即停止当前总线上的全部电机。 */
motor_result_t motor_stop_all(motor_bus_t *bus);

/** 触发之前使用 MOTOR_EXECUTE_SYNC 缓存的命令。 */
motor_result_t motor_trigger_sync(motor_bus_t *bus);

/** 将电机当前位置和目标位置清零。 */
motor_result_t motor_clear_position(motor_t *motor);

/** 解除堵转、过流和过热保护；故障原因未消除时可能再次进入保护。 */
motor_result_t motor_clear_protection(motor_t *motor);

/** 设置通信心跳保护时间，单位 ms；设为 0 表示关闭心跳保护。 */
motor_result_t motor_set_heartbeat(motor_t *motor, uint32_t heartbeat_ms, bool save_to_flash);

/** 设置某项状态的周期回传时间，period_ms 为 0 时停止该项周期回传。 */
motor_result_t motor_set_periodic_report(motor_t *motor,
                                         motor_information_t information,
                                         uint16_t period_ms);

// 下列函数发送一次读取命令，实际结果由接收解析函数写入 motor_t。
motor_result_t motor_read_speed(motor_t *motor);
motor_result_t motor_read_position(motor_t *motor);
motor_result_t motor_read_position_error(motor_t *motor);
motor_result_t motor_read_state(motor_t *motor);
motor_result_t motor_read_home_state(motor_t *motor);
motor_result_t motor_read_bus_voltage(motor_t *motor);
motor_result_t motor_read_bus_current(motor_t *motor);
motor_result_t motor_read_phase_current(motor_t *motor);
motor_result_t motor_read_temperature(motor_t *motor);
motor_result_t motor_read_battery_voltage(motor_t *motor);

/**
 * @brief 处理 CAN 转 TTL 模块解出的单个 CAN 扩展帧。
 *
 * 调用者应先去掉转换模块自己的串口帧头和校验，只传入真实 CAN ID、数据区
 * 和 DLC。本函数根据 CAN ID 中的电机地址找到对应 motor_t 并更新状态。
 */
motor_result_t motor_bus_process_can_frame(motor_bus_t *bus,
                                           uint32_t extended_id,
                                           const uint8_t *data,
                                           uint8_t length);

#endif //ENGINEERING_INNOVATION_MOTOR_H
