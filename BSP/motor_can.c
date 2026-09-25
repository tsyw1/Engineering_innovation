//
// Created by m1816 on 2026/9/14.
//
// Y42_V2.0 + Emm5.0 自由协议实现。
//
// 电机侧使用经典 CAN 29 位扩展帧：
//     CAN_ID = (电机地址 << 8) | 分包序号
// CAN 数据区不包含电机地址，首字节直接从功能码开始。
// 本文件负责生成/解析上述电机协议，CAN 转 TTL 模块自身的串口协议由回调适配。
//

#include "motor_can.h"

#include <string.h>

// Emm 运动控制功能码。
#define MOTOR_CMD_ENABLE                0xF3U
#define MOTOR_CMD_SPEED                 0xF6U
#define MOTOR_CMD_POSITION              0xFDU
#define MOTOR_CMD_STOP                  0xFEU
#define MOTOR_CMD_TRIGGER_SYNC          0xFFU
#define MOTOR_CMD_MULTI                 0xAAU

// 参数操作及辅助控制功能码。
#define MOTOR_CMD_CLEAR_POSITION        0x0AU
#define MOTOR_CMD_CLEAR_PROTECTION      0x0EU
#define MOTOR_CMD_PERIODIC_REPORT       0x11U
#define MOTOR_CMD_SET_HEARTBEAT         0x68U

// 常用状态读取功能码。
#define MOTOR_CMD_READ_BUS_VOLTAGE      0x24U
#define MOTOR_CMD_READ_BUS_CURRENT      0x26U
#define MOTOR_CMD_READ_PHASE_CURRENT    0x27U
#define MOTOR_CMD_READ_SPEED            0x35U
#define MOTOR_CMD_READ_POSITION         0x36U
#define MOTOR_CMD_READ_POSITION_ERROR   0x37U
#define MOTOR_CMD_READ_BATTERY_VOLTAGE  0x38U
#define MOTOR_CMD_READ_TEMPERATURE      0x39U
#define MOTOR_CMD_READ_STATE            0x3AU
#define MOTOR_CMD_READ_HOME_STATE       0x3BU
#define MOTOR_CMD_READ_ALL_STATE        0x3CU

#define MOTOR_MULTI_COMMAND_MAX_SIZE    64U

HAL_StatusTypeDef motor_can_ttl_send_command(UART_HandleTypeDef *huart,
                                             uint8_t address,
                                             const uint8_t *command,
                                             uint8_t length,
                                             uint32_t timeout_ms)
{
    uint8_t uart_frame[1U + 32U];

    /* 地址 0 是广播地址，地址 1-255 用于单台电机。 */
    if ((huart == NULL) || (command == NULL) ||
        (length == 0U) || (length > 32U))
    {
        return HAL_ERROR;
    }

    /* ZDT V4.0 模块 TTL 侧使用“地址+标准命令”，不需要额外帧头。 */
    uart_frame[0] = address;
    memcpy(&uart_frame[1], command, length);

    return HAL_UART_Transmit(huart,
                             uart_frame,
                             (uint16_t)(length + 1U),
                             timeout_ms);
}

/**
 * @brief 尝试占用共享串口总线。
 *
 * 六台电机共用同一个 USART2，长命令还可能拆成多个 CAN 帧。发送过程中不允许
 * 其他调用插入新帧，否则会破坏 Y42 的分包顺序。这里只在修改 tx_busy 时短暂
 * 关闭中断，不会在整个串口发送期间关闭中断。
 */
static motor_result_t motor_bus_lock(motor_bus_t *bus)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (bus->tx_busy != 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return MOTOR_BUSY;
    }

    bus->tx_busy = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return MOTOR_OK;
}

/** 释放共享串口总线。 */
static void motor_bus_unlock(motor_bus_t *bus)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    bus->tx_busy = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

/** 从协议大端字节流读取 16 位无符号整数。 */
static uint16_t motor_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

/** 从协议大端字节流读取 32 位无符号整数。 */
static uint32_t motor_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           data[3];
}

/** 将 16 位整数按高字节在前的协议格式写入缓冲区。 */
static void motor_write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

/** 将 32 位整数按高字节在前的协议格式写入缓冲区。 */
static void motor_write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

/** 根据 CAN ID 中解析出的电机地址查找对应的软件对象。 */
static motor_t *motor_bus_find_motor(motor_bus_t *bus, uint8_t id)
{
    uint8_t index;

    for (index = 0U; index < bus->motor_count; index++)
    {
        if ((bus->motors[index] != NULL) && (bus->motors[index]->id == id))
        {
            return bus->motors[index];
        }
    }

    return NULL;
}

/**
 * @brief 将一条完整的 Y42 命令拆成一个或多个 CAN 扩展帧发送。
 *
 * command 中不包含电机地址，例如 Emm 速度命令为：
 *     F6 方向 速度高 速度低 加速度 同步标志 6B
 *
 * 当命令不超过 8 字节时只发送第 0 包。当命令超过 8 字节时：
 * 1. 第 0 包直接发送命令前 8 字节；
 * 2. 后续包的第一个字节重复功能码；
 * 3. 后续包每包继续携带最多 7 字节剩余数据；
 * 4. CAN ID 低 8 位依次为 0、1、2……。
 */
static motor_result_t motor_bus_send_command(motor_bus_t *bus,
                                              uint8_t address,
                                              const uint8_t *command,
                                              uint8_t command_length)
{
    HAL_StatusTypeDef hal_status;
    motor_result_t result;
    uint8_t frame[MOTOR_CAN_FRAME_DATA_SIZE];
    uint8_t packet = 0U;
    uint8_t offset = 0U;
    uint8_t copy_length;
    uint8_t frame_length;
    uint32_t extended_id;

    if ((bus == NULL) || (bus->huart == NULL) || (bus->send_can_frame == NULL) ||
        (command == NULL) || (command_length < 2U))
    {
        /* ZDT CAN 转 TTL 模块使用完整串口命令，不需要 CAN 分包。 */
        if ((bus == NULL) || (bus->huart == NULL) ||
            (bus->send_ttl_command == NULL) || (command == NULL) ||
            (command_length < 2U))
        {
            return MOTOR_INVALID_PARAM;
        }

        result = motor_bus_lock(bus);
        if (result != MOTOR_OK)
        {
            return result;
        }

        hal_status = bus->send_ttl_command(bus->huart,
                                            address,
                                            command,
                                            command_length,
                                            bus->tx_timeout_ms);
        motor_bus_unlock(bus);

        if (hal_status == HAL_OK)
        {
            return MOTOR_OK;
        }

        return (hal_status == HAL_BUSY) ? MOTOR_BUSY : MOTOR_ERROR;
    }

    result = motor_bus_lock(bus);
    if (result != MOTOR_OK)
    {
        return result;
    }

    while (offset < command_length)
    {
        memset(frame, 0, sizeof(frame));

        if (packet == 0U)
        {
            // 首包可以使用完整的 8 字节 CAN 数据区。
            copy_length = (command_length > MOTOR_CAN_FRAME_DATA_SIZE) ?
                          MOTOR_CAN_FRAME_DATA_SIZE : command_length;
            memcpy(frame, command, copy_length);
            frame_length = copy_length;
        }
        else
        {
            // 后续 CAN 分包需要在首字节重复功能码。
            frame[0] = command[0];
            copy_length = (uint8_t)(command_length - offset);
            if (copy_length > (MOTOR_CAN_FRAME_DATA_SIZE - 1U))
            {
                copy_length = MOTOR_CAN_FRAME_DATA_SIZE - 1U;
            }
            memcpy(&frame[1], &command[offset], copy_length);
            frame_length = (uint8_t)(copy_length + 1U);
        }

        // Y42 固定使用 29 位扩展帧，地址位于 ID 的 bit8-bit15。
        extended_id = ((uint32_t)address << 8U) | packet;
        hal_status = bus->send_can_frame(bus->huart,
                                         extended_id,
                                         frame,
                                         frame_length,
                                         bus->tx_timeout_ms);
        if (hal_status != HAL_OK)
        {
            motor_bus_unlock(bus);
            return (hal_status == HAL_BUSY) ? MOTOR_BUSY : MOTOR_ERROR;
        }

        offset = (uint8_t)(offset + copy_length);
        packet++;
    }

    motor_bus_unlock(bus);
    return MOTOR_OK;
}

/** 发送“功能码 + 0x6B”格式的短读取命令。 */
static motor_result_t motor_send_read_command(motor_t *motor, uint8_t function)
{
    uint8_t command[2];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    command[0] = function;
    command[1] = MOTOR_FIXED_CHECKSUM;

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_bus_init(motor_bus_t *bus,
                              UART_HandleTypeDef *huart,
                              motor_can_send_callback_t send_can_frame,
                              uint32_t tx_timeout_ms)
{
    if ((bus == NULL) || (huart == NULL) || (send_can_frame == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 总线对象由用户静态分配，这里清除历史注册信息和忙标志。
    memset(bus, 0, sizeof(*bus));
    bus->huart = huart;
    bus->send_can_frame = send_can_frame;
    bus->tx_timeout_ms = tx_timeout_ms;

    return MOTOR_OK;
}

motor_result_t motor_bus_init_ttl(motor_bus_t *bus,
                                  UART_HandleTypeDef *huart,
                                  motor_ttl_send_callback_t send_ttl_command,
                                  uint32_t tx_timeout_ms)
{
    if ((bus == NULL) || (huart == NULL) || (send_ttl_command == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    // ZDT CAN 转 TTL 模块由串口侧接收完整的地址+命令帧。
    memset(bus, 0, sizeof(*bus));
    bus->huart = huart;
    bus->send_ttl_command = send_ttl_command;
    bus->tx_timeout_ms = tx_timeout_ms;

    return MOTOR_OK;
}

motor_result_t motor_init(motor_t *motor, motor_bus_t *bus, uint8_t id)
{
    if ((motor == NULL) || (bus == NULL) || (id == 0U) ||
        (bus->motor_count >= MOTOR_MAX_COUNT))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 同一条总线上不能注册两个相同 ID，否则返回帧无法正确归属。
    if (motor_bus_find_motor(bus, id) != NULL)
    {
        return MOTOR_INVALID_PARAM;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;
    motor->last_reply = MOTOR_REPLY_NONE;

    // 保存对象指针，接收 CAN 帧时会通过 ID 查找并更新此对象。
    bus->motors[bus->motor_count] = motor;
    bus->motor_count++;

    return MOTOR_OK;
}

motor_result_t motor_enable(motor_t *motor, bool enable, motor_sync_t sync)
{
    uint8_t command[5];

    if ((motor == NULL) || (motor->bus == NULL) || (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 协议格式：F3 AB 使能状态 同步标志 6B。
    command[0] = MOTOR_CMD_ENABLE;
    command[1] = 0xABU;
    command[2] = enable ? 1U : 0U;
    command[3] = (uint8_t)sync;
    command[4] = MOTOR_FIXED_CHECKSUM;

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_set_speed(motor_t *motor, int16_t speed_rpm, uint8_t acceleration, motor_sync_t sync)
{
    uint8_t command[7];
    uint16_t speed_abs;
    int32_t speed_value;
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) || (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 对外使用有符号转速，驱动内部拆分成方向字节和无符号速度绝对值。
    speed_value = speed_rpm;
    speed_abs = (uint16_t)((speed_value < 0) ? -speed_value : speed_value);
    if (speed_abs > MOTOR_EMM_MAX_SPEED_RPM)
    {
        return MOTOR_INVALID_PARAM;
    }

    // Emm 格式：F6 方向 速度高 速度低 加速度 同步标志 6B。
    command[0] = MOTOR_CMD_SPEED;
    command[1] = (speed_rpm < 0) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
    motor_write_u16_be(&command[2], speed_abs);
    command[4] = acceleration;
    command[5] = (uint8_t)sync;
    command[6] = MOTOR_FIXED_CHECKSUM;

    result = motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
    if (result == MOTOR_OK)
    {
        motor->target_speed_rpm = speed_rpm;
    }

    return result;
}

motor_result_t motor_set_speeds_sync(motor_t *const motors[],
                                     const int16_t speed_rpm[],
                                     uint8_t motor_count,
                                     uint8_t acceleration)
{
    uint8_t command[MOTOR_MULTI_COMMAND_MAX_SIZE];
    uint8_t index;
    uint8_t offset = 3U;
    uint16_t total_length;
    motor_result_t result;

    if ((motors == NULL) || (speed_rpm == NULL) ||
        (motor_count == 0U) || (motor_count > MOTOR_MAX_COUNT) ||
        (motors[0] == NULL) || (motors[0]->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    /*
     * 多电机原始命令格式：
     * 00 AA 总长度高 总长度低 [电机1完整命令] ... [电机N完整命令] 6B
     *
     * 每条 Emm 速度命令包含地址在内共 8 字节；total_length 需要把广播地址、
     * AA 功能码、两个长度字节、全部子命令和最后一个外层 6B 都计算在内。
     */
    total_length = (uint16_t)(4U + ((uint16_t)motor_count * 8U) + 1U);
    command[0] = MOTOR_CMD_MULTI;
    motor_write_u16_be(&command[1], total_length);

    for (index = 0U; index < motor_count; index++)
    {
        uint16_t speed_abs;
        int32_t speed_value;

        if ((motors[index] == NULL) || (motors[index]->bus != motors[0]->bus))
        {
            return MOTOR_INVALID_PARAM;
        }

        speed_value = speed_rpm[index];
        speed_abs = (uint16_t)((speed_value < 0) ? -speed_value : speed_value);
        if (speed_abs > MOTOR_EMM_MAX_SPEED_RPM)
        {
            return MOTOR_INVALID_PARAM;
        }

        // 子命令必须包含电机地址，每台电机可以使用不同速度和方向。
        command[offset++] = motors[index]->id;
        command[offset++] = MOTOR_CMD_SPEED;
        command[offset++] = (speed_rpm[index] < 0) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
        motor_write_u16_be(&command[offset], speed_abs);
        offset = (uint8_t)(offset + 2U);
        command[offset++] = acceleration;
        command[offset++] = MOTOR_EXECUTE_NOW;
        command[offset++] = MOTOR_FIXED_CHECKSUM;

    }

    command[offset++] = MOTOR_FIXED_CHECKSUM;
    // 多电机命令由地址 0 发出，六个目标速度在一条逻辑命令内同时生效。
    result = motor_bus_send_command(motors[0]->bus, 0U, command, offset);
    if (result == MOTOR_OK)
    {
        for (index = 0U; index < motor_count; index++)
        {
            motors[index]->target_speed_rpm = speed_rpm[index];
        }
    }

    return result;
}

motor_result_t motor_set_position(motor_t *motor,
                                  int64_t pulse,
                                  uint16_t speed_rpm,
                                  uint8_t acceleration,
                                  motor_position_mode_t mode,
                                  motor_sync_t sync)
{
    uint8_t command[12];
    uint32_t pulse_abs;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (speed_rpm > MOTOR_EMM_MAX_SPEED_RPM) ||
        (mode > MOTOR_POSITION_RELATIVE_CURRENT) ||
        (sync > MOTOR_EXECUTE_SYNC) ||
        (pulse > (int64_t)UINT32_MAX) ||
        (pulse < -(int64_t)UINT32_MAX))
    {
        return MOTOR_INVALID_PARAM;
    }

    // Emm 协议将方向和 32 位脉冲绝对值分开传输。
    pulse_abs = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;

    // 格式：FD 方向 速度 加速度 脉冲数 运动模式 同步标志 6B，共 12 字节。
    command[0] = MOTOR_CMD_POSITION;
    command[1] = (pulse < 0) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
    motor_write_u16_be(&command[2], speed_rpm);
    command[4] = acceleration;
    motor_write_u32_be(&command[5], pulse_abs);
    command[9] = (uint8_t)mode;
    command[10] = (uint8_t)sync;
    command[11] = MOTOR_FIXED_CHECKSUM;

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_stop(motor_t *motor, motor_sync_t sync)
{
    uint8_t command[4];
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) || (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    // FE 98 为立即停止命令；同步标志为 1 时先缓存停止动作。
    command[0] = MOTOR_CMD_STOP;
    command[1] = 0x98U;
    command[2] = (uint8_t)sync;
    command[3] = MOTOR_FIXED_CHECKSUM;

    result = motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
    if (result == MOTOR_OK)
    {
        motor->target_speed_rpm = 0;
    }

    return result;
}

motor_result_t motor_stop_all(motor_bus_t *bus)
{
    uint8_t command[4] = {MOTOR_CMD_STOP, 0x98U, MOTOR_EXECUTE_NOW, MOTOR_FIXED_CHECKSUM};
    uint8_t index;
    motor_result_t result;

    if (bus == NULL)
    {
        return MOTOR_INVALID_PARAM;
    }

    // 地址 0 为广播地址，所有在线电机都会执行立即停止。
    result = motor_bus_send_command(bus, 0U, command, sizeof(command));
    if (result != MOTOR_OK)
    {
        return result;
    }

    for (index = 0U; index < bus->motor_count; index++)
    {
        if (bus->motors[index] != NULL)
        {
            bus->motors[index]->target_speed_rpm = 0;
        }
    }

    return MOTOR_OK;
}

motor_result_t motor_trigger_sync(motor_bus_t *bus)
{
    uint8_t command[3] = {MOTOR_CMD_TRIGGER_SYNC, 0x66U, MOTOR_FIXED_CHECKSUM};

    // 广播 FF 66 6B，仅触发已经缓存命令的电机，其他电机不动作。
    return motor_bus_send_command(bus, 0U, command, sizeof(command));
}

motor_result_t motor_clear_position(motor_t *motor)
{
    uint8_t command[3] = {MOTOR_CMD_CLEAR_POSITION, 0x6DU, MOTOR_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_clear_protection(motor_t *motor)
{
    uint8_t command[3] = {MOTOR_CMD_CLEAR_PROTECTION, 0x52U, MOTOR_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_set_heartbeat(motor_t *motor, uint32_t heartbeat_ms, bool save_to_flash)
{
    uint8_t command[8];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 在 heartbeat_ms 内未收到正确命令，Y42 会主动急停；0 表示关闭。
    command[0] = MOTOR_CMD_SET_HEARTBEAT;
    command[1] = 0x38U;
    command[2] = save_to_flash ? 1U : 0U;
    motor_write_u32_be(&command[3], heartbeat_ms);
    command[7] = MOTOR_FIXED_CHECKSUM;

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_set_periodic_report(motor_t *motor,
                                         motor_information_t information,
                                         uint16_t period_ms)
{
    uint8_t command[6];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    // 由电机主动周期回传状态，可减少主控频繁轮询六台电机的串口占用。
    command[0] = MOTOR_CMD_PERIODIC_REPORT;
    command[1] = 0x18U;
    command[2] = (uint8_t)information;
    motor_write_u16_be(&command[3], period_ms);
    command[5] = MOTOR_FIXED_CHECKSUM;

    return motor_bus_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_result_t motor_read_speed(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_SPEED);
}

motor_result_t motor_read_position(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_POSITION);
}

motor_result_t motor_read_position_error(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_POSITION_ERROR);
}

motor_result_t motor_read_state(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_STATE);
}

motor_result_t motor_read_home_state(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_HOME_STATE);
}

motor_result_t motor_read_bus_voltage(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_BUS_VOLTAGE);
}

motor_result_t motor_read_bus_current(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_BUS_CURRENT);
}

motor_result_t motor_read_phase_current(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_PHASE_CURRENT);
}

motor_result_t motor_read_temperature(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_TEMPERATURE);
}

motor_result_t motor_read_battery_voltage(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_BATTERY_VOLTAGE);
}

motor_result_t motor_bus_process_can_frame(motor_bus_t *bus,
                                           uint32_t extended_id,
                                           const uint8_t *data,
                                           uint8_t length)
{
    motor_t *motor;
    uint8_t address;
    uint8_t packet;
    uint8_t function;
    uint32_t value_u32;
    int32_t value_s32;
    int64_t value_s64;

    if ((bus == NULL) || (data == NULL) || (length < 2U) ||
        (length > MOTOR_CAN_FRAME_DATA_SIZE))
    {
        return MOTOR_INVALID_PARAM;
    }

    /*
     * Y42 返回帧仍使用 (地址 << 8) | 分包号。
     * 当前解析的速度、位置、电流等返回数据都不超过 8 字节，因此只接受第 0 包。
     * 后续若增加“读取全部系统状态”等长返回命令，需要在这里增加多包重组缓冲区。
     */
    address = (uint8_t)(extended_id >> 8U);
    packet = (uint8_t)extended_id;
    if (packet != 0U)
    {
        return MOTOR_ERROR;
    }

    motor = motor_bus_find_motor(bus, address);
    if (motor == NULL)
    {
        return MOTOR_NOT_FOUND;
    }

    // 当前项目固定使用 0x6B 校验，最后一个数据字节必须为 0x6B。
    if (data[length - 1U] != MOTOR_FIXED_CHECKSUM)
    {
        return MOTOR_ERROR;
    }

    // CAN 数据区第一个字节是功能码，电机地址已经包含在 CAN ID 中。
    function = data[0];
    motor->last_function = function;
    motor->last_update_ms = HAL_GetTick();

    switch (function)
    {
        case MOTOR_CMD_READ_BUS_VOLTAGE:
            if (length != 4U) return MOTOR_ERROR;
            // 格式：24 电压高 电压低 6B，单位 mV。
            motor->bus_voltage_mv = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_BUS_CURRENT:
            if (length != 4U) return MOTOR_ERROR;
            // 格式：26 电流高 电流低 6B，单位 mA。
            motor->bus_current_ma = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_PHASE_CURRENT:
            if (length != 4U) return MOTOR_ERROR;
            // 格式：27 相电流高 相电流低 6B，单位 mA。
            motor->phase_current_ma = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_SPEED:
            if (length != 5U) return MOTOR_ERROR;
            // Emm 格式：35 符号 速度高 速度低 6B，速度单位为 RPM。
            value_s32 = motor_read_u16_be(&data[2]);
            motor->actual_speed_rpm = (int16_t)((data[1] == 0U) ? value_s32 : -value_s32);
            break;

        case MOTOR_CMD_READ_POSITION:
            if (length != 7U) return MOTOR_ERROR;
            // Emm 格式：36 符号 32位脉冲数 6B，保留符号后的完整 32 位范围。
            value_u32 = motor_read_u32_be(&data[2]);
            value_s64 = value_u32;
            motor->actual_position_pulse = (data[1] == 0U) ? value_s64 : -value_s64;
            break;

        case MOTOR_CMD_READ_POSITION_ERROR:
            if (length != 7U) return MOTOR_ERROR;
            // 位置误差同样使用“符号 + 32位脉冲绝对值”的格式。
            value_u32 = motor_read_u32_be(&data[2]);
            value_s64 = value_u32;
            motor->position_error_pulse = (data[1] == 0U) ? value_s64 : -value_s64;
            break;

        case MOTOR_CMD_READ_BATTERY_VOLTAGE:
            if (length != 4U) return MOTOR_ERROR;
            // Y42 专用掉电记录电池电压，单位 mV。
            motor->battery_voltage_mv = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_TEMPERATURE:
            if (length != 4U) return MOTOR_ERROR;
            // 手册定义 0 表示负温度，1 表示正温度，第三字节为温度绝对值。
            motor->temperature_c = (data[1] == 0U) ? -(int8_t)data[2] : (int8_t)data[2];
            break;

        case MOTOR_CMD_READ_STATE:
            if (length != 3U) return MOTOR_ERROR;
            // data[1] 可配合 MOTOR_STATE_xxx 宏判断使能、堵转和限位等状态。
            motor->state_flags = data[1];
            break;

        case MOTOR_CMD_READ_HOME_STATE:
            if (length != 3U) return MOTOR_ERROR;
            // data[1] 可配合 MOTOR_HOME_xxx 宏判断校准、回零和保护状态。
            motor->home_flags = data[1];
            break;

        case MOTOR_CMD_READ_ALL_STATE:
            if (length != 4U) return MOTOR_ERROR;
            // 3C 命令在一个返回帧中同时提供回零状态和电机状态。
            motor->home_flags = data[1];
            motor->state_flags = data[2];
            break;

        default:
            if (length == 3U)
            {
                // 控制类命令通常返回“原功能码 + 应答码 + 6B”。
                motor->last_reply = (motor_reply_t)data[1];
            }
            break;
    }

    return MOTOR_OK;
}
