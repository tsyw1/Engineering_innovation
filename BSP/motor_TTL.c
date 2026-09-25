//
// X42_V1.3 + Emm5.0 TTL 电机驱动实现。
//
// 本文件直接通过 HAL_UART_Transmit 发送“地址 + Emm 命令”，不经过
// motor_can.c，也不依赖 CAN 转 TTL 模块的额外帧头。六台电机共享同一个 UART。
//

#include "motor_TTL.h"

#include <string.h>

#define MOTOR_TTL_CMD_ENABLE             0xF3U
#define MOTOR_TTL_CMD_SPEED              0xF6U
#define MOTOR_TTL_CMD_POSITION           0xFDU
#define MOTOR_TTL_CMD_STOP               0xFEU
#define MOTOR_TTL_CMD_TRIGGER_SYNC      0xFFU
#define MOTOR_TTL_CMD_CLEAR_POSITION     0x0AU
#define MOTOR_TTL_CMD_CLEAR_PROTECTION   0x0EU
#define MOTOR_TTL_CMD_PERIODIC_REPORT    0x11U
#define MOTOR_TTL_CMD_SET_HEARTBEAT      0x68U
#define MOTOR_TTL_CMD_HOME               0x9AU
#define MOTOR_TTL_CMD_HOME_ABORT         0x9CU

#define MOTOR_TTL_CMD_READ_ENCODER       0x31U
#define MOTOR_TTL_CMD_READ_INPUT_PULSES  0x32U
#define MOTOR_TTL_CMD_READ_SPEED         0x35U
#define MOTOR_TTL_CMD_READ_POSITION      0x36U
#define MOTOR_TTL_CMD_READ_POSITION_ERR  0x37U
#define MOTOR_TTL_CMD_READ_BUS_VOLTAGE   0x24U
#define MOTOR_TTL_CMD_READ_BUS_CURRENT   0x26U
#define MOTOR_TTL_CMD_READ_PHASE_CURRENT 0x27U
#define MOTOR_TTL_CMD_READ_TEMPERATURE   0x39U
#define MOTOR_TTL_CMD_READ_STATE         0x3AU
#define MOTOR_TTL_CMD_READ_HOME_STATE    0x3BU
#define MOTOR_TTL_CMD_READ_ALL_STATE     0x3CU

static motor_ttl_bus_t *s_active_bus;

static uint16_t motor_ttl_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static uint32_t motor_ttl_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           data[3];
}

static void motor_ttl_write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

static void motor_ttl_write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

static motor_ttl_result_t motor_ttl_lock(motor_ttl_bus_t *bus)
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
        return MOTOR_TTL_BUSY;
    }

    bus->tx_busy = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return MOTOR_TTL_OK;
}

static void motor_ttl_unlock(motor_ttl_bus_t *bus)
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

static motor_ttl_motor_t *motor_ttl_find_motor(motor_ttl_bus_t *bus, uint8_t id)
{
    uint8_t index;

    if (bus == NULL)
    {
        return NULL;
    }

    for (index = 0U; index < bus->motor_count; index++)
    {
        if ((bus->motors[index] != NULL) && (bus->motors[index]->id == id))
        {
            return bus->motors[index];
        }
    }
    return NULL;
}

static int32_t motor_ttl_signed_value(uint8_t sign, uint32_t value)
{
    if (value > 0x7FFFFFFFU)
    {
        value = 0x7FFFFFFFU;
    }
    return (sign == 0U) ? (int32_t)value : -(int32_t)value;
}

static int32_t motor_ttl_angle_cdeg(int32_t raw)
{
    uint32_t magnitude;
    int32_t angle;

    /* Emm 位置反馈的低 16 位按一圈 0-65535 表示，换算为 0.01 度。 */
    magnitude = (raw < 0) ? (uint32_t)(-(int64_t)raw) : (uint32_t)raw;
    angle = (int32_t)(((uint64_t)(magnitude & 0xFFFFU) * 36000ULL) / 65536ULL);
    return (raw < 0) ? -angle : angle;
}

static uint8_t motor_ttl_expected_length(uint8_t function)
{
    switch (function)
    {
        case MOTOR_TTL_CMD_READ_BUS_VOLTAGE:
        case MOTOR_TTL_CMD_READ_BUS_CURRENT:
        case MOTOR_TTL_CMD_READ_PHASE_CURRENT:
        case MOTOR_TTL_CMD_READ_ENCODER:
            return 5U; /* 地址 + 功能码 + 16 位数据 + 6B */

        case MOTOR_TTL_CMD_READ_SPEED:
        case MOTOR_TTL_CMD_READ_TEMPERATURE:
            return (function == MOTOR_TTL_CMD_READ_SPEED) ? 6U : 5U;

        case MOTOR_TTL_CMD_READ_INPUT_PULSES:
        case MOTOR_TTL_CMD_READ_POSITION:
        case MOTOR_TTL_CMD_READ_POSITION_ERR:
            return 8U; /* 地址 + 功能码 + 符号 + 32 位数据 + 6B */

        case MOTOR_TTL_CMD_READ_STATE:
        case MOTOR_TTL_CMD_READ_HOME_STATE:
            return 4U;

        case MOTOR_TTL_CMD_READ_ALL_STATE:
            return 5U;

        case MOTOR_TTL_CMD_ENABLE:
        case MOTOR_TTL_CMD_SPEED:
        case MOTOR_TTL_CMD_POSITION:
        case MOTOR_TTL_CMD_STOP:
        case MOTOR_TTL_CMD_TRIGGER_SYNC:
        case MOTOR_TTL_CMD_CLEAR_POSITION:
        case MOTOR_TTL_CMD_CLEAR_PROTECTION:
        case MOTOR_TTL_CMD_PERIODIC_REPORT:
        case MOTOR_TTL_CMD_SET_HEARTBEAT:
        case MOTOR_TTL_CMD_HOME:
        case MOTOR_TTL_CMD_HOME_ABORT:
            return 4U; /* 地址 + 原功能码 + 应答码 + 6B */

        default:
            return 0U;
    }
}

static motor_ttl_result_t motor_ttl_send_command(motor_ttl_bus_t *bus,
                                                 uint8_t address,
                                                 const uint8_t *command,
                                                 uint8_t command_length)
{
    HAL_StatusTypeDef status;
    uint8_t frame[MOTOR_TTL_MAX_COMMAND_LENGTH + 1U];
    motor_ttl_result_t result;

    if ((bus == NULL) || (bus->huart == NULL) || (command == NULL) ||
        (command_length < 2U) ||
        (command_length > MOTOR_TTL_MAX_COMMAND_LENGTH))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    result = motor_ttl_lock(bus);
    if (result != MOTOR_TTL_OK)
    {
        return result;
    }

    /* TTL 侧没有 CAN 帧头，首字节就是地址；地址 0 表示广播。 */
    frame[0] = address;
    memcpy(&frame[1], command, command_length);
    status = HAL_UART_Transmit(bus->huart,
                               frame,
                               (uint16_t)(command_length + 1U),
                               bus->tx_timeout_ms);
    motor_ttl_unlock(bus);

    if (status == HAL_OK)
    {
        return MOTOR_TTL_OK;
    }
    return (status == HAL_BUSY) ? MOTOR_TTL_BUSY : MOTOR_TTL_ERROR;
}

static motor_ttl_result_t motor_ttl_send_read(motor_ttl_motor_t *motor,
                                              uint8_t function)
{
    const uint8_t command[2] = {function, MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_init(motor_ttl_bus_t *bus,
                                  UART_HandleTypeDef *huart,
                                  uint32_t tx_timeout_ms)
{
    if ((bus == NULL) || (huart == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    memset(bus, 0, sizeof(*bus));
    bus->huart = huart;
    bus->tx_timeout_ms = tx_timeout_ms;
    s_active_bus = bus;

    return motor_ttl_start_receive(bus);
}

motor_ttl_result_t motor_ttl_motor_init(motor_ttl_motor_t *motor,
                                        motor_ttl_bus_t *bus,
                                        uint8_t id)
{
    if ((motor == NULL) || (bus == NULL) || (id == 0U) ||
        (bus->motor_count >= MOTOR_TTL_MAX_COUNT) ||
        (motor_ttl_find_motor(bus, id) != NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;
    motor->last_reply = MOTOR_TTL_REPLY_NONE;
    bus->motors[bus->motor_count] = motor;
    bus->motor_count++;
    return MOTOR_TTL_OK;
}

motor_ttl_bus_t *motor_ttl_get_bus(void)
{
    return s_active_bus;
}

motor_ttl_motor_t *motor_ttl_get_motor(uint8_t id)
{
    return motor_ttl_find_motor(s_active_bus, id);
}

motor_ttl_result_t motor_ttl_start_receive(motor_ttl_bus_t *bus)
{
    if ((bus == NULL) || (bus->huart == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    bus->rx_length = 0U;
    bus->rx_expected_length = 0U;
    bus->rx_byte = 0U;

    return (HAL_UART_Receive_IT(bus->huart, &bus->rx_byte, 1U) == HAL_OK) ?
           MOTOR_TTL_OK : MOTOR_TTL_ERROR;
}

motor_ttl_result_t motor_ttl_enable(motor_ttl_motor_t *motor,
                                    bool enable,
                                    motor_ttl_sync_t sync)
{
    const uint8_t command[5] = {MOTOR_TTL_CMD_ENABLE, 0xABU,
                                enable ? 1U : 0U, (uint8_t)sync,
                                MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_TTL_EXECUTE_SYNC))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_set_speed(motor_ttl_motor_t *motor,
                                       int16_t speed_rpm,
                                       uint8_t acceleration,
                                       motor_ttl_sync_t sync)
{
    uint8_t command[7];
    uint16_t speed_abs;
    int32_t speed_value;
    motor_ttl_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_TTL_EXECUTE_SYNC))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    speed_value = speed_rpm;
    speed_abs = (uint16_t)((speed_value < 0) ? -speed_value : speed_value);
    if (speed_abs > MOTOR_TTL_MAX_SPEED_RPM)
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    command[0] = MOTOR_TTL_CMD_SPEED;
    command[1] = (speed_rpm < 0) ? MOTOR_TTL_DIRECTION_CCW : MOTOR_TTL_DIRECTION_CW;
    motor_ttl_write_u16_be(&command[2], speed_abs);
    command[4] = acceleration;
    command[5] = (uint8_t)sync;
    command[6] = MOTOR_TTL_FIXED_CHECKSUM;

    result = motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
    if (result == MOTOR_TTL_OK)
    {
        motor->target_speed_rpm = speed_rpm;
    }
    return result;
}

motor_ttl_result_t motor_ttl_set_speeds_sync(motor_ttl_motor_t *const motors[],
                                             const int16_t speed_rpm[],
                                             uint8_t motor_count,
                                             uint8_t acceleration)
{
    uint8_t index;
    motor_ttl_result_t result;

    if ((motors == NULL) || (speed_rpm == NULL) || (motor_count == 0U) ||
        (motor_count > MOTOR_TTL_MAX_COUNT) || (motors[0] == NULL) ||
        (motors[0]->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    /*
     * Emm 多机同步的正确格式不是 AA 长帧，而是：
     * 1. 分别发送每台电机的速度命令，并将同步标志设为 1；
     * 2. 使用广播地址发送 FF 66 6B；
     * 3. 所有已经缓存同步命令的电机同时开始执行。
     */
    for (index = 0U; index < motor_count; index++)
    {
        if ((motors[index] == NULL) ||
            (motors[index]->bus != motors[0]->bus) ||
            (speed_rpm[index] > (int16_t)MOTOR_TTL_MAX_SPEED_RPM) ||
            (speed_rpm[index] < -(int16_t)MOTOR_TTL_MAX_SPEED_RPM))
        {
            return MOTOR_TTL_INVALID_PARAM;
        }

        result = motor_ttl_set_speed(motors[index],
                                     speed_rpm[index],
                                     acceleration,
                                     MOTOR_TTL_EXECUTE_SYNC);
        if (result != MOTOR_TTL_OK)
        {
            return result;
        }

        /* 必须与下一条命令拉开间隔，避免多机同步命令粘包。 */
        HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);
    }

    /* 触发帧也要与最后一条缓存命令分开。 */
    HAL_Delay(MOTOR_TTL_INTER_COMMAND_DELAY_MS);
    return motor_ttl_trigger_sync(motors[0]->bus);
}

motor_ttl_result_t motor_ttl_set_position(motor_ttl_motor_t *motor,
                                          int64_t pulse,
                                          uint16_t speed_rpm,
                                          uint8_t acceleration,
                                          motor_ttl_position_mode_t mode,
                                          motor_ttl_sync_t sync)
{
    uint8_t command[12];
    uint32_t pulse_abs;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (speed_rpm > MOTOR_TTL_MAX_SPEED_RPM) ||
        (mode > MOTOR_TTL_POSITION_RELATIVE_CURRENT) ||
        (sync > MOTOR_TTL_EXECUTE_SYNC) ||
        (pulse > (int64_t)UINT32_MAX) ||
        (pulse < -(int64_t)UINT32_MAX))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    pulse_abs = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;
    command[0] = MOTOR_TTL_CMD_POSITION;
    command[1] = (pulse < 0) ? MOTOR_TTL_DIRECTION_CCW : MOTOR_TTL_DIRECTION_CW;
    motor_ttl_write_u16_be(&command[2], speed_rpm);
    command[4] = acceleration;
    motor_ttl_write_u32_be(&command[5], pulse_abs);
    command[9] = (uint8_t)mode;
    command[10] = (uint8_t)sync;
    command[11] = MOTOR_TTL_FIXED_CHECKSUM;

    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_stop(motor_ttl_motor_t *motor,
                                  motor_ttl_sync_t sync)
{
    const uint8_t command[4] = {MOTOR_TTL_CMD_STOP, 0x98U,
                                (uint8_t)sync, MOTOR_TTL_FIXED_CHECKSUM};
    motor_ttl_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_TTL_EXECUTE_SYNC))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    result = motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
    if (result == MOTOR_TTL_OK)
    {
        motor->target_speed_rpm = 0;
    }
    return result;
}

motor_ttl_result_t motor_ttl_stop_all(motor_ttl_bus_t *bus)
{
    const uint8_t command[4] = {MOTOR_TTL_CMD_STOP, 0x98U,
                                MOTOR_TTL_EXECUTE_NOW, MOTOR_TTL_FIXED_CHECKSUM};
    uint8_t index;
    motor_ttl_result_t result;

    if (bus == NULL)
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    result = motor_ttl_send_command(bus, 0U, command, sizeof(command));
    if (result == MOTOR_TTL_OK)
    {
        for (index = 0U; index < bus->motor_count; index++)
        {
            if (bus->motors[index] != NULL)
            {
                bus->motors[index]->target_speed_rpm = 0;
            }
        }
    }
    return result;
}

motor_ttl_result_t motor_ttl_trigger_sync(motor_ttl_bus_t *bus)
{
    const uint8_t command[3] = {MOTOR_TTL_CMD_TRIGGER_SYNC, 0x66U,
                                MOTOR_TTL_FIXED_CHECKSUM};

    return (bus == NULL) ? MOTOR_TTL_INVALID_PARAM :
           motor_ttl_send_command(bus, 0U, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_clear_position(motor_ttl_motor_t *motor)
{
    const uint8_t command[3] = {MOTOR_TTL_CMD_CLEAR_POSITION, 0x6DU,
                                MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_clear_protection(motor_ttl_motor_t *motor)
{
    const uint8_t command[3] = {MOTOR_TTL_CMD_CLEAR_PROTECTION, 0x52U,
                                MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_home(motor_ttl_motor_t *motor,
                                  uint8_t home_mode,
                                  motor_ttl_sync_t sync)
{
    const uint8_t command[4] = {MOTOR_TTL_CMD_HOME, home_mode,
                                (uint8_t)sync, MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL) || (home_mode > 5U) ||
        (sync > MOTOR_TTL_EXECUTE_SYNC))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_home_abort(motor_ttl_motor_t *motor)
{
    const uint8_t command[3] = {MOTOR_TTL_CMD_HOME_ABORT, 0x48U,
                                MOTOR_TTL_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_set_heartbeat(motor_ttl_motor_t *motor,
                                           uint32_t heartbeat_ms,
                                           bool save_to_flash)
{
    uint8_t command[8];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    command[0] = MOTOR_TTL_CMD_SET_HEARTBEAT;
    command[1] = 0x38U;
    command[2] = save_to_flash ? 1U : 0U;
    motor_ttl_write_u32_be(&command[3], heartbeat_ms);
    command[7] = MOTOR_TTL_FIXED_CHECKSUM;
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_set_periodic_report(motor_ttl_motor_t *motor,
                                                 motor_ttl_information_t information,
                                                 uint16_t period_ms)
{
    uint8_t command[6];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_TTL_INVALID_PARAM;
    }

    command[0] = MOTOR_TTL_CMD_PERIODIC_REPORT;
    command[1] = 0x18U;
    command[2] = (uint8_t)information;
    motor_ttl_write_u16_be(&command[3], period_ms);
    command[5] = MOTOR_TTL_FIXED_CHECKSUM;
    return motor_ttl_send_command(motor->bus, motor->id, command, sizeof(command));
}

motor_ttl_result_t motor_ttl_read_input_pulses(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_INPUT_PULSES);
}

motor_ttl_result_t motor_ttl_read_speed(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_SPEED);
}

motor_ttl_result_t motor_ttl_read_position(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_POSITION);
}

motor_ttl_result_t motor_ttl_read_position_error(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_POSITION_ERR);
}

motor_ttl_result_t motor_ttl_read_state(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_STATE);
}

motor_ttl_result_t motor_ttl_read_home_state(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_HOME_STATE);
}

motor_ttl_result_t motor_ttl_read_bus_voltage(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_BUS_VOLTAGE);
}

motor_ttl_result_t motor_ttl_read_bus_current(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_BUS_CURRENT);
}

motor_ttl_result_t motor_ttl_read_phase_current(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_PHASE_CURRENT);
}

motor_ttl_result_t motor_ttl_read_encoder(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_ENCODER);
}

motor_ttl_result_t motor_ttl_read_temperature(motor_ttl_motor_t *motor)
{
    return motor_ttl_send_read(motor, MOTOR_TTL_CMD_READ_TEMPERATURE);
}

motor_ttl_result_t motor_ttl_process_frame(motor_ttl_bus_t *bus,
                                           const uint8_t *data,
                                           uint8_t length)
{
    motor_ttl_motor_t *motor;
    uint8_t function;
    uint32_t value_u32;
    int32_t value_s32;

    if ((bus == NULL) || (data == NULL) || (length < 4U) ||
        (length > MOTOR_TTL_MAX_RX_LENGTH) ||
        (data[length - 1U] != MOTOR_TTL_FIXED_CHECKSUM))
    {
        return MOTOR_TTL_ERROR;
    }

    motor = motor_ttl_find_motor(bus, data[0]);
    if (motor == NULL)
    {
        return MOTOR_TTL_NOT_FOUND;
    }

    function = data[1];
    if (motor_ttl_expected_length(function) != length)
    {
        return MOTOR_TTL_ERROR;
    }

    motor->last_function = function;
    motor->last_update_ms = HAL_GetTick();

    switch (function)
    {
        case MOTOR_TTL_CMD_READ_BUS_VOLTAGE:
            motor->bus_voltage_mv = motor_ttl_read_u16_be(&data[2]);
            break;

        case MOTOR_TTL_CMD_READ_BUS_CURRENT:
            motor->bus_current_ma = motor_ttl_read_u16_be(&data[2]);
            break;

        case MOTOR_TTL_CMD_READ_PHASE_CURRENT:
            motor->phase_current_ma = motor_ttl_read_u16_be(&data[2]);
            break;

        case MOTOR_TTL_CMD_READ_ENCODER:
            motor->encoder_raw = motor_ttl_read_u16_be(&data[2]);
            break;

        case MOTOR_TTL_CMD_READ_INPUT_PULSES:
            value_u32 = motor_ttl_read_u32_be(&data[3]);
            motor->actual_input_pulses = motor_ttl_signed_value(data[2], value_u32);
            motor->actual_input_pulses_valid = 1U;
            break;

        case MOTOR_TTL_CMD_READ_SPEED:
            value_u32 = motor_ttl_read_u16_be(&data[3]);
            value_s32 = motor_ttl_signed_value(data[2], value_u32);
            motor->actual_speed_rpm = (int16_t)value_s32;
            break;

        case MOTOR_TTL_CMD_READ_POSITION:
            value_u32 = motor_ttl_read_u32_be(&data[3]);
            value_s32 = motor_ttl_signed_value(data[2], value_u32);
            motor->actual_position_raw = value_s32;
            motor->actual_position_angle_cdeg = motor_ttl_angle_cdeg(value_s32);
            motor->actual_position_valid = 1U;
            break;

        case MOTOR_TTL_CMD_READ_POSITION_ERR:
            value_u32 = motor_ttl_read_u32_be(&data[3]);
            value_s32 = motor_ttl_signed_value(data[2], value_u32);
            motor->position_error_raw = value_s32;
            motor->position_error_angle_cdeg = motor_ttl_angle_cdeg(value_s32);
            break;

        case MOTOR_TTL_CMD_READ_TEMPERATURE:
            motor->temperature_c = (data[2] == 0U) ?
                                   -(int8_t)data[3] : (int8_t)data[3];
            break;

        case MOTOR_TTL_CMD_READ_STATE:
            motor->state_flags = data[2];
            break;

        case MOTOR_TTL_CMD_READ_HOME_STATE:
            motor->home_flags = data[2];
            break;

        case MOTOR_TTL_CMD_READ_ALL_STATE:
            motor->home_flags = data[2];
            motor->state_flags = data[3];
            break;

        default:
            /* 控制命令返回：地址 + 原功能码 + 应答码 + 6B。 */
            motor->last_reply = (motor_ttl_reply_t)data[2];
            break;
    }

    return MOTOR_TTL_OK;
}

static void motor_ttl_reset_parser(motor_ttl_bus_t *bus)
{
    bus->rx_length = 0U;
    bus->rx_expected_length = 0U;
}

static void motor_ttl_feed_byte(motor_ttl_bus_t *bus, uint8_t data)
{
    uint8_t expected;

    if (bus->rx_length == 0U)
    {
        /* 只接受已经注册的地址，避免把模块噪声当成帧头。 */
        if (motor_ttl_find_motor(bus, data) == NULL)
        {
            return;
        }
        bus->rx_buffer[0] = data;
        bus->rx_length = 1U;
        return;
    }

    if (bus->rx_length == 1U)
    {
        expected = motor_ttl_expected_length(data);
        if (expected == 0U)
        {
            motor_ttl_reset_parser(bus);
            return;
        }
        bus->rx_buffer[1] = data;
        bus->rx_length = 2U;
        bus->rx_expected_length = expected;
        return;
    }

    if (bus->rx_length >= MOTOR_TTL_MAX_RX_LENGTH)
    {
        motor_ttl_reset_parser(bus);
        return;
    }

    bus->rx_buffer[bus->rx_length] = data;
    bus->rx_length++;
    if (bus->rx_length >= bus->rx_expected_length)
    {
        (void)motor_ttl_process_frame(bus, bus->rx_buffer, bus->rx_length);
        motor_ttl_reset_parser(bus);
    }
}

void motor_ttl_on_rx_complete(UART_HandleTypeDef *huart)
{
    if ((s_active_bus == NULL) || (huart != s_active_bus->huart))
    {
        return;
    }

    motor_ttl_feed_byte(s_active_bus, s_active_bus->rx_byte);
    (void)HAL_UART_Receive_IT(s_active_bus->huart, &s_active_bus->rx_byte, 1U);
}

void motor_ttl_on_error(UART_HandleTypeDef *huart)
{
    if ((s_active_bus == NULL) || (huart != s_active_bus->huart))
    {
        return;
    }

    motor_ttl_reset_parser(s_active_bus);
    (void)HAL_UART_Receive_IT(s_active_bus->huart, &s_active_bus->rx_byte, 1U);
}
