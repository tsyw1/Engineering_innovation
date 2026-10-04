//
// Y42_V2.0 / X42S CAN + Emm5.0 电机驱动实现。
//

#include "motor_can.h"

#include <string.h>

#define MOTOR_CMD_CLEAR_POSITION        0x0AU
#define MOTOR_CMD_CLEAR_PROTECTION      0x0EU
#define MOTOR_CMD_SET_ADDRESS           0xAEU
#define MOTOR_CMD_HOME                  0x9AU
#define MOTOR_CMD_HOME_ABORT            0x9CU
#define MOTOR_CMD_PERIODIC_REPORT       0x11U
#define MOTOR_CMD_BROADCAST_READ_ID     0x15U
#define MOTOR_CMD_READ_BUS_VOLTAGE      0x24U
#define MOTOR_CMD_READ_BUS_CURRENT      0x26U
#define MOTOR_CMD_READ_PHASE_CURRENT    0x27U
#define MOTOR_CMD_READ_ENCODER          0x31U
#define MOTOR_CMD_READ_SPEED            0x35U
#define MOTOR_CMD_READ_POSITION         0x36U
#define MOTOR_CMD_READ_POSITION_ERROR   0x37U
#define MOTOR_CMD_READ_BATTERY_VOLTAGE  0x38U
#define MOTOR_CMD_READ_TEMPERATURE      0x39U
#define MOTOR_CMD_READ_STATE            0x3AU
#define MOTOR_CMD_READ_HOME_STATE       0x3BU
#define MOTOR_CMD_READ_ALL_STATE        0x3CU
#define MOTOR_CMD_SET_HEARTBEAT         0x68U
#define MOTOR_CMD_MULTI                 0xAAU
#define MOTOR_CMD_ENABLE                0xF3U
#define MOTOR_CMD_SPEED                 0xF6U
#define MOTOR_CMD_POSITION              0xFDU
#define MOTOR_CMD_STOP                  0xFEU
#define MOTOR_CMD_TRIGGER_SYNC          0xFFU
#define MOTOR_INIT_ENABLE_SETTLE_MS    100U
#define MOTOR_INIT_DISABLE_SETTLE_MS   100U

static motor_bus_t *s_active_bus;

static motor_result_t motor_run_init_sequence(motor_t *motor);

static motor_result_t motor_bus_lock(motor_bus_t *bus)
{
    uint32_t primask;

    if (bus == NULL)
    {
        return MOTOR_INVALID_PARAM;
    }

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

static void motor_bus_unlock(motor_bus_t *bus)
{
    uint32_t primask;

    if (bus == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    bus->tx_busy = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint16_t motor_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static uint32_t motor_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           data[3];
}

static void motor_write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

static void motor_write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

static motor_t *motor_bus_find_motor(motor_bus_t *bus, uint8_t id)
{
    uint8_t index;

    if (bus == NULL)
    {
        return NULL;
    }

    for (index = 0U; index < bus->motor_count; index++)
    {
        if ((bus->motors[index] != NULL) &&
            (bus->motors[index]->id == id))
        {
            return bus->motors[index];
        }
    }
    return NULL;
}

HAL_StatusTypeDef motor_can_hal_send_frame(CAN_HandleTypeDef *hcan,
                                           uint32_t extended_id,
                                           const uint8_t *data,
                                           uint8_t length,
                                           uint32_t timeout_ms)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t start_tick;

    if ((hcan == NULL) || (data == NULL) || (length == 0U) ||
        (length > MOTOR_CAN_FRAME_DATA_SIZE) ||
        (extended_id > 0x1FFFFFFFU))
    {
        return HAL_ERROR;
    }

    tx_header.ExtId = extended_id;
    tx_header.StdId = 0U;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(hcan,
                                &tx_header,
                                (uint8_t *)data,
                                &tx_mailbox);
}

motor_result_t motor_bus_init(motor_bus_t *bus,
                              CAN_HandleTypeDef *hcan,
                              motor_can_send_callback_t send_can_frame,
                              uint32_t tx_timeout_ms)
{
    if ((bus == NULL) || (hcan == NULL) || (send_can_frame == NULL) ||
        (tx_timeout_ms == 0U))
    {
        return MOTOR_INVALID_PARAM;
    }

    memset(bus, 0, sizeof(*bus));
    bus->hcan = hcan;
    bus->send_can_frame = send_can_frame;
    bus->tx_timeout_ms = tx_timeout_ms;
    s_active_bus = bus;
    return MOTOR_OK;
}

motor_result_t motor_bus_start(motor_bus_t *bus)
{
    CAN_FilterTypeDef filter;

    if ((bus == NULL) || (bus->hcan == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /* 调试阶段接收全部 CAN 帧，后续可按电机地址收紧过滤范围。 */
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(bus->hcan, &filter) != HAL_OK)
    {
        return MOTOR_ERROR;
    }
    if (HAL_CAN_Start(bus->hcan) != HAL_OK)
    {
        return MOTOR_ERROR;
    }
    if (HAL_CAN_ActivateNotification(bus->hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR |
                                     CAN_IT_BUSOFF) != HAL_OK)
    {
        (void)HAL_CAN_Stop(bus->hcan);
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}

motor_result_t motor_bus_stop(motor_bus_t *bus)
{
    if ((bus == NULL) || (bus->hcan == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    (void)HAL_CAN_DeactivateNotification(bus->hcan,
                                         CAN_IT_RX_FIFO0_MSG_PENDING |
                                         CAN_IT_ERROR |
                                         CAN_IT_BUSOFF);
    return (HAL_CAN_Stop(bus->hcan) == HAL_OK) ? MOTOR_OK : MOTOR_ERROR;
}

static motor_result_t motor_bus_send_command(motor_bus_t *bus,
                                              uint8_t address,
                                              const uint8_t *command,
                                              uint8_t command_length)
{
    motor_result_t result;
    HAL_StatusTypeDef hal_result;
    uint8_t frame[MOTOR_CAN_FRAME_DATA_SIZE];
    uint8_t packet = 0U;
    uint8_t offset = 0U;
    uint8_t copy_length;
    uint8_t frame_length;
    uint32_t extended_id;

    if ((bus == NULL) || (bus->hcan == NULL) ||
        (bus->send_can_frame == NULL) || (command == NULL) ||
        (command_length < 2U) ||
        (command_length > MOTOR_CAN_COMMAND_MAX_SIZE))
    {
        return MOTOR_INVALID_PARAM;
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
            copy_length = (command_length > MOTOR_CAN_FRAME_DATA_SIZE) ?
                          MOTOR_CAN_FRAME_DATA_SIZE : command_length;
            memcpy(frame, command, copy_length);
            frame_length = copy_length;
        }
        else
        {
            frame[0] = command[0];
            copy_length = (uint8_t)(command_length - offset);
            if (copy_length > (MOTOR_CAN_FRAME_DATA_SIZE - 1U))
            {
                copy_length = MOTOR_CAN_FRAME_DATA_SIZE - 1U;
            }
            memcpy(&frame[1], &command[offset], copy_length);
            frame_length = (uint8_t)(copy_length + 1U);
        }

        extended_id = ((uint32_t)address << 8U) | packet;
        hal_result = bus->send_can_frame(bus->hcan,
                                         extended_id,
                                         frame,
                                         frame_length,
                                         bus->tx_timeout_ms);
        if (hal_result != HAL_OK)
        {
            motor_bus_unlock(bus);
            return (hal_result == HAL_BUSY) ? MOTOR_BUSY : MOTOR_ERROR;
        }

        offset = (uint8_t)(offset + copy_length);
        packet++;
    }

    motor_bus_unlock(bus);
    return MOTOR_OK;
}

static motor_result_t motor_send_read_command(motor_t *motor,
                                               uint8_t function)
{
    const uint8_t command[2] = {function, MOTOR_FIXED_CHECKSUM};

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_broadcast_read_id(motor_bus_t *bus)
{
    const uint8_t command[2] =
    {
        MOTOR_CMD_BROADCAST_READ_ID,
        MOTOR_FIXED_CHECKSUM
    };

    if (bus == NULL)
    {
        return MOTOR_INVALID_PARAM;
    }

    bus->discovered_id = 0U;
    bus->discovered_id_valid = 0U;
    return motor_bus_send_command(bus,
                                  MOTOR_BROADCAST_ADDRESS,
                                  command,
                                  sizeof(command));
}

bool motor_bus_get_discovered_id(const motor_bus_t *bus, uint8_t *id)
{
    if ((bus == NULL) || (id == NULL) ||
        (bus->discovered_id_valid == 0U))
    {
        return false;
    }

    *id = bus->discovered_id;
    return true;
}

motor_result_t motor_init(motor_t *motor, motor_bus_t *bus, uint8_t id)
{
    if ((motor == NULL) || (bus == NULL) || (id == 0U))
    {
        return MOTOR_INVALID_PARAM;
    }
    if ((bus->hcan == NULL) ||
        (bus->motor_count >= MOTOR_MAX_COUNT) ||
        (motor_bus_find_motor(bus, id) != NULL))
    {
        return MOTOR_INVALID_PARAM;
    }
    if (HAL_CAN_GetState(bus->hcan) != HAL_CAN_STATE_LISTENING)
    {
        return MOTOR_NOT_READY;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->id = id;
    motor->last_reply = MOTOR_REPLY_NONE;
    bus->motors[bus->motor_count++] = motor;

    /* 统一要求先启动 CAN，再注册电机并执行上电恢复流程。 */
    {
        motor_result_t result = motor_run_init_sequence(motor);
        if (result != MOTOR_OK)
        {
            bus->motors[--bus->motor_count] = NULL;
            memset(motor, 0, sizeof(*motor));
            return result;
        }
    }

    return MOTOR_OK;
}

motor_result_t motor_enable(motor_t *motor, bool enable, motor_sync_t sync)
{
    uint8_t command[5];

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    command[0] = MOTOR_CMD_ENABLE;
    command[1] = 0xABU;
    command[2] = enable ? 1U : 0U;
    command[3] = (uint8_t)sync;
    command[4] = MOTOR_FIXED_CHECKSUM;
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

static motor_result_t motor_run_init_sequence(motor_t *motor)
{
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    result = motor_enable(motor, true, MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return result;
    }
    HAL_Delay(MOTOR_INIT_ENABLE_SETTLE_MS);

    result = motor_enable(motor, false, MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return result;
    }
    HAL_Delay(MOTOR_INIT_DISABLE_SETTLE_MS);

    result = motor_enable(motor, true, MOTOR_EXECUTE_NOW);
    if (result != MOTOR_OK)
    {
        return result;
    }
    HAL_Delay(MOTOR_INIT_ENABLE_SETTLE_MS);

    return MOTOR_OK;
}

motor_result_t motor_set_speed(motor_t *motor,
                               int16_t speed_rpm,
                               uint8_t acceleration,
                               motor_sync_t sync)
{
    uint8_t command[7];
    uint16_t speed_abs;
    int32_t speed_value;
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    speed_value = speed_rpm;
    speed_abs = (uint16_t)((speed_value < 0) ? -speed_value : speed_value);
    if (speed_abs > MOTOR_EMM_MAX_SPEED_RPM)
    {
        return MOTOR_INVALID_PARAM;
    }

    command[0] = MOTOR_CMD_SPEED;
    command[1] = (speed_rpm < 0) ?
                 MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
    motor_write_u16_be(&command[2], speed_abs);
    command[4] = acceleration;
    command[5] = (uint8_t)sync;
    command[6] = MOTOR_FIXED_CHECKSUM;

    result = motor_bus_send_command(motor->bus,
                                    motor->id,
                                    command,
                                    sizeof(command));
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
    uint8_t command[MOTOR_CAN_COMMAND_MAX_SIZE];
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

    total_length = (uint16_t)(5U + ((uint16_t)motor_count * 8U));
    command[0] = MOTOR_CMD_MULTI;
    motor_write_u16_be(&command[1], total_length);

    for (index = 0U; index < motor_count; index++)
    {
        uint16_t speed_abs;
        int32_t speed_value;

        if ((motors[index] == NULL) ||
            (motors[index]->bus != motors[0]->bus))
        {
            return MOTOR_INVALID_PARAM;
        }

        speed_value = speed_rpm[index];
        speed_abs = (uint16_t)((speed_value < 0) ?
                              -speed_value : speed_value);
        if (speed_abs > MOTOR_EMM_MAX_SPEED_RPM)
        {
            return MOTOR_INVALID_PARAM;
        }

        command[offset++] = motors[index]->id;
        command[offset++] = MOTOR_CMD_SPEED;
        command[offset++] = (speed_rpm[index] < 0) ?
                            MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
        motor_write_u16_be(&command[offset], speed_abs);
        offset = (uint8_t)(offset + 2U);
        command[offset++] = acceleration;
        command[offset++] = MOTOR_EXECUTE_NOW;
        command[offset++] = MOTOR_FIXED_CHECKSUM;
    }

    command[offset++] = MOTOR_FIXED_CHECKSUM;
    result = motor_bus_send_command(motors[0]->bus,
                                    MOTOR_BROADCAST_ADDRESS,
                                    command,
                                    offset);
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

    pulse_abs = (pulse < 0) ? (uint32_t)(-pulse) : (uint32_t)pulse;
    command[0] = MOTOR_CMD_POSITION;
    command[1] = (pulse < 0) ?
                 MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW;
    motor_write_u16_be(&command[2], speed_rpm);
    command[4] = acceleration;
    motor_write_u32_be(&command[5], pulse_abs);
    command[9] = (uint8_t)mode;
    command[10] = (uint8_t)sync;
    command[11] = MOTOR_FIXED_CHECKSUM;
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_stop(motor_t *motor, motor_sync_t sync)
{
    uint8_t command[4];
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    command[0] = MOTOR_CMD_STOP;
    command[1] = 0x98U;
    command[2] = (uint8_t)sync;
    command[3] = MOTOR_FIXED_CHECKSUM;
    result = motor_bus_send_command(motor->bus,
                                    motor->id,
                                    command,
                                    sizeof(command));
    if (result == MOTOR_OK)
    {
        motor->target_speed_rpm = 0;
    }
    return result;
}

motor_result_t motor_stop_all(motor_bus_t *bus)
{
    const uint8_t command[4] =
    {
        MOTOR_CMD_STOP,
        0x98U,
        MOTOR_EXECUTE_NOW,
        MOTOR_FIXED_CHECKSUM
    };
    uint8_t index;
    motor_result_t result;

    if (bus == NULL)
    {
        return MOTOR_INVALID_PARAM;
    }

    result = motor_bus_send_command(bus,
                                    MOTOR_BROADCAST_ADDRESS,
                                    command,
                                    sizeof(command));
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
    const uint8_t command[3] =
    {
        MOTOR_CMD_TRIGGER_SYNC,
        0x66U,
        MOTOR_FIXED_CHECKSUM
    };

    return (bus == NULL) ? MOTOR_INVALID_PARAM :
           motor_bus_send_command(bus,
                                  MOTOR_BROADCAST_ADDRESS,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_clear_position(motor_t *motor)
{
    const uint8_t command[3] =
    {
        MOTOR_CMD_CLEAR_POSITION,
        0x6DU,
        MOTOR_FIXED_CHECKSUM
    };

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_clear_protection(motor_t *motor)
{
    const uint8_t command[3] =
    {
        MOTOR_CMD_CLEAR_PROTECTION,
        0x52U,
        MOTOR_FIXED_CHECKSUM
    };

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_set_address(motor_t *motor,
                                 uint8_t new_address,
                                 bool save_to_flash)
{
    const uint8_t command[5] =
    {
        MOTOR_CMD_SET_ADDRESS,
        0x4BU,
        save_to_flash ? 1U : 0U,
        new_address,
        MOTOR_FIXED_CHECKSUM
    };
    motor_result_t result;

    if ((motor == NULL) || (motor->bus == NULL) ||
        (new_address == MOTOR_BROADCAST_ADDRESS))
    {
        return MOTOR_INVALID_PARAM;
    }

    result = motor_bus_send_command(motor->bus,
                                    motor->id,
                                    command,
                                    sizeof(command));
    if (result == MOTOR_OK)
    {
        /* 命令发送成功后更新本地对象，后续查询使用新地址。 */
        motor->id = new_address;
    }
    return result;
}

motor_result_t motor_home(motor_t *motor,
                          uint8_t home_mode,
                          motor_sync_t sync)
{
    const uint8_t command[4] =
    {
        MOTOR_CMD_HOME,
        home_mode,
        (uint8_t)sync,
        MOTOR_FIXED_CHECKSUM
    };

    if ((motor == NULL) || (motor->bus == NULL) ||
        (home_mode > 5U) || (sync > MOTOR_EXECUTE_SYNC))
    {
        return MOTOR_INVALID_PARAM;
    }

    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_home_abort(motor_t *motor)
{
    const uint8_t command[3] =
    {
        MOTOR_CMD_HOME_ABORT,
        0x48U,
        MOTOR_FIXED_CHECKSUM
    };

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_set_heartbeat(motor_t *motor,
                                   uint32_t heartbeat_ms,
                                   bool save_to_flash)
{
    uint8_t command[8];

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return MOTOR_INVALID_PARAM;
    }

    command[0] = MOTOR_CMD_SET_HEARTBEAT;
    command[1] = 0x38U;
    command[2] = save_to_flash ? 1U : 0U;
    motor_write_u32_be(&command[3], heartbeat_ms);
    command[7] = MOTOR_FIXED_CHECKSUM;
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
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

    command[0] = MOTOR_CMD_PERIODIC_REPORT;
    command[1] = 0x18U;
    command[2] = (uint8_t)information;
    motor_write_u16_be(&command[3], period_ms);
    command[5] = MOTOR_FIXED_CHECKSUM;
    return motor_bus_send_command(motor->bus,
                                  motor->id,
                                  command,
                                  sizeof(command));
}

motor_result_t motor_read_speed(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_SPEED);
}

motor_result_t motor_read_encoder(motor_t *motor)
{
    return motor_send_read_command(motor, MOTOR_CMD_READ_ENCODER);
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
        (length > MOTOR_CAN_FRAME_DATA_SIZE) ||
        (extended_id > 0x1FFFFFFFU))
    {
        return MOTOR_INVALID_PARAM;
    }

    address = (uint8_t)(extended_id >> 8U);
    packet = (uint8_t)extended_id;
    if (packet != 0U)
    {
        return MOTOR_ERROR;
    }

    if (data[length - 1U] != MOTOR_FIXED_CHECKSUM)
    {
        return MOTOR_ERROR;
    }

    function = data[0];

    /* 广播读地址返回：扩展 ID 中带地址，数据为 15 Addr 6B。 */
    if (function == MOTOR_CMD_BROADCAST_READ_ID)
    {
        /*
         * CAN 协议通常返回 15 6B（地址已经在扩展帧 ID 中）；
         * 某些固件会保留 TTL 格式中的地址字节，返回 15 Addr 6B。
         */
        if ((address == 0U) ||
            ((length != 2U) && (length != 3U)) ||
            ((length == 3U) && (data[1] != address)))
        {
            return MOTOR_ERROR;
        }

        bus->discovered_id = address;
        bus->discovered_id_valid = 1U;
        return MOTOR_OK;
    }

    motor = motor_bus_find_motor(bus, address);
    if (motor == NULL)
    {
        return MOTOR_NOT_FOUND;
    }

    motor->last_function = function;
    motor->last_update_ms = HAL_GetTick();

    switch (function)
    {
        case MOTOR_CMD_READ_ENCODER:
        {
            uint16_t encoder_value;

            if (length != 4U) return MOTOR_ERROR;
            encoder_value = motor_read_u16_be(&data[1]);
            if (motor->encoder_valid != 0U)
            {
                /* int16_t 差值可自然处理 0/65535 的单圈回绕。 */
                motor->encoder_delta = (int16_t)(encoder_value -
                                                 motor->encoder_raw);
            }
            else
            {
                motor->encoder_delta = 0;
                motor->encoder_valid = 1U;
            }
            motor->encoder_raw = encoder_value;
            motor->encoder_update_ms = motor->last_update_ms;
            motor->encoder_update_count++;
            break;
        }

        case MOTOR_CMD_READ_BUS_VOLTAGE:
            if (length != 4U) return MOTOR_ERROR;
            motor->bus_voltage_mv = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_BUS_CURRENT:
            if (length != 4U) return MOTOR_ERROR;
            motor->bus_current_ma = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_PHASE_CURRENT:
            if (length != 4U) return MOTOR_ERROR;
            motor->phase_current_ma = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_SPEED:
            if (length != 5U) return MOTOR_ERROR;
            value_s32 = motor_read_u16_be(&data[2]);
            motor->actual_speed_rpm = (int16_t)((data[1] == 0U) ?
                                      value_s32 : -value_s32);
            motor->speed_update_ms = motor->last_update_ms;
            break;

        case MOTOR_CMD_READ_POSITION:
            if (length != 7U) return MOTOR_ERROR;
            value_u32 = motor_read_u32_be(&data[2]);
            value_s64 = value_u32;
            motor->actual_position_pulse = (data[1] == 0U) ?
                                           value_s64 : -value_s64;
            motor->position_update_ms = motor->last_update_ms;
            break;

        case MOTOR_CMD_READ_POSITION_ERROR:
            if (length != 7U) return MOTOR_ERROR;
            value_u32 = motor_read_u32_be(&data[2]);
            value_s64 = value_u32;
            motor->position_error_pulse = (data[1] == 0U) ?
                                          value_s64 : -value_s64;
            break;

        case MOTOR_CMD_READ_BATTERY_VOLTAGE:
            if (length != 4U) return MOTOR_ERROR;
            motor->battery_voltage_mv = motor_read_u16_be(&data[1]);
            break;

        case MOTOR_CMD_READ_TEMPERATURE:
            if (length != 4U) return MOTOR_ERROR;
            motor->temperature_c = (data[1] == 0U) ?
                                   -(int8_t)data[2] : (int8_t)data[2];
            break;

        case MOTOR_CMD_READ_STATE:
            if (length != 3U) return MOTOR_ERROR;
            motor->state_flags = data[1];
            break;

        case MOTOR_CMD_READ_HOME_STATE:
            if (length != 3U) return MOTOR_ERROR;
            motor->home_flags = data[1];
            break;

        case MOTOR_CMD_READ_ALL_STATE:
            if (length != 4U) return MOTOR_ERROR;
            motor->home_flags = data[1];
            motor->state_flags = data[2];
            break;

        default:
            if (length == 3U)
            {
                motor->last_reply = (motor_reply_t)data[1];
            }
            break;
    }

    return MOTOR_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[MOTOR_CAN_FRAME_DATA_SIZE];

    if ((s_active_bus == NULL) || (hcan != s_active_bus->hcan))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan,
                                 CAN_RX_FIFO0,
                                 &rx_header,
                                 rx_data) != HAL_OK)
        {
            s_active_bus->rx_error_count++;
            return;
        }

        s_active_bus->rx_frame_count++;
        if ((rx_header.IDE == CAN_ID_EXT) &&
            (rx_header.RTR == CAN_RTR_DATA))
        {
            if (motor_bus_process_can_frame(s_active_bus,
                                            rx_header.ExtId,
                                            rx_data,
                                            (uint8_t)rx_header.DLC) != MOTOR_OK)
            {
                s_active_bus->rx_error_count++;
            }
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if ((s_active_bus == NULL) || (hcan != s_active_bus->hcan))
    {
        return;
    }

    s_active_bus->last_can_error = HAL_CAN_GetError(hcan);
    s_active_bus->rx_error_count++;
}
