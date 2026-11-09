//
// 麦轮底盘控制实现，使用 motor_can 原生 CAN 电机接口。
//

#include "chassis_mecanum.h"

#include <math.h>
#include <string.h>

#define MECANUM_PI_F 3.14159265358979323846f
#define MECANUM_MOTOR_INIT_ATTEMPTS 10U
#define MECANUM_MOTOR_INIT_RETRY_DELAY_MS 100U
/* X42S/Y42 多机总线连续命令之间需要保留几毫秒处理时间。 */
#define MECANUM_CAN_COMMAND_GAP_MS 2U
#define MECANUM_SPEED_COMMAND_ATTEMPTS 3U
#define MECANUM_SPEED_COMMAND_RETRY_DELAY_MS 2U
/* Emm 位置命令拆成多帧 CAN 数据，多台电机连续发送时留出间隔。 */
#define MECANUM_POSITION_COMMAND_GAP_MS MECANUM_CAN_COMMAND_GAP_MS
/* 位置模式限制为约60 RPM，匹配任务层的路线速度。 */
#define MECANUM_POSITION_MAX_SPEED_MPS 0.246f

static float mecanum_abs_f(float value)
{
    return (value < 0.0f) ? -value : value;
}
static float mecanum_velocity_scale(float value)
{
    return (value > 0.0f) ? value : 1.0f;
}
static int16_t mecanum_round_rpm(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static uint16_t mecanum_position_speed_limit_rpm(
    const mecanum_chassis_t *chassis)
{
    float max_rpm;

    if ((chassis == NULL) || (chassis->config.wheel_radius_m <= 0.0f))
    {
        return 1U;
    }

    max_rpm = (MECANUM_POSITION_MAX_SPEED_MPS * 60.0f) /
              (2.0f * MECANUM_PI_F * chassis->config.wheel_radius_m);
    if (max_rpm < 1.0f)
    {
        return 1U;
    }
    if (max_rpm > (float)MOTOR_EMM_MAX_SPEED_RPM)
    {
        return MOTOR_EMM_MAX_SPEED_RPM;
    }
    return (uint16_t)max_rpm;
}

static bool mecanum_config_valid(const mecanum_config_t *config)
{
    uint8_t index;
    motor_bus_t *bus;

    if ((config == NULL) || (config->wheel_radius_m <= 0.0f) ||
        (config->half_length_m < 0.0f) || (config->half_width_m < 0.0f) ||
        (config->velocity_x_scale < 0.0f) ||
        (config->velocity_y_scale < 0.0f) ||
        (config->velocity_yaw_scale < 0.0f) ||
        (config->pulses_per_rev == 0U) || (config->max_wheel_rpm == 0U) ||
        (config->max_wheel_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return false;
    }

    bus = NULL;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if ((config->motor[index] == NULL) ||
            (config->motor[index]->bus == NULL) ||
            ((config->direction_sign[index] != 1) &&
             (config->direction_sign[index] != -1)))
        {
            return false;
        }

        if (bus == NULL)
        {
            bus = config->motor[index]->bus;
        }
        else if (config->motor[index]->bus != bus)
        {
            return false;
        }
    }

    return true;
}

static mecanum_result_t mecanum_send_wheel_rpm(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t motor_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;
    uint8_t attempt;
    motor_result_t result;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_rpm[index] = (int16_t)((int32_t)wheel_rpm[index] *
                                     chassis->config.direction_sign[index]);
    }

    /* 四台电机分别发送，避免多电机广播包在分包或某一路接收异常时
       导致单个车轮没有收到速度指令。 */
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        result = MOTOR_ERROR;
        for (attempt = 0U;
             attempt < MECANUM_SPEED_COMMAND_ATTEMPTS;
             attempt++)
        {
            result = motor_set_speed(chassis->config.motor[index],
                                     motor_rpm[index],
                                     chassis->config.acceleration,
                                     MOTOR_EXECUTE_NOW);
            if (result == MOTOR_OK)
            {
                break;
            }

            chassis->last_speed_command_motor_id =
                chassis->config.motor[index]->id;
            chassis->last_speed_command_result = result;
            if ((attempt + 1U) < MECANUM_SPEED_COMMAND_ATTEMPTS)
            {
                chassis->speed_command_retry_count++;
                HAL_Delay(MECANUM_SPEED_COMMAND_RETRY_DELAY_MS);
            }
        }
        if (result != MOTOR_OK)
        {
            chassis->speed_command_error_count++;
            return MECANUM_MOTOR_ERROR;
        }
        if (index < (MECANUM_WHEEL_COUNT - 1U))
        {
            HAL_Delay(MECANUM_CAN_COMMAND_GAP_MS);
        }
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->target_wheel_rpm[index] = wheel_rpm[index];
    }

    return MECANUM_OK;
}
mecanum_result_t mecanum_chassis_init(mecanum_chassis_t *chassis,
                                      const mecanum_config_t *config)
{
    if ((chassis == NULL) || !mecanum_config_valid(config))
    {
        return MECANUM_INVALID_PARAM;
    }

    memset(chassis, 0, sizeof(*chassis));
    memcpy(&chassis->config, config, sizeof(chassis->config));
    return MECANUM_OK;
}
mecanum_result_t mecanum_chassis_init_default(
    mecanum_chassis_t *chassis,
    motor_bus_t *bus,
    motor_t motors[MECANUM_WHEEL_COUNT])
{
    mecanum_config_t config;
    motor_result_t motor_result;
    uint8_t initial_motor_count;
    uint8_t index;
    uint8_t attempt;
    const uint8_t motor_ids[MECANUM_WHEEL_COUNT] =
    {
        MECANUM_DEFAULT_MOTOR_FL_ID,
        MECANUM_DEFAULT_MOTOR_FR_ID,
        MECANUM_DEFAULT_MOTOR_RL_ID,
        MECANUM_DEFAULT_MOTOR_RR_ID
    };
    const int8_t direction_sign[MECANUM_WHEEL_COUNT] =
        MECANUM_DEFAULT_DIRECTION_SIGNS;

    if ((chassis == NULL) || (bus == NULL) || (motors == NULL))
    {
        return MECANUM_INVALID_PARAM;
    }

    initial_motor_count = bus->motor_count;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_result = MOTOR_ERROR;
        for (attempt = 0U; attempt < MECANUM_MOTOR_INIT_ATTEMPTS; attempt++)
        {
            motor_result = motor_init(&motors[index], bus, motor_ids[index]);
            if (motor_result == MOTOR_OK)
            {
                break;
            }
            HAL_Delay(MECANUM_MOTOR_INIT_RETRY_DELAY_MS);
        }
        if (motor_result != MOTOR_OK)
        {
            /* 某一路失败时回滚本次已经注册的底盘电机，允许上层重试。 */
            for (index = initial_motor_count;
                 index < bus->motor_count;
                 index++)
            {
                bus->motors[index] = NULL;
            }
            bus->motor_count = initial_motor_count;
            memset(motors, 0,
                   sizeof(motor_t) * MECANUM_WHEEL_COUNT);
            return MECANUM_MOTOR_ERROR;
        }
    }

    memset(&config, 0, sizeof(config));
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        config.motor[index] = &motors[index];
        config.direction_sign[index] = direction_sign[index];
    }
    config.wheel_radius_m = MECANUM_DEFAULT_WHEEL_RADIUS_M;
    config.half_length_m = MECANUM_DEFAULT_HALF_LENGTH_M;
    config.half_width_m = MECANUM_DEFAULT_HALF_WIDTH_M;
    config.velocity_x_scale = MECANUM_DEFAULT_VELOCITY_X_SCALE;
    config.velocity_y_scale = MECANUM_DEFAULT_VELOCITY_Y_SCALE;
    config.velocity_yaw_scale = MECANUM_DEFAULT_VELOCITY_YAW_SCALE;
    config.pulses_per_rev = MECANUM_DEFAULT_ENCODER_COUNTS_PER_REV;
    config.max_wheel_rpm = MECANUM_DEFAULT_MAX_WHEEL_RPM;
    config.acceleration = MECANUM_DEFAULT_ACCELERATION;

    return mecanum_chassis_init(chassis, &config);
}

mecanum_result_t mecanum_chassis_enable(mecanum_chassis_t *chassis,
                                        bool enable)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if (motor_enable(chassis->config.motor[index],
                         enable,
                         MOTOR_EXECUTE_NOW) != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
    }

    return MECANUM_OK;
}
mecanum_result_t mecanum_chassis_stop(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    /* 只停止 1-4 号底盘电机，避免广播停止影响同总线的机械臂。 */
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if (motor_stop(chassis->config.motor[index],
                       MOTOR_EXECUTE_NOW) != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->target_wheel_rpm[index] = 0;
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_wheel_speed(
    mecanum_chassis_t *chassis,
    const int16_t wheel_rpm[MECANUM_WHEEL_COUNT])
{
    int16_t limited_rpm[MECANUM_WHEEL_COUNT];
    int32_t max_abs = 0;
    uint8_t index;
    float scale = 1.0f;

    if ((chassis == NULL) || (wheel_rpm == NULL) ||
        !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int32_t value = wheel_rpm[index];
        int32_t abs_value = (value < 0) ? -value : value;

        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > (int32_t)chassis->config.max_wheel_rpm)
    {
        scale = (float)chassis->config.max_wheel_rpm / (float)max_abs;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        limited_rpm[index] = mecanum_round_rpm((float)wheel_rpm[index] *
                                               scale);
    }

    if (mecanum_send_wheel_rpm(chassis, limited_rpm) != MECANUM_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    return (scale < 1.0f) ? MECANUM_CLAMPED : MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_position_increment(
    mecanum_chassis_t *chassis,
    const int32_t pulse[MECANUM_WHEEL_COUNT],
    uint16_t speed_rpm,
    uint8_t acceleration)
{
    uint8_t index;
    uint16_t command_speed_rpm;
    uint16_t max_position_speed_rpm;
    motor_result_t motor_result;

    if ((chassis == NULL) || (pulse == NULL) ||
        !mecanum_config_valid(&chassis->config) ||
        (speed_rpm == 0U) || (speed_rpm > MOTOR_EMM_MAX_SPEED_RPM))
    {
        return MECANUM_INVALID_PARAM;
    }

    command_speed_rpm = speed_rpm;
    max_position_speed_rpm = mecanum_position_speed_limit_rpm(chassis);
    if (command_speed_rpm > max_position_speed_rpm)
    {
        command_speed_rpm = max_position_speed_rpm;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int64_t motor_pulse = (int64_t)pulse[index] *
                              (int64_t)chassis->config.direction_sign[index];
        motor_result = motor_set_position(
            chassis->config.motor[index],
            motor_pulse,
            command_speed_rpm,
            acceleration,
            MOTOR_POSITION_RELATIVE_CURRENT,
            MOTOR_EXECUTE_SYNC);
        if (motor_result != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        HAL_Delay(MECANUM_POSITION_COMMAND_GAP_MS);
    }

    HAL_Delay(MECANUM_POSITION_COMMAND_GAP_MS);
    if (motor_trigger_sync(chassis->config.motor[0]->bus) != MOTOR_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->target_wheel_rpm[index] = 0;
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_set_velocity(mecanum_chassis_t *chassis,
                                              float vx_mps,
                                              float vy_mps,
                                              float wz_rad_s)
{
    float wheel_rad_s[MECANUM_WHEEL_COUNT];
    float max_abs = 0.0f;
    float geometry_factor;
    float scale = 1.0f;
    int16_t wheel_rpm[MECANUM_WHEEL_COUNT];
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    geometry_factor = chassis->config.half_length_m +
                      chassis->config.half_width_m;

    /* 标准 X 型麦轮运动学，轮速单位为 rad/s。 */
    wheel_rad_s[MECANUM_WHEEL_FL] = vx_mps - vy_mps -
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_FR] = vx_mps + vy_mps +
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RL] = vx_mps + vy_mps -
                                    geometry_factor * wz_rad_s;
    wheel_rad_s[MECANUM_WHEEL_RR] = vx_mps - vy_mps +
                                    geometry_factor * wz_rad_s;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float abs_value = mecanum_abs_f(wheel_rad_s[index]);
        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > 0.0f)
    {
        float max_rpm = (max_abs / chassis->config.wheel_radius_m) *
                        60.0f / (2.0f * MECANUM_PI_F);
        if (max_rpm > (float)chassis->config.max_wheel_rpm)
        {
            scale = (float)chassis->config.max_wheel_rpm / max_rpm;
        }
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float rpm = (wheel_rad_s[index] * scale /
                     chassis->config.wheel_radius_m) *
                    60.0f / (2.0f * MECANUM_PI_F);
        wheel_rpm[index] = mecanum_round_rpm(rpm);
    }

    if (mecanum_send_wheel_rpm(chassis, wheel_rpm) != MECANUM_OK)
    {
        return MECANUM_MOTOR_ERROR;
    }

    return (scale < 1.0f) ? MECANUM_CLAMPED : MECANUM_OK;
}

mecanum_result_t mecanum_chassis_read_feedback(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    /* 请求是异步的，HAL CAN 中断会更新 motor_t 的缓存字段。 */
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_t *motor = chassis->config.motor[index];

        /* 编码器速度解算只需要电机实际转速反馈。 */
        if (motor_read_speed(motor) != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
        }
        if (index < (MECANUM_WHEEL_COUNT - 1U))
        {
            HAL_Delay(MECANUM_CAN_COMMAND_GAP_MS);
        }
    }

    chassis->last_feedback_tick = HAL_GetTick();
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_feedback_task(mecanum_chassis_t *chassis)
{
    uint32_t now;

    if (chassis == NULL)
    {
        return MECANUM_INVALID_PARAM;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - chassis->last_feedback_tick) <
        MECANUM_DEFAULT_FEEDBACK_PERIOD_MS)
    {
        return MECANUM_NOT_READY;
    }

    return mecanum_chassis_read_feedback(chassis);
}

mecanum_result_t mecanum_chassis_update_encoder_velocity(
    mecanum_chassis_t *chassis)
{
    int16_t wheel_rpm[MECANUM_WHEEL_COUNT];
    uint32_t speed_update_ms[MECANUM_WHEEL_COUNT];
    float wheel_speed_mps[MECANUM_WHEEL_COUNT];
    float geometry_factor;
    float wheel_speed_scale;
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
    uint32_t primask;
    uint32_t now;
    uint8_t all_speed_valid;
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    now = HAL_GetTick();

    /* CAN 接收中断可能同时更新实际转速，先一次性复制四路快照。 */
    all_speed_valid = 1U;
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        const motor_t *motor = chassis->config.motor[index];

        wheel_rpm[index] = (int16_t)((int32_t)motor->actual_speed_rpm *
                                     chassis->config.direction_sign[index]);
        speed_update_ms[index] = motor->speed_update_ms;
        if (speed_update_ms[index] == 0U)
        {
            all_speed_valid = 0U;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (all_speed_valid == 0U)
    {
        return MECANUM_NOT_READY;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if ((uint32_t)(now - speed_update_ms[index]) >
            MECANUM_MAX_ENCODER_SPEED_AGE_MS)
        {
            return MECANUM_NOT_READY;
        }
    }

    wheel_speed_scale = (2.0f * MECANUM_PI_F *
                         chassis->config.wheel_radius_m) / 60.0f;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        wheel_speed_mps[index] = (float)wheel_rpm[index] * wheel_speed_scale;
    }

    /* 与 mecanum_chassis_set_velocity() 使用的 X 型麦轮运动学互为逆运算。 */
    vx_mps = (wheel_speed_mps[MECANUM_WHEEL_FL] +
              wheel_speed_mps[MECANUM_WHEEL_FR] +
              wheel_speed_mps[MECANUM_WHEEL_RL] +
              wheel_speed_mps[MECANUM_WHEEL_RR]) * 0.25f;
    vy_mps = (-wheel_speed_mps[MECANUM_WHEEL_FL] +
              wheel_speed_mps[MECANUM_WHEEL_FR] +
              wheel_speed_mps[MECANUM_WHEEL_RL] -
              wheel_speed_mps[MECANUM_WHEEL_RR]) * 0.25f;

    geometry_factor = chassis->config.half_length_m +
                      chassis->config.half_width_m;
    if (geometry_factor <= 0.0f)
    {
        return MECANUM_INVALID_PARAM;
    }

    wz_rad_s = (-wheel_speed_mps[MECANUM_WHEEL_FL] +
                wheel_speed_mps[MECANUM_WHEEL_FR] -
                wheel_speed_mps[MECANUM_WHEEL_RL] +
                wheel_speed_mps[MECANUM_WHEEL_RR]) /
               (4.0f * geometry_factor);

    /* 这些比例只修正编码器测量输出，不改变速度控制的轮速换算。 */
    vx_mps *= mecanum_velocity_scale(chassis->config.velocity_x_scale);
    vy_mps *= mecanum_velocity_scale(chassis->config.velocity_y_scale);
    wz_rad_s *= mecanum_velocity_scale(chassis->config.velocity_yaw_scale);

    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->encoder_velocity.wheel_rpm[index] = wheel_rpm[index];
        chassis->encoder_velocity.wheel_speed_mps[index] =
            wheel_speed_mps[index];
    }
    chassis->encoder_velocity.vx_mps = vx_mps;
    chassis->encoder_velocity.vy_mps = vy_mps;
    chassis->encoder_velocity.wz_rad_s = wz_rad_s;
    chassis->encoder_velocity.update_tick = now;
    chassis->encoder_velocity.valid = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_get_encoder_velocity(
    const mecanum_chassis_t *chassis,
    mecanum_encoder_velocity_t *velocity)
{
    uint32_t primask;

    if ((chassis == NULL) || (velocity == NULL))
    {
        return MECANUM_INVALID_PARAM;
    }
    if (chassis->encoder_velocity.valid == 0U)
    {
        return MECANUM_NOT_READY;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(velocity,
           (const void *)&chassis->encoder_velocity,
           sizeof(*velocity));
    if (primask == 0U)
    {
        __enable_irq();
    }

    return MECANUM_OK;
}
