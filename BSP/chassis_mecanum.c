//
// 麦轮底盘控制实现，使用 motor_can 原生 CAN 电机接口。
//

#include "chassis_mecanum.h"
#include "MECANUM_INERTIAL_NAV.h"

#include <math.h>
#include <string.h>

#define MECANUM_PI_F 3.14159265358979323846f
/* Emm 位置命令拆成多帧 CAN 数据，多台电机连续发送时留出间隔。 */
#define MECANUM_POSITION_COMMAND_GAP_MS 2U
/* 位置模式限制为约60 RPM，匹配任务层的路线速度。 */
#define MECANUM_POSITION_MAX_SPEED_MPS 0.246f
#define MECANUM_INERTIAL_MOVE_MAX_SPEED_MPS 0.25f
#define MECANUM_INERTIAL_MOVE_MAX_WZ_RAD_S  0.35f
#define MECANUM_INERTIAL_MOVE_KP_POSITION   1.20f
#define MECANUM_INERTIAL_MOVE_KP_YAW        1.50f
#define MECANUM_INERTIAL_MOVE_TOLERANCE_M   0.008f
#define MECANUM_INERTIAL_MOVE_YAW_TOLERANCE 0.025f
#define MECANUM_INERTIAL_MOVE_TIMEOUT_MS    30000U

static float mecanum_abs_f(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float mecanum_odometry_scale(float value)
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
        (config->odometry_x_scale < 0.0f) ||
        (config->odometry_y_scale < 0.0f) ||
        (config->odometry_yaw_scale < 0.0f) ||
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
        result = motor_set_speed(chassis->config.motor[index],
                                 motor_rpm[index],
                                 chassis->config.acceleration,
                                 MOTOR_EXECUTE_NOW);
        if (result != MOTOR_OK)
        {
            return MECANUM_MOTOR_ERROR;
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
    mecanum_chassis_reset_odometry(chassis);
    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_init_default(
    mecanum_chassis_t *chassis,
    motor_bus_t *bus,
    motor_t motors[MECANUM_WHEEL_COUNT])
{
    mecanum_config_t config;
    motor_result_t motor_result;
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

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_result = MOTOR_ERROR;
        for (attempt = 0U; attempt < 3U; attempt++)
        {
            motor_result = motor_init(&motors[index], bus, motor_ids[index]);
            if (motor_result == MOTOR_OK)
            {
                break;
            }
            HAL_Delay(20U);
        }
        if (motor_result != MOTOR_OK)
        {
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
    config.odometry_x_scale = MECANUM_DEFAULT_ODOMETRY_X_SCALE;
    config.odometry_y_scale = MECANUM_DEFAULT_ODOMETRY_Y_SCALE;
    config.odometry_yaw_scale = MECANUM_DEFAULT_ODOMETRY_YAW_SCALE;
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

static mecanum_result_t mecanum_chassis_move_inertial_increment(
    mecanum_chassis_t *chassis,
    const int32_t pulse[MECANUM_WHEEL_COUNT],
    uint16_t speed_rpm)
{
    mecanum_inertial_nav_t *nav;
    float pulse_distance_m;
    float body_x_pulse;
    float body_y_pulse;
    float body_yaw_pulse;
    float body_dx_m;
    float body_dy_m;
    float body_dyaw_rad;
    float target_x_m;
    float target_y_m;
    float target_yaw_rad;
    float max_speed_mps;
    uint32_t start_tick;
    mecanum_inertial_nav_result_t nav_result;

    if ((chassis == NULL) || (pulse == NULL) ||
        (chassis->inertial_nav == NULL) ||
        (chassis->imu_sensor == NULL))
    {
        return MECANUM_INVALID_PARAM;
    }

    nav = (mecanum_inertial_nav_t *)chassis->inertial_nav;
    pulse_distance_m = (2.0f * MECANUM_PI_F *
                        chassis->config.wheel_radius_m) /
                       (float)MECANUM_POSITION_PULSES_PER_REV;
    body_x_pulse = ((float)pulse[MECANUM_WHEEL_FL] +
                    (float)pulse[MECANUM_WHEEL_FR] +
                    (float)pulse[MECANUM_WHEEL_RL] +
                    (float)pulse[MECANUM_WHEEL_RR]) * 0.25f;
    body_y_pulse = (-(float)pulse[MECANUM_WHEEL_FL] +
                    (float)pulse[MECANUM_WHEEL_FR] +
                    (float)pulse[MECANUM_WHEEL_RL] -
                    (float)pulse[MECANUM_WHEEL_RR]) * 0.25f;
    body_yaw_pulse = (-(float)pulse[MECANUM_WHEEL_FL] +
                      (float)pulse[MECANUM_WHEEL_FR] -
                      (float)pulse[MECANUM_WHEEL_RL] +
                      (float)pulse[MECANUM_WHEEL_RR]) /
                     (4.0f * (chassis->config.half_length_m +
                              chassis->config.half_width_m));
    body_dx_m = body_x_pulse * pulse_distance_m;
    body_dy_m = body_y_pulse * pulse_distance_m;
    body_dyaw_rad = body_yaw_pulse * pulse_distance_m;

    start_tick = HAL_GetTick();
    while (nav->ekf_initialized == 0U)
    {
        (void)mecanum_chassis_feedback_task(chassis);
        (void)mecanum_inertial_nav_update(
            nav,
            chassis,
            chassis->imu_sensor);
        if ((uint32_t)(HAL_GetTick() - start_tick) >=
            MECANUM_INERTIAL_MOVE_TIMEOUT_MS)
        {
            return MECANUM_NOT_READY;
        }
        HAL_Delay(10U);
    }

    target_x_m = nav->x_m + cosf(nav->yaw_rad) * body_dx_m -
                 sinf(nav->yaw_rad) * body_dy_m;
    target_y_m = nav->y_m + sinf(nav->yaw_rad) * body_dx_m +
                 cosf(nav->yaw_rad) * body_dy_m;
    target_yaw_rad = nav->yaw_rad + body_dyaw_rad;
    max_speed_mps = ((float)speed_rpm / 60.0f) *
                    2.0f * MECANUM_PI_F * chassis->config.wheel_radius_m;
    if (max_speed_mps > MECANUM_INERTIAL_MOVE_MAX_SPEED_MPS)
    {
        max_speed_mps = MECANUM_INERTIAL_MOVE_MAX_SPEED_MPS;
    }
    if (max_speed_mps < 0.03f)
    {
        max_speed_mps = 0.03f;
    }

    nav_result = mecanum_inertial_nav_set_target(
        nav,
        target_x_m,
        target_y_m,
        target_yaw_rad,
        max_speed_mps,
        max_speed_mps,
        MECANUM_INERTIAL_MOVE_MAX_WZ_RAD_S,
        MECANUM_INERTIAL_MOVE_KP_POSITION,
        MECANUM_INERTIAL_MOVE_KP_YAW,
        MECANUM_INERTIAL_MOVE_TOLERANCE_M,
        MECANUM_INERTIAL_MOVE_YAW_TOLERANCE);
    if (nav_result != MECANUM_INERTIAL_NAV_OK)
    {
        return MECANUM_INVALID_PARAM;
    }

    start_tick = HAL_GetTick();
    while (nav->target_active != 0U)
    {
        (void)mecanum_chassis_feedback_task(chassis);
        (void)mecanum_inertial_nav_update(
            nav,
            chassis,
            chassis->imu_sensor);
        nav_result = mecanum_inertial_nav_control_step(nav, chassis);
        if (nav_result == MECANUM_INERTIAL_NAV_TARGET_REACHED)
        {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - start_tick) >=
            MECANUM_INERTIAL_MOVE_TIMEOUT_MS)
        {
            mecanum_inertial_nav_cancel_target(nav, chassis);
            return MECANUM_MOTOR_ERROR;
        }
        HAL_Delay(10U);
    }
    (void)mecanum_chassis_stop(chassis);
    return MECANUM_OK;
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
        chassis->last_position_increment_pulse[index] = pulse[index];
        chassis->target_wheel_rpm[index] = 0;
    }

    return MECANUM_OK;
}

void mecanum_chassis_attach_inertial_nav(
    mecanum_chassis_t *chassis,
    struct mecanum_inertial_nav *nav,
    jy901s_uart_t *imu_sensor)
{
    if (chassis == NULL)
    {
        return;
    }
    chassis->inertial_nav = nav;
    chassis->imu_sensor = imu_sensor;
    chassis->inertial_motion_enabled =
        ((nav != NULL) && (imu_sensor != NULL)) ? 1U : 0U;
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

        /* 使用调通版本的 Emm 0x36 多圈位置作为里程计输入。 */
        if ((motor_read_speed(motor) != MOTOR_OK) ||
            (motor_read_position(motor) != MOTOR_OK))
        {
            return MECANUM_MOTOR_ERROR;
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

mecanum_result_t mecanum_chassis_update_encoder_odometry(
    mecanum_chassis_t *chassis)
{
    float wheel_distance[MECANUM_WHEEL_COUNT];
    int64_t current_position_pulse[MECANUM_WHEEL_COUNT];
    uint32_t current_update_ms[MECANUM_WHEEL_COUNT];
    uint32_t current_update_count[MECANUM_WHEEL_COUNT];
    float geometry_factor;
    float dx;
    float dy;
    float dyaw;
    float dt_s;
    uint32_t now;
    uint8_t position_changed;
    uint8_t all_position_updated;
    uint8_t all_position_valid;
    uint32_t primask;
    uint8_t index;

    if ((chassis == NULL) || !mecanum_config_valid(&chassis->config))
    {
        return MECANUM_INVALID_PARAM;
    }

    position_changed = 0U;
    all_position_updated = 1U;
    all_position_valid = 1U;
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_t *motor = chassis->config.motor[index];

        /* 0x36 返回多圈实际位置，不需要单圈回绕处理。 */
        if (motor->position_valid == 0U)
        {
            all_position_valid = 0U;
            continue;
        }

        current_position_pulse[index] = motor->actual_position_pulse;
        current_update_ms[index] = motor->position_update_ms;
        current_update_count[index] = motor->position_update_count;
        chassis->actual_wheel_rpm[index] = (int16_t)(
            (int32_t)motor->actual_speed_rpm *
            chassis->config.direction_sign[index]);
        if ((chassis->odom_initialized != 0U) &&
            (current_update_count[index] !=
             chassis->last_encoder_update_count[index]))
        {
            position_changed = 1U;
        }
        else
        {
            all_position_updated = 0U;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (all_position_valid == 0U)
    {
        return MECANUM_NOT_READY;
    }

    if ((chassis->odom_initialized != 0U) &&
        ((position_changed == 0U) || (all_position_updated == 0U)))
    {
        return MECANUM_NOT_READY;
    }

    now = HAL_GetTick();
    /* 基准只需建立一次；不能用 encoder_odometry.valid 判断，
       因为 valid 要在完成第一次增量积分后才会置位。 */
    if (chassis->odom_initialized == 0U)
    {
        for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
        {
            chassis->last_position_pulse[index] = current_position_pulse[index];
            chassis->last_encoder_update_ms[index] = current_update_ms[index];
            chassis->last_encoder_update_count[index] =
                current_update_count[index];
        }
        chassis->encoder_odometry.update_tick = now;
        chassis->odom_initialized = 1U;
        return MECANUM_NOT_READY;
    }

    dt_s = (float)(now - chassis->encoder_odometry.update_tick) * 0.001f;
    if (dt_s <= 0.0f)
    {
        return MECANUM_NOT_READY;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int64_t position_delta = current_position_pulse[index] -
                                 chassis->last_position_pulse[index];

        wheel_distance[index] = (float)position_delta *
                                (2.0f * MECANUM_PI_F *
                                 chassis->config.wheel_radius_m) /
                                (float)chassis->config.pulses_per_rev;
        wheel_distance[index] *=
            (float)chassis->config.direction_sign[index];
    }

    geometry_factor = chassis->config.half_length_m +
                      chassis->config.half_width_m;
    if (geometry_factor <= 0.0f)
    {
        return MECANUM_INVALID_PARAM;
    }

    dx = (wheel_distance[MECANUM_WHEEL_FL] +
          wheel_distance[MECANUM_WHEEL_FR] +
          wheel_distance[MECANUM_WHEEL_RL] +
          wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    dy = (-wheel_distance[MECANUM_WHEEL_FL] +
          wheel_distance[MECANUM_WHEEL_FR] +
          wheel_distance[MECANUM_WHEEL_RL] -
          wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    dyaw = (-wheel_distance[MECANUM_WHEEL_FL] +
            wheel_distance[MECANUM_WHEEL_FR] -
            wheel_distance[MECANUM_WHEEL_RL] +
            wheel_distance[MECANUM_WHEEL_RR]) /
           (4.0f * geometry_factor);

    /* 标定比例只修正里程计输出，不参与速度控制。 */
    dx *= mecanum_odometry_scale(chassis->config.odometry_x_scale);
    dy *= mecanum_odometry_scale(chassis->config.odometry_y_scale);
    dyaw *= mecanum_odometry_scale(chassis->config.odometry_yaw_scale);

    /* 使用编码器自身的航向角作为积分基准，避免外部 IMU 覆盖原始结果。 */
    {
        float mid_yaw = chassis->encoder_odometry.yaw_rad + 0.5f * dyaw;
        float cos_yaw = cosf(mid_yaw);
        float sin_yaw = sinf(mid_yaw);

        chassis->encoder_odometry.x_m += dx * cos_yaw - dy * sin_yaw;
        chassis->encoder_odometry.y_m += dx * sin_yaw + dy * cos_yaw;
    }

    chassis->encoder_odometry.yaw_rad += dyaw;
    chassis->odom_x_m = chassis->encoder_odometry.x_m;
    chassis->odom_y_m = chassis->encoder_odometry.y_m;
    chassis->odom_yaw_rad = chassis->encoder_odometry.yaw_rad;
    chassis->last_delta_x_m = dx;
    chassis->last_delta_y_m = dy;
    chassis->last_delta_yaw_rad = dyaw;
    chassis->encoder_odometry.delta_x_m = dx;
    chassis->encoder_odometry.delta_y_m = dy;
    chassis->encoder_odometry.delta_yaw_rad = dyaw;
    chassis->encoder_odometry.vx_mps = dx / dt_s;
    chassis->encoder_odometry.vy_mps = dy / dt_s;
    chassis->encoder_odometry.wz_rad_s = dyaw / dt_s;
    chassis->encoder_odometry.update_tick = now;
    chassis->encoder_odometry.valid = 1U;

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->last_position_pulse[index] = current_position_pulse[index];
        chassis->last_encoder_update_ms[index] = current_update_ms[index];
        chassis->last_encoder_update_count[index] = current_update_count[index];
    }

    return MECANUM_OK;
}

mecanum_result_t mecanum_chassis_update_odometry(mecanum_chassis_t *chassis)
{
    return mecanum_chassis_update_encoder_odometry(chassis);
}

void mecanum_chassis_reset_odometry(mecanum_chassis_t *chassis)
{
    uint8_t index;

    if (chassis == NULL)
    {
        return;
    }

    chassis->odom_x_m = 0.0f;
    chassis->odom_y_m = 0.0f;
    chassis->odom_yaw_rad = 0.0f;
    chassis->last_delta_x_m = 0.0f;
    chassis->last_delta_y_m = 0.0f;
    chassis->last_delta_yaw_rad = 0.0f;
    chassis->odom_initialized = 0U;
    memset(&chassis->encoder_odometry, 0,
           sizeof(chassis->encoder_odometry));

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        chassis->last_encoder_raw[index] = 0U;
        chassis->last_position_pulse[index] = 0;
        chassis->last_position_increment_pulse[index] = 0;
        chassis->last_encoder_update_ms[index] = 0U;
        chassis->last_encoder_update_count[index] = 0U;
    }
}

mecanum_result_t mecanum_chassis_get_encoder_odometry(
    const mecanum_chassis_t *chassis,
    mecanum_encoder_odometry_t *odometry)
{
    if ((chassis == NULL) || (odometry == NULL))
    {
        return MECANUM_INVALID_PARAM;
    }
    if (chassis->encoder_odometry.valid == 0U)
    {
        return MECANUM_NOT_READY;
    }

    *odometry = chassis->encoder_odometry;
    return MECANUM_OK;
}

void mecanum_chassis_set_yaw(mecanum_chassis_t *chassis, float yaw_rad)
{
    if (chassis != NULL)
    {
        chassis->odom_yaw_rad = yaw_rad;
    }
}
