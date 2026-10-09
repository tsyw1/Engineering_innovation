#include "MECANUM_INERTIAL_NAV.h"

#include <math.h>
#include <string.h>

#define MECANUM_INERTIAL_PI_F       3.14159265358979323846f
#define MECANUM_INERTIAL_DEG_TO_RAD 0.01745329251994329577f
#define MECANUM_INERTIAL_RAD_TO_DEG 57.2957795130823208768f
#define MECANUM_INERTIAL_MAX_DT_S   0.2f
#define MECANUM_INERTIAL_MIN_DT_S   0.0005f

#define EKF_X       0U
#define EKF_Y       1U
#define EKF_YAW     2U
#define EKF_VX      3U
#define EKF_VY      4U
#define EKF_BIAS    5U

static float mecanum_nav_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float mecanum_nav_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float mecanum_nav_odometry_scale(float value)
{
    return (value > 0.0f) ? value : 1.0f;
}

static float mecanum_nav_wrap_angle(float angle)
{
    while (angle > MECANUM_INERTIAL_PI_F)
    {
        angle -= 2.0f * MECANUM_INERTIAL_PI_F;
    }
    while (angle < -MECANUM_INERTIAL_PI_F)
    {
        angle += 2.0f * MECANUM_INERTIAL_PI_F;
    }
    return angle;
}

static uint8_t mecanum_inertial_config_valid(
    const mecanum_config_t *config)
{
    uint8_t index;

    if ((config == NULL) || (config->wheel_radius_m <= 0.0f) ||
        ((config->half_length_m + config->half_width_m) <= 0.0f) ||
        (config->odometry_x_scale < 0.0f) ||
        (config->odometry_y_scale < 0.0f) ||
        (config->odometry_yaw_scale < 0.0f) ||
        (config->pulses_per_rev == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        if ((config->motor[index] == NULL) ||
            ((config->direction_sign[index] != 1) &&
             (config->direction_sign[index] != -1)))
        {
            return 0U;
        }
    }
    return 1U;
}

static void mecanum_inertial_copy_encoder_baseline(
    mecanum_inertial_nav_t *nav,
    const mecanum_chassis_t *chassis)
{
    uint32_t primask;
    uint8_t index;

    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        nav->last_position_pulse[index] =
            chassis->config.motor[index]->actual_position_pulse;
        nav->last_position_update_count[index] =
            chassis->config.motor[index]->position_update_count;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void mecanum_inertial_ekf_reset(mecanum_inertial_nav_t *nav)
{
    uint8_t index;

    memset(nav->ekf_state, 0, sizeof(nav->ekf_state));
    memset(nav->ekf_cov, 0, sizeof(nav->ekf_cov));
    for (index = 0U; index < MECANUM_INERTIAL_NAV_STATE_COUNT; index++)
    {
        nav->ekf_cov[index * MECANUM_INERTIAL_NAV_STATE_COUNT + index] =
            (index == EKF_YAW) ? 0.2f * 0.2f :
            ((index == EKF_BIAS) ?
             (0.5f * MECANUM_INERTIAL_DEG_TO_RAD) *
             (0.5f * MECANUM_INERTIAL_DEG_TO_RAD) : 0.1f * 0.1f);
    }
    nav->ekf_initialized = 0U;
}

static void mecanum_inertial_ekf_scalar_update(
    mecanum_inertial_nav_t *nav,
    uint8_t measurement_index,
    float measurement,
    float measurement_variance)
{
    float innovation;
    float innovation_variance;
    float gain[MECANUM_INERTIAL_NAV_STATE_COUNT];
    float covariance[MECANUM_INERTIAL_NAV_STATE_COUNT *
                     MECANUM_INERTIAL_NAV_STATE_COUNT];
    uint8_t row;
    uint8_t col;

    innovation = measurement - nav->ekf_state[measurement_index];
    innovation_variance = nav->ekf_cov[
        measurement_index * MECANUM_INERTIAL_NAV_STATE_COUNT +
        measurement_index] + measurement_variance;
    if (innovation_variance <= 0.0f)
    {
        return;
    }

    for (row = 0U; row < MECANUM_INERTIAL_NAV_STATE_COUNT; row++)
    {
        gain[row] = nav->ekf_cov[
            row * MECANUM_INERTIAL_NAV_STATE_COUNT + measurement_index] /
            innovation_variance;
        nav->ekf_state[row] += gain[row] * innovation;
    }

    memcpy(covariance, nav->ekf_cov, sizeof(covariance));
    for (row = 0U; row < MECANUM_INERTIAL_NAV_STATE_COUNT; row++)
    {
        for (col = 0U; col < MECANUM_INERTIAL_NAV_STATE_COUNT; col++)
        {
            nav->ekf_cov[row * MECANUM_INERTIAL_NAV_STATE_COUNT + col] =
                covariance[row * MECANUM_INERTIAL_NAV_STATE_COUNT + col] -
                gain[row] * covariance[measurement_index *
                                       MECANUM_INERTIAL_NAV_STATE_COUNT + col];
        }
    }
}

static void mecanum_inertial_ekf_predict(
    mecanum_inertial_nav_t *nav,
    float dt_s,
    float encoder_wz_rad_s,
    float gyro_raw_rad_s)
{
    float old_state[MECANUM_INERTIAL_NAV_STATE_COUNT];
    float transition[MECANUM_INERTIAL_NAV_STATE_COUNT *
                     MECANUM_INERTIAL_NAV_STATE_COUNT];
    float predicted_cov[MECANUM_INERTIAL_NAV_STATE_COUNT *
                        MECANUM_INERTIAL_NAV_STATE_COUNT];
    float yaw;
    float cos_yaw;
    float sin_yaw;
    float yaw_rate;
    uint8_t row;
    uint8_t col;
    uint8_t index;
    uint8_t inner;

    memcpy(old_state, nav->ekf_state, sizeof(old_state));
    memset(transition, 0, sizeof(transition));
    for (index = 0U; index < MECANUM_INERTIAL_NAV_STATE_COUNT; index++)
    {
        transition[index * MECANUM_INERTIAL_NAV_STATE_COUNT + index] = 1.0f;
    }

    yaw = old_state[EKF_YAW];
    cos_yaw = cosf(yaw);
    sin_yaw = sinf(yaw);
    yaw_rate = nav->gyro_weight *
               (gyro_raw_rad_s - old_state[EKF_BIAS]) +
               (1.0f - nav->gyro_weight) * encoder_wz_rad_s;

    nav->ekf_state[EKF_X] +=
        (cos_yaw * old_state[EKF_VX] - sin_yaw * old_state[EKF_VY]) * dt_s;
    nav->ekf_state[EKF_Y] +=
        (sin_yaw * old_state[EKF_VX] + cos_yaw * old_state[EKF_VY]) * dt_s;
    nav->ekf_state[EKF_YAW] = mecanum_nav_wrap_angle(
        old_state[EKF_YAW] + yaw_rate * dt_s);

    transition[EKF_X * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_YAW] =
        (-sin_yaw * old_state[EKF_VX] - cos_yaw * old_state[EKF_VY]) * dt_s;
    transition[EKF_X * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VX] =
        cos_yaw * dt_s;
    transition[EKF_X * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VY] =
        -sin_yaw * dt_s;
    transition[EKF_Y * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_YAW] =
        (cos_yaw * old_state[EKF_VX] - sin_yaw * old_state[EKF_VY]) * dt_s;
    transition[EKF_Y * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VX] =
        sin_yaw * dt_s;
    transition[EKF_Y * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VY] =
        cos_yaw * dt_s;
    transition[EKF_YAW * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_BIAS] =
        -nav->gyro_weight * dt_s;

    for (row = 0U; row < MECANUM_INERTIAL_NAV_STATE_COUNT; row++)
    {
        for (col = 0U; col < MECANUM_INERTIAL_NAV_STATE_COUNT; col++)
        {
            float value = 0.0f;
            for (index = 0U; index < MECANUM_INERTIAL_NAV_STATE_COUNT; index++)
            {
                for (inner = 0U; inner < MECANUM_INERTIAL_NAV_STATE_COUNT; inner++)
                {
                    value += transition[row * MECANUM_INERTIAL_NAV_STATE_COUNT +
                                        index] *
                             nav->ekf_cov[index * MECANUM_INERTIAL_NAV_STATE_COUNT +
                                          inner] *
                             transition[col * MECANUM_INERTIAL_NAV_STATE_COUNT +
                                        inner];
                }
            }
            predicted_cov[row * MECANUM_INERTIAL_NAV_STATE_COUNT + col] = value;
        }
    }

    memcpy(nav->ekf_cov, predicted_cov, sizeof(predicted_cov));
    nav->ekf_cov[EKF_X * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_X] += 1.0e-6f;
    nav->ekf_cov[EKF_Y * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_Y] += 1.0e-6f;
    nav->ekf_cov[EKF_YAW * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_YAW] += 1.0e-5f;
    nav->ekf_cov[EKF_VX * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VX] += 2.5e-3f;
    nav->ekf_cov[EKF_VY * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_VY] += 2.5e-3f;
    nav->ekf_cov[EKF_BIAS * MECANUM_INERTIAL_NAV_STATE_COUNT + EKF_BIAS] +=
        1.0e-9f + mecanum_nav_clamp(nav->bias_update_gain, 0.0f, 1.0f) *
        1.0e-5f;
}

mecanum_inertial_nav_result_t mecanum_inertial_nav_init(
    mecanum_inertial_nav_t *nav,
    const mecanum_config_t *config,
    float gyro_z_bias_dps,
    float gyro_weight,
    float bias_update_gain)
{
    if ((nav == NULL) || (mecanum_inertial_config_valid(config) == 0U) ||
        (gyro_weight < 0.0f) || (gyro_weight > 1.0f) ||
        (bias_update_gain < 0.0f))
    {
        return MECANUM_INERTIAL_NAV_INVALID_PARAM;
    }

    memset(nav, 0, sizeof(*nav));
    memcpy(&nav->config, config, sizeof(nav->config));
    nav->gyro_z_bias_dps = gyro_z_bias_dps;
    nav->gyro_weight = gyro_weight;
    nav->bias_update_gain = bias_update_gain;
    mecanum_inertial_ekf_reset(nav);
    nav->ekf_state[EKF_BIAS] = gyro_z_bias_dps * MECANUM_INERTIAL_DEG_TO_RAD;
    nav->initialized = 1U;
    return MECANUM_INERTIAL_NAV_OK;
}

mecanum_inertial_nav_result_t mecanum_inertial_nav_init_default(
    mecanum_inertial_nav_t *nav,
    const mecanum_config_t *config,
    float gyro_z_bias_dps)
{
    return mecanum_inertial_nav_init(
        nav,
        config,
        gyro_z_bias_dps,
        MECANUM_INERTIAL_NAV_DEFAULT_GYRO_WEIGHT,
        MECANUM_INERTIAL_NAV_DEFAULT_BIAS_UPDATE_GAIN);
}

void mecanum_inertial_nav_set_gyro_bias(
    mecanum_inertial_nav_t *nav,
    float gyro_z_bias_dps)
{
    if (nav != NULL)
    {
        nav->gyro_z_bias_dps = gyro_z_bias_dps;
        nav->ekf_state[EKF_BIAS] = gyro_z_bias_dps *
                                   MECANUM_INERTIAL_DEG_TO_RAD;
    }
}

void mecanum_inertial_nav_reset(mecanum_inertial_nav_t *nav)
{
    if (nav == NULL)
    {
        return;
    }

    nav->x_m = 0.0f;
    nav->y_m = 0.0f;
    nav->yaw_rad = 0.0f;
    nav->vx_mps = 0.0f;
    nav->vy_mps = 0.0f;
    nav->encoder_wz_rad_s = 0.0f;
    nav->gyro_wz_rad_s = 0.0f;
    nav->fused_wz_rad_s = 0.0f;
    nav->last_update_ms = 0U;
    nav->last_imu_frame_count = 0U;
    nav->encoder_initialized = 0U;
    nav->target_active = 0U;
    mecanum_inertial_ekf_reset(nav);
    nav->ekf_state[EKF_BIAS] = nav->gyro_z_bias_dps *
                               MECANUM_INERTIAL_DEG_TO_RAD;
}

mecanum_inertial_nav_result_t mecanum_inertial_nav_update(
    mecanum_inertial_nav_t *nav,
    const mecanum_chassis_t *chassis,
    const jy901s_uart_t *imu_sensor)
{
    jy901s_uart_t imu_data;
    float wheel_distance[MECANUM_WHEEL_COUNT];
    float geometry_factor;
    float body_dx;
    float body_dy;
    float encoder_dyaw;
    float dt_s;
    float gyro_raw_rad_s;
    float bias_measurement_rad_s;
    int64_t current_position_pulse[MECANUM_WHEEL_COUNT];
    uint32_t current_position_update_count[MECANUM_WHEEL_COUNT];
    uint32_t now;
    uint32_t dt_ms;
    uint8_t index;
    uint8_t encoder_updated;
    uint8_t all_encoder_updated;
    uint32_t primask;

    if ((nav == NULL) || (nav->initialized == 0U) ||
        (chassis == NULL) || (imu_sensor == NULL) ||
        (mecanum_inertial_config_valid(&nav->config) == 0U))
    {
        return MECANUM_INERTIAL_NAV_INVALID_PARAM;
    }
    if (jy901s_uart_get_data(imu_sensor, &imu_data) != JY901S_UART_OK)
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }
    if (imu_data.angular_velocity_frame_count == 0U)
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }
    /* 主循环可能远快于 IMU 100 Hz 输出，不能重复消费同一帧。 */
    if (imu_data.angular_velocity_frame_count ==
        nav->last_imu_frame_count)
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }

    /* 位置值和更新计数必须在 CAN 中断更新前一次性快照。 */
    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        current_position_pulse[index] =
            nav->config.motor[index]->actual_position_pulse;
        current_position_update_count[index] =
            nav->config.motor[index]->position_update_count;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    encoder_updated = 0U;
    all_encoder_updated = 1U;
    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        const motor_t *motor = nav->config.motor[index];
        /* 四路位置必须都是同一批次的新 0x36 反馈。 */
        if (motor->position_valid == 0U)
        {
            return MECANUM_INERTIAL_NAV_NOT_READY;
        }
        if (current_position_update_count[index] !=
            nav->last_position_update_count[index])
        {
            encoder_updated = 1U;
        }
        else
        {
            all_encoder_updated = 0U;
        }
    }
    if ((encoder_updated == 0U) || (all_encoder_updated == 0U))
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }

    now = HAL_GetTick();
    if (nav->encoder_initialized == 0U)
    {
        mecanum_inertial_copy_encoder_baseline(nav, chassis);
        nav->last_update_ms = now;
        nav->last_imu_frame_count = imu_data.angular_velocity_frame_count;
        nav->encoder_initialized = 1U;
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }

    dt_ms = now - nav->last_update_ms;
    nav->last_update_ms = now;
    dt_s = (float)dt_ms * 0.001f;
    if ((dt_s < MECANUM_INERTIAL_MIN_DT_S) ||
        (dt_s > MECANUM_INERTIAL_MAX_DT_S))
    {
        mecanum_inertial_copy_encoder_baseline(nav, chassis);
        /* 这次输入已经被检查过，避免在下一次循环重复消费同一帧 IMU。 */
        nav->last_imu_frame_count = imu_data.angular_velocity_frame_count;
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }

    for (index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        int64_t position_delta = current_position_pulse[index] -
                                 nav->last_position_pulse[index];
        wheel_distance[index] =
            (float)position_delta *
            (2.0f * MECANUM_INERTIAL_PI_F * nav->config.wheel_radius_m) /
            (float)nav->config.pulses_per_rev;
        wheel_distance[index] *= (float)nav->config.direction_sign[index];
        nav->last_position_pulse[index] = current_position_pulse[index];
        nav->last_position_update_count[index] =
            current_position_update_count[index];
    }

    geometry_factor = nav->config.half_length_m + nav->config.half_width_m;
    body_dx = (wheel_distance[MECANUM_WHEEL_FL] +
               wheel_distance[MECANUM_WHEEL_FR] +
               wheel_distance[MECANUM_WHEEL_RL] +
               wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    body_dy = (-wheel_distance[MECANUM_WHEEL_FL] +
               wheel_distance[MECANUM_WHEEL_FR] +
               wheel_distance[MECANUM_WHEEL_RL] -
               wheel_distance[MECANUM_WHEEL_RR]) * 0.25f;
    encoder_dyaw = (-wheel_distance[MECANUM_WHEEL_FL] +
                    wheel_distance[MECANUM_WHEEL_FR] -
                    wheel_distance[MECANUM_WHEEL_RL] +
                    wheel_distance[MECANUM_WHEEL_RR]) /
                   (4.0f * geometry_factor);

    body_dx *= mecanum_nav_odometry_scale(nav->config.odometry_x_scale);
    body_dy *= mecanum_nav_odometry_scale(nav->config.odometry_y_scale);
    encoder_dyaw *= mecanum_nav_odometry_scale(
        nav->config.odometry_yaw_scale);

    nav->vx_mps = body_dx / dt_s;
    nav->vy_mps = body_dy / dt_s;
    nav->encoder_wz_rad_s = encoder_dyaw / dt_s;
    gyro_raw_rad_s = imu_data.angular_velocity_dps[2] *
                     MECANUM_INERTIAL_DEG_TO_RAD;
    nav->gyro_wz_rad_s = gyro_raw_rad_s - nav->gyro_z_bias_dps *
                         MECANUM_INERTIAL_DEG_TO_RAD;
    nav->fused_wz_rad_s = nav->gyro_weight * nav->gyro_wz_rad_s +
                          (1.0f - nav->gyro_weight) * nav->encoder_wz_rad_s;

    if (nav->ekf_initialized == 0U)
    {
        nav->ekf_state[EKF_VX] = nav->vx_mps;
        nav->ekf_state[EKF_VY] = nav->vy_mps;
        nav->ekf_state[EKF_YAW] = 0.0f;
        nav->ekf_initialized = 1U;
    }
    mecanum_inertial_ekf_predict(nav, dt_s,
                                 nav->encoder_wz_rad_s,
                                 gyro_raw_rad_s);
    mecanum_inertial_ekf_scalar_update(nav, EKF_VX, nav->vx_mps, 0.03f * 0.03f);
    mecanum_inertial_ekf_scalar_update(nav, EKF_VY, nav->vy_mps, 0.03f * 0.03f);
    bias_measurement_rad_s = gyro_raw_rad_s - nav->encoder_wz_rad_s;
    mecanum_inertial_ekf_scalar_update(nav, EKF_BIAS,
                                       bias_measurement_rad_s,
                                       0.05f * 0.05f);

    nav->x_m = nav->ekf_state[EKF_X];
    nav->y_m = nav->ekf_state[EKF_Y];
    nav->yaw_rad = mecanum_nav_wrap_angle(nav->ekf_state[EKF_YAW]);
    nav->vx_mps = nav->ekf_state[EKF_VX];
    nav->vy_mps = nav->ekf_state[EKF_VY];
    nav->gyro_z_bias_dps = nav->ekf_state[EKF_BIAS] *
                           MECANUM_INERTIAL_RAD_TO_DEG;
    nav->last_imu_frame_count = imu_data.angular_velocity_frame_count;
    return MECANUM_INERTIAL_NAV_OK;
}

mecanum_inertial_nav_result_t mecanum_inertial_nav_set_target(
    mecanum_inertial_nav_t *nav,
    float target_x_m,
    float target_y_m,
    float target_yaw_rad,
    float max_vx_mps,
    float max_vy_mps,
    float max_wz_rad_s,
    float kp_position,
    float kp_yaw,
    float position_tolerance_m,
    float yaw_tolerance_rad)
{
    if ((nav == NULL) || (nav->initialized == 0U) ||
        (max_vx_mps <= 0.0f) || (max_vy_mps <= 0.0f) ||
        (max_wz_rad_s <= 0.0f) || (kp_position <= 0.0f) ||
        (kp_yaw <= 0.0f) || (position_tolerance_m <= 0.0f) ||
        (yaw_tolerance_rad <= 0.0f))
    {
        return MECANUM_INERTIAL_NAV_INVALID_PARAM;
    }

    nav->target_x_m = target_x_m;
    nav->target_y_m = target_y_m;
    nav->target_yaw_rad = mecanum_nav_wrap_angle(target_yaw_rad);
    nav->target_max_vx_mps = max_vx_mps;
    nav->target_max_vy_mps = max_vy_mps;
    nav->target_max_wz_rad_s = max_wz_rad_s;
    nav->target_kp_position = kp_position;
    nav->target_kp_yaw = kp_yaw;
    nav->target_position_tolerance_m = position_tolerance_m;
    nav->target_yaw_tolerance_rad = yaw_tolerance_rad;
    nav->target_active = 1U;
    return MECANUM_INERTIAL_NAV_OK;
}

void mecanum_inertial_nav_cancel_target(mecanum_inertial_nav_t *nav,
                                        mecanum_chassis_t *chassis)
{
    if (nav != NULL)
    {
        nav->target_active = 0U;
    }
    if (chassis != NULL)
    {
        (void)mecanum_chassis_stop(chassis);
    }
}

mecanum_inertial_nav_result_t mecanum_inertial_nav_control_step(
    mecanum_inertial_nav_t *nav,
    mecanum_chassis_t *chassis)
{
    float error_x_world;
    float error_y_world;
    float error_x_body;
    float error_y_body;
    float error_yaw;
    float distance_error;
    float vx_command;
    float vy_command;
    float wz_command;

    if ((nav == NULL) || (chassis == NULL) || (nav->initialized == 0U))
    {
        return MECANUM_INERTIAL_NAV_INVALID_PARAM;
    }
    if (nav->target_active == 0U)
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }

    error_x_world = nav->target_x_m - nav->x_m;
    error_y_world = nav->target_y_m - nav->y_m;
    error_yaw = mecanum_nav_wrap_angle(nav->target_yaw_rad - nav->yaw_rad);
    distance_error = sqrtf(error_x_world * error_x_world +
                           error_y_world * error_y_world);
    if ((distance_error <= nav->target_position_tolerance_m) &&
        (mecanum_nav_abs(error_yaw) <= nav->target_yaw_tolerance_rad))
    {
        nav->target_active = 0U;
        (void)mecanum_chassis_stop(chassis);
        return MECANUM_INERTIAL_NAV_TARGET_REACHED;
    }

    error_x_body = cosf(nav->yaw_rad) * error_x_world +
                   sinf(nav->yaw_rad) * error_y_world;
    error_y_body = -sinf(nav->yaw_rad) * error_x_world +
                   cosf(nav->yaw_rad) * error_y_world;
    vx_command = mecanum_nav_clamp(nav->target_kp_position * error_x_body,
                                   -nav->target_max_vx_mps,
                                   nav->target_max_vx_mps);
    vy_command = mecanum_nav_clamp(nav->target_kp_position * error_y_body,
                                   -nav->target_max_vy_mps,
                                   nav->target_max_vy_mps);
    wz_command = mecanum_nav_clamp(nav->target_kp_yaw * error_yaw,
                                   -nav->target_max_wz_rad_s,
                                   nav->target_max_wz_rad_s);

    if (mecanum_chassis_set_velocity(chassis,
                                     vx_command,
                                     vy_command,
                                     wz_command) == MECANUM_MOTOR_ERROR)
    {
        return MECANUM_INERTIAL_NAV_NOT_READY;
    }
    return MECANUM_INERTIAL_NAV_OK;
}

uint8_t mecanum_inertial_nav_target_reached(
    const mecanum_inertial_nav_t *nav)
{
    return (nav != NULL) ? (uint8_t)(nav->target_active == 0U) : 0U;
}
