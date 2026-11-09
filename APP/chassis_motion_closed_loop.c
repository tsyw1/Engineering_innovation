#include "chassis_motion_closed_loop.h"

#include "HEADING_HOLD_CONTROLLER.h"
#include "VELOCITY_ODOMETRY.h"
#include "chassis_odometry_motion_config.h"

#include <math.h>
#include <string.h>

#define MOTION_DEG_TO_RAD       0.01745329251994329577f
#define MOTION_ROTATE_SPEED_SCALE 2.0f

typedef struct
{
    mecanum_chassis_t *chassis;
    hwt101ct_t *yaw_sensor;
    heading_hold_controller_t heading;
    float target_yaw_deg;
    uint8_t initialized;
    uint8_t target_valid;
} chassis_motion_context_t;

static chassis_motion_context_t s_motion;

static float motion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float motion_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float motion_wrap_deg(float value)
{
    while (value > 180.0f)
    {
        value -= 360.0f;
    }
    while (value <= -180.0f)
    {
        value += 360.0f;
    }
    return value;
}

static uint32_t motion_age_ms(uint32_t now_ms, uint32_t update_ms)
{
    int32_t elapsed = (int32_t)(now_ms - update_ms);

    return (elapsed < 0) ? 0U : (uint32_t)elapsed;
}

static chassis_motion_result_t motion_get_yaw(
    hwt101ct_data_t *yaw_data)
{
    uint32_t now_ms;

    if ((s_motion.yaw_sensor == NULL) || (yaw_data == NULL) ||
        (hwt101ct_get_data(s_motion.yaw_sensor, yaw_data) != HWT101CT_OK) ||
        (yaw_data->yaw_valid == 0U))
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }
    now_ms = HAL_GetTick();
    if (motion_age_ms(now_ms, yaw_data->yaw_update_ms) >
        CHASSIS_ODOMETRY_MAX_YAW_AGE_MS)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }
    return CHASSIS_MOTION_OK;
}

static chassis_motion_result_t motion_update_feedback(
    mecanum_encoder_velocity_t *velocity,
    hwt101ct_data_t *yaw_data)
{
    if ((s_motion.chassis == NULL) || (velocity == NULL) ||
        (yaw_data == NULL))
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    (void)mecanum_chassis_feedback_task(s_motion.chassis);
    if (mecanum_chassis_update_encoder_velocity(s_motion.chassis) !=
        MECANUM_OK)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }
    if (mecanum_chassis_get_encoder_velocity(s_motion.chassis, velocity) !=
        MECANUM_OK)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }
    return motion_get_yaw(yaw_data);
}

static float motion_profile_speed(uint32_t elapsed_ms,
                                  float remaining_m,
                                  float max_speed_mps)
{
    float ramp_speed;
    float braking_speed;

    ramp_speed = CHASSIS_ODOMETRY_ACCEL_MPS2 *
                 (float)elapsed_ms * 0.001f;
    braking_speed = sqrtf(2.0f * CHASSIS_ODOMETRY_DECEL_MPS2 *
                          ((remaining_m > 0.0f) ? remaining_m : 0.0f));
    if (braking_speed > max_speed_mps)
    {
        braking_speed = max_speed_mps;
    }
    if (braking_speed > ramp_speed)
    {
        braking_speed = ramp_speed;
    }
    if (braking_speed < CHASSIS_ODOMETRY_MIN_SPEED_MPS)
    {
        braking_speed = CHASSIS_ODOMETRY_MIN_SPEED_MPS;
    }
    return braking_speed;
}

static chassis_motion_result_t motion_stop_stable(void)
{
    uint32_t start_ms = HAL_GetTick();
    mecanum_encoder_velocity_t velocity;

    for (;;)
    {
        if (mecanum_chassis_set_velocity(s_motion.chassis,
                                         0.0f, 0.0f, 0.0f) ==
            MECANUM_MOTOR_ERROR)
        {
            return CHASSIS_MOTION_CHASSIS_ERROR;
        }
        (void)mecanum_chassis_feedback_task(s_motion.chassis);
        (void)mecanum_chassis_update_encoder_velocity(s_motion.chassis);
        if ((mecanum_chassis_get_encoder_velocity(
                 s_motion.chassis, &velocity) == MECANUM_OK) &&
            (motion_abs(velocity.vx_mps) <=
             CHASSIS_ODOMETRY_STOP_SPEED_MPS) &&
            (motion_abs(velocity.vy_mps) <=
             CHASSIS_ODOMETRY_STOP_SPEED_MPS) &&
            (motion_abs(velocity.wz_rad_s) <= 0.05f) &&
            ((HAL_GetTick() - start_ms) >=
             CHASSIS_ODOMETRY_STOP_STABLE_MS))
        {
            return CHASSIS_MOTION_OK;
        }
        if ((HAL_GetTick() - start_ms) >=
            CHASSIS_ODOMETRY_STOP_TIMEOUT_MS)
        {
            return CHASSIS_MOTION_TIMEOUT;
        }
        HAL_Delay(CHASSIS_ODOMETRY_CONTROL_PERIOD_MS);
    }
}

static chassis_motion_result_t motion_prepare_target(
    uint8_t rotation,
    int16_t yaw_delta_deg,
    hwt101ct_data_t *yaw_data)
{
    chassis_motion_result_t result;

    result = motion_get_yaw(yaw_data);
    if (result != CHASSIS_MOTION_OK)
    {
        return result;
    }
    if (s_motion.target_valid == 0U)
    {
        s_motion.target_yaw_deg = yaw_data->yaw_deg;
        s_motion.target_valid = 1U;
    }
    if (rotation != 0U)
    {
        s_motion.target_yaw_deg = motion_wrap_deg(
            s_motion.target_yaw_deg + (float)yaw_delta_deg);
    }
    heading_hold_controller_enable(&s_motion.heading);
    if (heading_hold_controller_set_target(&s_motion.heading,
                                           s_motion.target_yaw_deg) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }
    return CHASSIS_MOTION_OK;
}

static chassis_motion_result_t motion_linear(
    int16_t body_x_mm,
    int16_t body_y_mm)
{
    velocity_odometry_t odometry;
    mecanum_encoder_velocity_t velocity;
    hwt101ct_data_t yaw_data;
    float start_x = 0.0f;
    float start_y = 0.0f;
    float target_yaw_rad;
    float direction_x;
    float direction_y;
    float normal_x;
    float normal_y;
    float target_m;
    float progress_m;
    float remaining_m;
    float cross_error;
    float cross_integral = 0.0f;
    float correction;
    float speed;
    float world_x;
    float world_y;
    float body_x;
    float body_y;
    float correction_wz;
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_control_ms = start_ms -
                               CHASSIS_ODOMETRY_CONTROL_PERIOD_MS;
    uint8_t position_ready = 0U;

    if ((body_x_mm == 0) == (body_y_mm == 0))
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    if (velocity_odometry_init(&odometry) != VELOCITY_ODOMETRY_OK)
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    if (motion_prepare_target(0U, 0, &yaw_data) != CHASSIS_MOTION_OK)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }

    target_m = (float)((body_x_mm != 0) ? motion_abs((float)body_x_mm) :
                       motion_abs((float)body_y_mm)) * 0.001f;
    if (body_y_mm > 0)
    {
        target_m += CHASSIS_ODOMETRY_LEFT_COMP_M;
    }
    target_yaw_rad = s_motion.target_yaw_deg * MOTION_DEG_TO_RAD;
    direction_x = ((body_x_mm > 0) ? 1.0f : (body_x_mm < 0) ? -1.0f : 0.0f) *
                  cosf(target_yaw_rad) -
                  ((body_y_mm > 0) ? 1.0f : (body_y_mm < 0) ? -1.0f : 0.0f) *
                  sinf(target_yaw_rad);
    direction_y = ((body_x_mm > 0) ? 1.0f : (body_x_mm < 0) ? -1.0f : 0.0f) *
                  sinf(target_yaw_rad) +
                  ((body_y_mm > 0) ? 1.0f : (body_y_mm < 0) ? -1.0f : 0.0f) *
                  cosf(target_yaw_rad);
    normal_x = -direction_y;
    normal_y = direction_x;

    for (;;)
    {
        uint32_t now_ms = HAL_GetTick();
        float dt_s;

        if ((now_ms - last_control_ms) <
            CHASSIS_ODOMETRY_CONTROL_PERIOD_MS)
        {
            HAL_Delay(CHASSIS_ODOMETRY_CONTROL_PERIOD_MS -
                      (now_ms - last_control_ms));
            continue;
        }
        last_control_ms = HAL_GetTick();
        if ((last_control_ms - start_ms) >
            CHASSIS_ODOMETRY_SEGMENT_TIMEOUT_MS)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_TIMEOUT;
        }
        if (motion_update_feedback(&velocity, &yaw_data) !=
            CHASSIS_MOTION_OK)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_SENSOR_ERROR;
        }
        if (velocity_odometry_update(&odometry,
                                     velocity.vx_mps,
                                     velocity.vy_mps,
                                     yaw_data.yaw_deg,
                                     last_control_ms) !=
            VELOCITY_ODOMETRY_OK)
        {
            continue;
        }
        if (position_ready == 0U)
        {
            (void)velocity_odometry_get_position(&odometry,
                                                 &start_x, &start_y);
            position_ready = 1U;
            continue;
        }
        (void)velocity_odometry_get_position(&odometry, &world_x, &world_y);
        progress_m = (world_x - start_x) * direction_x +
                     (world_y - start_y) * direction_y;
        remaining_m = target_m - progress_m;
        if (remaining_m <= CHASSIS_ODOMETRY_DISTANCE_TOL_M)
        {
            return motion_stop_stable();
        }

        dt_s = (float)CHASSIS_ODOMETRY_CONTROL_PERIOD_MS * 0.001f;
        cross_error = (world_x - start_x) * normal_x +
                      (world_y - start_y) * normal_y;
        cross_integral = motion_clamp(
            cross_integral + cross_error * dt_s,
            -CHASSIS_ODOMETRY_CROSS_INTEGRAL_LIMIT_M_S,
            CHASSIS_ODOMETRY_CROSS_INTEGRAL_LIMIT_M_S);
        speed = motion_profile_speed(last_control_ms - start_ms,
                                     remaining_m,
                                     CHASSIS_ODOMETRY_MAX_SPEED_MPS);
        correction = motion_clamp(
            -CHASSIS_ODOMETRY_CROSS_TRACK_KP * cross_error -
            CHASSIS_ODOMETRY_CROSS_TRACK_KI * cross_integral,
            -motion_clamp(speed * CHASSIS_ODOMETRY_CROSS_SPEED_RATIO,
                          0.0f,
                          CHASSIS_ODOMETRY_MAX_CROSS_SPEED_MPS),
            motion_clamp(speed * CHASSIS_ODOMETRY_CROSS_SPEED_RATIO,
                         0.0f,
                         CHASSIS_ODOMETRY_MAX_CROSS_SPEED_MPS));
        world_x = direction_x * speed + normal_x * correction;
        world_y = direction_y * speed + normal_y * correction;
        target_yaw_rad = yaw_data.yaw_deg * MOTION_DEG_TO_RAD;
        body_x = world_x * cosf(target_yaw_rad) +
                 world_y * sinf(target_yaw_rad);
        body_y = -world_x * sinf(target_yaw_rad) +
                 world_y * cosf(target_yaw_rad);
        if (heading_hold_controller_update(&s_motion.heading,
                                           yaw_data.yaw_deg,
                                           &correction_wz) !=
            HEADING_HOLD_CONTROLLER_OK)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_SENSOR_ERROR;
        }
        if (mecanum_chassis_set_velocity(s_motion.chassis,
                                         body_x, body_y, correction_wz) ==
            MECANUM_MOTOR_ERROR)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_CHASSIS_ERROR;
        }
    }
}

static chassis_motion_result_t motion_rotation(int16_t yaw_delta_deg)
{
    hwt101ct_data_t yaw_data;
    uint32_t start_ms = HAL_GetTick();
    uint32_t last_control_ms = start_ms -
                               CHASSIS_ODOMETRY_CONTROL_PERIOD_MS;
    int8_t direction = (yaw_delta_deg > 0) ? 1 : -1;

    if (yaw_delta_deg == 0)
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    if (motion_prepare_target(1U, yaw_delta_deg, &yaw_data) !=
        CHASSIS_MOTION_OK)
    {
        return CHASSIS_MOTION_SENSOR_ERROR;
    }

    for (;;)
    {
        mecanum_encoder_velocity_t velocity;
        float error_deg;
        float geometry;
        float remaining_m;
        float wheel_speed;
        float max_wz;
        float correction_wz;
        uint32_t now_ms = HAL_GetTick();

        if ((now_ms - last_control_ms) <
            CHASSIS_ODOMETRY_CONTROL_PERIOD_MS)
        {
            HAL_Delay(CHASSIS_ODOMETRY_CONTROL_PERIOD_MS -
                      (now_ms - last_control_ms));
            continue;
        }
        last_control_ms = HAL_GetTick();
        if ((last_control_ms - start_ms) >
            CHASSIS_ODOMETRY_SEGMENT_TIMEOUT_MS)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_TIMEOUT;
        }
        if (motion_update_feedback(&velocity, &yaw_data) !=
            CHASSIS_MOTION_OK)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_SENSOR_ERROR;
        }
        error_deg = motion_wrap_deg(s_motion.target_yaw_deg -
                                    yaw_data.yaw_deg);
        if (motion_abs(error_deg) <= 0.50f)
        {
            return motion_stop_stable();
        }
        geometry = s_motion.chassis->config.half_length_m +
                   s_motion.chassis->config.half_width_m;
        if (geometry <= 0.0f)
        {
            return CHASSIS_MOTION_INVALID_PARAM;
        }
        remaining_m = geometry * motion_abs(error_deg) * MOTION_DEG_TO_RAD;
        wheel_speed = motion_profile_speed(last_control_ms - start_ms,
                                           remaining_m,
                                           CHASSIS_ODOMETRY_MAX_SPEED_MPS *
                                           MOTION_ROTATE_SPEED_SCALE);
        max_wz = wheel_speed / geometry;
        if (heading_hold_controller_update(&s_motion.heading,
                                           yaw_data.yaw_deg,
                                           &correction_wz) !=
            HEADING_HOLD_CONTROLLER_OK)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_SENSOR_ERROR;
        }
        correction_wz *= MOTION_ROTATE_SPEED_SCALE;
        if ((motion_abs(error_deg) > 90.0f) &&
            (((correction_wz > 0.0f) ? 1 : -1) != direction))
        {
            correction_wz = motion_abs(correction_wz) * (float)direction;
        }
        correction_wz = motion_clamp(correction_wz, -max_wz, max_wz);
        if (mecanum_chassis_set_velocity(s_motion.chassis,
                                         0.0f, 0.0f, correction_wz) ==
            MECANUM_MOTOR_ERROR)
        {
            (void)motion_stop_stable();
            return CHASSIS_MOTION_CHASSIS_ERROR;
        }
    }
}

chassis_motion_result_t chassis_motion_closed_loop_init(
    mecanum_chassis_t *chassis,
    hwt101ct_t *yaw_sensor)
{
    if ((chassis == NULL) || (yaw_sensor == NULL))
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    memset(&s_motion, 0, sizeof(s_motion));
    if (heading_hold_controller_init(&s_motion.heading) !=
        HEADING_HOLD_CONTROLLER_OK)
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    s_motion.chassis = chassis;
    s_motion.yaw_sensor = yaw_sensor;
    s_motion.initialized = 1U;
    return CHASSIS_MOTION_OK;
}

chassis_motion_result_t chassis_motion_move_linear_mm(
    mecanum_chassis_t *chassis,
    int16_t body_x_mm,
    int16_t body_y_mm)
{
    if ((s_motion.initialized == 0U) || (chassis != s_motion.chassis))
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    return motion_linear(body_x_mm, body_y_mm);
}

chassis_motion_result_t chassis_motion_rotate_deg(
    mecanum_chassis_t *chassis,
    int16_t yaw_deg)
{
    if ((s_motion.initialized == 0U) || (chassis != s_motion.chassis))
    {
        return CHASSIS_MOTION_INVALID_PARAM;
    }
    return motion_rotation(yaw_deg);
}
