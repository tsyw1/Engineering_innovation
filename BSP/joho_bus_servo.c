#include "joho_bus_servo.h"

#include <stddef.h>

#define JOHO_BUS_SERVO_WRITE_DATA       0x03U
#define JOHO_BUS_SERVO_TARGET_POSITION  0x2AU
#define JOHO_BUS_SERVO_FRAME_LENGTH     11U
#define JOHO_BUS_SERVO_TX_TIMEOUT_MS    100U

static uint8_t joho_bus_servo_checksum(const uint8_t *data, uint8_t length)
{
    uint16_t sum = 0U;
    uint8_t index;

    for (index = 0U; index < length; index++)
    {
        sum += data[index];
    }

    return (uint8_t)(~sum);
}

joho_bus_servo_result_t joho_bus_servo_set_position(
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    uint16_t position,
    uint16_t run_time_ms)
{
    uint8_t frame[JOHO_BUS_SERVO_FRAME_LENGTH];

    if ((huart == NULL) || (servo_id == 0U) || (servo_id == 0xFFU))
    {
        return JOHO_BUS_SERVO_INVALID_PARAM;
    }

    if (position > JOHO_BUS_SERVO_POSITION_MAX)
    {
        position = JOHO_BUS_SERVO_POSITION_MAX;
    }

    /* FF FF ID LEN WRITE ADDR POS_H POS_L TIME_H TIME_L CHECKSUM */
    frame[0] = 0xFFU;
    frame[1] = 0xFFU;
    frame[2] = servo_id;
    frame[3] = 0x07U;
    frame[4] = JOHO_BUS_SERVO_WRITE_DATA;
    frame[5] = JOHO_BUS_SERVO_TARGET_POSITION;
    frame[6] = (uint8_t)(position >> 8U);
    frame[7] = (uint8_t)(position & 0xFFU);
    frame[8] = (uint8_t)(run_time_ms >> 8U);
    frame[9] = (uint8_t)(run_time_ms & 0xFFU);
    frame[10] = joho_bus_servo_checksum(&frame[2], 8U);

    if (HAL_UART_Transmit(huart,
                          frame,
                          JOHO_BUS_SERVO_FRAME_LENGTH,
                          JOHO_BUS_SERVO_TX_TIMEOUT_MS) != HAL_OK)
    {
        return JOHO_BUS_SERVO_UART_ERROR;
    }

    return JOHO_BUS_SERVO_OK;
}

joho_bus_servo_result_t joho_bus_servo_set_angle(
    UART_HandleTypeDef *huart,
    uint8_t servo_id,
    uint16_t angle_deg,
    uint16_t max_angle_deg,
    uint16_t run_time_ms)
{
    uint32_t position;

    if ((max_angle_deg == 0U) || (angle_deg > max_angle_deg))
    {
        return JOHO_BUS_SERVO_INVALID_PARAM;
    }

    position = ((uint32_t)angle_deg * JOHO_BUS_SERVO_POSITION_MAX) /
               max_angle_deg;

    return joho_bus_servo_set_position(huart,
                                       servo_id,
                                       (uint16_t)position,
                                       run_time_ms);
}
