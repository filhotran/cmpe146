/*
 * protocol.h
 * UART binary protocol: packet framing, CRC, parsing.
 *
 * PACKET FORMAT:
 *   [0xAA] [0x55] [MSG_ID] [LENGTH] [PAYLOAD...] [CRC32 (4 bytes LE)]
 *
 * CRC covers: MSG_ID + LENGTH + PAYLOAD (not sync bytes).
 * On MCU: computed with hardware CRC accelerator.
 * On host: computed with Python binascii.crc32().
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "imu_common.h"
#include <string.h>

/* ===== Packet Building ===== */

/*
 * Build a complete packet into a byte buffer.
 * Returns total packet size in bytes.
 */
static inline uint16_t packet_build(
    uint8_t *buf,
    uint8_t msg_id,
    const uint8_t *payload,
    uint8_t length,
    uint32_t crc_value)
{
    uint16_t idx = 0;

    buf[idx++] = SYNC_BYTE_0;
    buf[idx++] = SYNC_BYTE_1;
    buf[idx++] = msg_id;
    buf[idx++] = length;

    memcpy(&buf[idx], payload, length);
    idx += length;

    buf[idx++] = (uint8_t)(crc_value & 0xFF);
    buf[idx++] = (uint8_t)((crc_value >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((crc_value >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((crc_value >> 24) & 0xFF);

    return idx;
}

/* ===== Payload Builders ===== */

/* Orientation: 12 bytes (3 floats: pitch, roll, yaw) */
static inline uint8_t payload_build_orientation(
    uint8_t *payload,
    const FusedOrientation_t *orient)
{
    memcpy(&payload[0], &orient->pitch, 4);
    memcpy(&payload[4], &orient->roll,  4);
    memcpy(&payload[8], &orient->yaw,   4);
    return 12;
}

/* Raw sensor: 24 bytes (6 floats: ax, ay, az, gx, gy, gz) */
static inline uint8_t payload_build_raw_sensor(
    uint8_t *payload,
    const SensorSample_t *sample)
{
    memcpy(&payload[0],  &sample->accel_x, 4);
    memcpy(&payload[4],  &sample->accel_y, 4);
    memcpy(&payload[8],  &sample->accel_z, 4);
    memcpy(&payload[12], &sample->gyro_x,  4);
    memcpy(&payload[16], &sample->gyro_y,  4);
    memcpy(&payload[20], &sample->gyro_z,  4);
    return 24;
}

/* System health: 16 bytes */
static inline uint8_t payload_build_health(
    uint8_t *payload,
    const SystemHealth_t *health)
{
    memcpy(&payload[0],  &health->uptime_ms,     4);
    memcpy(&payload[4],  &health->sensor_errors,  4);
    memcpy(&payload[8],  &health->packets_sent,   4);
    memcpy(&payload[12], &health->sample_rate,    2);
    payload[14] = health->calibrated;
    payload[15] = health->filter_type;
    return 16;
}

/* ===== Packet Parser State Machine ===== */

typedef enum {
    RX_WAIT_SYNC0,
    RX_WAIT_SYNC1,
    RX_WAIT_ID,
    RX_WAIT_LENGTH,
    RX_COLLECTING_PAYLOAD,
    RX_COLLECTING_CRC
} RxState_t;

typedef struct {
    RxState_t state;
    uint8_t   msg_id;
    uint8_t   length;
    uint8_t   payload[MAX_PAYLOAD_SIZE];
    uint8_t   crc_bytes[CRC_SIZE];
    uint8_t   payload_idx;
    uint8_t   crc_idx;
} RxParser_t;

static inline void rx_parser_init(RxParser_t *rx)
{
    rx->state       = RX_WAIT_SYNC0;
    rx->payload_idx = 0;
    rx->crc_idx     = 0;
}

/*
 * Feed one byte into the parser.
 * Returns true when a complete packet has been received.
 * Access results via rx->msg_id, rx->length, rx->payload, rx->crc_bytes.
 */
static inline bool rx_parser_feed(RxParser_t *rx, uint8_t byte)
{
    switch (rx->state)
    {
    case RX_WAIT_SYNC0:
        if (byte == SYNC_BYTE_0)
            rx->state = RX_WAIT_SYNC1;
        break;

    case RX_WAIT_SYNC1:
        if (byte == SYNC_BYTE_1)
            rx->state = RX_WAIT_ID;
        else if (byte == SYNC_BYTE_0)
            rx->state = RX_WAIT_SYNC1;
        else
            rx->state = RX_WAIT_SYNC0;
        break;

    case RX_WAIT_ID:
        rx->msg_id = byte;
        rx->state  = RX_WAIT_LENGTH;
        break;

    case RX_WAIT_LENGTH:
        rx->length      = byte;
        rx->payload_idx = 0;
        rx->crc_idx     = 0;
        rx->state = (rx->length == 0) ? RX_COLLECTING_CRC : RX_COLLECTING_PAYLOAD;
        break;

    case RX_COLLECTING_PAYLOAD:
        rx->payload[rx->payload_idx++] = byte;
        if (rx->payload_idx >= rx->length)
            rx->state = RX_COLLECTING_CRC;
        break;

    case RX_COLLECTING_CRC:
        rx->crc_bytes[rx->crc_idx++] = byte;
        if (rx->crc_idx >= CRC_SIZE)
        {
            rx->state = RX_WAIT_SYNC0;
            return true;
        }
        break;
    }

    return false;
}

/* Extract 32-bit CRC from received bytes (little-endian) */
static inline uint32_t rx_parser_get_crc(const RxParser_t *rx)
{
    return (uint32_t)rx->crc_bytes[0]
         | ((uint32_t)rx->crc_bytes[1] << 8)
         | ((uint32_t)rx->crc_bytes[2] << 16)
         | ((uint32_t)rx->crc_bytes[3] << 24);
}

#endif /* PROTOCOL_H */
