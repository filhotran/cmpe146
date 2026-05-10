/*
 * imu_common.h
 * Shared data structures, constants, and message definitions.
 *
 * This is the single source of truth for the entire project.
 * Both the firmware (C) and host (Python) reference these values.
 *
 * CMPE 146 - Spring 2026
 */

#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#include <stdint.h>
#include <stdbool.h>

/* ===== MPU-6050 Register Addresses ===== */
#define MPU6050_ADDR            0x68
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

/* ===== Sensor Conversion Factors ===== */
/* Accel: +/- 2g range -> 16384 LSB/g */
#define ACCEL_SCALE             (1.0f / 16384.0f)
/* Gyro: +/- 500 deg/s range -> 65.5 LSB/(deg/s) */
#define GYRO_SCALE              (1.0f / 65.5f)

/* ===== Timing and Filter Parameters ===== */
#define SAMPLE_RATE_HZ          100
#define DT                      (1.0f / SAMPLE_RATE_HZ)
#define DEFAULT_ALPHA           0.98f
#define DEFAULT_BETA            0.1f

/* ===== Calibration Parameters ===== */
#define CALIBRATION_DISCARD     50      /* Samples to skip at startup */
#define CALIBRATION_SAMPLES     500     /* Samples to average for bias */

/* ===== FreeRTOS Queue Depths ===== */
#define SENSOR_QUEUE_DEPTH      4
#define ORIENT_QUEUE_DEPTH      4

/* ===== UART Protocol Constants ===== */
#define SYNC_BYTE_0             0xAA
#define SYNC_BYTE_1             0x55
#define MAX_PAYLOAD_SIZE        255
#define PACKET_HEADER_SIZE      4       /* sync0 + sync1 + msg_id + length */
#define CRC_SIZE                4

/* ===== Message IDs: MCU -> Host ===== */
#define MSG_ORIENTATION_EULER   0x01
#define MSG_ORIENTATION_QUAT    0x02
#define MSG_RAW_SENSOR          0x03
#define MSG_SYSTEM_HEALTH       0x04
#define MSG_COMMAND_ACK         0x05

/* ===== Message IDs: Host -> MCU ===== */
#define CMD_SET_SAMPLE_RATE     0x10
#define CMD_SET_FILTER_PARAM    0x11
#define CMD_TRIGGER_CALIBRATION 0x12
#define CMD_REQUEST_HEALTH      0x13

/* ===== Data Structures ===== */

/* Raw sensor sample: SensorTask -> FusionTask */
typedef struct {
    float accel_x;      /* g */
    float accel_y;
    float accel_z;
    float gyro_x;       /* deg/s (bias-corrected) */
    float gyro_y;
    float gyro_z;
    uint32_t timestamp;
} SensorSample_t;

/* Fused orientation: FusionTask -> CommTask */
typedef struct {
    float pitch;        /* degrees */
    float roll;
    float yaw;
    uint32_t timestamp;
} FusedOrientation_t;

/* Gyroscope calibration data */
typedef struct {
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
} CalibrationData_t;

/* Runtime config (shared between tasks, mutex-protected) */
typedef struct {
    uint8_t  filter_type;       /* 0 = complementary, 1 = madgwick */
    float    alpha;             /* complementary filter weight */
    float    beta;              /* madgwick filter gain */
    uint16_t sample_rate_hz;
    uint8_t  telemetry_mask;    /* which messages to send */
} SystemConfig_t;

/* UART packet */
typedef struct {
    uint8_t  msg_id;
    uint8_t  length;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint32_t crc;
} Packet_t;

/* System health */
typedef struct {
    uint32_t uptime_ms;
    uint32_t sensor_errors;
    uint32_t packets_sent;
    uint32_t packets_dropped;
    uint8_t  calibrated;
    uint8_t  filter_type;
    uint16_t sample_rate;
} SystemHealth_t;

#endif /* IMU_COMMON_H */
