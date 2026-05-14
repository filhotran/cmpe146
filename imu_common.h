/*
 * imu_common.h
 * Data structures and constants for the IMU project.
 * CMPE 146 - Spring 2026
 */

#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#include <stdint.h>
#include <stdbool.h>

/* MPU-6050 registers */
#define MPU6050_ADDR         0x68
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

/* Sensor scale factors */
#define ACCEL_SCALE  (1.0f / 16384.0f)   /* +/- 2g */
#define GYRO_SCALE   (1.0f / 65.5f)      /* +/- 500 deg/s */

/* Timing */
#define SAMPLE_RATE_HZ   100
#define DT               (1.0f / SAMPLE_RATE_HZ)
#define DEFAULT_ALPHA    0.98f

/* Calibration */
#define CAL_DISCARD  50
#define CAL_SAMPLES  500

/* UART protocol */
#define SYNC_0  0xAA
#define SYNC_1  0x55

/* Message IDs: MCU -> Host */
#define MSG_ORIENTATION  0x01
#define MSG_HEALTH       0x04
#define MSG_ACK          0x05

/* Message IDs: Host -> MCU */
#define CMD_SET_ALPHA    0x11
#define CMD_CALIBRATE    0x12
#define CMD_GET_HEALTH   0x13

/* Sensor sample: SensorTask -> FusionTask */
typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* deg/s, bias-corrected */
    uint32_t tick;
} SensorSample_t;

/* Fused result: FusionTask -> CommTask */
typedef struct {
    float pitch, roll, yaw;   /* degrees */
    uint32_t tick;
} Orientation_t;

/* Calibration biases */
typedef struct {
    float gx_bias, gy_bias, gz_bias;
} CalData_t;

/* System health */
typedef struct {
    uint32_t uptime_ms;
    uint32_t sensor_errors;
    uint32_t packets_sent;
    uint8_t  calibrated;
} Health_t;

#endif
