/*
 * mpu6050.h
 * MPU-6050 sensor driver: initialization, reading, and calibration.
 *
 * This sits on top of i2c_driver.h and handles:
 *   - Sensor power-up and register config
 *   - 14-byte burst read (3-axis accel + temp + 3-axis gyro)
 *   - Raw 16-bit big-endian -> float conversion
 *   - Gyroscope bias calibration
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "imu_common.h"
#include "i2c_driver.h"

/*
 * Initialize the MPU-6050.
 * Resets sensor, configures ranges, verifies WHO_AM_I.
 *
 * Returns true if WHO_AM_I returns 0x68.
 *
 * BOARD TODO: Uncomment the delay_cycles() call after reset.
 */
static inline bool mpu6050_init(void)
{
    uint8_t who;

    if (!i2c_write_byte(MPU6050_PWR_MGMT_1, 0x80))
        return false;

    /* BOARD TODO: Uncomment this line */
    /* delay_cycles(3200000); */  /* ~100ms for reset */

    if (!i2c_write_byte(MPU6050_PWR_MGMT_1, 0x01))
        return false;

    if (!i2c_write_byte(MPU6050_CONFIG, 0x00))
        return false;

    if (!i2c_write_byte(MPU6050_SMPLRT_DIV, 0x07))
        return false;

    if (!i2c_write_byte(MPU6050_GYRO_CONFIG, 0x08))
        return false;

    if (!i2c_write_byte(MPU6050_ACCEL_CONFIG, 0x00))
        return false;

    if (!i2c_read_byte(MPU6050_WHO_AM_I, &who))
        return false;

    return (who == MPU6050_ADDR);
}

/*
 * Read all 6 axes from the sensor.
 *
 * 14-byte burst read starting at 0x3B:
 *   [0-1] Accel X    [2-3] Accel Y    [4-5] Accel Z
 *   [6-7] Temp (skip) [8-9] Gyro X   [10-11] Gyro Y  [12-13] Gyro Z
 *
 * All values are 16-bit signed, big-endian.
 */
static inline bool mpu6050_read(SensorSample_t *sample)
{
    uint8_t buf[14];

    if (!i2c_read_burst(MPU6050_ACCEL_XOUT_H, buf, 14))
        return false;

    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    sample->accel_x = (float)raw_ax * ACCEL_SCALE;
    sample->accel_y = (float)raw_ay * ACCEL_SCALE;
    sample->accel_z = (float)raw_az * ACCEL_SCALE;
    sample->gyro_x  = (float)raw_gx * GYRO_SCALE;
    sample->gyro_y  = (float)raw_gy * GYRO_SCALE;
    sample->gyro_z  = (float)raw_gz * GYRO_SCALE;

    return true;
}

/*
 * Calibrate gyroscope by averaging stationary samples.
 * Discards first CALIBRATION_DISCARD samples, then averages
 * CALIBRATION_SAMPLES readings. Takes ~5.5 seconds at 100 Hz.
 *
 * BOARD TODO: Add delay between reads (usleep or delay_cycles).
 */
static inline bool mpu6050_calibrate(CalibrationData_t *cal)
{
    SensorSample_t sample;

    cal->gyro_bias_x   = 0.0f;
    cal->gyro_bias_y   = 0.0f;
    cal->gyro_bias_z   = 0.0f;
    cal->accel_offset_x = 0.0f;
    cal->accel_offset_y = 0.0f;
    cal->accel_offset_z = 0.0f;

    /* Discard initial samples */
    for (int i = 0; i < CALIBRATION_DISCARD; i++)
    {
        if (!mpu6050_read(&sample))
            return false;
        /* BOARD TODO: delay 10ms here */
    }

    /* Accumulate */
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        if (!mpu6050_read(&sample))
            return false;

        sum_gx += sample.gyro_x;
        sum_gy += sample.gyro_y;
        sum_gz += sample.gyro_z;

        /* BOARD TODO: delay 10ms here */
    }

    cal->gyro_bias_x = sum_gx / (float)CALIBRATION_SAMPLES;
    cal->gyro_bias_y = sum_gy / (float)CALIBRATION_SAMPLES;
    cal->gyro_bias_z = sum_gz / (float)CALIBRATION_SAMPLES;

    return true;
}

/* Subtract stored bias from a sensor reading */
static inline void mpu6050_apply_calibration(
    SensorSample_t *sample,
    const CalibrationData_t *cal)
{
    sample->gyro_x -= cal->gyro_bias_x;
    sample->gyro_y -= cal->gyro_bias_y;
    sample->gyro_z -= cal->gyro_bias_z;
}

#endif /* MPU6050_H */
