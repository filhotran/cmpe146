/*
 * mpu6050.h
 * MPU-6050 sensor driver.
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "imu_common.h"
#include "i2c_driver.h"

/* Initialize sensor. Returns true if WHO_AM_I == 0x68. */
static inline bool mpu6050_init(void)
{
    uint8_t who;

    i2c_write_byte(MPU6050_PWR_MGMT_1, 0x80);   /* reset */
    /* BOARD TODO: uncomment -> delay_cycles(3200000); */

    i2c_write_byte(MPU6050_PWR_MGMT_1, 0x01);   /* wake, PLL clock */
    i2c_write_byte(MPU6050_CONFIG, 0x00);
    i2c_write_byte(MPU6050_SMPLRT_DIV, 0x07);   /* 1000 Hz internal */
    i2c_write_byte(MPU6050_GYRO_CONFIG, 0x08);   /* +/- 500 deg/s */
    i2c_write_byte(MPU6050_ACCEL_CONFIG, 0x00);  /* +/- 2g */

    if (!i2c_read_byte(MPU6050_WHO_AM_I, &who))
        return false;
    return (who == MPU6050_ADDR);
}

/* Read all 6 axes. 14-byte burst: accel(6) + temp(2) + gyro(6). */
static inline bool mpu6050_read(SensorSample_t *s)
{
    uint8_t buf[14];
    if (!i2c_read_burst(MPU6050_ACCEL_XOUT_H, buf, 14))
        return false;

    s->ax = (float)((int16_t)((buf[0]  << 8) | buf[1]))  * ACCEL_SCALE;
    s->ay = (float)((int16_t)((buf[2]  << 8) | buf[3]))  * ACCEL_SCALE;
    s->az = (float)((int16_t)((buf[4]  << 8) | buf[5]))  * ACCEL_SCALE;
    s->gx = (float)((int16_t)((buf[8]  << 8) | buf[9]))  * GYRO_SCALE;
    s->gy = (float)((int16_t)((buf[10] << 8) | buf[11])) * GYRO_SCALE;
    s->gz = (float)((int16_t)((buf[12] << 8) | buf[13])) * GYRO_SCALE;
    return true;
}

/* Calibrate gyro bias. Keep sensor still during this. ~5.5 sec at 100 Hz. */
static inline bool mpu6050_calibrate(CalData_t *cal)
{
    SensorSample_t s;
    float sx = 0, sy = 0, sz = 0;
    int i;

    cal->gx_bias = 0;
    cal->gy_bias = 0;
    cal->gz_bias = 0;

    for (i = 0; i < CAL_DISCARD; i++) {
        if (!mpu6050_read(&s)) return false;
        /* BOARD TODO: uncomment -> usleep(10000); */
    }

    for (i = 0; i < CAL_SAMPLES; i++) {
        if (!mpu6050_read(&s)) return false;
        sx += s.gx;
        sy += s.gy;
        sz += s.gz;
        /* BOARD TODO: uncomment -> usleep(10000); */
    }

    cal->gx_bias = sx / (float)CAL_SAMPLES;
    cal->gy_bias = sy / (float)CAL_SAMPLES;
    cal->gz_bias = sz / (float)CAL_SAMPLES;
    return true;
}

/* Subtract bias from a reading */
static inline void mpu6050_apply_cal(SensorSample_t *s, const CalData_t *cal)
{
    s->gx -= cal->gx_bias;
    s->gy -= cal->gy_bias;
    s->gz -= cal->gz_bias;
}

#endif
