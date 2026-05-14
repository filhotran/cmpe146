/*
 * i2c_driver.h
 * I2C interface for MPU-6050.
 * Implement these functions in i2c_driver.c using MSPM0 DriverLib.
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

bool i2c_init(void);
bool i2c_write_byte(uint8_t reg, uint8_t data);
bool i2c_read_byte(uint8_t reg, uint8_t *data);
bool i2c_read_burst(uint8_t reg, uint8_t *buf, uint8_t len);
bool i2c_bus_recovery(void);

#endif
