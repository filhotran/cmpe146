/*
 * i2c_driver.h
 * I2C driver interface for MPU-6050 communication.
 *
 * ╔══════════════════════════════════════════════════════════╗
 * ║  BOARD REQUIRED: Implement these in i2c_driver.c        ║
 * ║  using MSPM0 DriverLib I2C functions or SysConfig.      ║
 * ║                                                          ║
 * ║  The rest of the firmware calls these functions and      ║
 * ║  doesn't care about the I2C implementation details.      ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * DriverLib functions you'll need:
 *   DL_I2C_enablePower()
 *   DL_I2C_setClockConfig()
 *   DL_I2C_fillControllerTXFIFO()
 *   DL_I2C_startControllerTransfer()
 *   DL_I2C_receiveControllerData()
 *   DL_I2C_isControllerBusBusy()
 *
 * Use I2C1 at 400 kHz (Fast Mode).
 * Configure pins via SysConfig or IOMUX manually.
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize I2C peripheral and configure pins.
 * Returns true on success. */
bool i2c_init(void);

/* Write one byte to a sensor register.
 * Returns true if ACK received. */
bool i2c_write_byte(uint8_t reg_addr, uint8_t data);

/* Read one byte from a sensor register.
 * Returns true on success. */
bool i2c_read_byte(uint8_t reg_addr, uint8_t *data);

/* Burst read len bytes starting at reg_addr.
 * Used for the 14-byte sensor data read.
 * Returns true on success. */
bool i2c_read_burst(uint8_t reg_addr, uint8_t *buf, uint8_t len);

/* Toggle SCL to unstick a stuck I2C bus.
 * Returns true if recovery succeeded. */
bool i2c_bus_recovery(void);

#endif /* I2C_DRIVER_H */
