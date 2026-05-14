# IMU Sensor Fusion System

CMPE 146 - Real-Time Embedded System Co-Design, Spring 2026

Real-time IMU sensor fusion on the TI MSPM0G3507 with FreeRTOS.

## Files

```
imu_common.h       Data structures and constants
i2c_driver.h       I2C interface (implement in i2c_driver.c)
mpu6050.h          MPU-6050 sensor driver
fusion.h           Complementary filter
main.c             FreeRTOS application (3 tasks)
imu_host.py        Python host (live plotting)
```

### Remaining Work
1. Create `i2c_driver.c` implementing the 5 functions in `i2c_driver.h`
2. In `mpu6050.h`, uncomment the lines marked `BOARD TODO`
3. Import the FreeRTOS skeleton project from the SDK
4. Replace `main.c`, add headers to include path, add `i2c_driver.c`
5. Build and flash

## Board Setup

### Wiring
| MPU-6050 | LaunchPad |
|----------|-----------|
| VCC | 3.3V |
| GND | GND |
| SDA | I2C1 SDA (configure via SysConfig) |
| SCL | I2C1 SCL (configure via SysConfig) |

## Running

1. Power on the board — red LED on during calibration (~5.5 sec)
2. LED off — data streaming begins at 100 Hz
3. On laptop: `python imu_host.py COM5`
4. Tilt the board — watch pitch/roll update on the live plot

## Host Commands

Type in the terminal while `imu_host.py` is running:
- `h` — request health status
- `c` — trigger recalibration
- `a 0.95` — change filter alpha
- `q` — quit

## Python Requirements

```
pip install pyserial matplotlib
```

## Lab Code Reference

| Component | Lab Source | What We Reused |
|-----------|-----------|----------------|
| UART config (clock, baud, TX/RX) | Lab 6 Ex 1.1 (Echo) | `DL_UART_Main_ClockConfig`, `DL_UART_Main_Config` structs, `uart_init()`, `DL_UART_Main_transmitData()`, `DL_UART_Main_receiveData()` |
| UART `send_message` pattern | Lab 6 Ex 1.2 (Send text) | Wait for TX FIFO not full, send one byte at a time |
| Hardware CRC | Lab 6 Ex 2.1 (CRC measurement) | `DL_CRC_init()`, `DL_CRC_setSeed32()`, `DL_CRC_feedData8()`, `DL_CRC_getResult32()` |
| FreeRTOS task creation | Lab 7 Ex 3.2 (Single task function) | `pthread_create()` with struct parameter passing, `usleep()`, `pthread_attr_setstacksize(1536)` |
| LED control (PA0, active-low) | Lab 2 Ex 1.1 (DriverLib) | `DL_GPIO_enablePower()`, `DL_GPIO_initDigitalOutput(IOMUX_PINCM1)`, `DL_GPIO_setPins/clearPins` |
| Timer interrupt pattern | Lab 3 Ex 3.1 (Interrupt) | ISR → semaphore → task wakeup pattern (used conceptually for 100 Hz trigger) |
