# IMU Sensor Fusion System

**CMPE 146 - Real-Time Embedded System Co-Design, Spring 2026**

Real-time IMU sensor fusion on the TI MSPM0G3507 LaunchPad with FreeRTOS.
Reads accelerometer + gyroscope from MPU-6050 via I2C, fuses with a complementary
filter, and streams orientation to a host PC over UART with CRC-protected binary packets.

## Project Structure

```
imu_project/
├── firmware/
│   ├── inc/                        # All header files
│   │   ├── imu_common.h            # Data structures, constants, message IDs
│   │   ├── fusion.h                # Complementary filter algorithm
│   │   ├── protocol.h              # Binary packet framing and parsing
│   │   ├── mpu6050.h               # MPU-6050 sensor driver
│   │   └── i2c_driver.h            # I2C interface (STUB - needs board)
│   └── src/
│       └── main.c                  # FreeRTOS tasks (Sensor, Fusion, Comm)
├── host/
│   └── imu_host.py                 # Python real-time plotter
├── tests/
│   ├── test_fusion_protocol.py     # Offline tests (no board needed)
│   └── fake_imu_serial.py          # Fake MCU for testing host app
├── docs/                           # Design docs, diagrams
├── .gitignore
└── README.md
```

## Status

| Component | File | Status | Board Needed? |
|-----------|------|--------|---------------|
| Data structures | `imu_common.h` | ✅ Complete | No |
| Complementary filter | `fusion.h` | ✅ Complete | No |
| UART protocol | `protocol.h` | ✅ Complete | No |
| Sensor driver | `mpu6050.h` | ✅ Complete | No (uses i2c_driver) |
| I2C driver | `i2c_driver.h` | ⬜ Interface only | **Yes** |
| FreeRTOS app | `main.c` | ✅ Complete | Yes (to run) |
| Python host | `imu_host.py` | ✅ Complete | No (test with fake_imu) |
| Offline tests | `test_fusion_protocol.py` | ✅ Complete | No |
| Fake MCU simulator | `fake_imu_serial.py` | ✅ Complete | No |

## Quick Start (No Board)

### 1. Run offline tests
```bash
cd tests
python test_fusion_protocol.py
```
This verifies the filter math, packet encoding/decoding, CRC, and sync recovery all work correctly.

### 2. Test the Python host with fake data
You need a virtual serial port pair:

**Mac/Linux:**
```bash
# Terminal 1: create virtual serial pair
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Note the two /dev/pts/X paths it prints

# Terminal 2: run fake MCU
python tests/fake_imu_serial.py /dev/pts/2

# Terminal 3: run host
python host/imu_host.py /dev/pts/3
```

**Windows:**
Install [com0com](https://com0com.sourceforge.net/) to create a virtual COM pair (e.g., COM3 ↔ COM4), then:
```bash
# Terminal 1
python tests/fake_imu_serial.py COM3

# Terminal 2
python host/imu_host.py COM4
```

You should see a live plot with smooth sinusoidal pitch/roll/yaw curves.

## Board Setup (For Teammates)

### 1. Wire the MPU-6050
| MPU-6050 Pin | LaunchPad Pin | Notes |
|---|---|---|
| VCC | 3.3V | From LaunchPad regulator |
| GND | GND | Common ground |
| SDA | I2C1 SDA | Configure via SysConfig |
| SCL | I2C1 SCL | Configure via SysConfig |

### 2. Implement `i2c_driver.c`
Create `firmware/src/i2c_driver.c` implementing the 4 functions in `i2c_driver.h`.
Use MSPM0 DriverLib I2C functions. See the header for details.

### 3. Uncomment delay calls
Search for `BOARD TODO` in `mpu6050.h` — uncomment the `delay_cycles()` calls.

### 4. Import into CCS
1. Import the FreeRTOS skeleton: `ti/mspm0_sdk/examples/rtos/LP_MSPM0G3507/kernel/posix_demo/freertos`
2. Replace `main.c` with `firmware/src/main.c`
3. Add `firmware/inc/` to include path (Project Properties → Build → Compiler → Include Options)
4. Add `i2c_driver.c` to source files
5. Build and flash

### 5. Run the host
```bash
pip install pyserial matplotlib
python host/imu_host.py COM5    # Use your actual COM port
```

## How It Works

1. **Power on** → Red LED on during calibration (~5.5 seconds, keep sensor still)
2. **LED off** → Calibration done, 100 Hz data streaming begins
3. **Python host** → Receives packets, verifies CRC, plots live orientation
4. **Tilt the board** → Watch pitch/roll respond in real-time

## Lab Code Reused

| What | Where From |
|------|------------|
| UART setup (clock, baud, TX/RX) | Lab 6, Exercise 1.1 |
| Hardware CRC (init, feed, result) | Lab 6, Exercise 2.1 |
| FreeRTOS tasks (pthread_create) | Lab 7, Exercise 3.2 |
| LED control (active-low PA0) | Lab 2, Exercise 1.1 |

## Host Commands

While `imu_host.py` is running, type in the terminal:
- `h` — Request system health
- `c` — Trigger recalibration
- `a 0.95` — Change filter alpha (0.0 to 1.0)
- `q` — Quit

## Python Requirements

```
pip install pyserial matplotlib
```
