"""
fake_imu_serial.py
Simulates the MCU by sending fake orientation packets over a serial port.
Use this to test the Python host (imu_host.py) without the board.

SETUP:
  1. Install a virtual serial port pair:
     - Windows: com0com (free) - creates COM3 <-> COM4 pair
     - Mac/Linux: socat
       socat -d -d pty,raw,echo=0 pty,raw,echo=0
       (note the /dev/pts/X numbers it prints)

  2. Run this script on one end:
     python fake_imu_serial.py COM3        # Windows
     python fake_imu_serial.py /dev/pts/2  # Linux/Mac

  3. Run the host on the other end:
     python imu_host.py COM4              # Windows
     python imu_host.py /dev/pts/3        # Linux/Mac

You should see the live plot showing a smooth sinusoidal tilt.
"""

import sys
import struct
import math
import time
import binascii

import serial

SYNC_0, SYNC_1 = 0xAA, 0x55
MSG_ORIENTATION_EULER = 0x01


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def build_packet(msg_id, payload):
    length = len(payload)
    crc_in = bytes([msg_id, length]) + payload
    crc = crc32(crc_in)
    pkt = bytearray([SYNC_0, SYNC_1, msg_id, length])
    pkt.extend(payload)
    pkt.extend(struct.pack('<I', crc))
    return bytes(pkt)


def main():
    if len(sys.argv) < 2:
        print("Usage: python fake_imu_serial.py <PORT>")
        print("  Creates a fake IMU stream for testing imu_host.py")
        sys.exit(1)

    port = sys.argv[1]
    ser = serial.Serial(port, 115200)
    print(f"Sending fake IMU data on {port}...")
    print("Run imu_host.py on the paired port to see the plot.")

    t0 = time.time()
    count = 0

    try:
        while True:
            t = time.time() - t0

            # Simulate tilting motion
            pitch = 30.0 * math.sin(2 * math.pi * t / 4.0)    # 4s period
            roll  = 15.0 * math.sin(2 * math.pi * t / 6.0)    # 6s period
            yaw   = 10.0 * t  # slow constant drift

            # Wrap yaw to [-180, 180]
            yaw = ((yaw + 180) % 360) - 180

            payload = struct.pack('<fff', pitch, roll, yaw)
            pkt = build_packet(MSG_ORIENTATION_EULER, payload)
            ser.write(pkt)

            count += 1
            if count % 100 == 0:
                print(f"[{count}] P={pitch:7.2f} R={roll:7.2f} Y={yaw:7.2f}")

            time.sleep(0.01)  # 100 Hz

    except KeyboardInterrupt:
        print(f"\nSent {count} packets. Done.")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
