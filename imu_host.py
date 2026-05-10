"""
imu_host.py
Real-time IMU orientation viewer.

Receives binary packets from the MCU, verifies CRC,
plots pitch/roll/yaw live with matplotlib.

Usage:
    python imu_host.py COM5              # Windows
    python imu_host.py /dev/ttyACM0      # Linux/Mac
    python imu_host.py COM5 --log        # Save CSV log

Requirements:
    pip install pyserial matplotlib
"""

import sys
import struct
import time
import csv
import threading
from collections import deque
from datetime import datetime

import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Protocol constants (must match imu_common.h)
SYNC_0, SYNC_1 = 0xAA, 0x55
MSG_ORIENTATION_EULER = 0x01
MSG_SYSTEM_HEALTH     = 0x04
MSG_COMMAND_ACK       = 0x05
CMD_SET_FILTER_PARAM    = 0x11
CMD_TRIGGER_CALIBRATION = 0x12
CMD_REQUEST_HEALTH      = 0x13


def crc32_compute(data: bytes) -> int:
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


class PacketParser:
    """State machine that parses byte stream into packets."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 'SYNC0'
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_bytes = bytearray()

    def feed(self, b):
        """Feed one byte. Returns parsed packet dict or None."""
        if self.state == 'SYNC0':
            if b == SYNC_0: self.state = 'SYNC1'
        elif self.state == 'SYNC1':
            if b == SYNC_1: self.state = 'ID'
            elif b == SYNC_0: pass
            else: self.state = 'SYNC0'
        elif self.state == 'ID':
            self.msg_id = b
            self.state = 'LEN'
        elif self.state == 'LEN':
            self.length = b
            self.payload = bytearray()
            self.crc_bytes = bytearray()
            self.state = 'CRC' if b == 0 else 'DATA'
        elif self.state == 'DATA':
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.state = 'CRC'
        elif self.state == 'CRC':
            self.crc_bytes.append(b)
            if len(self.crc_bytes) >= 4:
                rcv_crc = struct.unpack('<I', bytes(self.crc_bytes))[0]
                exp_crc = crc32_compute(
                    bytes([self.msg_id, self.length]) + bytes(self.payload))
                result = {
                    'msg_id': self.msg_id,
                    'payload': bytes(self.payload),
                    'crc_ok': rcv_crc == exp_crc,
                }
                self.reset()
                return result
        return None


def build_command(msg_id, payload=b''):
    """Build a packet to send to the MCU."""
    length = len(payload)
    crc_in = bytes([msg_id, length]) + payload
    crc = crc32_compute(crc_in)
    pkt = bytearray([SYNC_0, SYNC_1, msg_id, length])
    pkt.extend(payload)
    pkt.extend(struct.pack('<I', crc))
    return bytes(pkt)


class Plotter:
    """Real-time matplotlib plot showing last 5 seconds of data."""

    def __init__(self):
        self.pitch = deque(maxlen=500)
        self.roll  = deque(maxlen=500)
        self.yaw   = deque(maxlen=500)
        self.t     = deque(maxlen=500)
        self.t0    = time.time()
        self.lock  = threading.Lock()

        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(
            3, 1, figsize=(10, 8), sharex=True)
        self.fig.suptitle('IMU Sensor Fusion - Live Orientation', fontsize=14)

        self.lp, = self.ax1.plot([], [], 'b-', lw=1.5, label='Pitch')
        self.ax1.set_ylabel('Pitch (°)')
        self.ax1.set_ylim(-90, 90)
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)

        self.lr, = self.ax2.plot([], [], 'r-', lw=1.5, label='Roll')
        self.ax2.set_ylabel('Roll (°)')
        self.ax2.set_ylim(-90, 90)
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)

        self.ly, = self.ax3.plot([], [], 'g-', lw=1.5, label='Yaw')
        self.ax3.set_ylabel('Yaw (°)')
        self.ax3.set_ylim(-180, 180)
        self.ax3.set_xlabel('Time (s)')
        self.ax3.legend(loc='upper right')
        self.ax3.grid(True, alpha=0.3)

        plt.tight_layout()

    def add(self, p, r, y):
        with self.lock:
            self.t.append(time.time() - self.t0)
            self.pitch.append(p)
            self.roll.append(r)
            self.yaw.append(y)

    def _update(self, frame):
        with self.lock:
            if len(self.t) < 2:
                return self.lp, self.lr, self.ly
            t = list(self.t)
            self.lp.set_data(t, list(self.pitch))
            self.lr.set_data(t, list(self.roll))
            self.ly.set_data(t, list(self.yaw))
            xmin, xmax = t[-1] - 5.0, t[-1] + 0.1
            for ax in (self.ax1, self.ax2, self.ax3):
                ax.set_xlim(xmin, xmax)
        return self.lp, self.lr, self.ly

    def run(self):
        animation.FuncAnimation(self.fig, self._update,
            interval=50, blit=False, cache_frame_data=False)
        plt.show()


def reader_thread(ser, parser, plotter, csv_w=None):
    """Background thread: read serial, parse packets, feed plotter."""
    count, errors = 0, 0
    while True:
        try:
            if ser.in_waiting:
                for b in ser.read(ser.in_waiting):
                    pkt = parser.feed(b)
                    if pkt:
                        if not pkt['crc_ok']:
                            errors += 1
                            continue
                        count += 1
                        if pkt['msg_id'] == MSG_ORIENTATION_EULER:
                            p, r, y = struct.unpack('<fff', pkt['payload'][:12])
                            plotter.add(p, r, y)
                            if csv_w:
                                csv_w.writerow([time.time(), p, r, y])
                            if count % 100 == 0:
                                print(f"[{count}] P={p:7.2f} R={r:7.2f} "
                                      f"Y={y:7.2f} (err={errors})")
                        elif pkt['msg_id'] == MSG_SYSTEM_HEALTH:
                            d = pkt['payload']
                            up, errs, sent, rate = struct.unpack('<IIIH', d[:14])
                            print(f"\n[HEALTH] up={up}ms errs={errs} "
                                  f"sent={sent} rate={rate}Hz\n")
                        elif pkt['msg_id'] == MSG_COMMAND_ACK:
                            print(f"[ACK] cmd=0x{pkt['payload'][0]:02X}")
            else:
                time.sleep(0.001)
        except Exception as e:
            print(f"[ERR] {e}")
            time.sleep(0.01)


def main():
    if len(sys.argv) < 2:
        print("Usage: python imu_host.py <PORT> [--log]")
        print("  e.g.: python imu_host.py COM5")
        sys.exit(1)

    port = sys.argv[1]
    do_log = '--log' in sys.argv

    print(f"Connecting to {port} at 115200...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    print("Connected. Commands: h=health, c=calibrate, a <val>=alpha, q=quit")

    parser = PacketParser()
    plotter = Plotter()

    csv_file, csv_w = None, None
    if do_log:
        fn = f"imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        csv_file = open(fn, 'w', newline='')
        csv_w = csv.writer(csv_file)
        csv_w.writerow(['time', 'pitch', 'roll', 'yaw'])
        print(f"Logging to {fn}")

    threading.Thread(target=reader_thread,
        args=(ser, parser, plotter, csv_w), daemon=True).start()

    def cmd_loop():
        while True:
            try:
                c = input().strip().lower()
                if c == 'h':
                    ser.write(build_command(CMD_REQUEST_HEALTH))
                elif c == 'c':
                    ser.write(build_command(CMD_TRIGGER_CALIBRATION))
                elif c.startswith('a '):
                    v = float(c.split()[1])
                    ser.write(build_command(CMD_SET_FILTER_PARAM,
                        struct.pack('<f', v)))
                elif c == 'q':
                    plt.close('all')
                    break
            except (EOFError, ValueError):
                pass

    threading.Thread(target=cmd_loop, daemon=True).start()

    try:
        plotter.run()
    except KeyboardInterrupt:
        pass

    ser.close()
    if csv_file:
        csv_file.close()
    print("Done.")


if __name__ == '__main__':
    main()
