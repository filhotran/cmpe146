"""
imu_host.py
Receives orientation packets from the MCU, verifies CRC,
plots pitch/roll/yaw in real time.

Usage:
    python imu_host.py COM5              # Windows
    python imu_host.py /dev/ttyACM0      # Mac/Linux

Requirements:
    pip install pyserial matplotlib
"""

import sys
import struct
import time
import threading
import binascii
from collections import deque

import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Protocol constants (match imu_common.h)
SYNC_0, SYNC_1       = 0xAA, 0x55
MSG_ORIENTATION       = 0x01
MSG_HEALTH            = 0x04
MSG_ACK               = 0x05
CMD_SET_ALPHA         = 0x11
CMD_CALIBRATE         = 0x12
CMD_GET_HEALTH        = 0x13


def crc32(data):
    return binascii.crc32(data) & 0xFFFFFFFF


def build_cmd(msg_id, payload=b''):
    length = len(payload)
    crc = crc32(bytes([msg_id, length]) + payload)
    pkt = bytearray([SYNC_0, SYNC_1, msg_id, length])
    pkt.extend(payload)
    pkt.extend(struct.pack('<I', crc))
    return bytes(pkt)


class Parser:
    """Byte-by-byte packet parser. Returns parsed packet or None."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0  # 0=sync0, 1=sync1, 2=id, 3=len, 4=data, 5=crc
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_buf = bytearray()

    def feed(self, b):
        if self.state == 0:
            if b == SYNC_0: self.state = 1
        elif self.state == 1:
            if b == SYNC_1: self.state = 2
            elif b != SYNC_0: self.state = 0
        elif self.state == 2:
            self.msg_id = b; self.state = 3
        elif self.state == 3:
            self.length = b
            self.payload = bytearray()
            self.crc_buf = bytearray()
            self.state = 5 if b == 0 else 4
        elif self.state == 4:
            self.payload.append(b)
            if len(self.payload) >= self.length: self.state = 5
        elif self.state == 5:
            self.crc_buf.append(b)
            if len(self.crc_buf) >= 4:
                rcv = struct.unpack('<I', bytes(self.crc_buf))[0]
                exp = crc32(bytes([self.msg_id, self.length]) + bytes(self.payload))
                result = {'id': self.msg_id, 'data': bytes(self.payload), 'ok': rcv == exp}
                self.reset()
                return result
        return None


def reader(ser, parser, plot_data, lock):
    """Background thread: read serial, parse packets, store data."""
    count = 0
    while True:
        try:
            if ser.in_waiting:
                for b in ser.read(ser.in_waiting):
                    pkt = parser.feed(b)
                    if pkt and pkt['ok'] and pkt['id'] == MSG_ORIENTATION:
                        p, r, y = struct.unpack('<fff', pkt['data'][:12])
                        with lock:
                            plot_data['t'].append(time.time() - plot_data['t0'])
                            plot_data['p'].append(p)
                            plot_data['r'].append(r)
                            plot_data['y'].append(y)
                        count += 1
                        if count % 100 == 0:
                            print(f"[{count}] P={p:7.2f} R={r:7.2f} Y={y:7.2f}")
                    elif pkt and pkt['ok'] and pkt['id'] == MSG_HEALTH:
                        up, err, sent = struct.unpack('<III', pkt['data'][:12])
                        print(f"\n[HEALTH] uptime={up}ms errors={err} sent={sent}\n")
                    elif pkt and pkt['ok'] and pkt['id'] == MSG_ACK:
                        print(f"[ACK] cmd=0x{pkt['data'][0]:02X}")
            else:
                time.sleep(0.001)
        except Exception as e:
            print(f"[ERR] {e}")
            time.sleep(0.01)


def main():
    if len(sys.argv) < 2:
        print("Usage: python imu_host.py <PORT>")
        sys.exit(1)

    ser = serial.Serial(sys.argv[1], 115200, timeout=0.1)
    print(f"Connected to {sys.argv[1]}. Commands: h=health c=calibrate a <val>=alpha q=quit")

    N = 500
    lock = threading.Lock()
    data = {'t': deque(maxlen=N), 'p': deque(maxlen=N),
            'r': deque(maxlen=N), 'y': deque(maxlen=N), 't0': time.time()}

    parser = Parser()
    threading.Thread(target=reader, args=(ser, parser, data, lock), daemon=True).start()

    # Command input thread
    def cmds():
        while True:
            try:
                c = input().strip().lower()
                if c == 'h': ser.write(build_cmd(CMD_GET_HEALTH))
                elif c == 'c': ser.write(build_cmd(CMD_CALIBRATE))
                elif c.startswith('a '): ser.write(build_cmd(CMD_SET_ALPHA, struct.pack('<f', float(c.split()[1]))))
                elif c == 'q': plt.close('all'); break
            except: pass
    threading.Thread(target=cmds, daemon=True).start()

    # Plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle('IMU Sensor Fusion - Live Orientation')
    lp, = ax1.plot([], [], 'b-', lw=1.5); ax1.set_ylabel('Pitch (°)'); ax1.set_ylim(-90, 90); ax1.grid(True, alpha=0.3)
    lr, = ax2.plot([], [], 'r-', lw=1.5); ax2.set_ylabel('Roll (°)');  ax2.set_ylim(-90, 90); ax2.grid(True, alpha=0.3)
    ly, = ax3.plot([], [], 'g-', lw=1.5); ax3.set_ylabel('Yaw (°)');   ax3.set_ylim(-180, 180); ax3.set_xlabel('Time (s)'); ax3.grid(True, alpha=0.3)
    plt.tight_layout()

    def update(frame):
        with lock:
            if len(data['t']) < 2: return lp, lr, ly
            t = list(data['t'])
            lp.set_data(t, list(data['p']))
            lr.set_data(t, list(data['r']))
            ly.set_data(t, list(data['y']))
            xmin, xmax = t[-1] - 5, t[-1] + 0.1
            ax1.set_xlim(xmin, xmax); ax2.set_xlim(xmin, xmax); ax3.set_xlim(xmin, xmax)
        return lp, lr, ly

    animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == '__main__':
    main()
