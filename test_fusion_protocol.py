"""
test_fusion_protocol.py
Offline test: verifies the fusion math and protocol encoding/decoding
without any hardware. Run this on your laptop to confirm everything works.

Usage:
    python test_fusion_protocol.py

This simulates:
  1. Fake sensor data (tilting the board back and forth)
  2. Complementary filter processing
  3. Packet encoding -> decoding round-trip
  4. CRC verification
"""

import struct
import math
import binascii

# ===== Constants (must match imu_common.h) =====
SYNC_0, SYNC_1 = 0xAA, 0x55
MSG_ORIENTATION_EULER = 0x01
DT = 0.01  # 100 Hz
ALPHA = 0.98


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


# ===== Complementary Filter (Python version of fusion.h) =====
class ComplementaryFilter:
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.initialized = False

    def update(self, ax, ay, az, gx, gy, gz, alpha=ALPHA, dt=DT):
        pitch_accel = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        roll_accel  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

        if not self.initialized:
            self.pitch = pitch_accel
            self.roll  = roll_accel
            self.initialized = True
            return

        self.pitch = alpha * (self.pitch + gx * dt) + (1 - alpha) * pitch_accel
        self.roll  = alpha * (self.roll  + gy * dt) + (1 - alpha) * roll_accel
        self.yaw  += gz * dt


# ===== Packet Builder (Python version of protocol.h) =====
def build_packet(msg_id, payload):
    length = len(payload)
    crc_input = bytes([msg_id, length]) + payload
    crc = crc32(crc_input)
    pkt = bytearray([SYNC_0, SYNC_1, msg_id, length])
    pkt.extend(payload)
    pkt.extend(struct.pack('<I', crc))
    return bytes(pkt)


def parse_packet(pkt_bytes):
    """Simple parser for testing (assumes clean input, no sync scanning)."""
    if pkt_bytes[0] != SYNC_0 or pkt_bytes[1] != SYNC_1:
        return None
    msg_id = pkt_bytes[2]
    length = pkt_bytes[3]
    payload = pkt_bytes[4:4+length]
    crc_received = struct.unpack('<I', pkt_bytes[4+length:4+length+4])[0]

    crc_input = bytes([msg_id, length]) + payload
    crc_expected = crc32(crc_input)

    return {
        'msg_id': msg_id,
        'payload': payload,
        'crc_ok': crc_received == crc_expected,
    }


# ===== Tests =====
def test_filter_flat():
    """Sensor lying flat: accel_z = 1g, everything else near zero."""
    print("TEST: Filter with sensor flat...")
    filt = ComplementaryFilter()

    for i in range(200):
        filt.update(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    assert abs(filt.pitch) < 1.0, f"Pitch should be ~0, got {filt.pitch:.2f}"
    assert abs(filt.roll)  < 1.0, f"Roll should be ~0, got {filt.roll:.2f}"
    print(f"  PASS: pitch={filt.pitch:.4f}, roll={filt.roll:.4f}")


def test_filter_tilted():
    """Sensor tilted 45 degrees on pitch axis."""
    print("TEST: Filter with 45° pitch tilt...")
    filt = ComplementaryFilter()

    # ax = sin(45°) ≈ 0.707, az = cos(45°) ≈ 0.707
    ax = math.sin(math.radians(45))
    az = math.cos(math.radians(45))

    for i in range(500):
        filt.update(ax, 0.0, az, 0.0, 0.0, 0.0)

    assert abs(filt.pitch - 45.0) < 2.0, f"Pitch should be ~45, got {filt.pitch:.2f}"
    assert abs(filt.roll) < 2.0, f"Roll should be ~0, got {filt.roll:.2f}"
    print(f"  PASS: pitch={filt.pitch:.4f}, roll={filt.roll:.4f}")


def test_filter_gyro_drift_correction():
    """Gyro has constant bias, filter should correct via accel."""
    print("TEST: Gyro drift correction...")
    filt = ComplementaryFilter()

    # Sensor flat but gyro has 5 deg/s bias on pitch
    for i in range(1000):
        filt.update(0.0, 0.0, 1.0, 5.0, 0.0, 0.0)

    # With alpha=0.98, the accel correction should keep pitch near 0
    # It won't be perfect due to the constant bias, but should be bounded
    assert abs(filt.pitch) < 30.0, f"Pitch drifted too much: {filt.pitch:.2f}"
    print(f"  PASS: pitch={filt.pitch:.4f} (bounded despite gyro bias)")


def test_packet_roundtrip():
    """Build a packet, parse it back, verify CRC and data."""
    print("TEST: Packet round-trip...")

    pitch, roll, yaw = 12.34, -5.67, 89.01
    payload = struct.pack('<fff', pitch, roll, yaw)

    pkt = build_packet(MSG_ORIENTATION_EULER, payload)
    result = parse_packet(pkt)

    assert result is not None, "Parse failed"
    assert result['crc_ok'], "CRC mismatch"
    assert result['msg_id'] == MSG_ORIENTATION_EULER

    p, r, y = struct.unpack('<fff', result['payload'])
    assert abs(p - pitch) < 0.001
    assert abs(r - roll) < 0.001
    assert abs(y - yaw) < 0.001

    print(f"  PASS: decoded pitch={p:.2f}, roll={r:.2f}, yaw={y:.2f}")


def test_packet_corrupted():
    """Flip a byte in the packet, CRC should fail."""
    print("TEST: Corrupted packet detection...")

    payload = struct.pack('<fff', 1.0, 2.0, 3.0)
    pkt = bytearray(build_packet(MSG_ORIENTATION_EULER, payload))

    # Corrupt one payload byte
    pkt[6] ^= 0xFF

    result = parse_packet(bytes(pkt))
    assert result is not None
    assert not result['crc_ok'], "CRC should have failed on corrupted packet"
    print("  PASS: corrupted packet correctly rejected")


def test_state_machine_sync():
    """Parser should find sync pattern in garbage stream."""
    print("TEST: Sync recovery in noisy stream...")

    payload = struct.pack('<fff', 10.0, 20.0, 30.0)
    valid_pkt = build_packet(MSG_ORIENTATION_EULER, payload)

    # Prepend garbage bytes
    stream = bytes([0x12, 0x34, 0xAA, 0x00, 0xFF, 0x55, 0x13]) + valid_pkt

    # Feed byte by byte through state machine
    state = 'SYNC0'
    msg_id = 0
    length = 0
    pld = bytearray()
    crc_bytes = bytearray()
    found = False

    for b in stream:
        if state == 'SYNC0':
            if b == SYNC_0: state = 'SYNC1'
        elif state == 'SYNC1':
            if b == SYNC_1: state = 'ID'
            elif b == SYNC_0: state = 'SYNC1'
            else: state = 'SYNC0'
        elif state == 'ID':
            msg_id = b
            state = 'LEN'
        elif state == 'LEN':
            length = b
            pld = bytearray()
            crc_bytes = bytearray()
            state = 'CRC' if length == 0 else 'DATA'
        elif state == 'DATA':
            pld.append(b)
            if len(pld) >= length: state = 'CRC'
        elif state == 'CRC':
            crc_bytes.append(b)
            if len(crc_bytes) >= 4:
                rcv = struct.unpack('<I', bytes(crc_bytes))[0]
                exp = crc32(bytes([msg_id, length]) + bytes(pld))
                if rcv == exp:
                    found = True
                    break
                state = 'SYNC0'

    assert found, "Failed to find valid packet in noisy stream"
    p, r, y = struct.unpack('<fff', bytes(pld))
    assert abs(p - 10.0) < 0.001
    print(f"  PASS: found packet after garbage, pitch={p:.1f}")


def test_simulate_live_session():
    """Simulate 5 seconds of tilting motion and verify filter output."""
    print("TEST: Simulated 5-second tilt session...")
    filt = ComplementaryFilter()

    results = []

    for i in range(500):  # 5 seconds at 100 Hz
        t = i * DT

        # Simulate slow tilt: pitch goes 0 -> 30 -> 0 degrees
        target_pitch = 30.0 * math.sin(2 * math.pi * t / 5.0)

        ax = math.sin(math.radians(target_pitch))
        az = math.cos(math.radians(target_pitch))
        gx = (30.0 * 2 * math.pi / 5.0) * math.cos(2 * math.pi * t / 5.0)

        filt.update(ax, 0.0, az, gx, 0.0, 0.0)

        # Build packet for this sample
        payload = struct.pack('<fff', filt.pitch, filt.roll, filt.yaw)
        pkt = build_packet(MSG_ORIENTATION_EULER, payload)
        parsed = parse_packet(pkt)
        assert parsed['crc_ok']

        results.append(filt.pitch)

    # Check that the filter tracked the sinusoidal motion
    max_pitch = max(results)
    min_pitch = min(results)
    assert max_pitch > 20.0, f"Max pitch too low: {max_pitch:.2f}"
    assert min_pitch < -20.0, f"Min pitch too high: {min_pitch:.2f}"
    print(f"  PASS: pitch range [{min_pitch:.1f}, {max_pitch:.1f}] "
          f"(expected ~[-30, 30])")


# ===== Run All Tests =====
if __name__ == '__main__':
    print("=" * 50)
    print("IMU Sensor Fusion - Offline Test Suite")
    print("=" * 50)
    print()

    test_filter_flat()
    test_filter_tilted()
    test_filter_gyro_drift_correction()
    test_packet_roundtrip()
    test_packet_corrupted()
    test_state_machine_sync()
    test_simulate_live_session()

    print()
    print("=" * 50)
    print("ALL TESTS PASSED")
    print("=" * 50)
