#!/usr/bin/env python3
"""RS02 motor fault / calibration diagnostic tool.

Sends a disable command to each specified motor and parses the type-2 feedback
frame to check fault bits (including encoder calibration status).

Dependencies: Python 3 standard library only (socket, struct).

Usage:
    python3 rs02_fault_check.py can1:11 can2:21
    python3 rs02_fault_check.py can1:11,12,13,14,15 can2:21,22,23,24,25
"""

import math
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
#  CAN constants
# ---------------------------------------------------------------------------
CAN_EFF_FLAG = 0x80000000  # Extended frame format
CAN_RAW = 1
CAN_FRAME_FMT = "=IB3x8s"  # struct can_frame: {u32 id, u8 dlc, pad[3], u8 data[8]}
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

# ---------------------------------------------------------------------------
#  RS02 protocol constants (private protocol, extended frame)
# ---------------------------------------------------------------------------
HOST_ID = 0xFD  # Arbitrary host CAN ID (matches spec examples)

TYPE_FEEDBACK = 0x02  # Motor → Host: feedback
TYPE_DISABLE = 0x04   # Host → Motor: disable (safe, no motion)

# Type-2 feedback: fault bits in arb ID bits [21:16]
#   bit offset within the 6-bit fault field (bit21 = offset 5, bit16 = offset 0)
FAULT_DEFS = [
    (5, "未校定 (Encoder uncalibrated)"),
    (4, "積分過大障害 (Integral overload)"),
    (3, "磁気エンコーダ障害 (Magnetic encoder fault)"),
    (2, "過温度 (Over temperature)"),
    (1, "過電流 (Over current)"),
    (0, "欠圧障害 (Under voltage)"),
]

MODE_NAMES = {
    0: "リセット/復位 (Reset)",
    1: "キャリブレーション (Calibration)",
    2: "モータモード (Running)",
}

# Type-2 data scale (spec: Byte0-1 pos [-4π,4π], Byte2-3 vel [-44,44], etc.)
SCALE_POS = 4.0 * math.pi  # rad
SCALE_VEL = 44.0            # rad/s
SCALE_TORQUE = 17.0          # Nm


# ---------------------------------------------------------------------------
#  Low-level CAN helpers
# ---------------------------------------------------------------------------

def can_open(interface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((interface,))
    s.settimeout(0.1)
    return s


def can_send(sock: socket.socket, arb_id_29: int, data: bytes = b"\x00" * 8):
    frame = struct.pack(CAN_FRAME_FMT, arb_id_29 | CAN_EFF_FLAG, len(data), data.ljust(8, b"\x00"))
    sock.send(frame)


def can_recv(sock: socket.socket, timeout: float = 0.5):
    """Return (raw_arb_id, data_bytes) or None on timeout."""
    sock.settimeout(timeout)
    try:
        raw = sock.recv(CAN_FRAME_SIZE)
        arb_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        return arb_id & ~CAN_EFF_FLAG, data[: dlc & 0x0F]
    except socket.timeout:
        return None


def can_drain(sock: socket.socket):
    """Discard any pending frames."""
    sock.settimeout(0.01)
    while True:
        try:
            sock.recv(CAN_FRAME_SIZE)
        except (socket.timeout, BlockingIOError):
            break


# ---------------------------------------------------------------------------
#  RS02 protocol
# ---------------------------------------------------------------------------

def build_arb_id(comm_type: int, motor_id: int) -> int:
    """Build 29-bit arb ID for host→motor commands.

    Bit28-24: comm_type
    Bit15-8:  HOST_ID
    Bit7-0:   target motor_id
    """
    return (comm_type << 24) | (HOST_ID << 8) | motor_id


def parse_type2(arb_id: int, data: bytes) -> dict | None:
    """Parse type-2 feedback frame. Returns dict or None if not type-2."""
    comm_type = (arb_id >> 24) & 0x1F
    if comm_type != TYPE_FEEDBACK:
        return None

    motor_id = (arb_id >> 8) & 0xFF
    fault_bits = (arb_id >> 16) & 0x3F   # bits [21:16]
    mode_state = (arb_id >> 22) & 0x03   # bits [23:22]

    pos_u16 = (data[0] << 8) | data[1]
    vel_u16 = (data[2] << 8) | data[3]
    tq_u16 = (data[4] << 8) | data[5]
    temp_raw = (data[6] << 8) | data[7]

    return {
        "motor_id": motor_id,
        "mode": mode_state,
        "fault": fault_bits,
        "pos": (pos_u16 / 0x7FFF - 1.0) * SCALE_POS,
        "vel": (vel_u16 / 0x7FFF - 1.0) * SCALE_VEL,
        "torque": (tq_u16 / 0x7FFF - 1.0) * SCALE_TORQUE,
        "temp": temp_raw / 10.0,
    }


def query_motor(interface: str, motor_id: int) -> dict | None:
    """Send disable and read feedback for one motor."""
    sock = can_open(interface)
    try:
        can_drain(sock)
        can_send(sock, build_arb_id(TYPE_DISABLE, motor_id))

        for _ in range(20):
            result = can_recv(sock, timeout=0.5)
            if result is None:
                break
            raw_id, data = result
            fb = parse_type2(raw_id, data)
            if fb and fb["motor_id"] == motor_id:
                return fb
        return None
    finally:
        sock.close()


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def parse_args(args: list[str]) -> list[tuple[str, int]]:
    motors = []
    for arg in args:
        iface, _, ids = arg.partition(":")
        if not ids:
            print(f"Error: '{arg}' — expected canX:id1,id2,...", file=sys.stderr)
            sys.exit(1)
        for mid in ids.split(","):
            motors.append((iface, int(mid)))
    return motors


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip())
        sys.exit(1)

    motors = parse_args(sys.argv[1:])

    print("=" * 60)
    print("  RS02 Fault / Calibration Check")
    print("=" * 60)

    for iface, mid in motors:
        print(f"\n--- {iface} : motor {mid} ---")
        fb = query_motor(iface, mid)

        if fb is None:
            print("  ✗  No response (offline or wrong ID?)")
            continue

        # Mode
        mode = fb["mode"]
        print(f"  Mode:   {MODE_NAMES.get(mode, f'Unknown({mode})')}")

        # Fault
        f = fb["fault"]
        if f == 0:
            print("  Fault:  None ✓")
        else:
            print(f"  Fault:  0b{f:06b}")
            for bit_off, name in FAULT_DEFS:
                if f & (1 << bit_off):
                    print(f"          ✗ {name}")

        # State
        print(f"  Pos:    {fb['pos']:+.4f} rad  ({math.degrees(fb['pos']):+.1f}°)")
        print(f"  Vel:    {fb['vel']:+.4f} rad/s")
        print(f"  Torque: {fb['torque']:+.4f} Nm")
        print(f"  Temp:   {fb['temp']:.1f} °C")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
