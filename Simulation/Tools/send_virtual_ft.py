#!/usr/bin/env python3
"""Host-only virtual 6-axis F/T input sender. NEVER controls joints or hardware."""
import argparse
import math
import socket
import struct
import time

MAGIC = 0x56465431  # VFT1
VERSION = 1
PORT = 5013

def packet(sequence, enabled, wrench):
    return struct.pack("!4I6f", MAGIC, VERSION, sequence & 0xFFFFFFFF,
                       int(enabled), *wrench)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for axis in ("fx", "fy", "fz", "tx", "ty", "tz"):
        parser.add_argument("--" + axis, type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=2.0,
                        help="seconds applying wrench")
    parser.add_argument("--zero-tail", type=float, default=1.0,
                        help="seconds continuously sending zero wrench")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    args = parser.parse_args()
    wrench = (args.fx, args.fy, args.fz, args.tx, args.ty, args.tz)
    if any(not math.isfinite(x) for x in wrench):
        parser.error("All wrench components must be finite")
    if any(abs(x) > 5.0 for x in wrench[:3]) or any(
            abs(x) > 1.0 for x in wrench[3:]):
        parser.error("Demo limits: |F_i| <= 5 N and |T_i| <= 1 N.m")
    if args.duration < 0 or args.zero_tail < 0 or not (1 <= args.rate_hz <= 100):
        parser.error("invalid duration or rate")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        seq = 1
        start = time.monotonic()
        period = 1.0 / args.rate_hz
        next_send = start
        end = start + args.duration + args.zero_tail
        try:
            while time.monotonic() < end:
                now = time.monotonic()
                active = now - start < args.duration
                values = wrench if active else (0.0,) * 6
                sock.sendto(packet(seq, True, values), ("127.0.0.1", PORT))
                seq += 1
                next_send += period
                time.sleep(max(0.0, next_send - time.monotonic()))
        except KeyboardInterrupt:
            pass
        # A valid zero wrench is a release. Disconnect subsequently expires;
        # the Supervisor watchdog will fault rather than hold stale input.
        for _ in range(3):
            sock.sendto(packet(seq, True, (0.0,) * 6),
                        ("127.0.0.1", PORT))
            seq += 1
            time.sleep(period)
    print("Sender stopped; expect watchdog fault after 150 ms if guidance active.")

if __name__ == "__main__":
    main()
