#!/usr/bin/env python3
"""Headless zero-wrench virtual sensor; never starts a hardware interface."""
import socket
import struct
import time

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
    sequence = 0
    while True:
        sequence = (sequence + 1) & 0xffffffff
        sender.sendto(struct.pack('!4I6f', 0x56465431, 1, sequence, 1,
                                  0, 0, 0, 0, 0, 0), ('127.0.0.1', 5013))
        time.sleep(0.01)
