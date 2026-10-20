#!/usr/bin/env python3
"""Send X-Plane commands over its local UDP interface (port 49000).

Usage: xpcmd.py iias/mission/start_autoflight [more/commands ...]
"""
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for cmd in sys.argv[1:]:
    sock.sendto(b"CMND\0" + cmd.encode() + b"\0", ("127.0.0.1", 49000))
    print("sent", cmd)
