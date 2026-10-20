#!/usr/bin/env python3
"""Read (RREF) or set (DREF) X-Plane datarefs over UDP port 49000.

xpref.py get sim/private/controls/photometric/ev100 [more...]
xpref.py set sim/some/dataref 1.5
"""
import socket
import struct
import sys

XP = ("127.0.0.1", 49000)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 0))
sock.settimeout(2.0)
if sys.argv[1] == "set":
    name, value = sys.argv[2], float(sys.argv[3])
    sock.sendto(b"DREF\0" + struct.pack("<f", value) + name.encode().ljust(500, b"\0"), XP)
    print("set", name, value)
else:
    names = sys.argv[2:]
    for i, n in enumerate(names):
        sock.sendto(b"RREF\0" + struct.pack("<ii", 5, i) + n.encode().ljust(400, b"\0"), XP)
    got = {}
    try:
        while len(got) < len(names):
            data, _ = sock.recvfrom(4096)
            body = data[5:]
            for k in range(0, len(body), 8):
                idx, val = struct.unpack("<if", body[k:k + 8])
                got[idx] = val
    except socket.timeout:
        pass
    for i, n in enumerate(names):
        print(n, got.get(i))
    for i, n in enumerate(names):  # unsubscribe
        sock.sendto(b"RREF\0" + struct.pack("<ii", 0, i) + n.encode().ljust(400, b"\0"), XP)
