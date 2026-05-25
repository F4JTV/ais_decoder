#!/usr/bin/env python3
"""Minimal TCP server that mimics the Django map collector for testing.
Prints each newline-delimited JSON record it receives and validates it parses."""
import socket
import json
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 19999

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", PORT))
srv.listen(1)
print(f"[server] listening on 127.0.0.1:{PORT}", flush=True)

conn, addr = srv.accept()
print(f"[server] client connected: {addr}", flush=True)

buf = b""
count = 0
ok = 0
conn.settimeout(3.0)
try:
    while True:
        try:
            data = conn.recv(4096)
        except socket.timeout:
            break
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            count += 1
            try:
                rec = json.loads(line.decode())
                ok += 1
                print(f"[server] #{count} OK name={rec['name']!r} "
                      f"lat={rec['lat']} lon={rec['lon']} type={rec['type']} "
                      f"speed={rec['speed']} info={rec['info']!r}", flush=True)
            except Exception as e:
                print(f"[server] #{count} PARSE FAIL: {e}  raw={line!r}", flush=True)
finally:
    print(f"[server] received={count} valid_json={ok}", flush=True)
    conn.close()
    srv.close()
