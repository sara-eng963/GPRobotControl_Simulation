"""Actual GUI main + deterministic backend + real UDP. No WSLg claim."""
import json
import socket
import struct
import subprocess
import sys
import time

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1', int(sys.argv[2]) if len(sys.argv)>2 else 5013))
    sock.settimeout(.04)
    proc = subprocess.Popen([sys.argv[1]], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    packets = []
    try:
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            try:
                data = sock.recv(100)
                packets.append((time.monotonic(), struct.unpack('!4I6f',data)))
            except socket.timeout:
                if proc.poll() is not None: break
        output = proc.communicate(timeout=2)[0]
        assert proc.returncode==0, output
        assert 'RENDER_STALL_BEGIN' in output and 'RENDER_STALL_END' in output
        assert len(packets)>30, len(packets)
        gaps = [b[0]-a[0] for a,b in zip(packets,packets[1:])]
        result = {'packets':len(packets),'max_gap_ms':1000*max(gaps),
                  'first_seq':packets[0][1][2],'last_seq':packets[-1][1][2]}
        print(json.dumps(result), flush=True)
        assert max(gaps)<.150, 'rendering delayed heartbeat beyond watchdog'
        assert all(b[1][2]==a[1][2]+1 for a,b in zip(packets,packets[1:])), 'sequence ownership'
        assert any(p[1][4]>.4 for p in packets), 'force never applied'
        # No stale nonzero may survive an unresponsive GUI for >100 ms.
        runs=[]; start=None
        for t,p in packets:
            if p[4]!=0 and start is None: start=t
            if p[4]==0 and start is not None: runs.append(t-start);start=None
        assert len(runs)>=2, 'stalled input failed to expire and rearm'
        assert all(v==0 for _,p in packets[-3:] for v in p[4:]), 'shutdown failed to zero'
        print('PASS: actual GUI render stall, heartbeat, command expiry, focus loss, release and joined shutdown')
    finally:
        if proc.poll() is None: proc.kill();proc.wait()
