"""4F2A virtual wrench -> C admittance -> CANopen virtual motors, host only.
Not a physical robot, and not an automated graphical UI test."""
import csv
import os
import pathlib
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

if len(sys.argv) != 4:
    raise SystemExit("usage: test_supervisor_silkit_virtual_ft.py "
                     "<registry> <motor-bank> <supervisor>")
registry, motors, supervisor = [str(pathlib.Path(x).resolve())
                                for x in sys.argv[1:4]]
logs = {k: tempfile.TemporaryFile(mode="w+") for k in
        ("registry", "motors", "supervisor")}
procs = {}
stop_send = threading.Event()
wrench_lock = threading.Lock()
fx_command = [0.0]

def start(name, args, env=None):
    procs[name] = subprocess.Popen(args, env=env, stdout=logs[name],
                                   stderr=subprocess.STDOUT)

def alive():
    for name, p in procs.items():
        if p.poll() is not None:
            logs[name].seek(0)
            raise AssertionError(f"{name} exited: {logs[name].read()[-4000:]}")

def sender_loop():
    seq = 1
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while not stop_send.is_set():
            with wrench_lock:
                fx = fx_command[0]
            sock.sendto(struct.pack("!4I6f", 0x56465431, 1, seq, 1,
                                    fx, 0., 0., 0., 0., 0.),
                        ("127.0.0.1", 5013))
            seq += 1
            stop_send.wait(0.02)

def status_state(sock, want, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        alive()
        try:
            pkt = sock.recv(400)
        except socket.timeout:
            continue
        if len(pkt) != 57 * 4:
            continue
        words = struct.unpack("!57I", pkt)
        if words[3] == 11 and want != 11:
            raise AssertionError("Supervisor reached FAULT prematurely")
        if words[3] == want:
            return words
    raise AssertionError(f"state {want} not reached")

def rows(path):
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

def teaching_status(sock, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        words = status_state(sock, 3, deadline - time.monotonic())
        if predicate(words):
            return words
    raise AssertionError("teaching status condition not reached")

def wait_rows(path, predicate, timeout=8):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        alive()
        rr=rows(path)
        if predicate(rr):
            return rr
        time.sleep(0.05)
    raise AssertionError("timed out waiting for traced guidance rows")

try:
    with tempfile.TemporaryDirectory(prefix="virtual-ft-") as tmp:
        trace = pathlib.Path(tmp) / "guidance.csv"
        start("registry", [registry, "--listen-uri", "silkit://localhost:8500"])
        time.sleep(0.7)
        start("motors", [motors])
        time.sleep(1.0)
        env = os.environ.copy()
        env.update(HAND_GUIDING_DRY_RUN="1", HAND_GUIDING_SIM_MOTION="1",
                   HAND_GUIDING_VIRTUAL_FT="1",
                   HAND_GUIDING_TRACE_FILE=str(trace),
                   SILKIT_REGISTRY_URI="silkit://localhost:8500")
        start("supervisor", [supervisor], env)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as status, \
             socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as hmi:
            status.bind(("127.0.0.1", 5012))
            status.settimeout(0.2)
            status_state(status,2,25)  # IDLE
            thread=threading.Thread(target=sender_loop,daemon=True)
            thread.start()
            time.sleep(0.35)  # Deliver zero wrench before TEACHING starts
            hmi.sendto(struct.pack("!8I", 0x484D4933,3,1,1,0,0,0,0),
                       ("127.0.0.1",5010))
            initial = status_state(status,3,10)
            initial_q = [struct.unpack("!f",struct.pack("!I",v))[0]
                         for v in initial[37:43]]
            # Exercise idle TEACHING for seconds, not just five cycles.
            idle = wait_rows(trace,lambda rr:sum(
                x["valid"]=="1" for x in rr)>=1500)
            assert all(abs(float(x[f"q{i+1}"])-initial_q[i])<1e-5
                       for x in idle if x["valid"]=="1" for i in range(6)), \
                   "zero wrench changed the seeded joint reference"
            teaching_status(status,lambda w:w[20]==1 and w[7]==0)
            hmi.sendto(struct.pack("!8I",0x484D4933,3,4,2,0,0,0,0),
                       ("127.0.0.1",5010))  # Record at rest
            teaching_status(status,lambda w:w[45]==1)
            with wrench_lock: fx_command[0]=0.5
            applied=wait_rows(trace,lambda rr:sum(
                x["valid"]=="1" and float(x["fx"])>0.49 for x in rr)>=150)
            assert any(abs(float(x["vx"]))>1e-7 for x in applied
                       if x["valid"]=="1" and float(x["fx"])>0.49)
            teaching_status(status,lambda w:w[7]==1 and w[20]==0)
            # A raw Record event must still be rejected by Teaching's own
            # unsettled gate; hiding the button alone is insufficient.
            hmi.sendto(struct.pack("!8I",0x484D4933,3,4,3,0,0,0,0),
                       ("127.0.0.1",5010))
            receipt = teaching_status(status,lambda w:w[55]==3)
            rejected = teaching_status(status,lambda w:w[2]>receipt[2]+3)
            assert rejected[45]==1, "Record captured a point during force input"
            moved=False
            end=time.monotonic()+5
            while time.monotonic()<end and not moved:
                alive()
                try: pkt=status.recv(400)
                except socket.timeout: continue
                if len(pkt)!=57*4:continue
                words=struct.unpack("!57I",pkt)
                if words[3]==11:raise AssertionError("unexpected fault while applying force")
                q=[struct.unpack("!f",struct.pack("!I",v))[0] for v in words[37:43]]
                moved=any(abs(a-b)>0.00002 for a,b in zip(q,initial_q))
            assert moved, "virtual-motor feedback did not move"
            with wrench_lock: fx_command[0]=0.0
            released=wait_rows(trace,lambda rr:sum(
                x["valid"]=="1" and abs(float(x["fx"]))<1e-10 and
                int(x["cycle"])>int(applied[-1]["cycle"])
                for x in rr)>=100,6)
            later=[x for x in released if x["valid"]=="1" and
                   abs(float(x["fx"]))<1e-10 and
                   int(x["cycle"])>int(applied[-1]["cycle"])]
            assert len(later)>=100
            assert abs(float(later[-1]["vx"]))<abs(float(later[0]["vx"])), \
                   "velocity did not decay after zero wrench"
            teaching_status(status,lambda w:w[20]==1 and w[7]==0,18)
            stop_send.set()
            thread.join(timeout=2)
            fault = status_state(status,11,7) # watchdog -> Supervisor fault
            assert fault[20]==0, "FAULT falsely advertised Record as available"
            print("PASS virtual F/T SIL Kit: applied, moved, "
                  "idle/settled recording, moving rejection, lost-input fault")
finally:
    stop_send.set()
    for p in reversed(list(procs.values())):
        if p.poll() is None:p.terminate()
    for p in reversed(list(procs.values())):
        try:p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=2)
    if sys.exc_info()[0] is not None:
        for name,stream in logs.items():
            stream.seek(0)
            print(f"=== {name} ===\n{stream.read()[-4000:]}")
    for stream in logs.values():stream.close()
