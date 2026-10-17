"""SIL Kit 4E2B integration: synthetic force, real virtual feedback, NO
admittance-driven target dispatch. Requires external SIL Kit host binaries."""
import csv
import os
import pathlib
import socket
import struct
import subprocess
import sys
import tempfile
import time

if len(sys.argv) != 5:
    raise SystemExit(
        "usage: test_supervisor_silkit_guidance_dryrun.py "
        "<registry> <motor-bank> <supervisor> <none|stale_ft|bad_feedback>"
    )

registry, motors, supervisor, fault = (
    str(pathlib.Path(sys.argv[i]).resolve()) if i < 4 else sys.argv[i]
    for i in range(1, 5)
)
if fault not in ("none", "stale_ft", "bad_feedback"):
    raise SystemExit("unknown fault fixture")

logs = {name: tempfile.TemporaryFile(mode="w+") for name in (
    "registry", "motors", "supervisor"
)}
procs = {}

def start(name, argv, env=None):
    procs[name] = subprocess.Popen(
        argv, env=env, stdout=logs[name], stderr=subprocess.STDOUT
    )

def alive():
    for name, proc in procs.items():
        if proc.poll() is not None:
            logs[name].seek(0)
            raise AssertionError(
                f"{name} exited prematurely:\n{logs[name].read()[-3500:]}"
            )

def wait_state(sock, state, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        alive()
        try:
            packet = sock.recv(400)
            if len(packet) != 57 * 4:
                continue
            words = struct.unpack("!57I", packet)
            if words[3] == state:
                return words
        except socket.timeout:
            pass
    raise AssertionError(f"timeout waiting for Supervisor state {state}")

def read_rows(path):
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return [r for r in csv.DictReader(f)
                if r.get("cycle") and r.get("valid") is not None]

def wait_rows(path, condition, seconds=6):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        alive()
        rows = read_rows(path)
        if condition(rows):
            return rows
        time.sleep(0.05)
    raise AssertionError(
        f"Timed out: {fault}; trace rows={len(read_rows(path))}")

try:
    with tempfile.TemporaryDirectory(prefix="gp-4e2b-") as temp:
        trace = pathlib.Path(temp) / "guidance.csv"
        start("registry", [registry, "--listen-uri",
                           "silkit://localhost:8500"])
        time.sleep(0.75)
        alive()
        start("motors", [motors])
        time.sleep(1.0)
        alive()
        env = os.environ.copy()
        env.update({
            "GP_GUIDANCE_DRY_RUN": "1",
            "GP_GUIDANCE_TRACE_FILE": str(trace),
            "GP_GUIDANCE_INJECT": fault,
            "SILKIT_REGISTRY_URI": "silkit://localhost:8500"
        })
        start("supervisor", [supervisor], env=env)
        status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        status.bind(("127.0.0.1", 5012))
        status.settimeout(0.25)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            wait_state(status, 2, 25)  # HOMING -> IDLE
            sender.sendto(struct.pack(
                "!8I", 0x484D4933, 3, 1, 1, 0, 0, 0, 0
            ), ("127.0.0.1", 5010)) # select LINE / TEACHING
            wait_state(status, 3, 10)

            # New dry-run callback runs only in TEACHING; output is
            # diagnostic and the legacy raw guide interface is disabled.
            rows = wait_rows(trace, lambda rs: sum(
                r["valid"] == "1" for r in rs
            ) >= 5, 10)
            good = [r for r in rows if r["valid"] == "1"]
            assert len(good) >= 5, f"only {len(good)} valid cycles"
            assert any(float(r["vx"]) != 0.0 for r in good)
            assert len({r["time_us"] for r in good}) >= 5
            # Statically held motors while diagnostic q_ref evolves.
            assert len({r["q1"] for r in good}) >= 2

            if fault != "none":
                rows = wait_rows(trace, lambda rs: any(
                    r["fault"] == "1" for r in rs
                ), 5)
                first_fault = next(i for i, r in enumerate(rows)
                                   if r["fault"] == "1")
                assert first_fault > 0
                for row in rows[first_fault:first_fault + 4]:
                    assert row["valid"] == "0"
                    assert all(float(row[key]) == 0.0 for key in (
                        "vx", "q1", "q2", "q3", "q4", "q5", "q6"
                    ))
            print(f"PASS SIL Kit 4E2B {fault}: "
                  f"{len(good)} valid cycles, {len(rows)} rows", flush=True)
        finally:
            sender.close()
            status.close()
finally:
    for p in reversed(list(procs.values())):
        if p.poll() is None:
            p.terminate()
    for p in reversed(list(procs.values())):
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=2)
    if sys.exc_info()[0] is not None:
        for name, stream in logs.items():
            stream.seek(0)
            print(f"=== {name} ===\n{stream.read()[-3000:]}")
    for stream in logs.values():
        stream.close()
