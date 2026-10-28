"""4F1 sustained SIL Kit host experiment. Owns registry, virtual motors and
Supervisor; writes reproducible CSV and report if HAND_GUIDING_EXPERIMENT_OUTPUT_DIR is set.
This measures POSIX/WSL behavior, NOT STM32 real-time deadlines."""
from __future__ import annotations

import os
import pathlib
import socket
import struct
import subprocess
import sys
import tempfile
import time

if len(sys.argv) != 4:
    raise SystemExit("usage: test_supervisor_guidance_4f1_sustained.py "
                     "<registry> <motor_bank> <supervisor>")
registry, motors, supervisor = (
    str(pathlib.Path(a).resolve()) for a in sys.argv[1:]
)
# Default temporary experiment; explicit environment retains logs and charts.
saved = os.environ.get("HAND_GUIDING_EXPERIMENT_OUTPUT_DIR", "").strip()
temp = tempfile.TemporaryDirectory(prefix="hand-guiding-experiment-")
root = pathlib.Path(saved).resolve() if saved else pathlib.Path(temp.name)
root.mkdir(parents=True, exist_ok=True)
guidance_csv = root / "guidance.csv"
timing_csv = root / "timing.csv"
report_dir = root / "analysis"
logs = {n: tempfile.TemporaryFile(mode="w+") for n in (
    "registry", "motors", "supervisor"
)}
processes = {}

def start(name, command, env=None):
    processes[name] = subprocess.Popen(
        command, env=env, stdout=logs[name], stderr=subprocess.STDOUT
    )

def alive():
    for name, p in processes.items():
        if p.poll() is not None:
            logs[name].seek(0)
            raise AssertionError(f"{name} exited early:\n{logs[name].read()[-2500:]}")

def read_status(sock):
    data = sock.recv(400)
    return struct.unpack("!57I", data) if len(data) == 57 * 4 else None

def wait_state(sock, wanted, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        alive()
        try:
            status = read_status(sock)
            if status and status[3] == wanted:
                return status
        except socket.timeout:
            continue
    raise AssertionError(f"Timeout waiting for state {wanted}")

try:
    start("registry", [registry, "--listen-uri", "silkit://localhost:8500"])
    time.sleep(0.75)
    alive()
    start("motors", [motors])
    time.sleep(1.0)
    alive()
    env = os.environ.copy()
    env.update({
        "HAND_GUIDING_DRY_RUN": "1",
        "HAND_GUIDING_SIM_MOTION": "1",
        "HAND_GUIDING_INJECT": "none",
        "HAND_GUIDING_TRACE_FILE": str(guidance_csv),
        "HOST_TIMING_CSV": str(timing_csv),
        "SILKIT_REGISTRY_URI": "silkit://localhost:8500",
    })
    start("supervisor", [supervisor], env=env)
    status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        status.bind(("127.0.0.1", 5012))
        status.settimeout(0.25)
        wait_state(status, 2, 25)
        command.sendto(struct.pack("!8I", 0x484D4933, 3, 1, 1,
                                   0, 0, 0, 0), ("127.0.0.1", 5010))
        wait_state(status, 3, 10)
        start_time = time.monotonic()
        elapsed_goal = 4.0
        state_counts = {}
        while time.monotonic() - start_time < elapsed_goal:
            alive()
            try:
                w = read_status(status)
                if w:
                    state_counts[w[3]] = state_counts.get(w[3], 0) + 1
                    if w[3] in (11, 12):
                        raise AssertionError("Supervisor entered FAULT/ESTOP during sustained host guidance")
            except socket.timeout:
                pass
        # Allow HMI logger + background timing writer to drain recent samples.
        time.sleep(0.25)
    finally:
        status.close()
        command.close()
finally:
    for p in reversed(list(processes.values())):
        if p.poll() is None:
            p.terminate()
    for p in reversed(list(processes.values())):
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=2)

try:
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "scripts"))
    from analyze_guidance_4f1 import run
    result = run(guidance_csv, timing_csv, report_dir, 100, False)
    metrics = result["guidance"]
    assert metrics["span_s"] >= 2.5, f"short trace: {metrics['span_s']} seconds"
    assert metrics["fault_samples"] == 0, "unexpected fault in sustained test"
    assert metrics["nonmonotonic_order_pairs"] == 0, "unordered observer timestamps"
    assert any((v["measured_range_rad"] or 0) > 0.00002
               for v in metrics["axes"].values()), "virtual motor feedback did not change"
    assert result["timing"]["guidance_dispatch"]["samples"] > 100
    print(f"PASS 4F1 sustained host: {metrics['span_s']:.2f}s, "
          f"{metrics['valid_samples']} valid guidance samples, "
          f"{result['timing']['guidance_dispatch']['samples']} recorded dispatches",
          flush=True)
    if saved:
        print(f"REPORT: {report_dir / 'REPORT.md'}", flush=True)
except Exception:
    for name, file in logs.items():
        file.seek(0)
        print(f"===== {name} =====\n{file.read()[-5000:]}", flush=True)
    raise
finally:
    for file in logs.values():
        file.close()
    temp.cleanup()
