"""Stage 3B: headless end-to-end virtual F/T robustness over real SIL Kit CANopen.

Run as a CTest subprocess: <registry> <motor-bank> <supervisor> <scenario>.
Uses the existing VFT1 UDP interface. Does not modify or bypass any safety gate.
The --selftest mode validates the test's numerical assertions without SIL Kit.
"""
import csv
import math
import os
import pathlib
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

PORT_VFT = ("127.0.0.1", 5013)
PORT_CMD = ("127.0.0.1", 5010)
VFT_MAGIC = 0x56465431
HMI_MAGIC = 0x484D4933
COMPONENTS = ("fx", "fy", "fz", "tx", "ty", "tz")
TWIST = ("vx", "vy", "vz", "wx", "wy", "wz")
AXES = tuple(f"q{i}" for i in range(1, 7))
ACTUAL = tuple(f"actual_q{i}" for i in range(1, 7))
FAULTS = ("duplicate", "out_of_order", "nonfinite", "truncated",
          "disabled", "range", "dropout")
SCENARIOS = ("matrix", "mixed", "sustained") + FAULTS


def read_trace(path):
    if not path.exists():
        return []
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def eligible(rows):
    return [r for r in rows if r["accepted"] == "1" and r["valid"] == "1"]


def validate_trace(rows):
    good = eligible(rows)
    assert good, "no accepted guidance cycles"
    assert all(int(b["time_us"]) > int(a["time_us"])
               for a, b in zip(good, good[1:])), "guidance timestamps not strictly increasing"
    assert all(int(r["feedback_measurement_valid"]) == 1 for r in good), \
        "accepted cycle lacks valid actual motor feedback"
    assert all(int(r["feedback_max_age_us"]) <= 100000 for r in good), \
        "accepted cycle contains stale feedback beyond configured host limit"
    max_error = 0.0
    max_step = 0.0
    for r in good:
        values = [float(r[k]) for k in COMPONENTS + TWIST + AXES + ACTUAL]
        assert all(math.isfinite(v) for v in values), "nonfinite accepted output"
        max_error = max(max_error, *(abs(float(r[a]) - float(r[q]))
                                     for a, q in zip(ACTUAL, AXES)))
    for a, b in zip(good, good[1:]):
        if int(b["cycle"]) != int(a["cycle"]) + 1:
            continue  # trace recording can drop observer rows; don't fabricate per-tick steps
        max_step = max(max_step, *(abs(float(b[k]) - float(a[k])) for k in AXES))
    assert max_step < 0.0080001, f"commanded joint step exceeded synthetic gate fixture: {max_step}"
    assert max_error < 0.100001, f"joint tracking exceeded command gate fixture: {max_error}"
    return {"valid": len(good), "max_joint_step_rad": max_step,
            "max_ref_feedback_rad": max_error,
            "span_s": (int(good[-1]["time_us"]) - int(good[0]["time_us"])) / 1e6}


def test_assertions():
    def row(cycle, t, q=0., actual=0., fresh=1000):
        r = {k: "0" for k in COMPONENTS + TWIST + AXES + ACTUAL}
        r.update(cycle=str(cycle), time_us=str(t), accepted="1", valid="1",
                 feedback_measurement_valid="1", feedback_max_age_us=str(fresh))
        r["q1"] = str(q)
        r["actual_q1"] = str(actual)
        return r
    data = [row(1, 1000), row(2, 3000, .001, .0009)]
    assert validate_trace(data)["valid"] == 2
    for broken in ([data[0], row(2, 1000)], [data[0], row(2, 3000, .009)],
                   [data[0], row(2, 3000, 0.101, 0.)],
                   [data[0], row(2, 3000, 0., 0., 110000)],
                   [data[0], row(2, 3000, 0.01, float("nan"))]):
        try:
            validate_trace(broken)
        except AssertionError:
            pass
        else:
            raise AssertionError("analyzer silently accepted an invalid trace")
    print("PASS Stage 3B analyzer selftest")


class Sender:
    """One source, one monotonically advancing sequence, no competing UDP writers."""
    def __init__(self):
        self.lock = threading.Lock()
        self.wrench = (0.,) * 6
        self.attack = None
        self.stop = threading.Event()
        self.injected = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def begin(self):
        self.thread.start()

    def set_wrench(self, wrench):
        assert len(wrench) == 6 and all(math.isfinite(w) for w in wrench)
        with self.lock:
            self.wrench = tuple(wrench)

    def inject(self, kind):
        assert kind in FAULTS
        with self.lock:
            self.attack = kind
        assert self.injected.wait(2), "sender never injected disturbance"

    def close(self):
        self.stop.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2)
            assert not self.thread.is_alive(), "sender thread failed to stop"

    def _loop(self):
        seq = 1
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            while not self.stop.is_set():
                with self.lock:
                    w, attack = self.wrench, self.attack
                    self.attack = None
                if attack is not None:
                    if attack != "dropout":
                        packet_seq = seq - 1 if attack == "duplicate" else (
                            seq - 2 if attack == "out_of_order" else seq)
                        values = ((float("nan"),) + w[1:]) if attack == "nonfinite" else w
                        if attack == "range":
                            values = (6.0,) + w[1:]
                        enabled = 0 if attack == "disabled" else 1
                        if attack == "disabled":
                            values = (0.,) * 6
                        packet = struct.pack("!4I6f", VFT_MAGIC, 1, packet_seq,
                                             enabled, *values)
                        if attack == "truncated":
                            packet = packet[:-1]
                        sock.sendto(packet, PORT_VFT)
                    self.injected.set()
                    # Preserve a true dropout or single corrupted packet.
                    self.stop.wait()
                    break
                sock.sendto(struct.pack("!4I6f", VFT_MAGIC, 1, seq, 1, *w), PORT_VFT)
                seq += 1
                self.stop.wait(0.02)


class Rig:
    def __init__(self, registry, motors, supervisor):
        self.binaries = (registry, motors, supervisor)
        self.logs = {k: tempfile.TemporaryFile(mode="w+") for k in
                     ("registry", "motors", "supervisor")}
        self.processes = {}
        self.tmp = tempfile.TemporaryDirectory(prefix="virtual-ft-regression-")
        self.trace = pathlib.Path(self.tmp.name) / "guidance.csv"
        self.status = None
        self.hmi = None
        self.sender = Sender()
        self.hmi_sequence = 0

    def launch(self, name, argv, env=None):
        self.processes[name] = subprocess.Popen(argv, env=env,
                                                stdout=self.logs[name],
                                                stderr=subprocess.STDOUT)

    def check_alive(self):
        for name, proc in self.processes.items():
            if proc.poll() is not None:
                self.logs[name].seek(0)
                raise AssertionError(f"{name} exited: {self.logs[name].read()[-4500:]}")

    def state(self, wanted=None, timeout=12):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check_alive()
            try:
                raw = self.status.recv(400)
            except socket.timeout:
                continue
            if len(raw) != 57 * 4:
                continue
            words = struct.unpack("!57I", raw)
            if words[3] in (11, 12) and wanted not in (11, 12):
                raise AssertionError(f"Supervisor unexpectedly entered fault state {words[3]}")
            if wanted is None or words[3] == wanted:
                return words
        raise AssertionError(f"Supervisor state {wanted} timeout")

    def healthy_sleep(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.state(3, min(.5, max(.1, deadline - time.monotonic())))

    def command(self, command):
        self.hmi_sequence += 1
        self.hmi.sendto(struct.pack("!8I", HMI_MAGIC, 3, command,
                                    self.hmi_sequence, 0, 0, 0, 0), PORT_CMD)

    def wait_trace(self, predicate, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check_alive()
            r = read_trace(self.trace)
            if predicate(r):
                return r
            time.sleep(.075)
        raise AssertionError("trace condition timed out")

    def start(self):
        registry, motors, supervisor = self.binaries
        self.launch("registry", [registry, "--listen-uri", "silkit://localhost:8500"])
        time.sleep(.7)
        self.check_alive()
        self.launch("motors", [motors])
        time.sleep(1.)
        self.check_alive()
        env = os.environ.copy()
        env.update(HAND_GUIDING_DRY_RUN="1", HAND_GUIDING_SIM_MOTION="1",
                   HAND_GUIDING_VIRTUAL_FT="1", HAND_GUIDING_TRACE_FILE=str(self.trace),
                   SILKIT_REGISTRY_URI="silkit://localhost:8500")
        self.launch("supervisor", [supervisor], env)
        self.status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status.bind(("127.0.0.1", 5012))
        self.status.settimeout(.2)
        self.hmi = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state(2, 25)  # HOMING -> IDLE
        self.sender.begin()
        time.sleep(.35)  # Prime UDP stream with valid zero wrench.
        self.command(1)  # LINE / TEACHING
        self.state(3, 10)
        self.wait_trace(lambda rr: len(eligible(rr)) >= 30)

    def cleanup(self, error):
        self.sender.close()
        if self.hmi is not None:
            self.hmi.close()
        if self.status is not None:
            self.status.close()
        for p in reversed(list(self.processes.values())):
            if p.poll() is None:
                p.terminate()
        for p in reversed(list(self.processes.values())):
            try:
                p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=2)
        if error:
            for name, f in self.logs.items():
                f.seek(0)
                print(f"=== {name} ===\n{f.read()[-4500:]}", file=sys.stderr)
        for f in self.logs.values():
            f.close()
        self.tmp.cleanup()


def run_matrix(rig):
    # Force amplitudes are intentionally small synthetic fixtures, not HW limits.
    motions = [("+X", 0, .25), ("-X", 0, -.25),
               ("+Y", 1, .25), ("-Y", 1, -.25),
               ("+Z", 2, .25), ("-Z", 2, -.25),
               ("Tx", 3, .08), ("-Tx", 3, -.08),
               ("Ty", 4, .08), ("-Ty", 4, -.08),
               ("Tz", 5, .08), ("-Tz", 5, -.08)]
    for label, axis, magnitude in motions:
        before = eligible(read_trace(rig.trace))
        baseline = before[-1]
        start_cycle = int(baseline["cycle"])
        wrench = [0.] * 6
        wrench[axis] = magnitude
        rig.sender.set_wrench(wrench)
        # Collect >0.6 s actual accepted cycles, not arbitrary UI events.
        active = rig.wait_trace(lambda rows: sum(
            int(r["cycle"]) > start_cycle and r["valid"] == "1" and
            abs(float(r[COMPONENTS[axis]]) - magnitude) < 1e-5
            for r in rows) >= 300, 12)
        force_rows = [r for r in eligible(active) if int(r["cycle"]) > start_cycle
                      and abs(float(r[COMPONENTS[axis]]) - magnitude) < 1e-5]
        assert len(force_rows) >= 300
        assert float(force_rows[-1][TWIST[axis]]) * magnitude > 1e-7, \
            f"{label}: expected signed Cartesian velocity not observed"
        feedback_change = max(abs(float(force_rows[-1][actual]) -
                                  float(baseline[actual])) for actual in ACTUAL)
        assert feedback_change > 2e-5, f"{label}: motor feedback did not move"
        release_cycle = int(force_rows[-1]["cycle"])
        rig.sender.set_wrench((0.,) * 6)
        zeros = rig.wait_trace(lambda rows: sum(
            int(r["cycle"]) > release_cycle and r["valid"] == "1" and
            abs(float(r[COMPONENTS[axis]])) < 1e-10 for r in rows) >= 350, 12)
        after = [r for r in eligible(zeros) if int(r["cycle"]) > release_cycle and
                 abs(float(r[COMPONENTS[axis]])) < 1e-10]
        v0, v1 = abs(float(after[0][TWIST[axis]])), abs(float(after[-1][TWIST[axis]]))
        assert v1 < v0 * .80, f"{label}: motion failed to decay on release: {v0} -> {v1}"
        assert all(abs(float(r[COMPONENTS[axis]])) < 1e-10 for r in after)
        print(f"{label}: {len(force_rows)} cycles, feedback delta {feedback_change:.7f} rad; "
              f"release velocity {v0:.6g} -> {v1:.6g}", flush=True)
    stats = validate_trace(read_trace(rig.trace))
    print("PASS Stage 3B force/torque matrix", stats, flush=True)


def run_mixed(rig):
    # Verify a diagonal admittance response across simultaneous wrench axes.
    for force in (.06, .12, .18):
        rig.sender.set_wrench((force, 0., 0., 0., 0., .03))
        rig.healthy_sleep(.35)
    positive = eligible(read_trace(rig.trace))[-1]
    assert float(positive["fx"]) > .17 and float(positive["tz"]) > .029
    assert float(positive["vx"]) > 0 and float(positive["wz"]) > 0, \
        "simultaneous Fx/Tz not represented in admittance output"
    # A signed reversal should cross zero without an unsafe discontinuity.
    rig.sender.set_wrench((-.18, 0., 0., 0., 0., -.03))
    rig.healthy_sleep(1.1)
    negative = eligible(read_trace(rig.trace))[-1]
    assert float(negative["fx"]) < -.17 and float(negative["tz"]) < -.029
    assert float(negative["vx"]) < 0 and float(negative["wz"]) < 0, \
        "reversal did not reverse the admittance velocity"
    rig.sender.set_wrench((0.,) * 6)
    rig.healthy_sleep(1.0)
    stats = validate_trace(read_trace(rig.trace))
    assert stats["valid"] >= 700
    print("PASS Stage 3B ramp, mixed wrench and force reversal", stats, flush=True)


def run_sustained(rig):
    rig.sender.set_wrench((0.,) * 6)
    started = time.monotonic()
    # A reversible wrench waveform avoids driving the virtual robot indefinitely.
    for n in range(10):
        for fx, fy, duration in ((.12, .05, .8), (-.12, -.05, .8), (0., 0., .4)):
            rig.sender.set_wrench((fx, fy, 0., 0., 0., 0.))
            rig.healthy_sleep(duration)
    rig.sender.set_wrench((0.,) * 6)
    rig.healthy_sleep(1.0)
    duration = time.monotonic() - started
    trace = read_trace(rig.trace)
    stats = validate_trace(trace)
    assert stats["span_s"] >= 15., f"short trace {stats['span_s']}s"
    assert stats["valid"] >= 2500, f"too few accepted cycles: {stats['valid']}"
    assert not any(r["fault"] == "1" for r in trace), "unexpected pipeline fault"
    assert any(abs(float(r["fx"])) > .11 for r in eligible(trace))
    assert any(abs(float(r["fx"])) < 1e-8 for r in eligible(trace))
    assert any(max(abs(float(r[k]) - float(trace[0][k])) for k in ACTUAL) > 2e-5
               for r in eligible(trace)), "no actual virtual motor motion"
    print(f"PASS Stage 3B sustained: wall={duration:.1f}s", stats, flush=True)


def run_fault(rig, scenario):
    rig.sender.set_wrench((.25, 0., 0., 0., 0., 0.))
    rig.wait_trace(lambda rr: sum(float(r["fx"]) > .24 for r in eligible(rr)) >= 100)
    before = eligible(read_trace(rig.trace))
    assert before and abs(float(before[-1]["vx"])) > 1e-8
    rig.sender.inject(scenario)
    fault = rig.state(11, 6)
    assert fault[20] == 0, "record permission survived guidance failure"
    # Allow diagnostic CSV buffering to drain before checking that accepted
    # cycles stopped; otherwise older queued rows can resemble new dispatch.
    rig.command(1)  # Unauthorized TEACH must not recover FAULT.
    end = time.monotonic() + .5
    while time.monotonic() < end:
        rig.state(11, .5)
    drained_count = len(eligible(read_trace(rig.trace)))
    end = time.monotonic() + .4
    while time.monotonic() < end:
        rig.state(11, .5)
    after = eligible(read_trace(rig.trace))
    assert len(after) <= drained_count + 2, "accepted commands continued after FAULT"
    validate_trace(after)
    print(f"PASS Stage 3B {scenario}: FAULT latched, no post-fault guidance", flush=True)


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        test_assertions()
        return
    if len(sys.argv) != 5 or sys.argv[4] not in SCENARIOS:
        raise SystemExit("usage: script.py <registry> <motor-bank> <supervisor> "
                         f"<{ '|'.join(SCENARIOS) }>")
    registry, motors, supervisor, scenario = sys.argv[1:]
    rig = Rig(registry, motors, supervisor)
    error = None
    try:
        rig.start()
        if scenario == "matrix":
            run_matrix(rig)
        elif scenario == "mixed":
            run_mixed(rig)
        elif scenario == "sustained":
            run_sustained(rig)
        else:
            run_fault(rig, scenario)
    except BaseException as exc:
        error = exc
        raise
    finally:
        rig.cleanup(error is not None)


if __name__ == "__main__":
    main()
