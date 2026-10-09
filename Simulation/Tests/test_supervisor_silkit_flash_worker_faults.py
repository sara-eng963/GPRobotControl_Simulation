import pathlib
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

if len(sys.argv) != 5:
    raise SystemExit(
        "usage: test_supervisor_silkit_sequence.py "
        "<sil-kit-registry> <motor-bank> <supervisor>"
    )

TEST_MODE = sys.argv[4]
if TEST_MODE not in ("cancel", "program_fail"):
    raise SystemExit("Expected cancel or program_fail")
registry_bin = str(pathlib.Path(sys.argv[1]).resolve())
motor_bin = str(pathlib.Path(sys.argv[2]).resolve())
supervisor_bin = str(pathlib.Path(sys.argv[3]).resolve())

logs = {
    "registry": tempfile.TemporaryFile(mode="w+"),
    "motors": tempfile.TemporaryFile(mode="w+"),
    "supervisor": tempfile.TemporaryFile(mode="w+"),
}

processes = {}


def start(name, command):
    env = os.environ.copy()
    if name == "supervisor":
        env["GP_STORAGE_BACKEND"] = "flash"
        env["GP_FLASH_TEST_MODE"] = TEST_MODE
    processes[name] = subprocess.Popen(
        command,
        env=env,
        stdout=logs[name],
        stderr=subprocess.STDOUT,
    )


def require_alive(name):
    process = processes[name]
    if process.poll() is not None:
        logs[name].seek(0)
        raise RuntimeError(
            f"{name} exited early:\n{logs[name].read()[-4000:]}"
        )


def stop_all():
    for process in reversed(list(processes.values())):
        if process.poll() is None:
            process.terminate()

    deadline = time.time() + 5.0

    for process in reversed(list(processes.values())):
        if process.poll() is not None:
            continue

        timeout = max(0.0, deadline - time.time())

        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


def dump_log(name):
    log = logs[name]
    log.seek(0)
    text = log.read()
    print(f"\n===== {name} log =====")
    print(text[-5000:])
    return text


start(
    "registry",
    [
        registry_bin,
        "--listen-uri",
        "silkit://localhost:8500",
    ],
)

time.sleep(0.75)
require_alive("registry")

start(
    "motors",
    [motor_bin],
)

time.sleep(1.0)
require_alive("motors")

start(
    "supervisor",
    [supervisor_bin],
)

status_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)
status_socket.bind(
    ("127.0.0.1", 5012)
)
status_socket.settimeout(0.2)

command_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

sequence = 0


def send(command, *arguments):
    global sequence

    sequence += 1

    encoded = [
        struct.unpack(
            "!I",
            struct.pack("!f", value),
        )[0]
        if isinstance(value, float)
        else value
        for value in arguments
    ]

    command_socket.sendto(
        struct.pack(
            "!8I",
            0x484D4933,
            3,
            command,
            sequence,
            *(encoded + [0] * 4)[:4],
        ),
        ("127.0.0.1", 5010),
    )


def wait_for(predicate, seconds=10, monitor_validation=False):
    end = time.time() + seconds
    last = None
    prior_packet_time = None
    validation_packets = 0

    while time.time() < end:
        require_alive("registry")
        require_alive("motors")
        require_alive("supervisor")

        try:
            words = struct.unpack(
                "!57I",
                status_socket.recv(300),
            )

            last = words
            if monitor_validation and words[3] == 4:
                now = time.monotonic()
                if prior_packet_time is not None and now - prior_packet_time > 0.5:
                    raise RuntimeError(
                        "Supervisor status paused >500ms during flash validation"
                    )
                prior_packet_time = now
                validation_packets += 1
            if predicate(words):
                if monitor_validation and validation_packets < 2:
                    raise RuntimeError(
                        "No evidence of live Supervisor status during flash validation"
                    )
                return words

        except socket.timeout:
            pass

    raise RuntimeError(
        (
            "timeout",
            last
            and (
                last[3],
                last[14:26],
                last[26:34],
            ),
        )
    )


def as_float(word):
    return struct.unpack(
        "!f",
        struct.pack("!I", word),
    )[0]


passed = False

try:
    words = wait_for(
        lambda w: w[3] == 2,
        20,
    )

    print(
        "IDLE",
        [as_float(value) for value in words[34:37]],
        flush=True,
    )

    send(1)
    words = wait_for(
        lambda w: w[3] == 3 and w[20],
    )
    print("TEACH", flush=True)

    send(4)
    words = wait_for(
        lambda w: w[45] == 1,
    )
    print("REC1", flush=True)

    xyz = [
        as_float(value)
        for value in words[34:37]
    ]

    xyz[0] += 0.01

    guidance_deadline = time.time() + 10.0

    while True:
        send(
            0x80000001,
            *xyz,
        )

        try:
            words = wait_for(
                lambda w:
                    abs(as_float(w[34]) - xyz[0]) < 0.0003,
                0.25,
            )
            break
        except RuntimeError:
            if time.time() >= guidance_deadline:
                raise
            # Re-send the same teaching request so TPDO4 feedback continues
            # to update while the virtual actuator follower converges.

    print("GUIDED", flush=True)

    send(4)
    words = wait_for(
        lambda w: w[21],
    )
    print("REC2", flush=True)

    # First validation is deliberately interrupted or fails at the flash bus.
    send(5)
    if TEST_MODE == "cancel":
        wait_for(lambda w: w[3] == 4 and w[55] >= sequence, 8)
        # Observe the actual delayed-erase entry in the child's file without
        # disturbing its write offset (pread reads at an explicit position).
        deadline = time.monotonic() + 5.0
        while True:
            require_alive("supervisor")
            size = os.fstat(logs["supervisor"].fileno()).st_size
            data = os.pread(logs["supervisor"].fileno(), size, 0)
            if b"[FLASH_TEST] delayed erase entered" in data:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("flash erase delay was not entered")
            time.sleep(0.02)
        send(12)
        sent_at = time.monotonic()
        wait_for(lambda w: w[3] == 2 and w[55] >= sequence, 5)
        if time.monotonic() - sent_at > 0.5:
            raise RuntimeError("RESET acknowledgement took >500ms")
        print("[PASS] RESET processed while flash worker erase was in flight", flush=True)
        # Give the old worker time to complete its transaction and to cancel.
        time.sleep(0.8)
    else:
        words = wait_for(lambda w: w[3] == 2 and w[23] == 3 and
                         w[24] != 0 and w[55] >= sequence, 12)
        print("[PASS] flash page-program failure rejected trajectory, error=",
              words[24], flush=True)
        send(12)   # Clear invalid program before a new Teaching session.
        wait_for(lambda w: w[3] == 2 and w[55] >= sequence, 5)

    # Re-teach a new program and validate again. This demonstrates that a
    # canceled/failed worker did not leave flash or its owner permanently busy.
    send(1)
    words = wait_for(
        lambda w: w[3] == 3 and w[20],
    )
    print("TEACH", flush=True)

    send(4)
    words = wait_for(
        lambda w: w[45] == 1,
    )
    print("REC1", flush=True)

    xyz = [
        as_float(value)
        for value in words[34:37]
    ]

    xyz[0] += 0.01

    guidance_deadline = time.time() + 10.0

    while True:
        send(
            0x80000001,
            *xyz,
        )

        try:
            words = wait_for(
                lambda w:
                    abs(as_float(w[34]) - xyz[0]) < 0.0003,
                0.25,
            )
            break
        except RuntimeError:
            if time.time() >= guidance_deadline:
                raise
            # Re-send the same teaching request so TPDO4 feedback continues
            # to update while the virtual actuator follower converges.

    print("GUIDED", flush=True)

    send(4)
    words = wait_for(
        lambda w: w[21],
    )
    print("REC2", flush=True)

    send(5)
    words = wait_for(lambda w: w[3] == 2 and w[23] == 2 and
                     w[55] >= sequence, 25, monitor_validation=True)
    if words[31] != 0 or words[32] != 0:
        raise RuntimeError("recovery returned unexpectedly in preview/execution")
    print("[PASS] second validation committed after", TEST_MODE, flush=True)
    passed = True

except Exception as error:
    print(
        "FAIL:",
        error,
        flush=True,
    )

finally:
    stop_all()

    registry_text = dump_log(
        "registry"
    )

    motor_text = dump_log(
        "motors"
    )

    supervisor_text = dump_log(
        "supervisor"
    )

    for log in logs.values():
        log.close()

if not passed:
    raise SystemExit(1)

if "AVATAR SIL Kit motor bank ready" not in motor_text:
    raise SystemExit("FAIL: motor bank not ready")
if supervisor_text.count("[FLASH_SIM] committed+recovered") != 1:
    raise SystemExit("FAIL: expected exactly one successful recovered flash commit")
required = ("[FLASH_TEST] delayed erase entered" if TEST_MODE == "cancel"
            else "[FLASH_TEST] injected page-program transport failure")
if required not in supervisor_text:
    raise SystemExit("FAIL: requested bus-level injection did not execute")
if "wire_feed=ON" in supervisor_text:
    raise SystemExit("FAIL: test unexpectedly enabled production wire feed")
print("PASS: async flash worker", TEST_MODE, "fault-injection recovery", flush=True)
