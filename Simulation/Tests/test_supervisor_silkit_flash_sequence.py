import pathlib
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

if len(sys.argv) != 4:
    raise SystemExit(
        "usage: test_supervisor_silkit_sequence.py "
        "<sil-kit-registry> <motor-bank> <supervisor>"
    )

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


def wait_for(predicate, seconds=10):
    end = time.time() + seconds
    last = None

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

            if predicate(words):
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

    send(5)
    words = wait_for(
        lambda w: w[3] == 2,
        15,
    )
    print(
        "VALIDATION",
        words[23:25],
        flush=True,
    )

    if words[23] != 2:
        raise RuntimeError(
            "invalid path"
        )

    send(5)
    words = wait_for(
        lambda w:
            w[31] == 1 and
            w[3] == 2,
        20,
    )
    print(
        "PREVIEW DONE",
        flush=True,
    )

    send(9)
    words = wait_for(
        lambda w: w[3] == 6,
        10,
    )
    print(
        "PRODUCTION ENTER",
        flush=True,
    )

    send(10)
    wait_for(
        lambda w: w[3] == 9,
    )
    print("PAUSE", flush=True)

    send(10)
    time.sleep(0.1)
    wait_for(
        lambda w: w[3] == 9,
    )
    print(
        "DUPLICATE PAUSE REMAINS PAUSED",
        flush=True,
    )

    send(11)
    wait_for(
        lambda w: w[3] == 6,
    )
    print("RESUME", flush=True)

    wait_for(
        lambda w: w[3] == 2,
        25,
    )
    print(
        "PRODUCTION DONE",
        flush=True,
    )

    # The real Supervisor must release a partially consumed stream on HOME,
    # and automatically prepare it again for the next production execution.
    send(9)
    wait_for(
        lambda w: w[3] == 6 and w[55] >= sequence and as_float(w[33]) > 0,
        10,
    )
    send(13)
    wait_for(lambda w: w[3] == 2 and w[55] >= sequence, 15)
    print("EXECUTION HOME ABORT DONE", flush=True)

    send(9)
    wait_for(lambda w: w[3] == 6 and w[55] >= sequence and w[32] == 0, 10)
    wait_for(lambda w: w[3] == 2 and w[55] >= sequence, 25)
    print("REPEATED PRODUCTION AFTER ABORT DONE", flush=True)

    send(0x80000002)
    wait_for(
        lambda w: w[3] == 12,
    )
    print("ESTOP", flush=True)

    send(0x80000002)
    time.sleep(0.1)
    wait_for(
        lambda w: w[3] == 12,
    )
    print(
        "RELEASE REMAINS LATCHED",
        flush=True,
    )

    send(12)
    time.sleep(0.1)
    wait_for(
        lambda w: w[3] == 12,
    )
    print(
        "RESET WAITS FOR HOME",
        flush=True,
    )

    send(13)
    wait_for(
        lambda w: w[3] == 2,
        15,
    )
    print(
        "HOME RECOVERY DONE",
        flush=True,
    )

    send(
        0x80000003,
        1,
    )
    wait_for(
        lambda w: w[3] == 11,
    )
    print(
        "EXTERNAL FAULT LATCHED",
        flush=True,
    )

    send(
        0x80000003,
        0,
    )
    time.sleep(0.1)
    wait_for(
        lambda w: w[3] == 11,
    )
    print(
        "CLEAR ALONE DOES NOT RECOVER",
        flush=True,
    )

    send(12)
    time.sleep(0.1)
    wait_for(
        lambda w: w[3] == 11,
    )

    send(13)
    wait_for(
        lambda w: w[3] == 2,
        15,
    )
    print(
        "FAULT RESET/HOME RECOVERY DONE",
        flush=True,
    )

    print(
        "PASS: full Supervisor sequence over SIL Kit CAN1",
        flush=True,
    )

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

if "wire_feed=ON" not in supervisor_text:
    raise SystemExit(
        "FAIL: production never enabled wire feed"
    )

if "AVATAR SIL Kit motor bank ready" not in motor_text:
    raise SystemExit(
        "FAIL: SIL Kit motor bank did not report ready"
    )

if "[FLASH_SIM] committed+recovered" not in supervisor_text:
    raise SystemExit("FAIL: host flash storage was not committed and reloaded")
