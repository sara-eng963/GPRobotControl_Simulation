#!/usr/bin/env python3
"""Run existing protocol fixtures against the renamed target implementation."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "Firmware/STM32H745"
COMMON = FW / "Common"
APP = FW / "CM7/Application/Intercore"


def target_names(text):
    # Only the test fixtures and parity comparison are translated.
    # No host source/API or production source is rewritten.
    replacements = [
        ("GP_TRAJECTORY_", "TRAJECTORY_"), ("GP_TRAJ_", "TRAJECTORY_"), ("gp_traj_", "trajectory_"),
        ("GpTrajectory", "Trajectory"), ("GP_IPC_", "IPC_"),
        ("gp_ipc_", "ipc_"), ("GpIpc", "Ipc"),
        ("GP_VALIDATION_", "VALIDATION_"), ("gp_validation_", "validation_"),
        ("GpValidation", "Validation"), ("GP_PLAYBACK_", "PLAYBACK_"),
        ("gp_playback_", "playback_"), ("GpPlayback", "Playback"),
        ("GpCommitted", "Committed"), ("GP_CRC32_", "CRC32_"),
        ("GP_ACQUIRE", "ATOMIC_ACQUIRE"), ("GP_RELEASE", "ATOMIC_RELEASE"),
        ("GP_RELAXED", "ATOMIC_RELAXED"),
    ]
    for old, new in replacements:
        text = text.replace(old, new)
    return text


def tokens(text):
    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
    return re.findall(r'[A-Za-z_]\w*|0[xX][0-9a-fA-F]+|[0-9]+|[^\s]', text)


def main():
    for name, directory in [
        ("ipc_mailbox.h", COMMON / "Inc"),
        ("trajectory_stream.h", COMMON / "Inc"),
        ("validation_storage_wire.h", COMMON / "Inc"),
        ("ipc_mailbox_freertos.c", COMMON / "Src"),
        ("trajectory_stream.c", COMMON / "Src"),
        ("validation_storage_bridge.h", APP),
        ("validation_storage_bridge.c", APP),
        ("trajectory_storage_service.h", APP),
        ("trajectory_storage_service.c", APP),
        ("trajectory_stream_path_port.h", APP),
        ("trajectory_stream_path_port.c", APP),
    ]:
        expected = tokens(target_names((ROOT / "Intercore" / name).read_text()))
        actual = tokens((directory / name).read_text())
        assert expected == actual, f"Protocol token drift beyond identifier renames: {name}"
        print("PASS: normalized token parity:", name, flush=True)
    with tempfile.TemporaryDirectory(prefix="firmware-protocol-") as tmp:
        tmp = Path(tmp)
        sources = [COMMON / "Src/trajectory_stream.c",
                   APP / "validation_storage_bridge.c",
                   APP / "trajectory_storage_service.c",
                   APP / "trajectory_stream_path_port.c"]
        for name in ("trajectory_stream", "validation_storage_bridge",
                     "trajectory_storage_service", "trajectory_stream_path_port"):
            fixture = target_names((ROOT / "Intercore/Tests" / ("test_" + name + ".c")).read_text())
            fixture = fixture.replace('#include "../', '#include "')
            src = tmp / (name + ".c")
            src.write_text(fixture)
            exe = tmp / name
            subprocess.run(["cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-pthread", "-I" + str(COMMON / "Inc"), "-I" + str(APP),
                str(src)] + [str(p) for p in sources] + ["-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=30)
            print("PASS: target protocol fixture:", name, flush=True)


if __name__ == "__main__":
    main()
