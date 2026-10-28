#!/usr/bin/env python3
"""A02.2: strict production-master / shared-adapter ABI regression.

Run from any working directory. No files are written to the repository.
Strict mode is required for CI and exits nonzero on any ABI/layout mismatch
or behavioral failure. Reproduction of the original known defect remains
available by checking out the previous A02.1 commit (5e74a693).
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "Firmware/STM32H745/CM7/Application/CANComm/CANopen"
TESTS = ROOT / "Firmware/Tests"
ROOT_CAN = ROOT / "CANComm/CANopen"
PORTS = ROOT / "ServoDrive/JointDrive"
MASTER_SOURCES = [FW / f"canopen_{name}.c" for name in
                  ("master", "heartbeat", "nmt", "sdo")]
ADAPTERS = [PORTS / "canopen_drive_commissioning_port.c",
            PORTS / "canopen_joint_drive_port.c"]
REAL_DRIVE_DEPS = [ROOT / "ServoDrive/AvatarM/avatar_m_drive.c",
                   ROOT / "ServoDrive/AvatarM/avatar_m_pdo.c",
                   ROOT / "ServoDrive/CiA402/cia402.c"]


def run(command, *, allow_failure=False):
    print("COMMAND:", shlex.join(map(str, command)), flush=True)
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if result.stdout:
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
    if result.stderr:
        print(result.stderr, end="" if result.stderr.endswith("\n") else "\n",
              file=sys.stderr)
    if not allow_failure and result.returncode:
        raise RuntimeError(f"Command exited {result.returncode}")
    return result


def layout(cmd, workdir, variant):
    exe = workdir / ("probe_" + variant)
    flags = ["-DA02_PROBE_" + variant.upper()]
    run([*cmd, *flags, str(TESTS / "a02_1_canopen_layout_probe.c"),
         "-o", str(exe)])
    r = run([str(exe)])
    m = re.search(r"initialized=(\d+)", r.stdout)
    if m is None:
        raise RuntimeError("Layout probe produced no initialized offset")
    permission = re.search(r"permission=(\d+)", r.stdout)
    size = re.search(r"sizeof=(\d+)", r.stdout)
    align = re.search(r"align=(\d+)", r.stdout)
    sdo = re.search(r"sdo=(\d+)", r.stdout)
    owner = re.search(r"owner_check=(\d+)", r.stdout)
    if not all((permission, size, align, sdo, owner)):
        raise RuntimeError("Incomplete canonical CANopen layout probe: " + r.stdout)
    return {"initialized": int(m.group(1)), "permission": int(permission.group(1)),
            "sizeof": int(size.group(1)), "align": int(align.group(1)),
            "sdo": int(sdo.group(1)), "owner_check": int(owner.group(1))}


def main():
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True, check=True).stdout.strip()
    print("REPOSITORY_HEAD:", head)
    paths = [*MASTER_SOURCES, *ADAPTERS, *REAL_DRIVE_DEPS,
             TESTS / "a02_1_canopen_layout_probe.c",
             TESTS / "test_a02_1_canopen_adapter_abi.c"]
    for p in paths:
        if not p.is_file():
            raise RuntimeError("Required exact source absent: " + str(p))
    compiler = os.environ.get("CC", "cc")
    base = [compiler, "-std=c11", "-O0", "-g", "-Wall", "-Wextra", "-Werror",
            "-fno-strict-aliasing", "-I", str(ROOT)]
    with tempfile.TemporaryDirectory(prefix="a02_1_canopen_abi_") as tmp:
        workdir = Path(tmp)
        print("=== Headers: separate real compilation units ===")
        fw_layout = layout(base, workdir, "firmware")
        root_layout = layout(base, workdir, "root")
        print("FIRMWARE_LAYOUT:", fw_layout)
        print("ROOT_LAYOUT:", root_layout)
        if fw_layout != root_layout:
            raise RuntimeError("CANopen type ABI mismatch: " +
                               f"firmware={fw_layout}, root={root_layout}")

        print("=== Header include tracing: real production and root translation units ===")
        canonical_header = ROOT_CAN / "canopen_master_types.h"
        for src, required_header in [
            (MASTER_SOURCES[0], FW / "canopen_master.h"),
            *[(src, ROOT_CAN / "canopen_master.h") for src in ADAPTERS],
        ]:
            command = [*base, "-H", "-fsyntax-only", str(src)]
            print("COMMAND:", shlex.join(map(str, command)), flush=True)
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            if result.returncode:
                print(result.stderr, file=sys.stderr)
                raise RuntimeError("Compiler header tracing failed for " + str(src))
            header_paths = []
            for line in result.stderr.splitlines():
                match = re.match(r"^\.+\s+(.+canopen_master(?:_types)?\.h)$", line.strip())
                if match:
                    header_paths.append(Path(match.group(1)).resolve())
            print("HEADER_TRACE", src.name, "=>", *header_paths)
            if required_header.resolve() not in header_paths:
                raise RuntimeError("Unexpected CanopenMaster header in " + str(src))
            if canonical_header.resolve() not in header_paths:
                raise RuntimeError("Canonical CANopen type header missing from " + str(src))

        print("=== Production master with both genuine root adapters ===")
        exe = workdir / "adapter_abi_test"
        compilation = [*base, str(TESTS / "test_a02_1_canopen_adapter_abi.c"),
                       *map(str, MASTER_SOURCES), *map(str, ADAPTERS),
                       *map(str, REAL_DRIVE_DEPS), "-lm", "-o", str(exe)]
        run(compilation)
        test = run([str(exe)], allow_failure=True)
        if test.returncode:
            raise RuntimeError("ARCH-001 reproduced: required adapter contract failed")
        print("PASS: canonical CANopen layout, real adapters, and authorization vetoes")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit("FAIL A02.1: " + str(exc))
