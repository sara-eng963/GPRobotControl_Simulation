#!/usr/bin/env python3
"""A02.1: compile real STM32 master + real root adapters. TEST ONLY.

Run from any working directory. No files are written to the repository.
Default exits nonzero if production behavior fails. Optionally use
--expect-baseline-failure for a local reproduction that exits zero ONLY when
layout differs and the observed behavior test fails (never use as a CI gate).
"""
import argparse
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
    return int(m.group(1)), (int(permission.group(1)) if permission else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect-baseline-failure", action="store_true",
                        help="local evidence mode; not a required-passing CI check")
    args = parser.parse_args()
    expected_sha = "d8924d3acb4ec39e31462c49c680ccfb12f53316"
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True, check=True).stdout.strip()
    print("REPOSITORY_HEAD:", head)
    if head != expected_sha:
        raise RuntimeError("Expected exact A02.1 baseline; no branch switching performed")
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
        fw_offset, permission_offset = layout(base, workdir, "firmware")
        root_offset, _ = layout(base, workdir, "root")
        print(f"OFFSETS firmware_initialized={fw_offset} "
              f"firmware_permission={permission_offset} root_initialized={root_offset}")
        if fw_offset == root_offset or permission_offset != root_offset:
            raise RuntimeError("Expected offset substitution not demonstrated")

        print("=== Header include tracing: real production and root translation units ===")
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
                match = re.match(r"^\.+\s+(.+canopen_master\.h)$", line.strip())
                if match:
                    header_paths.append(Path(match.group(1)).resolve())
            print("HEADER_TRACE", src.name, "=>", *header_paths)
            if required_header.resolve() not in header_paths:
                raise RuntimeError("Unexpected CanopenMaster header in " + str(src))

        print("=== Production master with both genuine root adapters ===")
        exe = workdir / "adapter_abi_test"
        compilation = [*base, str(TESTS / "test_a02_1_canopen_adapter_abi.c"),
                       *map(str, MASTER_SOURCES), *map(str, ADAPTERS),
                       *map(str, REAL_DRIVE_DEPS), "-lm", "-o", str(exe)]
        run(compilation)
        test = run([str(exe)], allow_failure=True)
        if args.expect_baseline_failure:
            if test.returncode == 0:
                raise RuntimeError("Expected baseline failure absent")
            expected_output = [
                "production_initialized=1 permission=0 commissioning_initialized=0",
                "commissioning_configured=0 joint_configured=0",
                "A02.1 granted: commissioning_initialized=1 joint_configured=1",
                "A02.1 revoked: commissioning_initialized=0 joint_configured=0",
            ]
            output = test.stdout + test.stderr
            if not all(needle in output for needle in expected_output):
                raise RuntimeError("Baseline output did not isolate the expected ABI mismatch")
            expected_failures = {
                "init_when_denied", "configured_when_denied",
                "commissioning_configured_when_denied", "init_after_revoke",
                "configured_after_revoke", "init_when_denied == init_when_granted",
                "configured_when_denied == configured_when_granted",
            }
            actual_failures = re.findall(r"^FAIL A02\.1:\d+: (.+)$",
                                         test.stderr, flags=re.MULTILINE)
            if len(actual_failures) != len(expected_failures) or \
                    set(actual_failures) != expected_failures:
                raise RuntimeError("Unexpected failing assertion(s): " +
                                   repr(actual_failures))
            print("REPRODUCED ARCH-001: only expected adapter ABI assertions failed")
            return 0
        if test.returncode:
            raise RuntimeError("ARCH-001 reproduced: required adapter contract failed")
        print("PASS: production adapter ABI conformance")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit("FAIL A02.1: " + str(exc))
