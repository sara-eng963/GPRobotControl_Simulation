#!/usr/bin/env python3
"""Build and assert the five STM32H7 QSPI Renode firmware tests.

Functional Renode results only: not a physical W25Q512JV model, 500 Hz
benchmark, or RTOS/concurrent TrajectoryPrefetch integration test.

Isolates firmware builds, .resc scripts and logs in a temporary directory so
tracked source and the user's untracked build/flash files are untouched.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE = ROOT / "Simulation" / "Renode" / "Firmware"
SCRIPTS = ROOT / "Simulation" / "Renode"
PASS = 0x50415353

# (word offset from 0x24000000, required value). None = checked separately.
EXPECTED = {
    "qspi_jedec_test": {
        "magic": (0x00, 0x51535049), "status": (0x04, PASS),
        "jedec": (0x08, 0x002040EF),
    },
    "qspi_rw_test": {
        "magic": (0x00, 0x51535257), "status": (0x04, PASS),
        "stage": (0x08, 6), "wel_status": (0x0C, None),
        "mismatch": (0x10, 0xFFFFFFFF),
        "pattern0": (0x18, 0xEFBEADDE), "pattern1": (0x1C, 0x78563412),
        "pattern2": (0x20, 0xF0DEBC9A), "pattern3": (0x24, 0xF00FAA55),
    },
    "qspi_pv_sample_test": {
        "magic": (0x00, 0x5056534D), "status": (0x04, PASS),
        "sample_size": (0x08, 24), "count": (0x0C, 4),
        "bytes": (0x10, 96), "mismatch_sample": (0x14, 0xFFFFFFFF),
        "mismatch_joint": (0x18, 0xFFFFFFFF),
        "first_j1": (0x20, 1000), "first_j2": (0x24, -2000 & 0xFFFFFFFF),
        "first_j3": (0x28, 3000), "first_j4": (0x2C, -4000 & 0xFFFFFFFF),
        "first_j5": (0x30, 5000), "first_j6": (0x34, -6000 & 0xFFFFFFFF),
    },
    "qspi_storage_backend_test": {
        "magic": (0x00, 0x50563734), "status": (0x04, PASS),
        "callbacks_bound": (0x08, 1), "written": (0x0C, 12),
        "committed": (0x10, 1), "reloaded": (0x14, 1),
        "metadata_ok": (0x18, 1), "reloaded_count": (0x1C, 12),
        "sample_size": (0x20, 24),
        "mismatch_sample": (0x24, 0xFFFFFFFF),
        "mismatch_joint": (0x28, 0xFFFFFFFF),
        "last_j1": (0x2C, 11100),
        "last_j6": (0x30, -11150 & 0xFFFFFFFF),
    },
    "qspi_streaming_test": {
        "magic": (0x00, 0x50563735), "status": (0x04, PASS),
        "stored": (0x08, 1536), "reloaded": (0x0C, 1536),
        "capacity": (0x10, 512), "chunk": (0x14, 256),
        "refills": (0x18, 6), "pushed": (0x1C, 1536),
        "popped": (0x20, 1536), "underruns": (0x24, 0),
        "high_water": (0x28, 512), "remaining": (0x2C, 0),
        "mismatch_sample": (0x30, 0xFFFFFFFF),
        "mismatch_joint": (0x34, 0xFFFFFFFF),
        "final_j1": (0x38, 653500),
        "final_j6": (0x3C, -653535 & 0xFFFFFFFF),
    },
}

# The monitor's ReadDoubleWord output is a bare 0xXXXXXXXX result on its own
# line; echo delimiters avoid coupling assertions to human-readable .resc text.
RE_VALUE = re.compile(r"^\s*(0x[0-9a-fA-F]{1,8})\s*$")
RE_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
RE_MARKER = re.compile(r"BATCH4_WORD_([a-z0-9_]+)")


def parse_words(log: str, wanted: dict[str, tuple[int, int | None]]) -> dict[str, int]:
    values: dict[str, int] = {}
    pending: str | None = None
    for raw in log.splitlines():
        line = RE_ANSI.sub("", raw).strip()
        marker = RE_MARKER.search(line)
        if marker:
            name = marker.group(1)
            if pending is not None:
                raise ValueError(f"missing numeric response for {pending} before {name}")
            if name not in wanted or name in values:
                raise ValueError(f"unexpected or duplicate response marker: {name}")
            pending = name
            continue
        if pending is not None:
            result = RE_VALUE.fullmatch(line)
            if result:
                values[pending] = int(result.group(1), 16)
                pending = None
    if pending:
        raise ValueError(f"missing numeric response for {pending}")
    missing = set(wanted) - set(values)
    if missing:
        raise ValueError(f"missing firmware values: {', '.join(sorted(missing))}")
    return values


def check_words(values: dict[str, int], expected: dict[str, tuple[int, int | None]]) -> None:
    for name, (_, want) in expected.items():
        actual = values[name]
        if want is None:
            if name == "wel_status" and not actual & 0x02:
                raise ValueError(f"{name}: WEL bit missing, read {actual:#010x}")
        elif actual != want:
            detail = ""
            if name == "status":
                detail = " (ASCII: " + repr(actual.to_bytes(4, "big").decode("ascii", "replace")) + ")"
            raise ValueError(f"{name}: expected {want:#010x}, got {actual:#010x}{detail}")


def render_script(test: str, elf: Path, work: Path) -> Path:
    original = SCRIPTS / f"run_{test.removesuffix('_test')}_test.resc"
    # Existing names already fit run_<test>.resc; e.g. run_qspi_rw_test.resc.
    original = SCRIPTS / f"run_{test}.resc"
    script = original.read_text()
    elf_pattern = re.compile(r"(?m)^sysbus LoadELF @[^\s]+\.elf\s*$")
    script, count = elf_pattern.subn(f"sysbus LoadELF @{elf}", script)
    if count != 1:
        raise ValueError(f"{original}: expected exactly one sysbus LoadELF line")
    # Keep the real setup, firmware, and emulated-duration steps; replace only
    # the plain-text results section with stable machine-readable probes.
    script = script.split('\necho "Test ', 1)[0]
    if "emulation RunFor" not in script or "pause" not in script:
        raise ValueError(f"{original}: cannot identify bounded emulation phase")
    script += "\n\n"
    for name, (offset, _) in EXPECTED[test].items():
        script += f'echo "BATCH4_WORD_{name}"\nsysbus ReadDoubleWord {0x24000000 + offset:#010x}\n'
    script += 'echo "BATCH4_SCRIPT_COMPLETE"\n'
    out = work / f"{test}.resc"
    out.write_text(script)
    return out


def execute(cmd: list[str], log_path: Path, timeout: int, cwd: Path) -> str:
    try:
        p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, encoding="utf-8", errors="replace", timeout=timeout)
    except subprocess.TimeoutExpired as e:
        partial = e.stdout or b""
        if isinstance(partial, bytes):
            partial = partial.decode("utf-8", "replace")
        log_path.write_text(partial)
        raise ValueError(f"timeout after {timeout}s; log: {log_path}") from e
    log_path.write_text(p.stdout)
    if p.returncode:
        raise ValueError(f"command exited {p.returncode}; log: {log_path}")
    return p.stdout


def self_test() -> None:
    wanted = {"magic": (0, 0x12345678), "status": (4, PASS)}
    log = ('[INFO] Script: BATCH4_WORD_magic\n0x12345678\n'
           '[INFO] Script: BATCH4_WORD_status\n0x50415353\n'
           '[INFO] Script: BATCH4_SCRIPT_COMPLETE\n')
    check_words(parse_words(log, wanted), wanted)
    for malformed in [log.replace("0x50415353", "0x4641494C"),
                      log.replace("0x12345678", ""),
                      log.replace("BATCH4_WORD_status", "BATCH4_WORD_magic")]:
        try:
            check_words(parse_words(malformed, wanted), wanted)
        except ValueError:
            continue
        raise AssertionError("malformed output was accepted")
    print("[PASS] Runner self-test: accepts correct results, rejects failed/missing/duplicate values")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--renode", default=str(Path.home() / "renode_portable" / "renode"))
    ap.add_argument("--test", choices=tuple(EXPECTED), action="append", help="Run selected case only; repeatable")
    ap.add_argument("--timeout", type=int, default=90, help="Wall-clock seconds per Renode test")
    ap.add_argument("--self-test", action="store_true", help="Test assertion/parser logic without Renode")
    ap.add_argument("--workdir", type=Path, help="Dedicated directory for isolated output (must be empty/new)")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    for tool in ["make", "arm-none-eabi-gcc", "arm-none-eabi-objcopy"]:
        if shutil.which(tool) is None:
            print(f"[ERROR] Missing required executable: {tool}", file=sys.stderr)
            return 2
    renode = Path(args.renode)
    if not renode.is_file():
        print(f"[ERROR] Renode executable not found: {renode}", file=sys.stderr)
        return 2
    tests = args.test or list(EXPECTED)
    work = args.workdir or Path(tempfile.mkdtemp(prefix="batch4-renode-", dir="/tmp"))
    if args.workdir:
        if work.exists() and any(work.iterdir()):
            print("[ERROR] --workdir must be new/empty", file=sys.stderr)
            return 2
        work.mkdir(parents=True, exist_ok=True)
    print(f"[INFO] Logs and firmware builds: {work}", flush=True)
    failed = []
    for test in tests:
        build = work / test
        build.mkdir()
        try:
            print(f"[BUILD] {test}", flush=True)
            execute(["make", "-C", str(FIRMWARE / test), f"BUILD_DIR={build}"],
                    work / f"{test}.build.log", args.timeout, ROOT)
            elf = build / f"{test}.elf"
            if not elf.is_file():
                raise ValueError(f"missing built ELF: {elf}")
            resc = render_script(test, elf, work)
            print(f"[RUN] {test}", flush=True)
            out = execute([str(renode), "-p", "--disable-gui", "--console",
                           "-e", f"include @{resc}", "-e", "quit"],
                          work / f"{test}.renode.log", args.timeout, ROOT)
            if "BATCH4_SCRIPT_COMPLETE" not in out:
                raise ValueError("Renode script did not reach the final probe")
            values = parse_words(out, EXPECTED[test])
            check_words(values, EXPECTED[test])
            print(f"[PASS] {test}: {len(values)} checked memory words", flush=True)
        except (ValueError, OSError) as e:
            failed.append(test)
            print(f"[FAIL] {test}: {e}", flush=True)
            print(f"        See logs under: {work}", flush=True)
    print(f"[SUMMARY] {len(tests)-len(failed)}/{len(tests)} PASS; {len(failed)} FAIL")
    print(f"[LOGS] {work}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
