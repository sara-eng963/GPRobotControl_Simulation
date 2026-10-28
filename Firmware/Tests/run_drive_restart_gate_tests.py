#!/usr/bin/env python3
"""Execute genuine CM7-portable C2B policy and deliberate mutations.

This does NOT claim a CM7/CM4 link, a full BOOT integration test, or real STO.
Uses the production C2A header from the repository, unchanged.
"""
from pathlib import Path
import subprocess
import tempfile
import shutil

root = Path(__file__).resolve().parents[2]
include_dir = root / "Firmware/STM32H745/Common/Inc"
source = root / "Firmware/Tests/test_drive_restart_gate.c"
policy = include_dir / "drive_restart_gate.h"
reset = include_dir / "reset_diagnostics.h"
supervision = include_dir / "supervision_progress.h"
mutations = {
    "require released input before rise": (
        "gate->observed_release && !gate->previous_pressed &&",
        "true &&"),
    "revocation latch": (
        "if (gate->authorized) drive_restart_gate_inhibit(gate);",
        "if (false) drive_restart_gate_inhibit(gate);"),
    "current reset/IPC policy": (
        "if (!policy_without_operator) {",
        "if (policy_without_operator && false) {"),
    "live input provider": (
        "port == NULL || port->read == NULL ||",
        "false ||"),
    "permanent restart inhibition": (
        "if (gate == NULL || gate->inhibited) return false;",
        "if (gate == NULL) return false;"),
}

def run_variant(header: str, label: str, should_pass: bool):
    with tempfile.TemporaryDirectory(prefix="c2b_gate_") as d:
        d = Path(d)
        for h in (supervision, reset):
            shutil.copyfile(h, d / h.name)
        (d / "drive_restart_gate.h").write_text(header)
        binary = d / "test"
        compile_result = subprocess.run(
            ["gcc", "-std=c11", "-O0", "-g", "-Wall", "-Wextra", "-Werror",
             "-I", str(d), str(source), "-o", str(binary)],
            capture_output=True, text=True)
        if compile_result.returncode:
            raise RuntimeError(label + ": C compilation failed:\n" + compile_result.stderr)
        res = subprocess.run([str(binary)], capture_output=True, text=True)
        if (res.returncode == 0) != should_pass:
            raise AssertionError(f"{label}: unexpected exit {res.returncode}: {res.stdout} {res.stderr}")
        print("PASS:", label, "baseline" if should_pass else "mutation rejected")

original = policy.read_text()
run_variant(original, "live policy", True)
for label, (before, after) in mutations.items():
    assert original.count(before) >= 1, f"mutation target absent: {label}"
    run_variant(original.replace(before, after), label, False)
print("PASS: all C2B host policy mutations killed")

# Also exercise actual firmware BOOT decision code through a mock commissioning
# port, not just a self-contained policy header. Do not weaken on failure.
boot_src = root / "Firmware/STM32H745/CM7/Application/StateMachine/States/state_boot.c"
cia_src = root / "ServoDrive/CiA402/cia402.c"
boot_test = root / "Firmware/Tests/test_c2b_boot_path.c"
with tempfile.TemporaryDirectory(prefix="c2b_boot_") as tmp:
    cmd = ["gcc", "-std=c11", "-O0", "-g", "-Wall", "-Wextra", "-Werror",
           "-I", str(root / "Firmware/STM32H745/CM7/Application/StateMachine/States"),
           "-I", str(root / "StateMachine/States"),
           "-I", str(root / "ServoDrive/CiA402"),
           str(boot_src), str(cia_src), str(boot_test), "-o", str(Path(tmp)/"boot")]
    compile_result = subprocess.run(cmd, capture_output=True, text=True)
    if compile_result.returncode != 0:
        raise RuntimeError("actual BOOT compile failed:\n" + compile_result.stderr)
    result = subprocess.run([str(Path(tmp)/"boot")], capture_output=True, text=True)
    if result.returncode != 0:
        raise AssertionError("actual BOOT test failed: " + result.stderr)
    print(result.stdout.strip())
    # Deliberately remove the mandatory approval in the ENABLE path. The
    # production test must then fail; no success-only mock is acceptable.
    source = boot_src.read_text()
    target = "if (!boot->drive_enable_authorized)"
    assert source.count(target) >= 2, "must have wait and emission gate"
    mutated = source.replace(target, "if (false)")
    mutant_path = Path(tmp)/"state_boot.c"
    mutant_path.write_text(mutated)
    cmd[cmd.index(str(boot_src))] = str(mutant_path)
    compile_result = subprocess.run(cmd, capture_output=True, text=True)
    if compile_result.returncode != 0:
        raise RuntimeError("BOOT mutant failed compilation:\n" + compile_result.stderr)
    result = subprocess.run([str(Path(tmp)/"boot")], capture_output=True, text=True)
    if result.returncode == 0:
        raise AssertionError("BOOT emission gate mutation survived")
    print("PASS: BOOT enable-gate removal mutation rejected")
