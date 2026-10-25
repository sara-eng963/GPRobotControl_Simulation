#!/usr/bin/env python3
"""One-run Stage 3 Batch A SOFTWARE acceptance, no physical board required.

Checks the current source against an ALREADY BUILT independent CubeIDE project,
runs the ARM/host regressions and ELF symbol gates, then writes an audit report
beside (not inside) the verifier. DOES NOT build, flash, or run firmware.
"""
from datetime import datetime, timezone
from pathlib import Path
import filecmp
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "Firmware" / "Tests"

TESTS = (
    ("application_arm", "Stage 3 application ARM compilation", "check_application_arm_compilation.sh"),
    ("m7_composition", "M7 CANopen commissioning and fail-closed composition", "check_m7_board_canopen_composition.sh"),
    ("m7_application", "M7 Supervisor and board ARM integration", "check_m7_application_integration.sh"),
    ("m4_ipc", "M4 IPC application and task wiring", "check_m4_application_ipc.sh"),
    ("fdcan_mock", "Classic CAN HAL1 adapter failure paths", "check_fdcan_hal_adapter_mock.sh"),
    ("sram4_abi", "Shared SRAM4 cross-core ABI", "check_shared_layout.sh"),
    ("sram4_linkers", "SRAM4 linkers both cores, both memory modes", "check_sram4_linkers.sh"),
    ("sram4_objects", "Production shared objects ARM placement", "check_production_shared_objects.sh"),
    ("startup", "Intercore startup ARM checks", "check_intercore_startup_compile.sh"),
    ("notifications", "EXTI/FreeRTOS intercore notification checks", "check_intercore_notifications.sh"),
    ("source_staging", "Repeatable staging manifest test", "check_m7_application_staging.sh"),
    ("timers_config", "TIM2/TIM3 generated clock/IRQ/static source contract", "check_tim2_tim3_integration.py"),
    ("timers_faults", "Real C timing blocks with host fault injection", "run_m7_timer_fault_injection.py"),
)

COPIED_SOURCE = (
    ("Firmware/STM32H745/GP_STM32H745.ioc", "GP_STM32H745.ioc"),
    ("Firmware/STM32H745/CM7/Core/Src/main.c", "CM7/Core/Src/main.c"),
    ("Firmware/STM32H745/CM7/Core/Src/application_supervisor.c", "CM7/Core/Src/application_supervisor.c"),
    ("Firmware/STM32H745/CM7/Core/Inc/application_supervisor.h", "CM7/Core/Inc/application_supervisor.h"),
    ("StateMachine/supervisor_task.c", "CM7/Application/StateMachine/supervisor_task.c"),
    ("StateMachine/supervisor_task.h", "CM7/Application/StateMachine/supervisor_task.h"),
)

REQUIRED_M7 = (
    "HAL_FDCAN_Init", "HAL_FDCAN_MspInit", "gp_fdcan_classic_backend_open",
    "canopen_master_init", "application_supervisor_start",
    "TIM2_IRQHandler", "TIM3_IRQHandler", "HAL_TIM_PeriodElapsedCallback",
    "HAL_TIM_Base_Start_IT", "application_supervisor_timer_period_elapsed_isr",
    "supervisor_task_start",
)
REQUIRED_M4 = ("application_ipc_service_run", "gp_ipc_m4_receive")


def run(command, timeout=120):
    completed = subprocess.run(
        command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout, check=False,
    )
    return completed.returncode, completed.stdout


def validate_verifier(verifier, branch, m7, m4):
    if branch != "sara/can-replacement":
        raise RuntimeError("Expected sara/can-replacement, found " + branch)
    for required in (verifier / "GP_STM32H745.ioc",
                     verifier / "CM7/.project", verifier / "CM4/.project", m7, m4):
        if not required.is_file() or required.stat().st_size == 0:
            raise RuntimeError("Missing/empty project input: " + str(required))
    lines = []
    for canonical, relative in COPIED_SOURCE:
        src, dst = ROOT / canonical, verifier / relative
        if not src.is_file() or not dst.is_file():
            raise RuntimeError("Missing source counterpart: " + relative)
        if not filecmp.cmp(src, dst, shallow=False):
            raise RuntimeError("Stale/different verifier source: " + relative)
        lines.append("PASS source parity: " + relative)
        # If the edited verifier input is newer than the linked ELF, we cannot
        # associate it confidently with the previously user-built image.
        if relative.startswith("CM7/") or relative.endswith(".ioc"):
            if dst.stat().st_mtime_ns > m7.stat().st_mtime_ns:
                raise RuntimeError("M7 ELF older than source: " + relative +
                                   " (rebuild M7 before acceptance)")
    m4_main = verifier / "CM4/Core/Src/main.c"
    if m4_main.is_file() and m4_main.stat().st_mtime_ns > m4.stat().st_mtime_ns:
        raise RuntimeError("M4 ELF older than main.c; rebuild M4")
    lines.append("PASS ELF freshness vs staged timer/Supervisor sources")
    return "\n".join(lines)


def verify_elf_symbols(m7, m4):
    lines = []
    for core, elf, symbols in (
        ("CM7", m7, REQUIRED_M7), ("CM4", m4, REQUIRED_M4)
    ):
        code, listing = run(["arm-none-eabi-nm", "--defined-only", str(elf)])
        if code:
            raise RuntimeError("arm-none-eabi-nm failed for " + core + ":\n" + listing)
        for symbol in symbols:
            pattern = r"^[0-9a-fA-F]+\s+T\s+" + re.escape(symbol) + r"$"
            if re.search(pattern, listing, flags=re.MULTILINE) is None:
                raise RuntimeError("Missing strong " + core + " ELF symbol: " + symbol)
            lines.append("PASS strong " + core + ": " + symbol)
    code, size = run(["arm-none-eabi-size", str(m7), str(m4)])
    if code:
        raise RuntimeError("ELF size inspection failed:\n" + size)
    lines.append(size.rstrip())
    return "\n".join(lines)


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit("Usage: python3 Firmware/Tests/run_stage3_batch_a_acceptance.py VERIFIER [REPORT_DIR]")
    verifier = Path(sys.argv[1]).expanduser().resolve()
    report_root = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) == 3 else (
        verifier.parent / "GP_Stage3_BatchA_Acceptance"
    ).resolve()
    if report_root == verifier or verifier in report_root.parents:
        sys.exit("FAIL: report must not be written inside the CubeIDE verifier")
    if report_root == ROOT or ROOT in report_root.parents:
        sys.exit("FAIL: report must not be written inside the Git repository")
    _, head = run(["git", "rev-parse", "HEAD"])
    _, branch = run(["git", "branch", "--show-current"])
    head, branch = head.strip(), branch.strip()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    folder = report_root / (stamp + "_" + head[:10])
    folder.mkdir(parents=True, exist_ok=False)
    m7 = verifier / "CM7/Debug/GP_STM32H745_CM7.elf"
    m4 = verifier / "CM4/Debug/GP_STM32H745_CM4.elf"
    outcomes = []
    full_log = []
    print("Stage 3 Batch A — SOFTWARE-only acceptance")
    print("Commit:", head)
    print("Verifier:", verifier)
    print("Reports:", folder)
    print()

    def gate(name, title, action):
        try:
            code, output = action()
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
            code, output = 1, "FAIL: " + str(exc)
        status = "PASS" if code == 0 else "FAIL"
        outcomes.append((title, status, name + ".log"))
        (folder / (name + ".log")).write_text(output + "\n")
        full_log.append("=== " + title + " [" + status + "] ===\n" + output)
        print(status + ":", title)
        if code != 0:
            print("\n".join(output.splitlines()[-20:]))

    def snapshot():
        return 0, validate_verifier(verifier, branch, m7, m4)

    def symbol_gate():
        return 0, verify_elf_symbols(m7, m4)

    gate("00_snapshot", "Independent source parity + target ELF freshness", snapshot)
    for name, label, script in TESTS:
        args = (["python3"] if script.endswith(".py") else ["bash"]) + [str(SCRIPTS / script)]
        gate(name, label, lambda args=args: run(args))
    gate("14_elf_gate", "CANopen and fail-closed target ELF regression", lambda: run([
        "bash", str(SCRIPTS / "check_stage3_batch_a_target_elfs.sh"), str(verifier)
    ]))
    gate("15_timer_symbols", "TIM2/TIM3/M7 and M4 strong ELF symbol inventory", symbol_gate)

    failed = sum(status != "PASS" for _, status, _ in outcomes)
    now = datetime.now(timezone.utc).isoformat()
    report = [
        "# Stage 3 Batch A — Software Acceptance Report",
        "",
        "- Date (UTC): " + now,
        "- Repository: sara-eng963/GPRobotControl_Simulation",
        "- Branch: " + branch,
        "- Source commit: " + head,
        "- Independent CubeIDE verifier: " + str(verifier),
        "- Software result: " + ("PASS" if failed == 0 else "NOT ACCEPTED"),
        "- Passed: " + str(len(outcomes) - failed) + " / " + str(len(outcomes)),
        "",
        "## Acceptance gates",
        "",
        "| Gate | Outcome | Detailed log |",
        "| --- | --- | --- |",
    ]
    report.extend("| " + label + " | " + status + " | " + log + " |"
                  for label, status, log in outcomes)
    report += [
        "",
        "## Evidence boundary and unresolved hardware work",
        "",
        "This verifies selected ARM compile/host mock/ABI checks and the symbols",
        "of existing user-built target ELFs. It DOES NOT clean-build CubeIDE",
        "itself, run physical firmware, measure 500 Hz ISR/task jitter or",
        "CANopen bus throughput, commission drives, or certify safety.",
        "The real physical board provider remains absent and defaults to",
        "fail-closed motion disabled. External NOR, M4 storage producer,",
        "real hardware IPC and safety interlocks are not acceptance claims.",
        "",
        "## Handoff",
        "",
        "After a PASS, proceed to Stage 3 Batch B: M7 trajectory execution,",
        "M4/CM7 shared validation and M4 storage/trajectory producer/HMI IPC.",
        "Then Batch C: all-up regression, memory budgets and final clean builds.",
    ]
    report_path = folder / "BatchA_Acceptance.md"
    report_path.write_text("\n".join(report) + "\n")
    (folder / "full_run.log").write_text("\n\n".join(full_log) + "\n")
    print()
    print(("PASS" if failed == 0 else "FAIL") + ": Batch A software acceptance (" +
          str(len(outcomes) - failed) + "/" + str(len(outcomes)) + " gates)")
    print("Markdown report:", report_path)
    print("Full log:", folder / "full_run.log")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
