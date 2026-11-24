"""Functional acceptance for actual ARM application workloads (not startup)."""
from __future__ import annotations

from .common import BenchmarkError

FIELDS = (
    "magic", "version", "status", "failure_line", "phase", "completed_mask",
    "assertions", "supervisor_cycles", "can_target_frames", "can_syncs",
    "ft_frames", "guidance_ok_cycles", "validations", "validated_samples",
    "playbacks", "playback_samples", "max_validation_queue", "max_trajectory_ring",
    "max_ft_ring", "max_program_segments", "max_supervisor_queue", "observed_transitions",
    "safe_output_calls", "storage_writes", "storage_reads", "storage_commits", "last_state",
    "last_boot_phase", "last_fault", "last_validation_result", "last_validation_error",
    "max_raw_geometry", "ft_fault_verified", "restart_denied", "phase_tick",
    "last_execution_error", "last_execution_phase", "peer_progress_sequence",
    "c_heap_peak_break_growth_bytes", "failure_caller_pc", "waiting_state",
    "expected_playback_samples", "skipped_playback_samples",
    "storage_read_fault_injected",
    "heap_after_first_cycle_bytes", "heap_after_second_cycle_bytes", "heap_at_start_bytes", "task_count",
)
PHASES = (
    "application startup and CANopen commissioning", "idle hold cycles",
    "teaching and F/T hand-guiding processing", "numerical validation and durable commit",
    "preview/production playback", "repeated modes and pause/resume",
    "bounded buffer capacity", "fault containment and denied restart",
)
EXPECTED_TASKS = {
    "defaultTask": 2048, "IDLE": 512, "Tmr Svc": 1024,
    "BenchScenario": 16384, "M4IPC": 2048, "M4Progress": 1024,
    "FT_RX": 2048, "NorValidate": 3072, "StorageValidate": 16384, "Supervisor": 16384,
}
UNVERIFIED = [
    "Native H745 dual-kernel scheduling, CM4 machine-code/FPU stack demand, HSEM wake/IRQ routing, and cache/DMA coherency; CM4 services here are recompiled for Cortex-M7 and share its kernel.",
    "Physical board commissioning, vendor-specific F/T decoder, motor dynamics, collision geometry, stop/retraction timing and welding hardware; deterministic test-only providers supply these interfaces.",
    "Circular/arc teaching and validation, all possible geometries/IK failures, full 10000-sample artifact capacity, storage power-loss recovery and exhaustive fault interleavings are not exercised by this ARM scenario.",
    "ARM feedback/heartbeat loss, peer reboot/loss, forged/stale safety evidence, timer deadline violations and recovery after a new boot are unexercised; native regression evidence does not substitute for those ARM paths.",
    "Hand-guiding executes the production observation-only pipeline. Production deliberately forbids dispatching its candidate targets; virtual manual movement supplies the taught endpoints.",
    "Hardware TIM2/TIM3 release and deadlines are unverified. Supervisor uses its existing FreeRTOS tick/DelayUntil fallback; simulated time is not hardware timing.",
    "Task fill-pattern watermarks are observed usage, not guaranteed worst-case stack depth; unwritten frames, interrupts/MSP and other future paths remain unbounded.",
    "The measured application ELF includes test providers, a virtual NOR model, scenario stack and relocated CM4 state. Its Flash/SRAM totals are not a minimum replacement-MCU requirement.",
]


def decode(words):
    if len(words) != 48:
        raise BenchmarkError("Truncated application result buffer")
    data = dict(zip(FIELDS, words))
    if data["magic"] != 0x4150504D or data["version"] != 1 or data["status"] not in (0, 1, 2):
        raise BenchmarkError("Invalid application evidence header")
    if data["completed_mask"] & ~0x1FE:
        raise BenchmarkError("Unknown application phase evidence")
    return data


def validate(result):
    """Never promote a scheduler-only probe without independently checked work."""
    final = result["samples"][-1]
    if "application_words" not in final:
        raise BenchmarkError("No ARM application workload evidence")
    data = decode(final["application_words"])
    result["application"] = data
    result["coverage"] = [dict(name=name, status="PASS" if data["completed_mask"] & (1 << i) else "UNVERIFIED")
                          for i, name in enumerate(PHASES, 1)]
    result["unverified"] = UNVERIFIED
    if data["status"] != 1 or data["failure_line"]:
        raise BenchmarkError(f"ARM application did not pass: status={data['status']}, phase={data['phase']}, "
                             f"line={data['failure_line']}, state={data['last_state']}, fault={data['last_fault']:#x}")
    if not result["scheduler_and_ticks_verified"] or not final["heap_initialized"]:
        raise BenchmarkError("ARM application scheduling/heap initialization unverified")
    if data["completed_mask"] != 0x1FE or data["assertions"] < 1000:
        raise BenchmarkError("Missing asserted ARM application workload coverage")
    wanted = {"validations":2, "playbacks":3, "storage_commits":2,
              "max_validation_queue":64, "max_trajectory_ring":512, "max_ft_ring":512,
              "max_program_segments":32, "max_supervisor_queue":32, "max_raw_geometry":512,
              "ft_fault_verified":1, "restart_denied":1}
    for name, expected in wanted.items():
        if data[name] != expected:
            raise BenchmarkError(f"ARM workload {name}: observed {data[name]}, expected {expected}")
    for name in ("ft_frames", "guidance_ok_cycles", "safe_output_calls", "peer_progress_sequence"):
        if not data[name]:
            raise BenchmarkError(f"No observed ARM work: {name}")
    if data["validated_samples"] < 1024 or data["storage_writes"] != data["validated_samples"]:
        raise BenchmarkError("Validation/storage sample evidence mismatch")
    if data["playback_samples"] + data["skipped_playback_samples"] != data["expected_playback_samples"]:
        raise BenchmarkError("Playback actual consumption/skip accounting mismatch")
    if data["observed_transitions"] < 15:
        raise BenchmarkError("Insufficient operating-mode transitions")
    if data["task_count"]!=len(EXPECTED_TASKS):
        raise BenchmarkError("Unexpected actual FreeRTOS task count")
    if not (data["heap_at_start_bytes"] == data["heap_after_first_cycle_bytes"] == data["heap_after_second_cycle_bytes"] == final["current_free_heap_bytes"]):
        raise BenchmarkError("FreeRTOS heap failed to return to baseline across operating cycles")
    tasks = final["tasks"]
    if {t["name"] for t in tasks} != set(EXPECTED_TASKS) or len(tasks) != len(EXPECTED_TASKS):
        raise BenchmarkError("Missing, duplicate or unexpected ARM application tasks")
    for task in tasks:
        if task["allocated_stack_bytes"] != EXPECTED_TASKS[task["name"]]:
            raise BenchmarkError(f"Unexpected stack reservation: {task['name']}")
        if not task["active"] or task["switches"] == 0 or task["high_water_words"] == 0:
            raise BenchmarkError(f"Task not executed/live, or exhausted stack: {task['name']}")
    if final["dynamic_allocation_failures"] or final["task_creation_failures"]:
        raise BenchmarkError("Unexpected ARM application allocation failure")
    result["classification"] = "ARM_APPLICATION_WORKLOADS_MEASURED"
    result["validation"] = "PASS"
    result["memory_interpretation"] = "Task static stacks and ucHeap already reside in the application ELF SRAM sections; runtime usage is a subset and is not added again."


def validate_negative(result):
    """Accept only the deliberately injected reader failure after real validation."""
    final=result["samples"][-1]
    data=decode(final["application_words"])
    result["application"]=data
    if (data["status"] != 2 or data["phase"] != 5 or data["storage_read_fault_injected"] != 1 or
            data["storage_commits"] != 1 or data["validations"] != 1 or data["last_state"] != 11 or
            not data["last_fault"] or not data["failure_line"] or data["completed_mask"] != 30):
        raise BenchmarkError(f"Expected ARM storage-reader failure was not established: {data}")
    if final["dynamic_allocation_failures"] or final["task_creation_failures"]:
        raise BenchmarkError("Negative reader test failed for an unrelated allocation error")
    result["classification"]="EXPECTED_APPLICATION_FAILURE_OBSERVED"
    result["validation"]="PASS"


def execute(runner, production, renode, out, seconds=60, timeout=600, publish=None):
    from pathlib import Path
    from .variants import build
    from .static import analyze, aggregate
    from .runtime import probe
    c=production[0]
    print("[APPLICATION] Build real ARM tasks with deterministic providers", flush=True)
    v=build(runner,Path(c["elf"]).parent,"CM7",out/"build",application=True)
    image=analyze(runner,Path(v["elf"]),"CM7",out/"audit")
    print(f"[APPLICATION] Run asserted workloads ({seconds:g} simulated seconds maximum)",flush=True)
    result=probe(runner,Path(v["elf"]),"CM7",renode,out/"run",seconds=seconds,timeout=timeout,instrumented=True)
    result.update(variant=v, image=image, static=aggregate([image]), topology="CM7 single kernel; CM4 services recompiled and relocated")
    if publish:
        publish(result)
    # Retain partial evidence even if acceptance fails.
    import json
    def retain():
        (out/"application-results.json").write_text(json.dumps(result,indent=2)+"\n")
    try:
        validate(result)
    finally:
        app=result.get("application", {})
        if app.get("failure_caller_pc"):
            _, location=runner.run(["arm-none-eabi-addr2line","-f","-i","-e",v["elf"],hex(app["failure_caller_pc"])],out/"failure-location.log")
            result["failure_location"]=location.splitlines()[1:]
        retain()
    print("[APPLICATION] Workloads passed; inject a separate storage-reader failure",flush=True)
    nv=build(runner,Path(c["elf"]).parent,"CM7",out/"negative-build",application=True,application_negative=True)
    negative=probe(runner,Path(nv["elf"]),"CM7",renode,out/"negative-run",seconds=15,timeout=timeout,instrumented=True)
    negative["variant"]=nv
    result["negative_test"]=negative
    try:
        validate_negative(negative)
    finally:
        retain()
    return result
