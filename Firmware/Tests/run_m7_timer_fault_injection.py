#!/usr/bin/env python3
"""Host fault injection of selected real M7 timer and Supervisor C code.

Compile extracts from production sources with mocked HAL/FreeRTOS calls.
This cannot prove physical interrupt timing, WCET, CAN load or robot safety.
"""
from pathlib import Path
import subprocess
import tempfile
import sys

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "Firmware/STM32H745/CM7/Core/Src/application_supervisor.c"
SUP = ROOT / "StateMachine/supervisor_task.c"
TEMPLATE = Path(__file__).with_name("m7_timer_fault_host.c")


def extract(text, marker):
    begin = text.find(marker)
    if begin == -1 or text.find(marker, begin + len(marker)) != -1:
        raise ValueError("Expected one source block: " + marker)
    start = text.find("{", begin + len(marker))
    if start == -1:
        raise ValueError("No opening brace: " + marker)
    level = 0
    for pos in range(start, len(text)):
        if text[pos] == "{":
            level += 1
        elif text[pos] == "}":
            level -= 1
            if level == 0:
                return text[begin:pos + 1]
    raise ValueError("Unbalanced source: " + marker)


def main():
    app, sup = APP.read_text(), SUP.read_text()
    functions = "\n\n".join(extract(app, marker) for marker in (
        "static uint64_t gp_time_now_us(void *unused)",
        "static bool gp_timer_start(void *unused)",
        "static uint32_t gp_timer_wait(",
        "void application_supervisor_timer_period_elapsed_isr(",
    ))
    task = sup.split("static void supervisor_task_entry(void *argument)", 1)[1]
    wait_part = extract(
        task.split("const uint64_t cycle_start_us", 1)[0],
        "if (hardware_timed) {"
    )
    elapsed_part = extract(
        task.split("TRACE_END(GP_TIMING_SUPERVISOR, true);", 1)[1],
        "if (hardware_timed) {"
    )
    source = TEMPLATE.read_text()
    for key, value in {
        "/* PRODUCTION_TIMER_FUNCTIONS */": functions,
        "/* PRODUCTION_WAIT_BLOCK */": wait_part,
        "/* PRODUCTION_ELAPSED_BLOCK */": elapsed_part,
    }.items():
        if source.count(key) != 1:
            raise ValueError("Unexpected harness placeholder: " + key)
        source = source.replace(key, value)
    with tempfile.TemporaryDirectory(prefix="gp_timer_fault_") as folder:
        test_c = Path(folder) / "timer_fault.c"
        exe = Path(folder) / "timer_fault"
        test_c.write_text(source)
        subprocess.run(
            ["cc", "-std=c11", "-O0", "-Wall", "-Wextra", "-Werror",
             str(test_c), "-o", str(exe)], check=True
        )
        subprocess.run([str(exe)], check=True)
    print("PASS: production-C timing blocks passed host fault injection")
    print("NOTE: hardware jitter and safety are NOT verified")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit("FAIL: host timer fault injection: " + str(exc))
