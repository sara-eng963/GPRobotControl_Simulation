"""Bounded Renode execution and strict evidence decoding, no GDB dependency."""
from __future__ import annotations

import re
import struct
from pathlib import Path
from .common import BenchmarkError, sha256
from .static import symbols

COUNTERS = ("xSchedulerRunning", "xTickCount", "xFreeBytesRemaining", "xMinimumEverFreeBytesRemaining", "pxEnd")
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def parse_words(log, expected):
    values, pending = {}, None
    for line in ANSI.sub("", log).splitlines():
        marker = re.fullmatch(r"\s*MB_WORD_(\w+)\s*", line)
        if marker:
            key = marker.group(1)
            if pending or key not in expected or key in values:
                raise BenchmarkError(f"Unexpected/duplicate/incomplete monitor marker {key}")
            pending = key
        elif pending:
            value = re.fullmatch(r"\s*0x([\da-fA-F]{1,8})\s*", line)
            if value:
                values[pending] = int(value.group(1), 16)
                pending = None
    if pending or set(values) != set(expected):
        raise BenchmarkError(f"Incomplete monitor samples: missing {sorted(set(expected)-set(values))}")
    if not re.search(r"^MB_COMPLETE\s*$", ANSI.sub("", log), re.M):
        raise BenchmarkError("Missing Renode completion marker")
    if re.search(r"\[ERROR\]|Error while|There was an error|Could not execute|CPU abort|Aborted", log, re.I):
        raise BenchmarkError("Renode reported an execution/monitor error")
    return values


def stack_watermark(data):
    """FreeRTOS downward-stack 0xA5 scan, in StackType_t units (4 bytes)."""
    n = 0
    for byte in data:
        if byte != 0xA5:
            break
        n += 1
    return n // 4


def decode_trace(data, ram, base):
    if len(data) != 1312:
        raise BenchmarkError("Truncated instrumentation buffer")
    magic, version, count, overflow, failures, creates, mallocs, frees = struct.unpack_from("<8I", data)
    if magic != 0x4D424E43 or version != 1 or not 1 <= count <= 32 or overflow:
        raise BenchmarkError("Invalid/incomplete task trace")
    tasks = []
    for i in range(count):
        handle, stack, size, allocation, switches, active, name = struct.unpack_from("<6I16s", data, 32+i*40)
        name = name.split(b"\0", 1)[0].decode("ascii", "strict")
        if not name or allocation not in (0, 1, 2) or active not in (0, 1) or size < 32 or size % 4:
            raise BenchmarkError("Invalid task record")
        offset = stack - base
        if offset < 0 or offset + size > len(ram):
            raise BenchmarkError(f"Task stack outside captured RAM: {name}")
        watermark = stack_watermark(ram[offset:offset+size]) if active else None
        tasks.append(dict(name=name, handle=handle, stack_address=stack, allocated_stack_bytes=size,
                          allocation={0:"dynamic", 1:"static stack/dynamic TCB", 2:"static"}[allocation],
                          switches=switches, active=bool(active), high_water_words=watermark,
                          minimum_remaining_stack_bytes=None if watermark is None else watermark*4,
                          observed_used_stack_bytes=None if watermark is None else size-watermark*4))
    return dict(tasks=tasks, dynamic_allocation_failures=failures, task_creation_failures=creates,
                allocation_calls=mallocs, free_calls=frees, trace_overflow=overflow)


def make_platform(renode, core, out):
    # This remains a single-core platform; never presented as an H745 model.
    text = 'using "platforms/cpus/stm32h7.repl"\n'
    if core == "CM4":
        text += 'cpu:\n    cpuType: "cortex-m4"\n'
        for index, address in enumerate((0x10000000, 0x10020000, 0x10040000), 1):
            text += f'sram{index}: @ {{ sysbus 0x{address:08X}; sysbus 0x{address+0x20000000:08X} }}\n'
    # QSPI implementation reused from the repository platform.
    text += (Path(__file__).resolve().parents[2] / "Simulation/Renode/stm32h7_qspi_flash.repl").read_text().split("\n", 1)[1]
    path = out / "platform.repl"
    path.write_text(text)
    return path


def probe(runner, elf, core, renode, out, seconds=0.3, timeout=120, instrumented=False, seed=None):
    out.mkdir(parents=True, exist_ok=True)
    syms = symbols(runner.run(["arm-none-eabi-nm", "-S", "-a", elf], out / "symbols.log")[1])
    missing = set(COUNTERS) - syms.keys()
    if missing:
        raise BenchmarkError(f"Missing FreeRTOS counters: {missing}")
    if instrumented and "membench_trace" not in syms:
        raise BenchmarkError("Missing instrumentation symbol")
    platform = make_platform(renode, core, out)
    script = ['mach create "memory-benchmark"', 'emulation SetSeed 745',
              f'machine LoadPlatformDescription @{platform}', f'sysbus LoadELF @{elf}',
              f'sysbus.cpu VectorTableOffset {"0x08000000" if core == "CM7" else "0x08100000"}']
    if seed:
        if seed.stat().st_size != 32768:
            raise BenchmarkError("Bad shared SRAM4 seed snapshot")
        script += [f'sysbus LoadBinary @{seed} 0x38000000']
    application = "membench_application" in syms
    if application:
        helper=out/"run_until.py"
        helper.write_text('''# Renode/IronPython monitor extension; no target instrumentation.
def mc_membench_run_until(address, budget, marker):
    bus = self.Machine["sysbus"]
    address = int(str(address), 0)
    remaining = float(budget)
    if bus.ReadDoubleWord(address + 8) != 0:
        remaining = 0.05
    while remaining > 0.0000001:
        step = min(1.0, remaining)
        monitor.Parse('emulation RunFor "%.9f"' % step)
        monitor.Parse('pause')
        mc_membench_run_until.elapsed += step
        remaining -= step
        if bus.ReadDoubleWord(address + 8) != 0:
            break
    print 'MB_WORD_' + str(marker)
    print '0x%08X' % int(round(mc_membench_run_until.elapsed * 1000000))
mc_membench_run_until.elapsed = 0.0
''')
        script += [f'include @{helper}']
    expected = set()
    base, size = (0x24000000, 0x80000) if core == "CM7" else (0x10000000, 0x48000)
    for i in range(3):
        if application:
            key=f"s{i}_elapsed_us"
            expected.add(key)
            script += [f'membench_run_until 0x{syms["membench_application"]["address"]:08X} {seconds/3:.9f} "{key}"', 'pause']
        else:
            script += [f'emulation RunFor "{seconds/3:.9f}"', 'pause']
        for name in (*COUNTERS, "pc"):
            key = f"s{i}_{name}"
            expected.add(key)
            command = 'sysbus.cpu PC' if name == "pc" else f'sysbus ReadDoubleWord 0x{syms[name]["address"]:08X}'
            script += [f'echo "MB_WORD_{key}"', command]
        if instrumented:
            script += [f'dump_file 0x{syms["membench_trace"]["address"]:08X} 1312 "{out / f"trace-{i}.bin"}"',
                       f'dump_file 0x{base:08X} {size} "{out / f"ram-{i}.bin"}"']
            if "membench_application" in syms:
                script += [f'dump_file 0x{syms["membench_application"]["address"]:08X} {syms["membench_application"]["size"]} "{out / f"application-{i}.bin"}"']
    script += [f'dump_file 0x38000000 32768 "{out / "shared.bin"}"', 'echo "MB_COMPLETE"']
    resc = out / "run.resc"
    resc.write_text("\n".join(script) + "\n")
    _, log = runner.run([renode, "-p", "--disable-gui", "--console", "-e", f"include @{resc}", "-e", "quit"],
                        out / "execution.log", timeout=timeout)
    values = parse_words(log, expected)
    samples = []
    heap_capacity = syms["ucHeap"]["size"]
    for i in range(3):
        words = {name:values[f"s{i}_{name}"] for name in (*COUNTERS, "pc")}
        elapsed=values[f"s{i}_elapsed_us"]/1_000_000 if application else (i+1)*seconds/3
        sample = dict(simulated_seconds=elapsed, **words)
        minimum, free = words["xMinimumEverFreeBytesRemaining"], words["xFreeBytesRemaining"]
        if minimum > free or free > heap_capacity:
            raise BenchmarkError("Impossible heap counters")
        initialized = words["pxEnd"] != 0
        sample.update(heap_arena_bytes=heap_capacity, heap_initialized=initialized,
                      minimum_ever_free_heap_bytes=minimum if initialized else None,
                      observed_peak_heap_bytes=heap_capacity-minimum if initialized else None,
                      current_free_heap_bytes=free if initialized else None)
        if instrumented:
            sample.update(decode_trace((out/f"trace-{i}.bin").read_bytes(), (out/f"ram-{i}.bin").read_bytes(), base))
            if "membench_application" in syms:
                sample["application_words"]=list(struct.unpack("<48I",(out/f"application-{i}.bin").read_bytes()))
        samples.append(sample)
    running = all(s["xSchedulerRunning"] == 1 for s in samples)
    advancing = all(b["xTickCount"] > a["xTickCount"] for a, b in zip(samples, samples[1:]))
    if any(b["xMinimumEverFreeBytesRemaining"] > a["xMinimumEverFreeBytesRemaining"]
           for a, b in zip(samples, samples[1:]) if a["heap_initialized"]):
        raise BenchmarkError("Heap low-water counter increased (reset or invalid snapshot)")
    result = dict(core=core, instrumented=instrumented, samples=samples,
                  scheduler_and_ticks_verified=running and advancing,
                  classification="SCHEDULER_OBSERVED_ONLY" if running and advancing else "STARTUP_BLOCKED",
                  elf_sha256=sha256(Path(elf)), log=str(out/"execution.log"), shared_snapshot=str(out/"shared.bin"),
                  warnings=sorted(set(re.findall(r"^.*\[WARNING\].*$", log, re.M))))
    if not result["scheduler_and_ticks_verified"]:
        result["final_location"] = runner.run(["arm-none-eabi-addr2line", "-f", "-e", elf, hex(samples[-1]["pc"])],
                                               out/"final-location.log")[1].splitlines()[1:]
    return result


def validate_diagnostic(result, negative=False):
    if not result["scheduler_and_ticks_verified"]:
        raise BenchmarkError(f"{result['core']} diagnostic scheduler/ticks not verified")
    samples = result["samples"]
    for sample in samples:
        if not sample["heap_initialized"]:
            raise BenchmarkError("Diagnostic heap not initialized")
        names = {t["name"] for t in sample["tasks"] if t["active"] and t["switches"] > 0}
        wanted = {"defaultTask", "IDLE", "Tmr Svc"} | ({"M4Progress"} if result["core"] == "CM4" else set())
        if names != wanted or len(sample["tasks"]) != len(wanted):
            raise BenchmarkError(f"Missing/unexpected executed task evidence: {names}; expected {wanted}")
        if any(t["active"] and t["high_water_words"] == 0 for t in sample["tasks"]):
            raise BenchmarkError("Observed task stack exhausted")
        if sample["dynamic_allocation_failures"] != (2 if negative else 0):
            raise BenchmarkError("Unexpected allocation failure count")
        if sample["task_creation_failures"] != (1 if negative else 0):
            raise BenchmarkError("Unexpected task creation failure count")
    before = {t["handle"]:t for t in samples[0]["tasks"]}
    for t in samples[-1]["tasks"]:
        old = before.get(t["handle"])
        if old and old["active"] and t["active"]:
            if t["high_water_words"] > old["high_water_words"] or t["switches"] < old["switches"]:
                raise BenchmarkError("Nonmonotonic task evidence")
            if t["name"] in {"defaultTask", "IDLE", "M4Progress"} and t["switches"] == old["switches"]:
                raise BenchmarkError(f"Task stopped being scheduled across snapshots: {t['name']}")
    result["validation"] = "PASS"
