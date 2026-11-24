"""Human-readable view of the same machine-readable evidence."""
from __future__ import annotations


LIMITATIONS = [
    "Full original dual-core controller peak memory remains UNVERIFIED. The separate application image measures the workloads and tasks listed in report.md under its explicit test composition.",
    "Startup-only diagnostics do not create Supervisor, StorageValidate, FT_RX or NorValidate. Their application measurements must come from the separate asserted ARM application image.",
    "Physical commissioned providers remain absent. The application test image links explicit virtual-device providers; production safety gates remain unchanged.",
    "CM7 and CM4 startup diagnostics run separately. The application image instead relocates CM4 services onto the M7 kernel; native dual-core startup, IRQs, caches, and races remain UNVERIFIED.",
    "CM7 omits its second D2CKRDY poll only in a generated main.c; CM4 omits STOP entry only in its generated main.c. These are test firmware changes, not validated startup behavior.",
    "Native workload fixtures exercise production algorithms and policies with virtual devices/mocked scheduling. Their memory usage is not an STM32 measurement.",
    "FreeRTOS heap peak includes allocator metadata, alignment, and initial arena overhead. It excludes C-library malloc, MSP/interrupt stacks and unused statically allocated task stacks; ucHeap is already in .bss and must not be added again.",
    "Stack watermarks use FreeRTOS 0xA5 fill and 32-bit words with the CPU paused; untouched/unwritten stack frames, deleted-task history, interrupts and future paths can require more memory.",
    "Linker ._user_heap_stack includes nominal heap/stack minima at the low end, while startup MSP points to _estack. The separate top-of-bank minimum MSP reservation is included conservatively; actual MSP peak is unmeasured.",
    "Linked sections include internal linker padding; flash spans include inter-section gaps; committed SRAM unions include gaps in occupied linker regions and declared shared partitions once. Fixed-address span is a layout requirement, not the minimum relocatable capacity.",
    "Renode peripheral warnings are retained. Simulated ticks/time do not establish hardware timing, WCET, cache/DMA coherency, physical safety, or worst-case memory bounds.",
    "No replacement MCU purchase approval or minimum SKU is established by these results.",
]


def render_detailed(data):
    identity = data.get("identity", {})
    lines = ["# STM32H745 memory benchmark", "", f"Outcome: **{data['outcome']}**",
             f"Full production runtime: **{data['full_runtime_status']}**", "",
             f"UTC: {data['date_utc']}", f"Branch: `{identity.get('branch', 'unknown')}`",
             f"Commit: `{identity.get('commit', 'unknown')}`; dirty tree: **{bool(identity.get('status'))}**",
             f"Compiler: {identity.get('compiler', 'unknown')}",
             f"Input manifest SHA-256: `{identity.get('input_manifest_sha256', 'unavailable')}`", "",
             "Exact commands: `commands.json`, production `commands.txt`, diagnostic `variant_commands.txt` and per-stage logs.",
             "Input file hashes: `inputs.json`; artifacts: `artifacts.json`. Build is Release (-Os), from this working tree, including its pre-existing edits.", ""]
    if data.get("errors"):
        lines += ["## Failures", ""] + [f"- {e}" for e in data["errors"]] + [""]
    if data.get("static"):
        s = data["static"]
        lines += ["## Production linked memory", "", "| Core | Flash payload | Flash span | SRAM sections | FreeRTOS arena (included) | ELF SHA-256 |",
                  "|---|---:|---:|---:|---:|---|"]
        for c in data["production"]:
            lines.append(f"| {c['core']} | {c['flash_payload_bytes']:,} | {c['flash_span_bytes']:,} | {c['sram_section_bytes']:,} | {c['freertos_heap_arena_bytes']:,} | `{c['elf_sha256']}` |")
        lines += ["", f"Combined Flash image spans: **{s['flash_image_spans_bytes']:,} bytes**; payload: **{s['flash_payload_bytes']:,} bytes**.",
                  f"Combined SRAM sections: **{s['sram_section_bytes']:,} bytes**; with shared partition and separate minimum MSP reservations: **{s['sram_committed_bytes']:,} bytes**.",
                  f"Fixed Flash address span from 0x08000000: **{s['flash_fixed_address_span_bytes']:,} bytes** (includes the gap before CM4).", "",
                  "| Physical RAM bank | Capacity | Allocated sections | Additional reservations | Committed union | Span from bank base |",
                  "|---|---:|---:|---:|---:|---:|"]
        for b in s["banks"]:
            lines.append(f"| {b['bank']} @ 0x{b['base']:08X} | {b['capacity_bytes']:,} | {b['section_bytes']:,} | {b['reserved_extra_bytes']:,} | {b['committed_bytes']:,} | {b['address_span_bytes']:,} |")
        lines += ["", "Shared SRAM4 partitions are derived from both linker maps and owned by CM7. CM4 D2 aliases are normalized; overlapping non-owner allocations fail the audit.", ""]
        lines += ["Static stack storage below is already included in section bytes. Storage presence does not establish task creation or execution.", "",
                  "| Core | Stack storage ELF symbol | Reserved bytes |", "|---|---|---:|"]
        for c in data["production"]:
            for stack in c.get("stack_storage_symbols", []):
                lines.append(f"| {c['core']} | `{stack['name']}` | {stack['size']} |")
        lines += [""]
    lines += ["## Unmodified production boot attempts", ""]
    for r in data.get("production_boot", []):
        lines += [f"- {r['core']}: **{r['classification']}**; final PC `0x{r['samples'][-1]['pc']:08X}`; location: {r.get('final_location', [])}; log `{r['log']}`."]
    lines += ["", "## Diagnostic ARM FreeRTOS measurements", "",
              "These values apply only to separately built diagnostic images under their observed limited workload. They are not production workload peaks.", ""]
    for r in data.get("diagnostics", []):
        v = r["variant"]
        lines += [f"### {r['core']} {'negative allocation test' if v['negative'] else 'available-task run'}", "",
                  f"Classification: **{r['classification']}**; validation: **{r.get('validation', 'FAIL')}**.",
                  f"ELF SHA-256: `{r['elf_sha256']}`; simulated duration {r['samples'][-1]['simulated_seconds']:.3f}s.",
                  "Generated-image changes: " + "; ".join(v["changes"]) + ".", ""]
        final = r["samples"][-1]
        lines += [f"Heap arena: **{final['heap_arena_bytes']:,} B**; observed arena-minus-minimum peak: **{final['observed_peak_heap_bytes']:,} B**; minimum free: **{final['minimum_ever_free_heap_bytes']:,} B**.",
                  f"Allocation failures: **{final.get('dynamic_allocation_failures', 'unknown')}**; task creation failures: **{final.get('task_creation_failures', 'unknown')}**.",
                  "Tick samples: " + ", ".join(str(s["xTickCount"]) for s in r["samples"]) + ".", "",
                  "| Task | Allocation | Stack allocated (B) | Minimum free (words / B) | Observed used (B) | Switch-ins |",
                  "|---|---|---:|---:|---:|---:|"]
        for t in final.get("tasks", []):
            lines.append(f"| {t['name']} | {t['allocation']} | {t['allocated_stack_bytes']} | {t['high_water_words']} / {t['minimum_remaining_stack_bytes']} | {t['observed_used_stack_bytes']} | {t['switches']} |")
        if v.get("memory_delta"):
            lines += ["", f"Diagnostic minus production linked memory: {v['memory_delta']} (instrumentation/build perturbation, not workload demand)."]
        lines += ["", f"Peripheral warning lines retained: {len(r['warnings'])}; log `{r['log']}`.", ""]
    lines += ["## Workload and regression evidence", "", "| Check | Result | Actual execution scope | ARM workload memory |",
              "|---|---|---|---|"]
    for check in data.get("checks", []):
        lines.append(f"| {check['name']} | {check['status']} | {check['scope']} | No |")
    for w in data.get("workloads", []):
        lines.append(f"| {w['name']} | {w['status']} | {w['execution_domain']} | No |")
        if w.get("integration"):
            i = w["integration"]
            lines += ["", f"Connected native run: {i.get('teach_cycles')} teaching cycles; {i.get('validated_samples')} validated samples; {i.get('stream_replayed_samples')} samples replayed from virtual durable storage; {i.get('blocked_recovery_attempts')} blocked recovery attempts.", ""]
    lines += ["", "## Pass/fail criteria", "",
              "- PASS_LIMITED_EVIDENCE requires fresh production builds, ELF/map audits, regression/negative tests, both diagnostic schedulers with advancing multi-sample ticks, executed task traces, valid nonzero stack margins, zero unexpected allocation failures, and all supported workload fixtures passing.",
              "- Negative ARM image must observe exactly two allocation failures and one task-creation failure while scheduling continues.",
              "- Missing markers, duplicate responses, monitor errors, impossible counters, trace overflow, unexpected tasks/stacks, ownership conflicts, build/test errors or timeouts produce FAILED and exit 1.",
              "- Full runtime requires representative production ARM workloads on both cores with real IPC and verified task coverage. It remains UNVERIFIED here. `--require-full-runtime` exits 2 for this unmet gate; default exits 0 only for the explicitly limited evidence contract.",
              "- Static-only mode reports STATIC_ONLY, excludes runtime acceptance, and cannot satisfy `--require-full-runtime`.", "",
              "## Unverified requirements and interpretation", ""]
    lines += [f"- {item}" for item in LIMITATIONS]
    lines += ["", "See `docs/MEMORY_BENCHMARK.md` for model feasibility and measurement methods.", ""]
    return "\n".join(lines)


def render(data):
    if "application_runtime" not in data:
        return render_detailed(data)
    r=data["application_runtime"]
    a=r.get("application", {})
    identity=data.get("identity", {})
    lines=["# ARM application memory benchmark", "", f"Outcome: **{data['outcome']}**",
        f"Application evidence: **{r['classification']}**; full original dual-core runtime: **{data['full_runtime_status']}**.", "",
        f"Revision `{identity.get('commit')}` on `{identity.get('branch')}`; dirty tree: {bool(identity.get('status'))}.",
        f"Compiler: {identity.get('compiler')}; Release `-Os`, Cortex-M7, hard-float FPv5-D16.",
        "Emulator: Renode STM32H7, one Cortex-M7/FreeRTOS kernel; actual CM4 services recompiled for M7. Seed 745. Exact configuration and binary hashes are in results.json.", ""]
    if data.get("errors"):
        lines += ["Failures: " + "; ".join(data["errors"]), ""]
    lines += ["## Linked memory (bytes)", "", "| Image | Flash payload | Flash span | SRAM sections | SRAM with padding/reservations |",
        "|---|---:|---:|---:|---:|"]
    p=data.get("static", {})
    if p:
        lines.append(f"| Original production CM7 + CM4 | {p['flash_payload_bytes']:,} | {p['flash_image_spans_bytes']:,} | {p['sram_section_bytes']:,} | {p['sram_committed_bytes']:,} |")
    im=r["image"]; s=r["static"]
    lines.append(f"| Instrumented application composition | {im['flash_payload_bytes']:,} | {im['flash_span_bytes']:,} | {im['sram_section_bytes']:,} | {s['sram_committed_bytes']:,} |")
    lines += ["", "The application image includes virtual devices, test configuration, trace storage and the scenario task. These image totals describe different builds and must not be added together.", "",
        "## Actual ARM runtime", ""]
    final=r["samples"][-1]
    lines += [f"FreeRTOS arena **{final['heap_arena_bytes']:,} B**; observed peak **{final['observed_peak_heap_bytes']:,} B**; minimum free **{final['minimum_ever_free_heap_bytes']:,} B**; allocation failures **{final['dynamic_allocation_failures']}**; task creation failures **{final['task_creation_failures']}**.",
        f"Observed C-library break growth peak: **{a.get('c_heap_peak_break_growth_bytes', 'unverified')} B** (separate `_sbrk` observer).", "",
        "| Task | Reservation B | Minimum free words / B | Observed used B | Switch-ins |",
        "|---|---:|---:|---:|---:|"]
    for t in final["tasks"]:
        lines.append(f"| {t['name']} | {t['allocated_stack_bytes']} | {t['high_water_words']} / {t['minimum_remaining_stack_bytes']} | {t['observed_used_stack_bytes']} | {t['switches']} |")
    lines += ["", "The generated `defaultTask`, `BenchScenario`, and `M4IPC` wrapper are test composition overhead. Supervisor, StorageValidate, FT_RX, NorValidate and M4Progress execute the production entry points. Static task stacks and `ucHeap` already reside in `.bss`; their observed usage is a subset, never an additional SRAM allocation.", "",
        "## Asserted workload coverage", "", "| Workload | ARM result |", "|---|---|"]
    for c in r.get("coverage", []):
        lines.append(f"| {c['name']} | {c['status']} |")
    lines += ["", f"Validated **{a.get('validated_samples',0)}** samples in **{a.get('validations',0)}** committed trajectories; completed **{a.get('playbacks',0)}** preview/production playbacks; consumed **{a.get('playback_samples',0)}** samples, explicitly skipped **{a.get('skipped_playback_samples',0)}**.",
        f"Observed **{a.get('ft_frames',0)}** decoded F/T frames, **{a.get('guidance_ok_cycles',0)}** successful guidance cycles and **{a.get('observed_transitions',0)}** observed mode changes.",
        f"Capacity observations: validation queue **{a.get('max_validation_queue',0)}/64**, trajectory ring **{a.get('max_trajectory_ring',0)}/512**, F/T ring **{a.get('max_ft_ring',0)}/512**, taught segments **{a.get('max_program_segments',0)}/32**, Supervisor queue **{a.get('max_supervisor_queue',0)}/32**, raw geometry **{a.get('max_raw_geometry',0)}/512**.",
        f"Free heap before operations / after first cycle / after second cycle: **{a.get('heap_at_start_bytes')} / {a.get('heap_after_first_cycle_bytes')} / {a.get('heap_after_second_cycle_bytes')} B**.",
        "Fault assertions cover queue overload, rejected restart, overflow of the actual FT_RX ring and E-stop precedence. A separate ARM image must fail when the storage reader is deliberately failed; it cannot count as a successful workload run.", ""]
    negative=r.get("negative_test", {})
    lines.append(f"ARM reader-failure negative test: **{negative.get('validation', 'UNVERIFIED')}** ({negative.get('classification', 'not executed')}).")
    tests=data.get("regression_tests_executed",0)
    native=(f"**{sum(w['status']=='PASS' for w in data['workloads'])}/{len(data['workloads'])}** stages passed"
            if data.get("workloads") else "not requested in this run")
    lines += [f"Tool regressions: **{tests}** executed. Additional native/standalone checks: {native}. Startup-only evidence is retained in diagnostics.md.", "",
        "## Reproduction and limitations", "", "```sh", "python3 scripts/memory_benchmark.py",
        "# ARM application + negative test only, still with fresh production builds:",
        "python3 scripts/memory_benchmark.py --application-only", "```", "",
        "See commands.json for exact commands and logs, inputs.json for source hashes, application/run/run.resc and platform.repl for emulator setup, and results.json for all samples/ELF identities.", ""]
    for limitation in r.get("unverified", []):
        lines.append("- "+limitation)
    lines += ["", "No minimum replacement MCU or guaranteed worst-case memory is inferred. `--require-full-runtime` remains nonzero while the original topology and unexercised paths remain unverified.", ""]
    return "\n".join(lines)
