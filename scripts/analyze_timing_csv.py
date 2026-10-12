#!/usr/bin/env python3
"""Analyze measured POSIX/SIL Kit runtime timing. Standard library required;
matplotlib is optional for PNG plots.

This script NEVER infers STM32H745 hard real-time guarantees from WSL/Linux.
Usage:
    python3 scripts/analyze_timing_csv.py .simulation-logs/timing-metrics.csv
    python3 scripts/analyze_timing_csv.py timing.csv --output-dir results --no-plots
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

V1_FIELDS = (
    "schema_version", "source", "kind", "sequence", "period_us",
    "start_us", "finish_us", "duration_us", "interval_us",
    "scheduled_us", "start_lateness_us", "finish_lateness_us",
    "missed_deadline", "success", "dropped_total",
)
V2_FIELDS = (
    "schema_version", "source", "kind", "sequence", "period_us",
    "start_us", "finish_us", "duration_us", "interval_us",
    "execution_budget_overrun", "success", "dropped_total",
)
V3_FIELDS = V2_FIELDS + ("value",)
OUTPUT_FIELDS = (
    "source", "kind", "samples", "span_s", "period_us",
    "duration_p50_us", "duration_p95_us", "duration_p99_us", "duration_max_us",
    "execution_budget_overruns", "execution_budget_overrun_pct",
    "interval_samples", "interval_mean_us", "interval_p99_us",
    "interval_max_us", "interval_abs_jitter_p99_us",
    "intervals_over_150pct_period", "operation_failures",
    "logger_dropped", "missing_sequences",
    "value_total", "value_p50", "value_p95", "value_p99", "value_max",
)


def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    a = sorted(values)
    location = p * (len(a) - 1) / 100.0
    below, above = math.floor(location), math.ceil(location)
    return float(a[below] + (a[above] - a[below]) * (location - below))


def pretty(number: float, decimals: int = 1) -> str:
    return "n/a" if math.isnan(number) else f"{number:.{decimals}f}"


def parse(path: Path) -> dict[str, list[dict]]:
    groups: dict[str, list[dict]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        headers = set(reader.fieldnames or [])
        if set(V3_FIELDS).issubset(headers):
            schema = 3
            fields = V3_FIELDS
        elif set(V2_FIELDS).issubset(headers):
            schema = 2
            fields = V2_FIELDS
        elif set(V1_FIELDS).issubset(headers):
            schema = 1
            fields = V1_FIELDS
        else:
            raise ValueError("Unsupported timing CSV: expected schema v1, v2 or v3 fields")
        for line, original in enumerate(reader, start=2):
            if not original["source"]:
                continue
            try:
                if int(original["schema_version"]) != schema:
                    raise ValueError("schema_version does not match the header")
                item = {k: int(original[k]) if original[k] not in ("", None)
                        else None for k in fields
                        if k not in ("source", "kind")}
            except ValueError as error:
                raise ValueError(f"{path}:{line}: {error}") from error
            item["source"] = original["source"]
            item["kind"] = original["kind"]
            groups[item["source"]].append(item)
    for values in groups.values():
        values.sort(key=lambda d: d["sequence"])
    return groups


def summarize(source: str, values: list[dict]) -> dict:
    """Host-only observations. DO NOT trust schema-v1 missed_deadline.

    Old raw CSVs calculated missed_deadline against first_start + n*period,
    accumulating WSL FreeRTOS/POSIX clock drift. Recompute robust metrics
    from observed execution durations and consecutive wall-clock starts.
    """
    starts = [r["start_us"] for r in values]
    durations = [r["duration_us"] for r in values]
    period = values[0]["period_us"]
    # Only compare genuinely consecutive recorded events. A dropped row or
    # interrupted trace must never masquerade as a one-period interval.
    intervals = [b["start_us"] - a["start_us"]
                 for a, b in zip(values, values[1:])
                 if b["sequence"] == a["sequence"] + 1
                 and b["start_us"] >= a["start_us"]]
    if period:
        jitter = [abs(interval - period) for interval in intervals]
        overruns = sum(duration > period for duration in durations)
        long_gaps = sum(interval > period * 1.5 for interval in intervals)
    else:
        jitter = []
        overruns = None
        long_gaps = None
    missing = sum(max(0, b["sequence"] - a["sequence"] - 1)
                  for a, b in zip(values, values[1:]))
    measured_values = [r["value"] for r in values if r.get("value") is not None]
    return {
        "source": source, "kind": values[0]["kind"],
        "samples": len(values),
        "span_s": (max(starts) - min(starts)) / 1e6,
        "period_us": period or "",
        "duration_p50_us": percentile(durations, 50),
        "duration_p95_us": percentile(durations, 95),
        "duration_p99_us": percentile(durations, 99),
        "duration_max_us": max(durations),
        "execution_budget_overruns": overruns if overruns is not None else "",
        "execution_budget_overrun_pct":
            (100.0 * overruns / len(values)) if overruns is not None
            else float("nan"),
        "interval_samples": len(intervals),
        "interval_mean_us": statistics.fmean(intervals) if intervals
            else float("nan"),
        "interval_p99_us": percentile(intervals, 99),
        "interval_max_us": max(intervals) if intervals else float("nan"),
        "interval_abs_jitter_p99_us": percentile(jitter, 99),
        "intervals_over_150pct_period":
            long_gaps if long_gaps is not None else "",
        "operation_failures": sum(r["success"] == 0 for r in values),
        "logger_dropped": max(r["dropped_total"] for r in values),
        "missing_sequences": missing,
        "value_total": sum(measured_values) if measured_values else "",
        "value_p50": percentile(measured_values, 50),
        "value_p95": percentile(measured_values, 95),
        "value_p99": percentile(measured_values, 99),
        "value_max": max(measured_values) if measured_values else "",
    }


def can_bursts(values: list[dict], max_gap_us: int = 5000,
               min_burst: int = 10) -> dict:
    """Check near-500Hz traffic only within contiguous command bursts.
    Motion may pause for seconds; those gaps are not 2-ms cycle violations.
    """
    bursts: list[list[dict]] = []
    chunk: list[dict] = []
    for item in values:
        if (chunk and item["start_us"] - chunk[-1]["start_us"] > max_gap_us):
            if len(chunk) >= min_burst:
                bursts.append(chunk)
            chunk = []
        chunk.append(item)
    if len(chunk) >= min_burst:
        bursts.append(chunk)
    intervals = [b["start_us"] - a["start_us"] for burst in bursts
                 for a, b in zip(burst, burst[1:])]
    return {
        "bursts": len(bursts), "intervals": len(intervals),
        "abs_jitter_p99_us": percentile([abs(x - 2000) for x in intervals], 99),
        "interval_p50_us": percentile(intervals, 50),
        "interval_p95_us": percentile(intervals, 95),
        "interval_p99_us": percentile(intervals, 99),
        "interval_max_us": max(intervals) if intervals else float("nan"),
        "intervals_over_2500us": sum(x > 2500 for x in intervals),
        "excluded_boundaries": sum(
            b["start_us"] - a["start_us"] > max_gap_us
            for a, b in zip(values, values[1:])),

    }


def write_csv(path: Path, summaries: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        for s in summaries:
            writer.writerow(s)


def make_plots(groups: dict[str, list[dict]], folder: Path) -> list[Path]:
    try:
        # Force a headless renderer BEFORE importing pyplot. WSL/X11 display
        # probing can hang even when the user only wants saved PNG files.
        import matplotlib
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
    except ImportError:
        print("Charts skipped: install matplotlib with 'python3 -m pip install matplotlib'.")
        return []
    generated: list[Path] = []
    for source, items in groups.items():
        if items[0]["kind"] in ("counter", "gauge"):
            continue
        durations = [i["duration_us"] for i in items]
        if not durations:
            continue
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot([(i["start_us"] - items[0]["start_us"]) / 1e6 for i in items],
                durations, linewidth=0.7)
        ax.set(title=f"{source}: execution time (WSL host)", xlabel="Elapsed time (s)",
               ylabel="Execution time (us)")
        if items[0]["period_us"]:
            ax.axhline(items[0]["period_us"], linestyle="--",
                       label=f"Period {items[0]['period_us']} us")
            ax.legend()
        fig.tight_layout()
        path = folder / f"{source}_execution_time.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        generated.append(path)
        if items[0]["period_us"]:
            fig, ax = plt.subplots(figsize=(9, 4))
            x = [(i["start_us"] - items[0]["start_us"]) / 1e6 for i in items]
            xs = [((b["start_us"] - items[0]["start_us"]) / 1e6,
                   b["start_us"] - a["start_us"])
                  for a, b in zip(items, items[1:])
                  if b["sequence"] == a["sequence"] + 1 and
                  b["start_us"] >= a["start_us"]]
            if xs:
                ax.plot([p[0] for p in xs], [p[1] for p in xs], linewidth=0.7)
            ax.axhline(items[0]["period_us"], linestyle="--",
                       label="Nominal period")
            ax.legend()
            ax.set(title=f"{source}: observed interval between starts",
                   xlabel="Elapsed time (s)", ylabel="Start-to-start interval (us)")
            fig.tight_layout()
            path = folder / f"{source}_start_interval.png"
            fig.savefig(path, dpi=160)
            plt.close(fig)
            generated.append(path)
    return generated


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path, help="Measured timing-metrics.csv")
    parser.add_argument("--output-dir", type=Path,
                        help="Where to write summary CSV, report and plots")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    if not args.csv_file.is_file():
        parser.error(f"Timing CSV does not exist: {args.csv_file}")
    out_dir = args.output_dir or args.csv_file.parent / "timing-analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    groups = parse(args.csv_file)
    if not groups:
        parser.error("CSV has no measurement rows. Run SIL Kit with GP_TIMING_CSV set.")
    summaries = [summarize(name, items) for name, items in sorted(groups.items())]
    write_csv(out_dir / "summary_metrics.csv", summaries)

    lines = [
        "# Timing interpretation — PC/WSL experiment",
        "",
        "Source: " + str(args.csv_file),
        "",
        "**Host observations only — not STM32H745 worst-case execution time "
        "or hardware deadline certification.**",
        "",
        "**Raw schema:** version " + str(next(iter(groups.values()))[0]["schema_version"]) + ". "
        "For legacy v1 files, missed_deadline and first-anchored lateness fields "
        "accumulate POSIX/WSL tick drift and are deliberately **ignored**. "
        "Schema v2/v3 no longer emit those fields. Numeric v3 "
        "diagnostics are stored separately in the value column.",
        "",
        "Execution-budget overrun means a measured task BODY took longer than "
        "its nominal period after it started. It does not determine whether "
        "the task woke at its actual RTOS release instant.",
        "A long start interval means consecutive recorded starts were spaced "
        "by more than 1.5x the nominal period; it is NOT a proven deadline miss.",
        "",
        "| Task | Rows | P99 exec (us) | Max exec (us) | P99 interval (us) | Body overruns | Long intervals (>1.5x) | Lost rows |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        if s["kind"] in ("counter", "gauge"):
            continue
        over = str(s["execution_budget_overruns"]) if s["period_us"] else "N/A"
        gap = str(s["intervals_over_150pct_period"]) if s["period_us"] else "N/A"
        lines.append(
            f"| {s['source']} | {s['samples']} | "
            f"{pretty(s['duration_p99_us'])} | "
            f"{pretty(float(s['duration_max_us']))} | "
            f"{pretty(s['interval_p99_us'])} | {over} | {gap} | "
            f"{s['logger_dropped']} |"
        )
    lines.extend(["", "## Interpretation", ""])
    for s in summaries:
        if s["kind"] == "counter" or s["kind"] == "gauge":
            continue
        if s["period_us"]:
            lines.append(
                f"- **{s['source']}**: {s['execution_budget_overruns']} measured "
                f"body overruns / {s['samples']} invocations "
                f"({pretty(s['execution_budget_overrun_pct'], 3)}%). "
                f"Start intervals >1.5x nominal: "
                f"{s['intervals_over_150pct_period']} / {s['interval_samples']}; "
                f"mean actual interval {pretty(s['interval_mean_us'])} us "
                f"versus target {s['period_us']} us. "
                f"These are separate indicators, NOT an exact count of "
                f"missed RTOS deadlines."
            )
        else:
            lines.append(
                f"- **{s['source']}**: event-driven/best-effort source; "
                f"no artificial periodic deadline assigned. "
                f"P99 operation time {pretty(s['duration_p99_us'])} us."
            )
        if s["logger_dropped"] or s["missing_sequences"]:
            lines.append(f"  Warning: recorder dropped {s['logger_dropped']} records "
                         f"(observed sequence gaps {s['missing_sequences']}). "
                         "Treat percentiles as incomplete.")
        if s["operation_failures"]:
            lines.append(f"  Warning: {s['operation_failures']} instrumented calls returned failure.")
    if "can_targets" in groups:
        c = can_bursts(groups["can_targets"])
        lines.extend([
            "", "## 2 ms CAN target-command bursts",
            "",
            f"Detected {c['bursts']} burst(s) with at least 10 commands, "
            f"{c['intervals']} within-burst intervals.",
            f"P50/P95/P99 intervals: {pretty(c['interval_p50_us'])}/"
            f"{pretty(c['interval_p95_us'])}/{pretty(c['interval_p99_us'])} us. "
            f"P99 absolute deviation: {pretty(c['abs_jitter_p99_us'])} us. "
            f"Maximum within-burst interval: {pretty(c['interval_max_us'])} us. "
            f"Intervals > 2,500 us: {c['intervals_over_2500us']}.",
            "These are intervals between host calls to send six RPDO4 frames "
            "and SYNC, **not** physical CAN frame arrival timestamps. "
            f"Excluded across-burst boundaries (>5 ms): {c['excluded_boundaries']}. "
            "These must not be interpreted as good cadence or zero skips. "
            "When bursts are interrupted, no 500 Hz conclusion "
            "should be drawn from a whole-run average.",
        ])
    if any(name in groups for name in (
            "path_commands", "path_feedback_late",
            "path_feedback_wait", "path_period_wait")):
        counts = {
            name: len(groups.get(name, []))
            for name in (
                "path_commands", "path_feedback_late",
                "path_feedback_wait", "path_period_wait")
        }
        lines.extend([
            "", "## Path-execution 2 ms gating diagnosis", "",
            f"Actual trajectory command calls: {counts['path_commands']}.",
            f"Feedback not fresh while 2 ms command already due: "
            f"{counts['path_feedback_late']} Supervisor polling events.",
            f"Feedback not yet fresh before the command was due: "
            f"{counts['path_feedback_wait']} polling events.",
            f"Feedback ready but waiting for the 2 ms period: "
            f"{counts['path_period_wait']} polling events.",
            "",
            "These are **poll counts** at successive Supervisor iterations, "
            "**not** numbers of independently missed commands or motor failures. "
            "In legacy recordings, the feedback gate was checked before "
            "the period gate; new absolute-clock runs should record zero "
            "feedback-gate wait events.",
        ])
        if "path_commands" in groups:
            timing = can_bursts(groups["path_commands"])
            lines.extend([
                f"Trajectory-only burst intervals: {timing['intervals']} "
                f"across {timing['bursts']} bursts.",
                f"Trajectory command P50/P95/P99 intervals: "
                f"{pretty(timing['interval_p50_us'])}/"
                f"{pretty(timing['interval_p95_us'])}/"
                f"{pretty(timing['interval_p99_us'])} us; "
                f"max {pretty(timing['interval_max_us'])} us; "
                f">2,500 us: {timing['intervals_over_2500us']}; "
                f"excluded >5-ms boundaries: {timing['excluded_boundaries']}.",
                "Unlike the generic can_targets source, path_commands "
                "excludes guided/homing/other CAN transactions.",
            ])
    else:
        lines.extend([
            "", "## Path-execution 2 ms gating diagnosis", "",
            "No path-specific event counters in this recording. "
            "This is expected for pre-instrumentation baseline/flash CSVs. "
            "One NEW scenario run is needed for attribution.",
        ])

    if "path_samples_skipped" in groups or any(
            f"path_feedback_age_axis_{axis}" in groups
            for axis in range(1, 7)):
        skipped = sum(row.get("value") or 0
                      for row in groups.get("path_samples_skipped", []))
        lines.extend(["", "## Absolute trajectory scheduling and TPDO age", "",
                      f"Explicitly skipped samples: {skipped}.",
                      "Skipped samples indicate missed trajectory deadlines; "
                      "they are NOT successful commands or a performance improvement.",
                      "",
                      "| Axis | TPDO-age P50 (us) | P95 (us) | P99 (us) | Max (us) | Lost rows |",
                      "|---|---:|---:|---:|---:|---:|"])
        for axis in range(1, 7):
            key = f"path_feedback_age_axis_{axis}"
            if key not in groups:
                lines.append(f"| {axis} | N/A | N/A | N/A | N/A | N/A |")
                continue
            item = next(x for x in summaries if x["source"] == key)
            lines.append(
                f"| {axis} | {pretty(item['value_p50'])} | "
                f"{pretty(item['value_p95'])} | "
                f"{pretty(item['value_p99'])} | "
                f"{item['value_max']} | {item['logger_dropped']} |")
        lines.append("Age includes CAN receive processing and host RTOS tick "
                     "quantization; physical TPDO bus-arrival times are not measured.")
    lines.extend([
        "", "## What to conclude",
        "",
        "A test pass establishes functional host-simulation success, not a "
        "timing guarantee. Review task-body budgets and observed start-to-start "
        "intervals separately. It cannot prove worst-case STM32 timing, "
        "Cortex-M7/M4 contention, "
        "QSPI cache effects, or 1 Mbit/s CAN on-wire latency. "
        "For MCU sign-off, repeat with a DWT cycle counter, board GPIO traces, "
        "and real bus measurements.",
        "",
    ])
    report = out_dir / "timing_report.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    charts = [] if args.no_plots else make_plots(groups, out_dir)
    print("[TIMING] Ignoring flawed schema-v1 cumulative 'missed_deadline' field (if present).")
    print("[TIMING] Body overruns != actual RTOS deadline misses.")
    for s in summaries:
        if s["kind"] in ("counter", "gauge"):
            continue
        flag = "WARN" if (s["execution_budget_overruns"] or s["logger_dropped"]) else "INFO"
        overruns = (s["execution_budget_overruns"]
                    if s["period_us"] else "N/A")
        print(f"[{flag}] {s['source']:<14} N={s['samples']:<7} "
              f"p99={pretty(s['duration_p99_us']):>9}us "
              f"max={pretty(float(s['duration_max_us'])):>9}us "
              f"body_overruns={overruns} "
              f"p99_interval={pretty(s['interval_p99_us'])}us "
              f"log_drops={s['logger_dropped']}")
    if "can_targets" in groups:
        print("CAN event-burst analysis:", can_bursts(groups["can_targets"]))
    path_sources = (
        "path_commands", "path_feedback_late",
        "path_feedback_wait", "path_period_wait")
    if any(source in groups for source in path_sources):
        print("Path-execution event polls (not missed commands):",
              {source: len(groups.get(source, [])) for source in path_sources})
        if "path_commands" in groups:
            print("Trajectory-only command spacing:",
                  can_bursts(groups["path_commands"]))
    else:
        print("No path-specific diagnostics (legacy CSV); collect a new run.")
    if "path_samples_skipped" in groups:
        print("Skipped samples:", sum(x.get("value") or 0
              for x in groups["path_samples_skipped"]))
    for axis in range(1, 7):
        key = f"path_feedback_age_axis_{axis}"
        if key in groups:
            item = next(x for x in summaries if x["source"] == key)
            print(f"Axis {axis} TPDO age: p50 {pretty(item['value_p50'])}, "
                  f"p95 {pretty(item['value_p95'])}, "
                  f"p99 {pretty(item['value_p99'])}, "
                  f"max {item['value_max']} us")
    print("Saved:", out_dir / "summary_metrics.csv")
    print("Saved:", report)
    if charts:
        print(f"Saved {len(charts)} PNG timing plots in {out_dir}")
    print("Interpretation: host-only observations, NOT hard real-time verification.")


if __name__ == "__main__":
    main()
