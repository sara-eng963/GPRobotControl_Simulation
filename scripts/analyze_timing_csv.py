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

FIELDS = (
    "schema_version", "source", "kind", "sequence", "period_us",
    "start_us", "finish_us", "duration_us", "interval_us",
    "scheduled_us", "start_lateness_us", "finish_lateness_us",
    "missed_deadline", "success", "dropped_total",
)
OUTPUT_FIELDS = (
    "source", "kind", "samples", "span_s", "period_us",
    "duration_p50_us", "duration_p95_us", "duration_p99_us", "duration_max_us",
    "start_late_p95_us", "start_late_p99_us", "start_late_max_us",
    "interval_mean_us", "interval_abs_jitter_p99_us",
    "deadline_misses", "deadline_miss_pct", "operation_failures",
    "logger_dropped", "missing_sequences",
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
        absent = set(FIELDS) - set(reader.fieldnames or [])
        if absent:
            raise ValueError(f"Unsupported timing CSV; missing columns: {sorted(absent)}")
        for line, original in enumerate(reader, start=2):
            if not original["source"]:
                continue
            try:
                if int(original["schema_version"]) != 1:
                    raise ValueError("unsupported schema_version")
                item = {k: int(original[k]) if original[k] not in ("", None)
                        else None for k in FIELDS
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
    starts = [r["start_us"] for r in values]
    durations = [r["duration_us"] for r in values]
    period = values[0]["period_us"]
    intervals = [r["interval_us"] for r in values if r["sequence"] > 0]
    if period:
        jitter = [abs(interval - period) for interval in intervals]
        late = [r["start_lateness_us"] for r in values
                if r["start_lateness_us"] is not None]
        misses = sum(r["missed_deadline"] or 0 for r in values)
    else:
        jitter = []
        late = []
        misses = None
    missing = sum(max(0, b["sequence"] - a["sequence"] - 1)
                  for a, b in zip(values, values[1:]))
    return {
        "source": source, "kind": values[0]["kind"],
        "samples": len(values),
        "span_s": (max(starts) - min(starts)) / 1e6,
        "period_us": period or "",
        "duration_p50_us": percentile(durations, 50),
        "duration_p95_us": percentile(durations, 95),
        "duration_p99_us": percentile(durations, 99),
        "duration_max_us": max(durations),
        "start_late_p95_us": percentile(late, 95),
        "start_late_p99_us": percentile(late, 99),
        "start_late_max_us": max(late) if late else float("nan"),
        "interval_mean_us": statistics.fmean(intervals) if intervals else float("nan"),
        "interval_abs_jitter_p99_us": percentile(jitter, 99),
        "deadline_misses": misses if misses is not None else "",
        "deadline_miss_pct": (100.0 * misses / len(values)) if misses is not None else float("nan"),
        "operation_failures": sum(r["success"] == 0 for r in values),
        "logger_dropped": max(r["dropped_total"] for r in values),
        "missing_sequences": missing,
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
        "interval_p99_us": percentile(intervals, 99),
        "interval_max_us": max(intervals) if intervals else float("nan"),
        "intervals_over_2500us": sum(x > 2500 for x in intervals),
    }


def write_csv(path: Path, summaries: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        for s in summaries:
            writer.writerow(s)


def make_plots(groups: dict[str, list[dict]], folder: Path) -> list[Path]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Charts skipped: install matplotlib with 'python3 -m pip install matplotlib'.")
        return []
    generated: list[Path] = []
    for source, items in groups.items():
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
            lateness = [i["start_lateness_us"] for i in items]
            ax.plot(x, lateness, linewidth=0.7)
            ax.axhline(0, linestyle="--")
            ax.set(title=f"{source}: release lateness vs initial periodic schedule",
                   xlabel="Elapsed time (s)", ylabel="Start lateness (us)")
            fig.tight_layout()
            path = folder / f"{source}_start_lateness.png"
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
        "**These measurements are host observations, NOT STM32H745 WCET "
        "certification or physical CAN deadline guarantees.**",
        "The recorder itself adds timestamp/queue overhead. All deadlines "
        "are measured against a 1 kHz or 100 Hz nominal clock derived from "
        "the first observed start; catch-up and OS scheduling matter.",
        "",
        "| Task | Rows | P99 exec (us) | Max exec (us) | P99 start lateness (us) | Misses | Lost log rows |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        misses = str(s["deadline_misses"]) if s["period_us"] else "N/A"
        lines.append(
            f"| {s['source']} | {s['samples']} | "
            f"{pretty(s['duration_p99_us'])} | "
            f"{pretty(float(s['duration_max_us']))} | "
            f"{pretty(s['start_late_p99_us'])} | {misses} | "
            f"{s['logger_dropped']} |"
        )
    lines.extend(["", "## Interpretation", ""])
    for s in summaries:
        if s["period_us"]:
            lines.append(
                f"- **{s['source']}**: {s['deadline_misses']} observed nominal "
                f"period deadline misses / {s['samples']} samples "
                f"({pretty(s['deadline_miss_pct'], 3)}%). "
                f"P99 start lateness {pretty(s['start_late_p99_us'])} us; "
                f"maximum execution {pretty(float(s['duration_max_us']))} us."
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
            f"P99 absolute deviation from 2,000 us: {pretty(c['abs_jitter_p99_us'])} us. "
            f"Maximum within-burst interval: {pretty(c['interval_max_us'])} us. "
            f"Intervals > 2,500 us: {c['intervals_over_2500us']}.",
            "These are intervals between host calls to send six RPDO4 frames "
            "and SYNC, **not** physical CAN frame arrival timestamps. "
            "When command bursts are interrupted, no 500 Hz conclusion "
            "should be drawn from a whole-run average.",
        ])
    lines.extend([
        "", "## What to conclude",
        "",
        "A pass on WSL means only that the observed host run stayed within "
        "the selected nominal bounds under this workload. "
        "It cannot prove worst-case STM32 timing, Cortex-M7/M4 contention, "
        "QSPI cache effects, or 1 Mbit/s CAN on-wire latency. "
        "For MCU sign-off, repeat with a DWT cycle counter, board GPIO traces, "
        "and real bus measurements.",
        "",
    ])
    report = out_dir / "timing_report.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    charts = [] if args.no_plots else make_plots(groups, out_dir)
    for s in summaries:
        flag = "WARN" if (s["deadline_misses"] or s["logger_dropped"]) else "INFO"
        print(f"[{flag}] {s['source']:<14} N={s['samples']:<7} "
              f"p99={pretty(s['duration_p99_us']):>9}us "
              f"max={pretty(float(s['duration_max_us'])):>9}us "
              f"misses={s['deadline_misses'] if s['period_us'] else 'N/A'} "
              f"log_drops={s['logger_dropped']}")
    if "can_targets" in groups:
        print("CAN event-burst analysis:", can_bursts(groups["can_targets"]))
    print("Saved:", out_dir / "summary_metrics.csv")
    print("Saved:", report)
    if charts:
        print(f"Saved {len(charts)} PNG timing plots in {out_dir}")
    print("Interpretation: host-only observations, NOT hard real-time verification.")


if __name__ == "__main__":
    main()
