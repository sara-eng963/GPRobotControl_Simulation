#!/usr/bin/env python3
"""4F1 SIL Kit diagnostics: host-only timing and pre-dispatch joint tracking.

The recorded q_ref is a *candidate*, not proof it was dispatched.
q_actual is read before this cycle's candidate is sent. Thus error metrics
are concurrent reference-minus-pre-dispatch-feedback observations, NOT
settled-point tracking accuracy, bandwidth, stability, or hardware safety.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from analyze_timing_csv import parse as parse_timing, summarize as summarize_timing


def quantile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def analyze_guidance(path: Path) -> tuple[dict, list[dict]]:
    required = {"time_us", "cycle", "accepted", "valid", "fault",
                "feedback_measurement_valid", "feedback_max_age_us"}
    required |= {f"q{i}" for i in range(1, 7)}
    required |= {f"actual_q{i}" for i in range(1, 7)}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not required.issubset(set(reader.fieldnames or ())):
            raise ValueError("Missing 4F1 guidance columns: " +
                             ", ".join(sorted(required - set(reader.fieldnames or ()))))
        rows = list(reader)
    if not rows:
        raise ValueError("Empty guidance trace (TEACHING may never have started)")
    observations = []
    bad_order = 0
    for i, row in enumerate(rows):
        try:
            row["time_us"] = int(row["time_us"])
            row["cycle"] = int(row["cycle"])
            row["accepted"] = int(row["accepted"])
            row["valid"] = int(row["valid"])
            row["fault"] = int(row["fault"])
            row["feedback_measurement_valid"] = int(row["feedback_measurement_valid"])
            row["feedback_max_age_us"] = int(row["feedback_max_age_us"])
            for axis in range(1, 7):
                for prefix in ("q", "actual_q"):
                    key = f"{prefix}{axis}"
                    row[key] = float(row[key])
                    if not math.isfinite(row[key]):
                        raise ValueError(f"nonfinite {key}")
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path}, row {i+2}: {exc}") from exc
        if i and (row["time_us"] <= rows[i-1]["time_us"] or
                  row["cycle"] <= rows[i-1]["cycle"]):
            bad_order += 1
        if row["valid"] and row["feedback_measurement_valid"]:
            observations.append(row)
    gaps = [b["time_us"] - a["time_us"] for a, b in zip(rows, rows[1:])
            if b["cycle"] == a["cycle"] + 1 and b["time_us"] >= a["time_us"]]
    missing = sum(max(0, b["cycle"] - a["cycle"] - 1)
                  for a, b in zip(rows, rows[1:]))
    axes = {}
    for axis in range(1, 7):
        ref = [r[f"q{axis}"] for r in observations]
        meas = [r[f"actual_q{axis}"] for r in observations]
        error = [a-b for a, b in zip(ref, meas)]
        abs_error = [abs(x) for x in error]
        axes[f"J{axis}"] = {
            "samples": len(error),
            "rms_error_rad": math.sqrt(statistics.fmean(x*x for x in error))
                if error else None,
            "p95_abs_error_rad": quantile(abs_error, 95),
            "max_abs_error_rad": max(abs_error) if error else None,
            "reference_range_rad": max(ref)-min(ref) if ref else None,
            "measured_range_rad": max(meas)-min(meas) if meas else None,
        }
    summary = {
        "trace_file": str(path),
        "samples": len(rows),
        "valid_samples": len(observations),
        "rejected": sum(not r["accepted"] for r in rows),
        "fault_samples": sum(bool(r["fault"]) for r in rows),
        "span_s": (rows[-1]["time_us"] - rows[0]["time_us"]) / 1e6,
        "consecutive_interval_count": len(gaps),
        "interval_p50_us": quantile(gaps, 50),
        "interval_p99_us": quantile(gaps, 99),
        "interval_max_us": max(gaps) if gaps else None,
        "observed_interval_gt_3000us": sum(x > 3000 for x in gaps),
        "missing_trace_cycles": missing,
        "nonmonotonic_order_pairs": bad_order,
        "max_feedback_age_us": max((r["feedback_max_age_us"] for r in rows), default=0),
        "axes": axes,
    }
    return summary, rows


def analyze_host_timing(path: Path) -> dict:
    if not path.exists():
        raise ValueError(f"Missing host timing CSV: {path}")
    groups = parse_timing(path)
    keys = ("supervisor_2ms", "guidance_observer", "guidance_dispatch")
    if not all(k in groups for k in keys):
        raise ValueError(f"Missing timing sources: {sorted(set(keys)-set(groups))}")
    return {key: summarize_timing(key, groups[key]) for key in keys}


def plots(rows: list[dict], folder: Path, axes: dict) -> list[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    valid = [r for r in rows if r["valid"] and r["feedback_measurement_valid"]]
    if len(valid) < 2:
        return []
    axis = max(range(1, 7), key=lambda a:
               axes[f"J{a}"]["measured_range_rad"] or 0)
    x = [(r["time_us"] - valid[0]["time_us"]) / 1e6 for r in valid]
    plt.figure(figsize=(10, 4))
    plt.plot(x, [r[f"q{axis}"] for r in valid],
             label="computed joint reference (pre-dispatch)")
    plt.plot(x, [r[f"actual_q{axis}"] for r in valid],
             label="measured joint feedback (pre-dispatch)", alpha=0.8)
    plt.xlabel("Observed time from first valid sample (s)")
    plt.ylabel(f"J{axis} angle (rad)")
    plt.title("4F1 SIL Kit — candidate reference vs observed joint")
    plt.legend()
    plt.tight_layout()
    name = folder / f"J{axis}_reference_vs_feedback.png"
    plt.savefig(name, dpi=160)
    plt.close()
    plt.figure(figsize=(10, 3))
    plt.plot(x, [r[f"q{axis}"] - r[f"actual_q{axis}"] for r in valid])
    plt.xlabel("Observed time from first valid sample (s)")
    plt.ylabel("q_ref - q_actual (rad)")
    plt.title("Same-cycle pre-dispatch difference (not settled error)")
    plt.tight_layout()
    e = folder / f"J{axis}_pre_dispatch_error.png"
    plt.savefig(e, dpi=160)
    plt.close()
    return [str(name), str(e)]


def run(guidance: Path, timing: Path, output: Path,
        min_valid: int, no_plots: bool) -> dict:
    observed, rows = analyze_guidance(guidance)
    host = analyze_host_timing(timing)
    if observed["valid_samples"] < min_valid:
        raise ValueError(f"Only {observed['valid_samples']} valid guidance cycles; "
                         f"requested >= {min_valid}")
    output.mkdir(parents=True, exist_ok=True)
    result = {"scope": "Host SIL Kit / POSIX only; no STM32 timing or physical safety claim",
              "guidance": observed, "timing": host}
    (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n",
                                          encoding="utf-8")
    ax = observed["axes"]
    lines = [
        "# Stage 4F1 — Hand-guiding host timing & tracking",
        "",
        "**Scope:** SIL Kit / POSIX host measurements only. Not a 500 Hz STM32 proof, "
        "not a physical safety assessment or human interaction stability test.",
        "",
        f"Guidance trace: `{guidance}`  ",
        f"Host timing trace: `{timing}`",
        "",
        f"Observed duration: {observed['span_s']:.3f} s; "
        f"valid pre-dispatch feedback/reference pairs: {observed['valid_samples']}; "
        f"rejected: {observed['rejected']}; fault rows: {observed['fault_samples']}.",
        f"Host trace missing cycle numbers: {observed['missing_trace_cycles']}; "
        f"nonmonotonic pairs: {observed['nonmonotonic_order_pairs']}; "
        f"max feedback age: {observed['max_feedback_age_us']} us.",
        "",
        "## Joint tracking observations",
        "",
        "| Axis | RMS qref-qactual (rad) | P95 abs (rad) | Max abs (rad) | "
        "qref range (rad) | actual range (rad) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, v in ax.items():
        fmt = lambda x: "n/a" if x is None else f"{x:.7f}"
        lines.append(f"| {name} | {fmt(v['rms_error_rad'])} | "
                     f"{fmt(v['p95_abs_error_rad'])} | "
                     f"{fmt(v['max_abs_error_rad'])} | "
                     f"{fmt(v['reference_range_rad'])} | "
                     f"{fmt(v['measured_range_rad'])} |")
    lines += ["", "These comparisons use feedback sampled **before** the corresponding "
              "joint reference was dispatched. The sample may come from a prior "
              "motor update and the command may subsequently be rejected. "
              "Do not interpret as settled accuracy or controller bandwidth.",
              "", "## Timing observations", "",
              "| Source | Rows | P95 body (us) | P99 body (us) | Max body (us) "
              "| P99 interval (us) | Overruns | Dropped log rows |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for key, t in host.items():
        fmt = lambda x: "n/a" if x is None or (
            isinstance(x, float) and math.isnan(x)) else f"{x:.1f}"
        lines.append(f"| {key} | {t['samples']} | "
                     f"{fmt(t['duration_p95_us'])} | "
                     f"{fmt(t['duration_p99_us'])} | "
                     f"{fmt(t['duration_max_us'])} | "
                     f"{fmt(t['interval_p99_us'])} | "
                     f"{t['execution_budget_overruns'] or 'n/a'} | "
                     f"{t['logger_dropped']} |")
    lines += ["", "The host clock measures observed task-body duration and start intervals; "
              "it cannot identify true RTOS release times or MCU deadline misses. "
              "Logger drops and missing sequences must be reviewed before using "
              "percentiles as representative.",
              ""]
    files = [] if no_plots else plots(rows, output, ax)
    if files:
        lines.append("## Plots")
        for filename in files:
            lines.append(f"![{Path(filename).stem}]({Path(filename).name})")
    (output / "REPORT.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return result


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--guidance", type=Path, required=True)
    p.add_argument("--timing", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--min-valid", type=int, default=50)
    p.add_argument("--no-plots", action="store_true")
    opt = p.parse_args()
    result = run(opt.guidance, opt.timing, opt.output_dir,
                 opt.min_valid, opt.no_plots)
    s = result["guidance"]
    print(f"4F1 HOST: {s['valid_samples']} valid / {s['samples']} rows, "
          f"{s['span_s']:.2f} s, fault={s['fault_samples']}, "
          f"missing_rows={s['missing_trace_cycles']}; "
          f"report={opt.output_dir / 'REPORT.md'}")


if __name__ == "__main__":
    main()
