#!/usr/bin/env python3
"""Synthetic schema/analysis regression; NOT a benchmark run."""
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

COLUMNS = [
    "schema_version", "source", "kind", "sequence", "period_us",
    "start_us", "finish_us", "duration_us", "interval_us",
    "scheduled_us", "start_lateness_us", "finish_lateness_us",
    "missed_deadline", "success", "dropped_total",
]


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: test_analyzer.py scripts/analyze_timing_csv.py")
    with tempfile.TemporaryDirectory(prefix="gp-timing-analyzer-") as folder:
        root = Path(folder)
        source = root / "synthetic.csv"
        with source.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows([
                [1, "supervisor", "periodic", 0, 1000, 100, 400,
                 300, 0, 100, 0, -700, 1, 1, 0],
                [1, "supervisor", "periodic", 1, 1000, 1100, 1300,
                 200, 1000, 1100, 0, -800, 1, 1, 0],
                [1, "supervisor", "periodic", 2, 1000, 2100, 3300,
                 1200, 1000, 2100, 0, 200, 1, 1, 0],
                [1, "can_targets", "event", 0, 0, 300, 400,
                 100, 0, "", "", "", "", 1, 0],
            ])
        report_dir = root / "report"
        subprocess.run(
            [sys.executable, sys.argv[1], str(source), "--output-dir",
             str(report_dir), "--no-plots"],
            check=True, capture_output=True, text=True, timeout=8,
        )
        with (report_dir / "summary_metrics.csv").open(
                newline="", encoding="utf-8") as f:
            data = {row["source"]: row for row in csv.DictReader(f)}
        assert data["supervisor"]["samples"] == "3"
        assert data["supervisor"]["execution_budget_overruns"] == "1"
        assert data["supervisor"]["interval_samples"] == "2"
        assert data["supervisor"]["interval_mean_us"] == "1000.0"
        assert data["supervisor"]["intervals_over_150pct_period"] == "0"
        assert data["supervisor"]["duration_max_us"] == "1200"
        assert data["can_targets"]["execution_budget_overruns"] == ""
        assert (report_dir / "timing_report.md").exists()
    print("PASS: analyzer ignores legacy drifted misses and counts actual body overruns")


if __name__ == "__main__":
    main()
