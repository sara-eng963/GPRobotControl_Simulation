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

        # Schema v2 omits cumulative lateness/false deadline fields.
        v2_source = root / "schema_v2.csv"
        v2_fields = [
            "schema_version", "source", "kind", "sequence", "period_us",
            "start_us", "finish_us", "duration_us", "interval_us",
            "execution_budget_overrun", "success", "dropped_total",
        ]
        with v2_source.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(v2_fields)
            writer.writerows([
                [2, "supervisor", "periodic", 0, 1000,
                 100, 400, 300, 0, 0, 1, 0],
                [2, "supervisor", "periodic", 1, 1000,
                 1300, 2600, 1300, 1200, 1, 1, 0],
                [2, "path_feedback_late", "event", 0, 0,
                 2000, 2001, 1, 0, "", 1, 0],
                [2, "path_feedback_wait", "event", 0, 0,
                 1000, 1001, 1, 0, "", 1, 0],
                [2, "path_period_wait", "event", 0, 0,
                 1100, 1101, 1, 0, "", 1, 0],
                [2, "path_commands", "event", 0, 0,
                 2100, 2150, 50, 0, "", 1, 0],
                [2, "path_commands", "event", 1, 0,
                 4100, 4150, 50, 2000, "", 1, 0],
            ])
        subprocess.run(
            [sys.executable, sys.argv[1], str(v2_source), "--output-dir",
             str(root / "v2-report"), "--no-plots"],
            check=True, capture_output=True, text=True, timeout=8,
        )
        with (root / "v2-report" / "summary_metrics.csv").open(
                newline="", encoding="utf-8") as f:
            v2_data = {row["source"]: row for row in csv.DictReader(f)}
        assert v2_data["supervisor"]["execution_budget_overruns"] == "1"
        assert v2_data["supervisor"]["interval_mean_us"] == "1200.0"
        assert v2_data["path_commands"]["samples"] == "2"
        assert v2_data["path_feedback_late"]["samples"] == "1"
        report = (root / "v2-report" / "timing_report.md").read_text()
        assert "Path-execution 2 ms gating diagnosis" in report
        assert "Feedback not fresh while 2 ms command already due: 1" in report
        assert "Actual trajectory command calls: 2" in report

    print("PASS: v1/v2 analyzer and path-execution wait reason counts")


if __name__ == "__main__":
    main()
