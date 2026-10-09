#!/usr/bin/env python3

import argparse
import csv
import os
import sys
from pathlib import Path

DEFAULT_LOG = Path("trajectory_memory_log.csv")
FLASH_SIZES_MIB = (8, 16, 32, 64)
RESERVE_FRACTION = 0.25


def mib(value: int) -> float:
    return value / (1024.0 * 1024.0)


def load_rows(path: Path):
    if not path.exists():
        return []

    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def reset_log(path: Path) -> int:
    if path.exists():
        path.unlink()
        print(f"Reset: removed {path}")
    else:
        print(f"Reset: {path} did not exist")
    return 0


def print_report(path: Path) -> int:
    rows = load_rows(path)

    if not rows:
        print("No validated trajectory history found.")
        print(f"Expected log: {path}")
        print("Run the full simulator, validate at least one path, then run this report again.")
        return 1

    total_raw = 0
    total_allocated = 0
    total_samples = 0
    total_duration = 0.0
    total_path_length = 0.0

    print("=" * 72)
    print("TEST 7.6 - HMI-GENERATED TRAJECTORY MEMORY REPORT")
    print("=" * 72)
    print(f"History file: {path}")
    print()

    for index, row in enumerate(rows, start=1):
        raw_bytes = int(row["raw_bytes"])
        allocated_bytes = int(row["flash_allocated_bytes"])
        sample_count = int(row["sample_count"])
        duration_s = float(row["duration_s"])
        path_length_m = float(row["path_length_m"])

        total_raw += raw_bytes
        total_allocated += allocated_bytes
        total_samples += sample_count
        total_duration += duration_s
        total_path_length += path_length_m

        print(f"Trajectory {index}")
        print(f"  program id:       {row['program_id']}")
        print(f"  segments:         {row['segments']}")
        print(f"  segment count:    {row['segment_count']}")
        print(f"  path length:      {path_length_m:.3f} m")
        print(f"  duration:         {duration_s:.3f} s")
        print(f"  samples:          {sample_count}")
        print(f"  raw storage:      {mib(raw_bytes):.3f} MiB")
        print(f"  flash allocation: {mib(allocated_bytes):.3f} MiB")
        print()

    reserve_bytes = int(total_allocated * RESERVE_FRACTION)
    planning_bytes = total_allocated + reserve_bytes

    print("-" * 72)
    print("COMBINED CAPACITY PLANNING")
    print("-" * 72)
    print(f"Validated trajectories: {len(rows)}")
    print(f"Combined path length:   {total_path_length:.3f} m")
    print(f"Combined motion time:   {total_duration:.3f} s ({total_duration / 60.0:.3f} min)")
    print(f"Total samples:          {total_samples}")
    print(f"Total raw storage:      {mib(total_raw):.3f} MiB")
    print(f"Sector/header alloc:    {mib(total_allocated):.3f} MiB")
    print(f"+25% planning reserve:  {mib(reserve_bytes):.3f} MiB")
    print(f"Capacity to plan for:   {mib(planning_bytes):.3f} MiB")
    print()

    print("Candidate flash sizes")
    for size_mib in FLASH_SIZES_MIB:
        device_bytes = size_mib * 1024 * 1024
        utilization = 100.0 * planning_bytes / device_bytes
        remaining = device_bytes - planning_bytes
        status = "FIT" if remaining >= 0 else "NO FIT"
        print(
            f"  {size_mib:>2} MiB -> {utilization:6.2f}% used, "
            f"remaining {mib(max(remaining, 0)):.3f} MiB, {status}"
        )

    print()
    print("Interpretation note:")
    print("  The simulator still keeps only one active payload in external_flash.bin.")
    print("  This report sums the per-trajectory allocations as the capacity that would")
    print("  be required if those validated programs were stored together in QSPI NOR.")
    print()
    print("TEST 7.6 DATASET REPORT: COMPLETE")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Report memory usage for HMI-generated validated trajectories."
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=DEFAULT_LOG,
        help=f"CSV history path (default: {DEFAULT_LOG})",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Delete the existing history log and start a fresh Test 7.6 session.",
    )

    args = parser.parse_args()

    if args.reset:
        return reset_log(args.log)

    return print_report(args.log)


if __name__ == "__main__":
    sys.exit(main())
