"""Synthetic fixture: verify 4F1 analyzer math and rejection rules.
This does NOT simulate a robot or measure timing."""
import csv
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from analyze_guidance_4f1 import run, analyze_guidance

TIMING_FIELDS = (
    "schema_version source kind sequence period_us start_us finish_us "
    "duration_us interval_us execution_budget_overrun success dropped_total value"
).split()

with tempfile.TemporaryDirectory(prefix="4f1-fixture-") as folder:
    d = Path(folder)
    f = d / "guide.csv"
    t = d / "timing.csv"
    cols = ("time_us cycle accepted valid fault "
            "fx fy fz tx ty tz vx vy vz wx wy wz "
            "q1 q2 q3 q4 q5 q6 feedback_measurement_valid feedback_max_age_us "
            "actual_q1 actual_q2 actual_q3 actual_q4 actual_q5 actual_q6").split()
    with f.open("w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=cols)
        w.writeheader()
        for i in range(50):
            r = dict.fromkeys(cols, "0")
            r.update(time_us=str(i * 2000 + 10000), cycle=str(i+1),
                     accepted="1", valid="1", feedback_measurement_valid="1",
                     feedback_max_age_us="1000")
            for j in range(1, 7):
                r[f"q{j}"] = str(j * 0.1 + i * 0.0001)
                r[f"actual_q{j}"] = str(j * 0.1 + i * 0.0001 - 0.002)
            w.writerow(r)
    with t.open("w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=TIMING_FIELDS)
        w.writeheader()
        for src, period, kind in (
            ("supervisor_2ms", 2000, "periodic"),
            ("guidance_observer", 0, "activity"),
            ("guidance_dispatch", 0, "event")):
            for i in range(50):
                w.writerow(dict(schema_version="3", source=src, kind=kind,
                    sequence=i, period_us=period, start_us=i*2000,
                    finish_us=i*2000+100, duration_us=100,
                    interval_us=(2000 if i else 0),
                    execution_budget_overrun=("0" if period else ""),
                    success=1, dropped_total=0, value=""))
    result = run(f, t, d / "report", min_valid=40, no_plots=True)
    assert result["guidance"]["valid_samples"] == 50
    assert abs(result["guidance"]["axes"]["J1"]["rms_error_rad"] - 0.002) < 1e-9
    assert result["guidance"]["missing_trace_cycles"] == 0
    assert result["timing"]["supervisor_2ms"]["samples"] == 50
    assert (d / "report" / "REPORT.md").exists()
    # Corruption is not silently treated as trustworthy telemetry.
    with f.open("w", newline="") as handle:
        handle.write("cycle,time_us\n1,1\n")
    try:
        analyze_guidance(f)
        raise AssertionError("missing measured-joint columns were accepted")
    except ValueError:
        pass
print("PASS: 4F1 synthetic timing/tracking analyzer and invalid-file rejection")
