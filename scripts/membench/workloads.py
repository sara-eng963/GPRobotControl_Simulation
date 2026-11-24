"""Reuse existing production-source fixtures; never attribute host RAM to ARM."""
from __future__ import annotations

import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
from .common import BenchmarkError


def execute(runner, renode, out):
    out.mkdir(parents=True, exist_ok=True)
    records = []

    def case(name, argv, scope, expected=(), timeout=360, env=None):
        print(f"[WORKLOAD] {name}", flush=True)
        log = out / f"{name}.log"
        code, output = runner.run(argv, log, timeout=timeout, env=env, check=False)
        missing = [marker for marker in expected if marker not in output]
        status = "PASS" if code == 0 and not missing else "FAIL"
        records.append(dict(name=name, status=status, execution_domain=scope,
                            log=str(log), expected_evidence=list(expected), missing_evidence=missing,
                            observed_evidence=re.findall(r"^(?:PASS:.*|P04_SCENARIO .*|P05_SUPERVISOR .*|\[PASS\].*)$", output, re.M),
                            arm_memory_measurement=False))
        return status == "PASS"

    case("production-control", [sys.executable, "Firmware/Tests/run_state_machine_alignment_tests.py"],
         "native production state/Supervisor/CANopen sources; deterministic devices and injected clock; mocked scheduler",
         ["PASS: production test_joint_drive_teaching", "PASS: production test_joint_drive_homing",
          "PASS: production test_joint_drive_approach", "PASS: production test_path_validation",
          *[f"P04_SCENARIO P04-{i:02d} PASS" for i in range(1, 11)],
          "P04_SCENARIO P04-BOUNDARY PASS"], timeout=600)
    case("production-faults", [sys.executable, "Firmware/Tests/run_p05_supervisor_source.py"],
         "native production Supervisor; feedback loss, peer recovery, deadline, task-entry scenarios; mocked scheduler",
         [f"P05_SUPERVISOR {name} PASS" for name in (
             "feedback_loss", "cm4_lost_and_recovered", "deadline_and_no_reauth", "real_task_entry")])
    case("cm4-validation-playback", ["bash", "Firmware/Tests/check_cm4_validation_service.sh"],
         "native CM4 service startup, virtual storage ownership/rollback, static/dynamic/missing-provider cases",
         ["PASS: invalid providers, allocation rollback, one owner, M4-only IO",
          "PASS: missing providers/invalid task period leave validation disabled"])
    case("runtime-policy-negative", [sys.executable, "Firmware/Tests/run_p03_runtime_evidence_tests.py"],
         "native production IPC safety policy; executed assertion rejection of fourteen compiled mutations",
         ["PASS: production P03 policy tests", "PASS: rejected mutation peer_age"])
    env = os.environ.copy()
    env["IPC_RUNTIME_BUILD_DIR"] = str(out / "ipc-build")
    case("freertos-ipc", ["bash", "Intercore/Tests/run_ipc_freertos_runtime.sh"],
         "real POSIX FreeRTOS message-buffer IPC; different kernel and port from STM32; no HSEM/cache verification", env=env)
    host = out / "ft-build"
    if case("ft-configure", ["cmake", "-S", "HandGuiding", "-B", host, "-DCMAKE_BUILD_TYPE=Debug"],
            "native F/T virtual transport and guidance fixtures"):
        if case("ft-build", ["cmake", "--build", host, "--parallel", "4"], "native F/T and guidance fixtures"):
            junit = out / "ft-tests.xml"
            if case("ft-scenarios", ["ctest", "--test-dir", host, "--output-on-failure", "--no-tests=error",
                                     "--output-junit", junit], "native F/T serial, DMA bridge, admittance, wrench, guidance and sensor fault tests"):
                root = ET.parse(junit).getroot()
                tests = root.findall("testcase")
                if len(tests) != 12 or any(t.find("failure") is not None or t.find("skipped") is not None for t in tests):
                    records[-1]["status"] = "FAIL"
                    records[-1]["missing_evidence"] = ["Expected twelve executed passing F/T tests"]
                records[-1]["test_names"] = [t.get("name") for t in tests]
    case("qspi-storage-streaming", [sys.executable, "scripts/run_renode_qspi_tests.py",
                                    "--renode", renode, "--workdir", out / "qspi"],
         "standalone ARM QSPI/storage/trajectory stream tests in Renode; no production scheduler",
         ["[SUMMARY] 5/5 PASS; 0 FAIL"], timeout=600)
    # The established connected runner deliberately builds outside the checkout.
    # Honor its policy, then retain all generated evidence/build logs in our run.
    with tempfile.TemporaryDirectory(prefix="membench-connected-") as temp:
        evidence = Path(temp) / "evidence"
        case("connected-virtual-controller", ["bash", "scripts/run_p05_virtual_end_to_end.sh", evidence],
             "native connected virtual CANopen six-axis control, F/T teaching, validation, durable trajectory playback and seven injected faults; production Supervisor with mocked scheduling",
             ["PASS: complete P05 software-only acceptance", "P05 host virtual chain PASS"], timeout=600)
        if evidence.is_dir():
            retained = out / "connected-evidence"
            shutil.copytree(evidence, retained)
            if (retained / "integration.json").is_file():
                import json
                records[-1]["integration"] = json.loads((retained / "integration.json").read_text())
            records[-1]["retained_evidence"] = str(retained)
    return records
