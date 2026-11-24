#!/usr/bin/env python3
"""Build, audit and execute evidence-driven STM32H745 memory benchmarks.

Default: production CM7/CM4 Release builds, exact physical-bank accounting,
unmodified boot attempts, instrumented isolated ARM task runs, allocation-failure
negative test, actual production application tasks/workloads on ARM FreeRTOS,
native production-source workloads and standalone Renode storage tests.
Exit 0 = requested evidence checks passed, never full-controller qualification.
Exit 1 = build/audit/test/evidence failure; exit 2 = required full runtime absent.
Reports and all generated firmware are isolated under build/memory-benchmark/.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
from pathlib import Path
import shutil
import sys
import traceback

from membench.common import BenchmarkError, Runner, identity, input_manifest, sha256
from membench.static import aggregate, analyze
from membench.variants import build as build_variant
from membench.runtime import probe, validate_diagnostic
from membench.workloads import execute as execute_workloads
from membench.report import render, render_detailed
from membench.application import execute as execute_application

ROOT = Path(__file__).resolve().parents[1]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--renode", type=Path, default=Path.home()/"renode_portable/renode")
    parser.add_argument("--output", type=Path, help="New/empty result directory")
    parser.add_argument("--seconds", type=float, default=0.6, help="Total simulated duration per ARM run (default 0.6)")
    parser.add_argument("--timeout", type=int, default=120, help="Wall timeout per Renode process")
    parser.add_argument("--static-only", action="store_true", help="Fresh builds/audit/tests only; no runtime/workload acceptance")
    parser.add_argument("--application-only", action="store_true", help="Fresh builds/audit and ARM application workloads/negative test; skip startup diagnostics and native fixtures")
    parser.add_argument("--application-seconds", type=float, default=60, help="Simulated duration for asserted application workloads (default 60)")
    parser.add_argument("--application-timeout", type=int, default=600, help="Wall timeout per application Renode process (default 600s)")
    parser.add_argument("--require-full-runtime", action="store_true", help="Exit 2 unless full production ARM workload coverage is verified")
    args = parser.parse_args()
    if not 0.15 <= args.seconds <= 10 or args.timeout <= 0:
        parser.error("--seconds must be 0.15..10 and --timeout must be positive")
    if args.static_only and args.application_only:
        parser.error("--static-only and --application-only are mutually exclusive")
    if not 1 <= args.application_seconds <= 180 or args.application_timeout <= 0:
        parser.error("--application-seconds must be 1..180 and --application-timeout must be positive")
    args.repo = args.repo.expanduser().resolve()
    args.renode = args.renode.expanduser().resolve()
    if args.repo != ROOT:
        parser.error("Run this checkout's script; --repo must match the script's repository")
    return args


def write_reports(out, data, runner):
    runner.save_commands()
    (out/"report.md").write_text(render(data))
    (out/"diagnostics.md").write_text(render_detailed(data))
    artifacts = {str(p.relative_to(out)):sha256(p) for p in sorted(out.rglob("*"))
                 if p.is_file() and p.name not in {"artifacts.json", "results.json", "report.md"}}
    (out/"artifacts.json").write_text(json.dumps(artifacts, indent=2)+"\n")
    (out/"results.json").write_text(json.dumps(data, indent=2)+"\n")


def main():
    args = arguments()
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    out = (args.output or args.repo/"build/memory-benchmark"/stamp).expanduser().resolve()
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise SystemExit(f"Output must be a new/empty directory: {out}")
    out.mkdir(parents=True, exist_ok=True)
    runner = Runner(args.repo, out)
    data = dict(schema_version=1, date_utc=stamp, outcome="FAILED", full_runtime_status="UNVERIFIED",
                production=[], production_boot=[], diagnostics=[], workloads=[], checks=[], errors=[],
                contract="asserted ARM application workload measurements, separate startup diagnostics; no full dual-core qualification",
                invocation=sys.argv, parameters=dict(seconds=args.seconds, timeout=args.timeout,
                    static_only=args.static_only, application_only=args.application_only,
                    application_seconds=args.application_seconds, application_timeout=args.application_timeout,
                    require_full_runtime=args.require_full_runtime))
    code = 1
    try:
        required = ["arm-none-eabi-gcc", "arm-none-eabi-nm", "arm-none-eabi-objdump", "arm-none-eabi-addr2line", "git"]
        if not args.static_only and not args.application_only:
            required += ["cc", "cmake", "ctest", "make", "bash", "timeout", "arm-none-eabi-objcopy"]
        for tool in required:
            if shutil.which(tool) is None:
                raise BenchmarkError(f"Required executable missing: {tool}")
        if not args.static_only and not args.renode.is_file():
            raise BenchmarkError(f"Renode executable missing: {args.renode}")
        data["identity"] = identity(runner)
        print(f"[OUTPUT] {out}", flush=True)
        _, regression = runner.run([sys.executable, "-m", "unittest", "discover", "-s", "scripts/tests", "-p", "test_memory_benchmark.py", "-v"],
                                    out/"regression.log")
        match = re.search(r"Ran (\d+) tests?", regression)
        if not match or int(match.group(1)) == 0:
            raise BenchmarkError("No benchmark regression tests executed")
        data["regression_tests_executed"] = int(match.group(1))
        data["checks"].append(dict(name="benchmark regressions and negative evidence tests", status="PASS", scope="native tool unit/integration tests"))
        for core in ("CM7", "CM4"):
            print(f"[BUILD] production {core} Release", flush=True)
            target = out/"production"/core
            runner.run([sys.executable, "Firmware/Tests/build_stm32.py", "--core", core,
                        "--config", "Release", "--output", target], out/f"{core}.build.log", timeout=600)
            result = analyze(runner, target/f"STM32H745_{core}.elf", core, out/"audit"/core)
            if any(name.startswith("membench_") for name in result["symbols"]):
                raise BenchmarkError("Production build unexpectedly contains benchmark instrumentation")
            data["production"].append(result)
        data["static"] = aggregate(data["production"])
        data["checks"].append(dict(name="production ELF/map and shared ownership audit", status="PASS", scope="both ARM Release images"))
        if args.static_only:
            data["outcome"] = "STATIC_ONLY"
        else:
            installed = args.renode.parent/"platforms/cpus/stm32h7.repl"
            if not installed.is_file():
                raise BenchmarkError("Renode stm32h7 platform not found adjacent to executable")
            data["renode"] = dict(executable=str(args.renode), executable_sha256=sha256(args.renode),
                                  platform=str(installed), platform_sha256=sha256(installed))
            for binary in ("Infrastructure.dll", "Renode.dll"):
                path = args.renode.parent/binary
                if path.is_file():
                    data["renode"][binary+"_sha256"] = sha256(path)
            _, version=runner.run([args.renode, "--version"], out/"renode-version.log", timeout=args.timeout)
            data["renode"]["version_output"]=version.split("\n",1)[1].strip()
            for c in ([] if args.application_only else data["production"]):
                core = c["core"]
                print(f"[RENODE] unmodified production {core} boot", flush=True)
                data["production_boot"].append(probe(runner, Path(c["elf"]), core, args.renode,
                    out/"production-boot"/core, args.seconds, args.timeout))
            seed = None
            for c in ([] if args.application_only else data["production"]):
                core = c["core"]
                print(f"[RENODE] instrumented isolated {core} task run", flush=True)
                v = build_variant(runner, Path(c["elf"]).parent, core, out/"diagnostic-build"/core)
                vm = analyze(runner, Path(v["elf"]), core, out/"diagnostic-audit"/core)
                v["memory_delta"] = dict(flash_span_bytes=vm["flash_span_bytes"]-c["flash_span_bytes"],
                                          sram_section_bytes=vm["sram_section_bytes"]-c["sram_section_bytes"])
                r = probe(runner, Path(v["elf"]), core, args.renode, out/"diagnostic-run"/core,
                          args.seconds, args.timeout, instrumented=True, seed=seed)
                r["variant"] = v
                data["diagnostics"].append(r)
                validate_diagnostic(r)
                if core == "CM7":
                    seed = Path(r["shared_snapshot"])
            if not args.application_only:
                c = data["production"][0]
                print("[RENODE] allocation-failure negative image", flush=True)
                v = build_variant(runner, Path(c["elf"]).parent, "CM7", out/"negative-build", negative=True)
                vm = analyze(runner, Path(v["elf"]), "CM7", out/"negative-audit")
                v["memory_delta"] = dict(flash_span_bytes=vm["flash_span_bytes"]-c["flash_span_bytes"],
                                          sram_section_bytes=vm["sram_section_bytes"]-c["sram_section_bytes"])
                r = probe(runner, Path(v["elf"]), "CM7", args.renode, out/"negative-run",
                          args.seconds, args.timeout, instrumented=True)
                r["variant"] = v
                data["diagnostics"].append(r)
                validate_diagnostic(r, negative=True)
            data["application_runtime"] = execute_application(runner,data["production"],args.renode,
                out/"application",args.application_seconds,args.application_timeout,
                publish=lambda result:data.update(application_runtime=result))
            if not args.application_only:
                data["workloads"] = execute_workloads(runner, args.renode, out/"workloads")
            failed = [w["name"] for w in data["workloads"] if w["status"] != "PASS"]
            if failed:
                raise BenchmarkError(f"Workload/regression failures: {failed}")
            data["outcome"] = "PASS_ARM_APPLICATION_WORKLOADS"
        manifest = json.loads((out/"inputs.json").read_text())
        _, listing = runner.run(["git", "ls-files", "--cached", "--others", "--exclude-standard"], out/"git-end-inputs.log")
        final_manifest = input_manifest(args.repo, listing.split("\n", 1)[1].strip().splitlines())
        changed = [name for name in sorted(manifest.keys() | final_manifest.keys())
                   if manifest.get(name) != final_manifest.get(name)]
        if changed:
            raise BenchmarkError(f"Build inputs changed during execution: {changed}")
        data["checks"].append(dict(name="input immutability", status="PASS", scope=f"{len(manifest)} hashed working-tree inputs"))
        code = 2 if args.require_full_runtime else 0
    except (BenchmarkError, OSError, ValueError, KeyError) as exc:
        data["outcome"] = "FAILED"
        data["errors"].append(str(exc))
        print(f"[FAIL] {exc}", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        data["errors"].append("Interrupted by user")
        code = 130
    except Exception as exc:
        data["outcome"] = "FAILED"
        data["errors"].append(f"Unexpected {type(exc).__name__}: {exc}")
        (out/"exception.log").write_text(traceback.format_exc())
        print(f"[FAIL] {exc}; see {out/'exception.log'}", file=sys.stderr, flush=True)
    finally:
        data["exit_code"] = code
        write_reports(out, data, runner)
        print(f"[RESULT] {data['outcome']}; full production runtime {data['full_runtime_status']}", flush=True)
        print(f"[REPORT] {out/'report.md'}", flush=True)
        print(f"[DATA] {out/'results.json'}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
