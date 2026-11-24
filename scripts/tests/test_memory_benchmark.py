"""Regression and fail-closed tests for the benchmark evidence boundary."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from membench.common import BenchmarkError, Runner
from membench.static import aggregate, length, map_regions, physical, sections, split_banks, symbols, union
from membench.runtime import decode_trace, parse_words, stack_watermark, validate_diagnostic
from membench.variants import replace_once
from membench.report import render
from membench.application import FIELDS, EXPECTED_TASKS, decode as decode_application, validate as validate_application, validate_negative


class MemoryAccounting(unittest.TestCase):
    def test_interval_union_counts_overlap_and_padding_once(self):
        self.assertEqual(union([(8, 12), (0, 8), (4, 9), (20, 24)]), [(0, 12), (20, 24)])
        self.assertEqual(length([(0, 4), (8, 12)]), 8)

    def test_cm4_alias_and_bank_boundary(self):
        self.assertEqual(physical(0x10047E00, "CM4"), 0x30047E00)
        self.assertEqual(split_banks(0x3001FFF0, 32),
                         [("SRAM1", 0x3001FFF0, 0x30020000), ("SRAM2", 0x30020000, 0x30020010)])
        self.assertEqual(physical(0x10000000, "CM7"), 0x10000000)

    def test_unmapped_and_out_of_bank_rejected(self):
        for address, size in [(0x30047FF0, 32), (0x12345678, 16), (0x2407FFF0, 32)]:
            with self.subTest(address=address), self.assertRaises(BenchmarkError):
                split_banks(address, size)

    def test_shared_partitions_deduplicated_but_colliding_storage_rejected(self):
        reservation = dict(start=0x38000000, end=0x38001000)
        allocation = dict(core="CM7", bank="SRAM4", start=0x38000000, end=0x38000100)
        cores = [dict(allocations=[allocation], reservations=[reservation], flash_ranges=[(0x08000000, 0x08000004)], flash_payload_bytes=4, flash_span_bytes=4),
                 dict(allocations=[], reservations=[reservation], flash_ranges=[(0x08100000, 0x08100004)], flash_payload_bytes=4, flash_span_bytes=4)]
        summary = aggregate(cores)
        self.assertEqual(summary["sram_section_bytes"], 256)
        self.assertEqual(summary["sram_committed_bytes"], 4096)
        cores[1]["allocations"] = [dict(allocation, core="CM4")]
        with self.assertRaises(BenchmarkError): aggregate(cores)

    def test_flash_overlap_rejected(self):
        core = dict(allocations=[], reservations=[], flash_ranges=[(0x08000000, 0x08000004)], flash_payload_bytes=4, flash_span_bytes=4)
        with self.assertRaises(BenchmarkError): aggregate([core, core])

    def test_shared_map_disagreement_and_other_owner_msp_rejected(self):
        a = dict(allocations=[], reservations=[dict(start=0x38000000, end=0x38001000,
                 kind="shared partition", region="SRAM4_IPC", owner="CM7")],
                 flash_ranges=[(0x08000000, 0x08000004)], flash_payload_bytes=4, flash_span_bytes=4)
        b = copy.deepcopy(a)
        b["flash_ranges"] = [(0x08100000, 0x08100004)]
        b["reservations"][0]["end"] += 32
        with self.assertRaises(BenchmarkError): aggregate([a, b])
        a["reservations"] = [dict(start=0x30047A00, end=0x30047E00, kind="minimum MSP", owner="CM4")]
        b["reservations"] = []
        b["allocations"] = [dict(core="CM7", bank="SRAM3", start=0x30047A00, end=0x30047E00)]
        with self.assertRaises(BenchmarkError): aggregate([a, b])

    def test_objdump_map_and_absolute_symbols(self):
        parsed = sections("  1 .text 00000008 08000000 08000000 00001000 2**2\n CONTENTS, ALLOC, LOAD, READONLY, CODE\n")
        self.assertEqual(parsed[0]["size"], 8)
        regions = map_regions("Memory Configuration\nRAM 0x10000000 0x00047e00 xrw\nLinker script and memory map\n")
        self.assertEqual(regions[0]["size"], 0x47E00)
        syms = symbols("00000400 A _Min_Stack_Size\n24000000 00003c00 b ucHeap\n")
        self.assertEqual(syms["_Min_Stack_Size"]["address"], 1024)
        self.assertEqual(syms["ucHeap"]["size"], 15360)
        for invalid in ("", "Memory Configuration\n*default* 0x0 0xffffffff\nLinker script and memory map"):
            with self.assertRaises(BenchmarkError): map_regions(invalid)
        with self.assertRaises(BenchmarkError): sections("not an ELF")


class RuntimeEvidence(unittest.TestCase):
    def test_monitor_accepts_only_complete_unique_bare_responses(self):
        valid = "MB_WORD_one\n0x12345678\nMB_WORD_two\n0x00000001\nMB_COMPLETE\n"
        self.assertEqual(parse_words(valid, {"one", "two"})["one"], 0x12345678)
        invalid = [valid.replace("0x12345678", ""), valid.replace("MB_WORD_two", "MB_WORD_one"),
                   valid.replace("MB_COMPLETE", ""), valid+"[ERROR] command failed\n",
                   valid.replace("0x12345678", "[WARNING] read at 0x12345678"),
                   valid.replace("MB_WORD_two", "MB_WORD_unexpected")]
        for log in invalid:
            with self.subTest(log=log), self.assertRaises(BenchmarkError): parse_words(log, {"one", "two"})

    def test_stack_units_and_alignment(self):
        self.assertEqual(stack_watermark(b"\xa5"*15+b"\0"+b"\xa5"*16), 3)
        self.assertEqual(stack_watermark(b"\0"*64), 0)

    def test_trace_exact_requested_stack_size_and_corruption(self):
        data = bytearray(1312)
        struct.pack_into("<8I", data, 0, 0x4D424E43, 1, 1, 0, 0, 0, 2, 0)
        struct.pack_into("<6I16s", data, 32, 0x24000100, 0x24000000, 64, 2, 3, 1, b"task")
        task = decode_trace(data, b"\xa5"*48+b"\0"*16, 0x24000000)["tasks"][0]
        self.assertEqual(task["allocated_stack_bytes"], 64)
        self.assertEqual(task["minimum_remaining_stack_bytes"], 48)
        for offset, value in [(0, 0), (4, 2), (8, 33), (12, 1), (36, 0x10000000), (40, 0x100000), (44, 99)]:
            mutant = data.copy()
            struct.pack_into("<I", mutant, offset, value)
            with self.subTest(offset=offset), self.assertRaises(BenchmarkError): decode_trace(mutant, b"\xa5"*64, 0x24000000)
        with self.assertRaises(BenchmarkError): decode_trace(data[:-1], b"", 0)

    @staticmethod
    def diagnostic(negative=False):
        samples = []
        for i in range(3):
            samples.append(dict(heap_initialized=True, tasks=[dict(name=name, active=True, switches=i+1,
                handle=n, high_water_words=100-i) for n, name in enumerate(("defaultTask", "IDLE", "Tmr Svc"))],
                dynamic_allocation_failures=2 if negative else 0, task_creation_failures=1 if negative else 0))
        return dict(core="CM7", scheduler_and_ticks_verified=True, samples=samples)

    def test_scheduler_flag_alone_never_passes_diagnostic(self):
        r = self.diagnostic(); r["scheduler_and_ticks_verified"] = False
        with self.assertRaises(BenchmarkError): validate_diagnostic(r)

    def test_missing_task_exhaustion_unexpected_failures_nonmonotonic_watermarks(self):
        mutations = [("tasks", []), ("dynamic_allocation_failures", 1), ("task_creation_failures", 1), ("heap_initialized", False)]
        for field, value in mutations:
            r = self.diagnostic(); r["samples"][-1][field] = value
            with self.subTest(field=field), self.assertRaises(BenchmarkError): validate_diagnostic(r)
        r = self.diagnostic(); r["samples"][-1]["tasks"][0]["high_water_words"] = 0
        with self.assertRaises(BenchmarkError): validate_diagnostic(r)
        r = self.diagnostic(); r["samples"][-1]["tasks"][0]["high_water_words"] = 101
        with self.assertRaises(BenchmarkError): validate_diagnostic(r)

    def test_negative_image_has_separate_expected_failure_contract(self):
        validate_diagnostic(self.diagnostic())
        validate_diagnostic(self.diagnostic(True), negative=True)
        with self.assertRaises(BenchmarkError): validate_diagnostic(self.diagnostic(True))
        with self.assertRaises(BenchmarkError): validate_diagnostic(self.diagnostic(), negative=True)

    def test_unexpected_or_stalled_task_rejected(self):
        r = self.diagnostic()
        r["samples"][-1]["tasks"].append(dict(name="unexpected", active=True, switches=1))
        with self.assertRaises(BenchmarkError): validate_diagnostic(r)
        r = self.diagnostic()
        r["samples"][-1]["tasks"][0]["switches"] = 1
        with self.assertRaises(BenchmarkError): validate_diagnostic(r)

    def test_startup_patch_fails_if_source_anchor_changes(self):
        self.assertEqual(replace_once("a b c", "b", "d"), "a d c")
        for text in ("a c", "a b b c"):
            with self.assertRaises(BenchmarkError): replace_once(text, "b", "d")


class ExecutionFailures(unittest.TestCase):
    def test_cli_missing_dependency_still_writes_failure_artifacts(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "missing-renode"
            p = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1]/"memory_benchmark.py"),
                                "--renode", str(Path(d)/"does-not-exist"), "--output", str(out)],
                               capture_output=True, text=True, timeout=30)
            self.assertEqual(p.returncode, 1, p.stdout+p.stderr)
            result = json.loads((out/"results.json").read_text())
            self.assertEqual(result["outcome"], "FAILED")
            self.assertEqual(result["full_runtime_status"], "UNVERIFIED")
            self.assertTrue(result["errors"])
            self.assertTrue((out/"report.md").is_file())
            self.assertTrue((out/"commands.json").is_file())

    def test_timeout_and_nonzero_commands_retain_logs(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); runner = Runner(root, root)
            with self.assertRaises(BenchmarkError):
                runner.run([sys.executable, "-c", "print('partial', flush=True); import time; time.sleep(5)"], root/"timeout.log", timeout=0.05)
            self.assertIn("BENCHMARK TIMEOUT", (root/"timeout.log").read_text())
            with self.assertRaises(BenchmarkError):
                runner.run([sys.executable, "-c", "raise SystemExit(7)"], root/"exit.log")
            self.assertEqual(json.loads((root/"commands.json").read_text())[-1]["exit_code"], 7)

    def test_failure_report_always_marks_full_runtime_unverified(self):
        report = render(dict(outcome="FAILED", full_runtime_status="UNVERIFIED", date_utc="test", errors=["missing compiler"]))
        self.assertIn("missing compiler", report)
        self.assertIn("UNVERIFIED", report)
        self.assertNotIn("Outcome: **PASS", report)


class ApplicationEvidence(unittest.TestCase):
    @staticmethod
    def evidence():
        data=dict.fromkeys(FIELDS,0)
        data.update(magic=0x4150504D,version=1,status=1,completed_mask=0x1FE,assertions=50000,
            validations=2,playbacks=3,storage_commits=2,max_validation_queue=64,max_trajectory_ring=512,
            max_ft_ring=512,max_program_segments=32,max_supervisor_queue=32,max_raw_geometry=512,
            ft_fault_verified=1,restart_denied=1,ft_frames=20000,guidance_ok_cycles=300,
            safe_output_calls=2,peer_progress_sequence=10000,validated_samples=1692,storage_writes=1692,
            playback_samples=2540,expected_playback_samples=2540,observed_transitions=20,task_count=10,
            heap_at_start_bytes=13120,heap_after_first_cycle_bytes=13120,heap_after_second_cycle_bytes=13120)
        final=dict(heap_initialized=True,current_free_heap_bytes=13120,dynamic_allocation_failures=0,
            task_creation_failures=0,tasks=[dict(name=n,allocated_stack_bytes=size,active=True,
                switches=100,high_water_words=100) for n,size in EXPECTED_TASKS.items()],
            application_words=[data[k] for k in FIELDS])
        return dict(samples=[final],scheduler_and_ticks_verified=True)

    @staticmethod
    def mutate(result,field,value):
        result["samples"][-1]["application_words"][FIELDS.index(field)]=value

    def test_actual_workload_contract_distinct_from_scheduler_only(self):
        r=self.evidence(); validate_application(r)
        self.assertEqual(r["classification"],"ARM_APPLICATION_WORKLOADS_MEASURED")
        self.assertEqual(len(r["coverage"]),8)
        for field,value in (("status",0),("completed_mask",0x1E),("playbacks",0),
                            ("validations",1),("storage_commits",0),("guidance_ok_cycles",0),
                            ("playback_samples",2500),("max_ft_ring",34),("max_program_segments",31),
                            ("ft_fault_verified",0),("heap_after_second_cycle_bytes",13000)):
            r=self.evidence(); self.mutate(r,field,value)
            with self.subTest(field=field),self.assertRaises(BenchmarkError): validate_application(r)

    def test_linked_but_unexecuted_or_wrong_stack_task_rejected(self):
        for kind in ("missing","not-run","wrong-stack","exhausted","allocation"):
            r=self.evidence(); final=r["samples"][-1]
            if kind=="missing": final["tasks"].pop()
            if kind=="not-run": final["tasks"][-1]["switches"]=0
            if kind=="wrong-stack": final["tasks"][-1]["allocated_stack_bytes"]=8192
            if kind=="exhausted": final["tasks"][-1]["high_water_words"]=0
            if kind=="allocation": final["dynamic_allocation_failures"]=1
            with self.subTest(kind=kind),self.assertRaises(BenchmarkError): validate_application(r)

    def test_reader_negative_requires_actual_injected_failure_after_validation(self):
        r=self.evidence()
        for k,v in dict(status=2,phase=5,storage_read_fault_injected=1,storage_commits=1,
                        validations=1,last_state=11,last_fault=0x60004,failure_line=40,completed_mask=30).items():
            self.mutate(r,k,v)
        validate_negative(r)
        self.assertEqual(r["classification"],"EXPECTED_APPLICATION_FAILURE_OBSERVED")
        self.mutate(r,"storage_read_fault_injected",0)
        with self.assertRaises(BenchmarkError): validate_negative(r)

    def test_application_wire_header_and_missing_result_rejected(self):
        with self.assertRaises(BenchmarkError): decode_application([0]*48)
        with self.assertRaises(BenchmarkError): decode_application([0]*47)
        r=self.evidence(); del r["samples"][-1]["application_words"]
        with self.assertRaises(BenchmarkError): validate_application(r)


if __name__ == "__main__": unittest.main()
