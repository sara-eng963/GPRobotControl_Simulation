"""ELF allocation audit in physical address space (including CM4 aliases)."""
from __future__ import annotations

import re
from pathlib import Path
from .common import BenchmarkError, sha256

BANKS = (
    ("ITCM", 0x00000000, 0x10000), ("DTCM", 0x20000000, 0x20000),
    ("AXI", 0x24000000, 0x80000), ("SRAM1", 0x30000000, 0x20000),
    ("SRAM2", 0x30020000, 0x20000), ("SRAM3", 0x30040000, 0x8000),
    ("SRAM4", 0x38000000, 0x10000),
    ("FLASH1", 0x08000000, 0x100000), ("FLASH2", 0x08100000, 0x100000),
)
SHARED = {"SRAM4_IPC", "SRAM4_TRAJECTORY", "SRAM4_RESERVED"}


def physical(address, core):
    return address + 0x20000000 if core == "CM4" and 0x10000000 <= address < 0x10048000 else address


def union(ranges):
    merged = []
    for start, end in sorted(ranges):
        if end < start:
            raise BenchmarkError("Inverted memory interval")
        if end == start:
            continue
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
        else:
            merged.append((start, end))
    return merged


def length(ranges):
    return sum(b - a for a, b in union(ranges))


def split_banks(start, size):
    end = start + size
    pieces = [(name, max(start, base), min(end, base + capacity))
              for name, base, capacity in BANKS if start < base + capacity and end > base]
    if sum(b - a for _, a, b in pieces) != size:
        raise BenchmarkError(f"Unmapped/cross-invalid physical allocation: {start:#x}+{size:#x}")
    return pieces


def sections(text):
    result = []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"\s*\d+\s+(\S+)\s+([\da-fA-F]+)\s+([\da-fA-F]+)\s+([\da-fA-F]+)\s+[\da-fA-F]+\s+2\*\*\d+", line)
        if m:
            name, size, vma, lma = m.groups()
            result.append(dict(name=name, size=int(size, 16), vma=int(vma, 16),
                               lma=int(lma, 16), flags=lines[i + 1].strip()))
    if not result or not any(s["name"] == ".text" for s in result):
        raise BenchmarkError("Missing/unparseable ELF section headers")
    return result


def map_regions(text):
    try:
        block = text.split("Memory Configuration\n", 1)[1].split("Linker script and memory map", 1)[0]
    except IndexError as exc:
        raise BenchmarkError("Missing linker memory configuration") from exc
    regions = []
    for m in re.finditer(r"^([\w]+)\s+0x([\da-fA-F]+)\s+0x([\da-fA-F]+)\s+(\w+)\s*$", block, re.M):
        name, origin, size, attributes = m.groups()
        regions.append(dict(name=name, origin=int(origin, 16), size=int(size, 16), attributes=attributes))
    if not regions:
        raise BenchmarkError("No linker regions parsed")
    return regions


def symbols(text):
    result = {}
    for line in text.splitlines():
        m = re.fullmatch(r"([\da-fA-F]+)\s+(?:([\da-fA-F]+)\s+)?(\w)\s+(\S+)", line.strip())
        if m:
            address, size, kind, name = m.groups()
            result[name] = dict(address=int(address, 16), size=int(size or "0", 16), kind=kind)
    return result


def analyze(runner, elf: Path, core: str, out: Path):
    out.mkdir(parents=True, exist_ok=True)
    sec = sections(runner.run(["arm-none-eabi-objdump", "-h", elf], out / "sections.log")[1])
    syms = symbols(runner.run(["arm-none-eabi-nm", "-S", "-a", elf], out / "symbols.log")[1])
    regions = map_regions(elf.with_suffix(".map").read_text())
    allocations, flash, reservations = [], [], []
    for s in sec:
        if not s["size"] or "ALLOC" not in s["flags"]:
            continue
        addr = physical(s["vma"], core)
        containing = [r for r in regions if r["origin"] <= s["vma"] and s["vma"] + s["size"] <= r["origin"] + r["size"]]
        if len(containing) != 1:
            raise BenchmarkError(f"{core} {s['name']} outside exactly one linker region")
        reg = containing[0]["name"]
        for bank, a, b in split_banks(addr, s["size"]):
            if not bank.startswith("FLASH"):
                if reg in SHARED and core != "CM7":
                    raise BenchmarkError(f"CM4 allocates CM7-owned shared storage: {s['name']}")
                allocations.append(dict(core=core, section=s["name"], start=a, end=b, bank=bank, region=reg))
        if "LOAD" in s["flags"]:
            for bank, a, b in split_banks(s["lma"], s["size"]):
                if bank.startswith("FLASH"):
                    flash.append((a, b))
    if not flash:
        raise BenchmarkError(f"No flash image for {core}")
    for r in regions:
        if r["name"] in SHARED:
            start = physical(r["origin"], core)
            reservations.append(dict(start=start, end=start + r["size"], kind="shared partition", owner="CM7", region=r["name"]))
        elif not r["name"].startswith("FLASH"):
            placed = [a for a in allocations if a["region"] == r["name"]]
            if placed:
                reservations.append(dict(start=physical(r["origin"], core), end=max(a["end"] for a in placed),
                    kind="linked region placement including padding", owner=core, region=r["name"]))
    # MSP is initialized to _estack, not to the ._user_heap_stack section.
    for required in ("_estack", "_Min_Stack_Size", "_Min_Heap_Size", "ucHeap"):
        if required not in syms:
            raise BenchmarkError(f"Missing required memory symbol {core}:{required}")
    top = physical(syms["_estack"]["address"] - 1, core) + 1
    bottom = top - syms["_Min_Stack_Size"]["address"]
    split_banks(bottom, top - bottom)
    if any(a["start"] < top and a["end"] > bottom for a in allocations):
        raise BenchmarkError(f"{core} MSP reservation collides with sections")
    reservations.append(dict(start=bottom, end=top, kind="minimum MSP", owner=core))
    return dict(core=core, elf=str(elf), elf_sha256=sha256(elf), map_sha256=sha256(elf.with_suffix(".map")),
                flash_payload_bytes=length(flash), flash_span_bytes=max(b for _, b in flash)-min(a for a, _ in flash),
                flash_ranges=union(flash), sram_section_bytes=sum(a["end"]-a["start"] for a in allocations),
                allocations=allocations, reservations=reservations, linker_regions=regions,
                stack_storage_symbols=[dict(name=n, **v) for n, v in syms.items()
                    if "stack" in n.lower() and v["kind"] in "bBdD" and v["size"] >= 128],
                freertos_heap_arena_bytes=syms["ucHeap"]["size"],
                c_heap_minimum_bytes=syms["_Min_Heap_Size"]["address"],
                msp_minimum_bytes=top-bottom, symbols=syms)


def aggregate(cores):
    alloc = [a for c in cores for a in c["allocations"]]
    for i, a in enumerate(alloc):
        for b in alloc[i + 1:]:
            if a["start"] < b["end"] and b["start"] < a["end"]:
                raise BenchmarkError(f"Physical SRAM ownership collision: {a} / {b}")
    reserved = [r for c in cores for r in c["reservations"]]
    partitions = {}
    for r in reserved:
        if r.get("kind") == "shared partition":
            key, interval = r["region"], (r["start"], r["end"])
            if key in partitions and partitions[key] != interval:
                raise BenchmarkError(f"Cores disagree on shared partition {key}")
            partitions[key] = interval
        for a in alloc:
            if (r.get("owner") and a["core"] != r["owner"] and
                    a["start"] < r["end"] and r["start"] < a["end"]):
                raise BenchmarkError(f"Allocation collides with another owner's reservation: {a} / {r}")
    for i, a in enumerate(reserved):
        for b in reserved[i+1:]:
            if (a.get("owner") and b.get("owner") and a["owner"] != b["owner"] and
                    a["start"] < b["end"] and b["start"] < a["end"]):
                raise BenchmarkError(f"Cross-core reservation collision: {a} / {b}")
    rows = []
    for bank, base, capacity in BANKS:
        if bank.startswith("FLASH"):
            continue
        ar = [(a["start"], a["end"]) for a in alloc if a["bank"] == bank]
        rr = [(a, b) for r in reserved for name, a, b in split_banks(r["start"], r["end"]-r["start"]) if name == bank]
        used = union(ar + rr)
        rows.append(dict(bank=bank, base=base, capacity_bytes=capacity, section_bytes=length(ar),
                         committed_bytes=length(used), reserved_extra_bytes=length(used)-length(ar),
                         address_span_bytes=max((b-base for _, b in used), default=0), ranges=used))
    flashes = [r for c in cores for r in c["flash_ranges"]]
    if length(flashes) != sum(c["flash_payload_bytes"] for c in cores):
        raise BenchmarkError("Flash images overlap")
    return dict(banks=rows, flash_payload_bytes=length(flashes),
                flash_image_spans_bytes=sum(c["flash_span_bytes"] for c in cores),
                flash_fixed_address_span_bytes=max(b for _, b in flashes)-0x08000000,
                sram_section_bytes=sum(r["section_bytes"] for r in rows),
                sram_committed_bytes=sum(r["committed_bytes"] for r in rows))
