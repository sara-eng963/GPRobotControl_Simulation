#!/usr/bin/env python3
"""Stage C1: audit actual linked ELF addresses by STM32 linker MEMORY bank.

Static evidence only: never reports runtime stack high-water, WCET, CAN timing,
NOR persistence, independent-core reset safety, or hardware functionality.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

MEMORY = re.compile(r'^\s*(\w+)\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)\s*,\s*LENGTH\s*=\s*(\d+)([KkMm]?)\b')
SECTION = re.compile(r'^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+(\S+)')
LOAD = re.compile(r'^\s*LOAD\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)')
SHARED = {'.gp_shared_ipc': (0x38000000, 0x1000),
          '.gp_shared_trajectory': (0x38001000, 0x4000),
          '.gp_shared_validation': (0x38005000, 0x3000)}

def parse_memory(source):
    banks = {}
    for line in source.splitlines():
        match = MEMORY.match(line.split('/*', 1)[0])
        if match:
            name, address, number, unit = match.groups()
            banks[name] = {'start': int(address, 16),
                           'size': int(number) * {'': 1, 'K': 1024, 'M': 1048576}[unit.upper()]}
    if 'FLASH' not in banks:
        raise ValueError('missing FLASH bank in linker MEMORY statement')
    ordered = sorted((v['start'], v['start'] + v['size'], k) for k, v in banks.items())
    for (_, end, name), (start, _, other) in zip(ordered, ordered[1:]):
        if end > start:
            raise ValueError(f'overlapping linker regions {name} and {other}')
    return banks

def parse_sections(output):
    result = []
    for line in output.splitlines():
        match = SECTION.match(line)
        if match:
            name, kind, addr, _offset, size, _entsize, flags = match.groups()
            if 'A' in flags and int(size, 16) > 0:
                result.append({'name': name, 'type': kind,
                               'addr': int(addr, 16), 'size': int(size, 16)})
    if not result:
        raise ValueError('no allocated sections in ELF')
    return result

def audit_sections(sections, banks, core):
    assignments = {k: [] for k in banks}
    for sec in sections:
        first, end = sec['addr'], sec['addr'] + sec['size']
        matches = [k for k, v in banks.items()
                   if first >= v['start'] and end <= v['start'] + v['size']]
        if len(matches) != 1:
            raise ValueError(f"{core}: {sec['name']} at 0x{first:08x} outside one region")
        assignments[matches[0]].append(sec)
    report = {}
    for name, bank in banks.items():
        items = sorted(assignments[name], key=lambda s: s['addr'])
        for left, right in zip(items, items[1:]):
            if left['addr'] + left['size'] > right['addr']:
                raise ValueError(f"{core}: overlapping {left['name']} and {right['name']}")
        high = max((v['addr'] + v['size'] - bank['start'] for v in items), default=0)
        report[name] = {'origin': f"0x{bank['start']:08x}", 'capacity': bank['size'],
                        'section_bytes': sum(v['size'] for v in items),
                        'end_offset': high, 'linker_headroom': bank['size'] - high}
    if core == 'CM7':
        for name, (address, capacity) in SHARED.items():
            matches = [s for s in sections if s['name'] == name]
            if len(matches) != 1 or matches[0]['type'] != 'NOBITS' or \
               matches[0]['addr'] != address or matches[0]['size'] > capacity:
                raise ValueError(f'CM7: wrong NOLOAD shared section {name}')
        if next(s['size'] for s in sections if s['name'] == '.gp_shared_validation') != 0x3000:
            raise ValueError('CM7 validation section must reserve full 12 KiB')
    elif any(0x38000000 <= s['addr'] < 0x38008000 for s in sections):
        raise ValueError('CM4 must not allocate M7-owned shared SRAM4 sections')
    return report

def flash_load(output, bank):
    intervals = []
    for line in output.splitlines():
        match = LOAD.match(line)
        if not match:
            continue
        _offset, _vaddr, address, size, _memsz = (int(v, 16) for v in match.groups())
        if size and bank['start'] <= address < bank['start'] + bank['size']:
            if address + size > bank['start'] + bank['size']:
                raise ValueError('flash file-backed LOAD segment exceeds configured bank')
            intervals.append((address, address + size))
    if not intervals:
        raise ValueError('no flash file-backed PT_LOAD segments')
    intervals.sort()
    begin, end = intervals[0]
    consumed = 0
    for start, finish in intervals[1:]:
        if start <= end:
            end = max(end, finish)
        else:
            consumed += end - begin
            begin, end = start, finish
    consumed += end - begin
    return {'load_file_bytes': consumed,
            'high_water': max(e for _, e in intervals) - bank['start']}

def selftest():
    banks = parse_memory('FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K\n'
                         'RAM (xrw) : ORIGIN = 0x24000000, LENGTH = 512K')
    sections = parse_sections(' [ 1] .text PROGBITS 08000000 010000 000080 00 AX 0 0 4\n'
                              ' [ 2] .bss NOBITS 24000000 020000 000010 00 WA 0 0 4')
    assert len(sections) == 2 and banks['FLASH']['size'] == 1048576
    assert audit_sections(sections, banks, 'HOST')['RAM']['section_bytes'] == 16
    try:
        audit_sections(sections + [{'name': '.bad', 'type': 'NOBITS',
                                    'addr': 0x24080000, 'size': 1}], banks, 'HOST')
    except ValueError:
        pass
    else:
        raise AssertionError('overflow must fail')
    line = ' LOAD 0x010000 0x08000000 0x08000000 0x000080 0x000080 R E 0x1000'
    assert flash_load(line, banks['FLASH'])['load_file_bytes'] == 128
    try:
        flash_load(line.replace('0x000080 0x000080', '0x200000 0x200000'),
                   banks['FLASH'])
    except ValueError:
        pass
    else:
        raise AssertionError('flash overflow must fail')
    print('PASS: Stage C1 parser unit tests (bank limits, ELF sections and LOAD ranges)')

def run(tool, *args):
    return subprocess.check_output([tool, *map(str, args)], text=True,
                                   stderr=subprocess.PIPE)

def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--self-test', action='store_true')
    cli.add_argument('--cm7', type=Path)
    cli.add_argument('--cm4', type=Path)
    cli.add_argument('--cm7-linker', type=Path)
    cli.add_argument('--cm4-linker', type=Path)
    cli.add_argument('--out', type=Path)
    cli.add_argument('--readelf', default='arm-none-eabi-readelf')
    args = cli.parse_args()
    if args.self_test:
        selftest()
        return
    if not all([args.cm7, args.cm4, args.cm7_linker, args.cm4_linker, args.out]):
        cli.error('both target ELFs, both FLASH linker scripts, and --out required')
    audits = {}
    for core in ('CM7', 'CM4'):
        elf = getattr(args, core.lower())
        ld = getattr(args, core.lower() + '_linker')
        if not elf.is_file() or not ld.is_file():
            raise ValueError(f'{core}: missing linked ELF or authoritative linker')
        banks = parse_memory(ld.read_text())
        sections = parse_sections(run(args.readelf, '-W', '-S', elf))
        allocations = audit_sections(sections, banks, core)
        loads = flash_load(run(args.readelf, '-W', '-l', elf), banks['FLASH'])
        audits[core] = {'elf': str(elf.resolve()),
                        'sha256': hashlib.sha256(elf.read_bytes()).hexdigest(),
                        'banks': allocations, 'flash_segments': loads}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'memory_regions.json').write_text(json.dumps(audits, indent=2) + '\n')
    lines = ['# Stage C1 linked STM32H745 memory audit', '',
             'STATIC only: linked section occupancy is NOT runtime task-stack high-water,',
             'CPU worst-case execution, physical memory endurance or timing.', '']
    for core, report in audits.items():
        lines += ['## ' + core, '', 'ELF SHA256: ' + report['sha256'], '',
                  '| Bank | Capacity B | Section bytes | End offset B | Linker headroom B |',
                  '|---|---:|---:|---:|---:|']
        for name, bank in report['banks'].items():
            lines.append(f"| {name} ({bank['origin']}) | {bank['capacity']} | "
                         f"{bank['section_bytes']} | {bank['end_offset']} | {bank['linker_headroom']} |")
        lines += ['', 'FLASH PT_LOAD bytes (including initialized data): ' +
                  str(report['flash_segments']['load_file_bytes']), '']
    lines += ['## Deferred evidence', '',
              '- Per-FreeRTOS-task stack watermarks and heap usage, after hardware bring-up.',
              '- Independent M4/M7 reset/MPU/cache coherency, interrupts, and powered tests.',
              '- 500 Hz scheduling WCET/jitter and real six-motor CAN bus timing.', '']
    (args.out/'memory_regions.md').write_text('\n'.join(lines))
    print('PASS: Stage C1 bank-by-bank linked ELF occupancy and SRAM4 ownership audit')
    for core, report in audits.items():
        print(f"{core}: flash PT_LOAD bytes={report['flash_segments']['load_file_bytes']}")
    print('REPORT:', args.out/'memory_regions.md')

if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print('FAIL: Stage C1 memory audit:', exc, file=sys.stderr)
        sys.exit(1)
