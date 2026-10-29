#!/usr/bin/env python3
"""Verify P05 DMA/service actually linked to CM7, in D2 SRAM, in both builds."""
import argparse
import re
import subprocess
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--build-root',required=True,type=Path)
a=p.parse_args()
elf=a.build_root/'CM7'/'STM32H745_CM7.elf'
if not elf.is_file(): raise SystemExit(f'Missing ARM ELF {elf}')
nm=subprocess.check_output(['arm-none-eabi-nm','-S','--defined-only',str(elf)],text=True)
sections=subprocess.check_output(['arm-none-eabi-readelf','-W','-S',str(elf)],text=True)
needed=('ft_uart2_dma_start','ft_uart2_dma_service','ft_uart2_dma_stop',
        'HAL_UART_RxHalfCpltCallback','HAL_UART_RxCpltCallback',
        'HAL_UART_ErrorCallback','application_ft_service_start')
for name in needed:
    if not re.search(rf'^\w+\s+\w+\s+[Tt]\s+{name}$',nm,re.M):
        raise SystemExit('Missing linked CM7 symbol: '+name)
# Read the section's actual linked address; a C-level annotation is insufficient.
m=re.search(r'\]\s+\.dma_rx_d2\s+NOBITS\s+([0-9a-fA-F]+)'
            r'\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+'
            r'\s+\w+\s+\d+\s+\d+\s+(\d+)',sections)
if not m:
    raise SystemExit('Missing DMA NOBITS section from linked CM7 ELF')
address,size,align=int(m[1],16),int(m[2],16),int(m[3])
if not (address==0x30047E00 and address+size==0x30048000 and
        address%32==0 and size==512 and align>=32):
    raise SystemExit(f'Non-DMA-safe placement: {address:#x} size={size} align={align}')
# CM4's 0x10000000 RAM is an alias of this same D2 bank. Check the
# actual stack ceiling and every allocated CM4 section, not just CM7 range.
elf4=a.build_root/'CM4'/'STM32H745_CM4.elf'
if not elf4.is_file(): raise SystemExit(f'Missing ARM ELF {elf4}')
nm4=subprocess.check_output(['arm-none-eabi-nm','--defined-only',str(elf4)],text=True)
stack=re.search(r'^([0-9a-fA-F]+)\s+\w\s+_estack$',nm4,re.M)
if not stack or int(stack[1],16)!=0x10047E00:
    raise SystemExit('CM4 stack ceiling overlaps or fails to reserve CM7 DMA alias')
sections4=subprocess.check_output(['arm-none-eabi-readelf','-W','-S',str(elf4)],text=True)
for match in re.finditer(r'\[\s*\d+\]\s+\S+\s+\S+\s+([0-9a-fA-F]+)\s+'
                         r'[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+\S+\s+(\w+)',sections4):
    start,length,flags=int(match[1],16),int(match[2],16),match[3]
    if 'A' in flags and length:
        physical=start+0x20000000 if 0x10000000<=start<0x10048000 else start
        if physical<address+size and physical+length>address:
            raise SystemExit('CM4 allocated section aliases CM7 DMA buffer')
# Both flash-boot and RAM-debug linker variants must reserve the same tail.
fw=Path(__file__).resolve().parents[1]/'STM32H745'
for variant in ('FLASH','RAM'):
    text=(fw/'CM4'/f'STM32H745ZITX_{variant}.ld').read_text()
    expected='288K - 512' if variant=='FLASH' else '160K - 512'
    if expected not in text:
        raise SystemExit('CM4 linker variant lacks D2 alias reservation: '+variant)
print('PASS: CM4 allocated sections and stack exclude DMA physical/alias interval')
print(f'PASS: CM7 linked DMA service and 512-byte cache-aligned RX at {address:#010x}')
print('NOTE: Linker placement does not validate physical bus/MPU timing')
