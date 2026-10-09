#!/usr/bin/env python3
"""Upgrade restored pre-CAN Renode test sources for W25Q512JV v2 storage.
Run once from repository root after unpacking the implementation archive.
"""
from pathlib import Path

root=Path(__file__).resolve().parent.parent

repl=root/'Simulation/Renode/stm32h7_qspi_flash.repl'
if repl.exists():
    s=repl.read_text()
    s=s.replace('manufacturerId: 0x20','manufacturerId: 0xEF')
    s=s.replace('memoryType: 0xBB','memoryType: 0x40')
    s=s.replace('JEDEC manufacturer/type values are placeholders for the generic Renode',
      'JEDEC manufacturer/type values imitate the W25Q512JV, but the generic Renode')
    s=s.replace('SPI-NOR model. They do not select the final hardware part for the robot.',
      'SPI-NOR model is not a validated Winbond-specific behavioral model.')
    repl.write_text(s)

jedec=root/'Simulation/Renode/Firmware/qspi_jedec_test/qspi_jedec_test.c'
if jedec.exists():
    s=jedec.read_text().replace('EXPECTED_MANUFACTURER 0x20U','EXPECTED_MANUFACTURER 0xEFU')
    s=s.replace('EXPECTED_MEMORY_TYPE  0xBBU','EXPECTED_MEMORY_TYPE  0x40U')
    jedec.write_text(s)
resc=root/'Simulation/Renode/run_qspi_jedec_test.resc'
if resc.exists():
    s=resc.read_text().replace('0x0020BB20','0x002040EF').replace('20 BB 20','EF 40 20')
    resc.write_text(s)

for name in ('qspi_storage_backend_test','qspi_streaming_test'):
    path=root/f'Simulation/Renode/Firmware/{name}/{name}.c'
    if not path.exists(): continue
    s=path.read_text()
    # The old test had dummy sample/artifact CRC values. New storage rejects these.
    s=s.replace('#include <stdint.h>',
      '#include <stdint.h>\n#include "../../Storage/w25_artifact_crc.h"',1)
    anchor='''        const PvExecutionSample sample =
            make_sample(sample_index);'''
    assert s.count(anchor)==1, f'Unexpected old test loop layout in {path}'
    s=s.replace(anchor,anchor+'''
        sample_crc = w25q512jv_crc32(sample_crc,&sample,sizeof(sample));''')
    # Declare CRC accumulator just before primary write-samples loop.
    main_start=s.index('int main(void)')
    first_loop=s.index('    for (',s.index('storage_interface.begin(storage_interface.context)',main_start))
    s=s[:first_loop]+'    uint32_t sample_crc=0xFFFFFFFFUL;\n\n'+s[first_loop:]
    import re
    s,n=re.subn(r'    metadata\.artifact_crc = 0x[0-9A-Fa-f]+UL;','',s,count=1)
    assert n==1, f'Expected old dummy artifact crc in {path}'
    s,n=re.subn(r'    metadata\.sample_data_crc = 0x[0-9A-Fa-f]+UL;',
               '    metadata.sample_data_crc = ~sample_crc;',s,count=1)
    assert n==1, f'Expected old dummy data crc in {path}'
    anchor='    metadata.segments[0].reserved = 0U;'
    assert s.count(anchor)==1, f'Expected segment data in {path}'
    s=s.replace(anchor,anchor+'\n    metadata.artifact_crc = w25_artifact_crc(&metadata);')
    # Verify the reloaded metadata CRC rather than outdated hardcoded constants.
    s=re.sub(r'(reloaded_storage\.metadata\.artifact_crc) == 0x[0-9A-Fa-f]+UL',
          r'\1 == w25_artifact_crc(&reloaded_storage.metadata)',s)
    s=re.sub(r'(reloaded_storage\.metadata\.sample_data_crc) == 0x[0-9A-Fa-f]+UL',
          r'\1 == metadata.sample_data_crc',s)
    path.write_text(s)

    make=root/f'Simulation/Renode/Firmware/{name}/Makefile'
    m=make.read_text()
    m=m.replace('CFLAGS := \\', 'CFLAGS := -DW25Q_STM32H7_BACKEND -DW25Q_RENODE_VIRTUAL_TIME \\',1)
    m=m.replace('    $(BUILD_DIR)/qspi_nor_validated_storage.o \\',
      '    $(BUILD_DIR)/qspi_nor_validated_storage.o \\\n    $(BUILD_DIR)/w25q512jv_flash.o \\\n    $(BUILD_DIR)/stm32h7_w25q_bus.o \\',1)
    anchor='$(BUILD_DIR)/qspi_nor_validated_storage.o: ../../Storage/qspi_nor_validated_storage.c | $(BUILD_DIR)'
    assert anchor in m
    extra='''
$(BUILD_DIR)/w25q512jv_flash.o: ../../Storage/w25q512jv_flash.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32h7_w25q_bus.o: ../../Storage/stm32h7_w25q_bus.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
'''
    m=m.replace(anchor,extra+'\n'+anchor)
    make.write_text(m)

print('Renode sources and old tests updated. Verify the generic Renode model supports four-byte commands.')
