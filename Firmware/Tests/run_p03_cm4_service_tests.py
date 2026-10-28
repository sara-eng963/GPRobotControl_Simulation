#!/usr/bin/env python3
"""Host execution of actual production CM4 task service, scheduling/IPC fakes only."""
from pathlib import Path
import os, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'Firmware/STM32H745'
TEST=ROOT/'Firmware/Tests/test_p03_cm4_service.c'
with tempfile.TemporaryDirectory(prefix='p03-cm4-') as d:
    output=Path(d)/'p03-cm4'
    flags=['-std=c11','-O1','-g','-Wall','-Wextra','-Werror',
        '-I'+str(ROOT/'Firmware/Tests/p03_cm4_mocks'),
        '-I'+str(FW/'CM4/Core/Src'),'-I'+str(FW/'CM4/Core/Inc'),
        '-I'+str(FW/'CM4/Application/Intercore'),
        '-I'+str(FW/'CM4/Application/StateMachine'),
        '-I'+str(FW/'Common/Inc'),'-I'+str(FW/'CM7/Application/StateMachine')]
    cmd=[os.environ.get('CC','cc'),*flags,str(TEST),
         str(FW/'CM4/Application/Intercore/hmi_lifecycle.c'),'-o',str(output)]
    subprocess.run(cmd,check=True,timeout=60)
    subprocess.run([str(output)],check=True,timeout=15)
