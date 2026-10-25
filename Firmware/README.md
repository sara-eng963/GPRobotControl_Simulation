# STM32H745ZIT6 dual-core firmware

Firmware/STM32H745 is the authoritative target tree, integrated from the working
STM32H745ZIT6_BoardConfig CubeMX project. MCU: STM32H745ZIT6, LQFP144;
STM32CubeH7 1.13.0, vendored FreeRTOS 10.6.2. The .ioc, clocks, PLLs, peripheral
initialization, pin assignments and NVIC settings are preserved from that source.
The external Windows project is not modified.

**Software builds are not hardware commissioning or permission for robot motion.**
Physical F/T sensing, CM4 flash storage, final pins, CAN electrical/drive
commissioning and hardware safety validation remain incomplete. No default
board or storage supplier fabricates readiness.

See [executed migration results](MIGRATION_STATUS.md) for build/test evidence.

## Authoritative structure

- STM32H745/GP_STM32H745ZIT6_BoardConfig.ioc: exact MCU/generator configuration.
- STM32H745/CM7: Supervisor/CANopen integration, generated initialization,
  ZITX startup and FLASH/RAM linker scripts.
- STM32H745/CM4: generated initialization, IPC/HMI and separate validation task.
- STM32H745/Common: single target IPC/trajectory headers and transport sources,
  SRAM4 map and EXTI notifications.
- STM32H745/Drivers and Middlewares: target HAL, CMSIS and FreeRTOS dependencies.

CM4 links the two storage worker sources from CM7/Application/Intercore;
each image compiles its own copy, with one M4 task owning physical callbacks.
CM7 never calls flash storage in its control loop; unused M4 operations are
discarded from its ELF. The 25 portable ControlCore/ServoDrive/HandGuiding C
units are Eclipse-linked to the repository implementation. Forwarding headers
preserve include paths without another editable definition.
The root host API remains unchanged. Firmware-specific renamed Supervisor and
Intercore variants stay in the target tree; do not stage host copies over them.

## Build locally

From the repository root, with Python 3 and ARM GNU GCC/newlib installed:

~~~bash
python3 Firmware/Tests/build_stm32.py --core CM7
python3 Firmware/Tests/build_stm32.py --core CM4
python3 Firmware/Tests/check_firmware_integration.py
~~~

The build reads source entries, linked resources, includes and defines from each
checked-in CubeIDE project. It compiles and links independent images using the
same CPU/FPU ABI, nano/nosys specifications and ZITX FLASH linker scripts.
Defaults: Debug (-O0); --config Release selects -Os. Outputs and command records:
build/stm32/{Debug,Release}/{CM7,CM4}/. Override ARM_NONE_EABI_GCC if needed.
This is an ARM GCC equivalent build; the CubeIDE GUI and its particular packaged
compiler were not executed.

For STM32CubeIDE, import all three .project files under STM32H745 with
Existing Projects into Workspace. Preserve their relative directory layout
inside this complete repository because portable source links reach the
repository root. Build each core's Debug/Release configuration independently.
Linked resources use PARENT-n-PROJECT_LOC; no Windows SDK or staging path
is needed. The linker uses workspace_loc/ProjName. Do not copy old Debug makefiles
or rerun historical staging scripts.

Do not regenerate CubeMX merely to build. After intentional regeneration,
review user code, source links, Common transport ownership, includes and all
four linker scripts, then rerun the checks below.

## CM4 storage worker and safety gates

StartDefaultTask initializes IPC, attempts application_validation_service_start,
then enters the existing 1 ms IPC/HMI loop. The independent priority-1 task
steps validation_m4_step and playback_m4_step, then delays by 1 ms.

Both board_m4_supply_validation_backend and board_m4_supply_trajectory_reader
must be real strong board implementations for the SAME durable artifact.
Mandatory callbacks and capacities are checked before task creation.
Missing providers, invalid shared objects, invalid reader/backend or allocation
failure prevent startup; worker_ready stays false (or is revoked on allocation
failure). IPC continues when storage is absent. A second call cannot create a
competing worker. The static stack is provisionally 768 words; measure it on board.

Supervisor startup still requires real interlocks, physical input snapshots,
commissioned CANopen configuration, valid IPC, timing gates and ready storage.
No motion, flashing or hardware provider has been enabled by migration.

## Shared SRAM4 contract

| Partition | Address | Capacity | CM7 allocated bytes |
| --- | --- | --- | --- |
| IPC | 0x38000000 | 4096 | 2208 |
| Trajectory | 0x38001000 | 16384 | 12352 |
| Validation/reserved | 0x38005000 | 12288 | 12288 |
| Unallocated | 0x38008000 | 32768 | 0 |

CM7 defines the shared objects once. Its ELF sections are NOBITS and
32-byte aligned. CM4 defines no shared objects: its zero-size NOLOAD reservations
may appear as empty PROGBITS in GNU readelf. They contain no loadable shared data.
Both FLASH/RAM linker pairs retain the same partitions. CM7 MPU Region 1 retains
the shareable, non-cacheable, non-executable 32 KiB window.
Hardware cache coherency, dual-core resets and IPC signalling remain unverified.

## Verification

~~~bash
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh
bash Firmware/Tests/check_intercore_startup_compile.sh
bash Firmware/Tests/check_intercore_notifications.sh
bash Firmware/Tests/check_cm4_validation_service.sh
python3 Firmware/Tests/check_portable_protocol.py
python3 Firmware/Tests/check_tim2_tim3_integration.py
python3 Firmware/Tests/run_m7_timer_fault_injection.py
~~~

The CM4 native policy test uses test-only scheduler/provider mocks and exercises
static/dynamic allocation, missing providers, bad periods, malformed providers,
allocation failure, duplicate ownership and M7 queue/M4 IO separation.
Run host CMake/CTest as described in ../README.md for IPC, control, storage,
hand-guiding, SIL Kit and optional Renode regressions.

Tests/historical preserves superseded staging/release scripts.
STAGE*.md, FDCAN1_CUBEMX_CONFIGURATION.md and TIM2_TIM3_CUBEMX_CONFIGURATION.md
are historical milestone evidence. Their old II-package, staging paths, clocks
and pin assumptions are not current build instructions. Physical pin mapping
is unfinished; HARDWARE_ARRIVAL_VERIFICATION_PLAN.md is a planning checklist
that must be checked against the actual ZIT6 board before hardware work.
