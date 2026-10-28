#!/usr/bin/env python3
"""Check repository-local CubeIDE graphs and the actual linked ARM ELFs."""
import argparse
from pathlib import Path
import re
import subprocess
from build_stm32 import FW, REPO, project_sources

PARTITIONS = [
    ("ipc", 0x38000000, 0x1000, "intercore_ipc_shared", 0x8A0),
    ("trajectory", 0x38001000, 0x4000, "intercore_trajectory_shared", 0x3040),
    ("validation", 0x38005000, 0x3000, "intercore_validation_storage", 0x3000),
]


def output(*args):
    return subprocess.check_output(args, text=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-root", type=Path, default=REPO / "build/stm32/Debug")
    args = parser.parse_args()
    assert len(list(FW.glob("*.ioc"))) == 1
    ioc = next(FW.glob("*.ioc")).read_text()
    assert "Mcu.CPN=STM32H745ZIT6" in ioc and "Mcu.Package=LQFP144" in ioc
    for core in ("CM7", "CM4"):
        for config in ("Debug", "Release"):
            sources, includes, _ = project_sources(core, config)
            assert includes and all(p.is_dir() for p in includes)
            assert len([s for s in sources if Path(s).name.startswith("startup_")]) == 1
            assert all("iitx" not in str(s).lower() for s in sources)
            for name in ("ipc_mailbox_freertos.c", "trajectory_stream.c"):
                assert sum(p.name == name for p in sources.values()) == 1
            assert sum(p.name == "validation_storage_bridge.c" for p in sources.values()) == 1
            assert sum(p.name == "trajectory_storage_service.c" for p in sources.values()) == 1
            # The M7 must compile the entire numerical observer chain; M4
            # remains an IPC/storage owner, not a hand-guiding controller.
            observer_sources = (
                "ft_sensor.c", "sensor_config.c", "wrench_frame.c",
                "admittance_control.c", "admittance_matlab_reference.c",
                "cartesian_joint_mapper.c", "hand_guiding_pipeline.c",
                "supervisor_dry_run_adapter.c"
            )
            for observer in observer_sources:
                assert sum(p.name == observer for p in sources.values()) == (
                    1 if core == "CM7" else 0), (core, observer)
        elf = args.build_root / core / ("STM32H745_" + core + ".elf")
        assert elf.is_file(), f"Build first: {elf}"
        nm = output("arm-none-eabi-nm", "-S", "--defined-only", str(elf))
        sections = output("arm-none-eabi-readelf", "-W", "-S", str(elf))
        for name, address, capacity, symbol, size in PARTITIONS:
            match = re.search(r"\] \.shared_" + name +
                r"\s+(?:NOBITS|PROGBITS)\s+([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+).*? (\d+)\s*$",
                sections, re.M)
            assert match, f"{core}: missing NOLOAD section {name}"
            addr, actual_size, align = int(match[1], 16), int(match[2], 16), int(match[3])
            assert addr == address and actual_size <= capacity
            if core == "CM7":
                assert actual_size == size and align >= 32
                assert re.search(r"\.shared_" + name + r"\s+NOBITS", sections)
                assert re.search(rf"^{address:08x} {size:08x} B {symbol}$", nm, re.M)
            else:
                assert actual_size == 0 and symbol not in nm, "CM4 must not allocate M7 objects"
            linker_text = (FW / core / "STM32H745ZITX_FLASH.ld").read_text()
            assert ".shared_" + name + " (NOLOAD)" in linker_text
            label = "NOBITS" if actual_size else "empty NOLOAD reservation"
            print(f"PASS: {core} .shared_{name}: 0x{addr:08x}, {actual_size} bytes, {label}")
        required = (["Reset_Handler", "main", "vTaskStartScheduler",
                     "application_supervisor_start", "supervisor_task_init",
                     "intercore_shared_memory_initialize", "EXTI1_IRQHandler"]
                    if core == "CM7" else
                    ["Reset_Handler", "main", "vTaskStartScheduler",
                     "application_ipc_service_run", "application_validation_service_start",
                     "validation_m4_init", "validation_m4_step", "validation_m4_shutdown",
                     "playback_m4_init", "playback_m4_step", "intercore_startup_validate",
                     "EXTI0_IRQHandler"])
        for name in required:
            assert re.search(rf"^[0-9a-f]+ [0-9a-f]+ [TW] {name}$", nm, re.M), name
        if core == "CM7":
            assert not re.search(r" T (validation_m4_step|playback_m4_step)$", nm, re.M)
            # These are linked, retained numerical functions, not simply
            # compiled but discarded by --gc-sections.
            for symbol in (
                "hand_guiding_dry_run_init", "hand_guiding_dry_run_tick",
                "hand_guiding_pipeline_step", "admittance_step",
                "cartesian_joint_mapper_step"
            ):
                assert re.search(r"^[0-9a-f]+ [0-9a-f]+ [TW] " +
                                 re.escape(symbol) + r"$", nm, re.M), symbol
        # Physical suppliers must remain undefined; no default success provider.
        assert not re.search(r" [TW] board_(m4_supply_|supply_commissioned_config)", nm)
        print(f"PASS: {core}: startup/application symbols and ownership")
    supervisor7 = (FW / "CM7/Application/StateMachine/supervisor_task.c").read_text()
    assert "service_guidance_dry_run(context);" in supervisor7
    service = supervisor7.split("static void service_guidance_dry_run(", 1)[1].split(
        "static void supervisor_task_entry(", 1)[0]
    assert "port->tick(port->context, &cycle)" in service
    assert "joint_drive_port_read_axis" in service
    assert "take_candidate(" not in service
    assert "joint_drive_port_send_targets(" not in service
    assert "guidance_dry_run.take_candidate != NULL" in supervisor7
    # The observer is opt-in and cannot supply a fake board provider.
    app7 = (FW / "CM7/Core/Src/application_supervisor.c").read_text()
    assert "board_supply_commissioned_config == NULL" in app7
    assert "hand_guiding_dry_run_init(&g_guidance_observer" in app7
    assert "hand_guiding_dry_run_port(&g_guidance_observer)" in app7
    print("PASS: M7 hand-guiding dry-run observer linked; no candidate dispatch")
    main7 = (FW / "CM7/Core/Src/main.c").read_text()
    mpu = main7[main7.index("void MPU_Config(void)\n{"):]
    for token in ("0x38000000", "MPU_REGION_SIZE_32KB", "MPU_TEX_LEVEL1",
                  "MPU_ACCESS_SHAREABLE", "MPU_ACCESS_NOT_CACHEABLE",
                  "MPU_INSTRUCTION_ACCESS_DISABLE"):
        assert token in mpu, f"Missing SRAM4 MPU setting: {token}"
    main4 = (FW / "CM4/Core/Src/main.c").read_text()
    body = main4[main4.index("void StartDefaultTask(void *argument)\n{"):]
    assert body.index("application_ipc_service_initialize()") < body.index(
        "application_validation_service_start()") < body.index("application_ipc_service_run()")
    forbidden = re.compile(r"\b(?:GP_\w*|gp_\w*|Gp\w*)")
    for p in FW.rglob("*"):
        if p.suffix in (".c", ".h") and not any(
                s in p.parts for s in ("Drivers", "Middlewares", "Debug", "Release")):
            assert not forbidden.search(p.read_text()), f"Legacy identifier: {p}"
    print("PASS: portable linked resources, single source ownership, ZIT6 configuration")
    print("PASS: SRAM4 MPU source policy, CM4 task wiring, no migrated legacy identifiers")
    print("NOTE: ELF/static checks do not validate physical MPU, IPC, timing or motion.")


if __name__ == "__main__":
    main()
