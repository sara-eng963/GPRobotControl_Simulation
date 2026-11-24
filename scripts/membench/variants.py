"""Derive isolated diagnostic builds from the exact production command graph."""
from __future__ import annotations

import concurrent.futures
import shlex
from pathlib import Path
from .common import BenchmarkError, sha256


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise BenchmarkError(f"Diagnostic source anchor count {text.count(old)}: {old}")
    return text.replace(old, new)


def build(runner, production: Path, core: str, out: Path, negative=False, application=False, application_negative=False):
    if application_negative and not application:
        raise BenchmarkError("Application reader failure requires an application image")
    out.mkdir(parents=True, exist_ok=True)
    fw = runner.repo / "Firmware/STM32H745"
    support = runner.repo / "Simulation/Renode/MemoryBenchmark"
    source = fw / core / "Core/Src/main.c"
    config_source = fw / core / "Core/Inc/FreeRTOSConfig.h"
    original = source.read_text()
    changes = []
    if core == "CM7":
        text = replace_once(original,
            "while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));",
            "/* DIAGNOSTIC ONLY: CM4 absent, omit second D2 ready poll. */")
        changes.append("Omitted CM7 second D2CKRDY wait; no CM4 wake-up synchronization measured")
    else:
        text = replace_once(original,
            "HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);",
            "/* DIAGNOSTIC ONLY: no CM7/HSEM wake; initialized SRAM4 snapshot supplied. */")
        changes.append("Omitted CM4 STOP entry; SRAM4 seeded from paused CM7 diagnostic snapshot")
    if negative:
        text = replace_once(text, "void StartDefaultTask(void *argument)\n{",
                            "void membench_negative(void);\nvoid StartDefaultTask(void *argument)\n{\n  membench_negative();")
        changes.append("Injected one heap exhaustion and one oversized task creation")
    if application:
        if core != "CM7" or negative:
            raise BenchmarkError("Application image requires CM7 and cannot inject allocation failures")
        start=text.index("void StartDefaultTask(void *argument)\n{")
        end=text.index("\n /* MPU Configuration */",start)
        body=text[start:end]
        setup=body.index("  /* Persistent configuration; avoid a large task-stack allocation. */")
        loop=body.index("  for (;;)",setup)
        # Preserve the exact original progress observer, policy and 10ms loop.
        # Only optional board/service startup is moved to the test composition.
        body=body[:setup]+"  membench_application_start();\n\n"+body[loop:]
        text=text[:start]+"void membench_application_start(void);\n"+body+text[end:]
        changes.append("Isolated application composition: actual CM7/CM4 service tasks on one CM7 FreeRTOS kernel; deterministic hardware providers")
        if application_negative:
            changes.append("Negative application image injects storage read failure at the board reader boundary")
    patched = out / "main.c"
    patched.write_text(text)
    kernel_source = fw / "Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
    kernel = out / "tasks.c"
    kernel.write_text(replace_once(kernel_source.read_text(), "    if( pxCreatedTask != NULL )",
        "    /* TEST ONLY: capture requested depth before alignment loses bytes. */\n"
        "    membench_create(pxNewTCB, pxNewTCB->pxStack, ulStackDepth * sizeof(StackType_t),\n"
        "                    pxNewTCB->pcTaskName, pxNewTCB->ucStaticallyAllocated);\n\n"
        "    if( pxCreatedTask != NULL )"))
    changes.append("Generated tasks.c captures exact requested stack depth; kernel source unchanged")
    config = out / "FreeRTOSConfig.h"
    config.write_text(replace_once(config_source.read_text(), "#endif /* FREERTOS_CONFIG_H */",
                                  f'#include "{support / "trace.h"}"\n#endif /* FREERTOS_CONFIG_H */'))
    graph = [shlex.split(line) for line in (production / "commands.txt").read_text().splitlines()]
    compiles = [c for c in graph if "-c" in c]
    links = [c for c in graph if any(v.startswith("-Wl,-Map=") for v in c)]
    if not compiles or len(links) != 1:
        raise BenchmarkError("Unrecognized production build command graph")
    objects = {}
    compile_jobs = []
    template = None
    for command in compiles:
        command = command.copy()
        src = Path(command[command.index("-c")+1])
        old = command[command.index("-o")+1]
        target = out / Path(old).relative_to(production)
        target.parent.mkdir(parents=True, exist_ok=True)
        objects[old] = str(target)
        command[command.index("-o")+1] = str(target)
        if src.suffix == ".c":
            command[1:1] = ["-include", str(config)]
            template = command.copy()
        if src == source:
            command[command.index("-c")+1] = str(patched)
        elif src == kernel_source:
            command[command.index("-c")+1] = str(kernel)
        compile_jobs.append(command)
    if template is None:
        raise BenchmarkError("No C compiler command")
    template[template.index("-c")+1] = str(support / "trace.c")
    extra = out / "trace.o"
    template[template.index("-o")+1] = str(extra)
    compile_jobs.append(template)
    extras = [extra]
    if application:
        extra_sources = [support/"application_providers.c", support/"application_scenarios.c",
            fw/"CM4/Core/Src/application_ipc_service.c", fw/"CM4/Core/Src/application_validation_service.c",
            fw/"CM4/Application/Intercore/hmi_lifecycle.c",
            runner.repo/"ControlCore/Config/robot_config.c",
            runner.repo/"ServoDrive/AvatarM/Sim/avatar_m_node_sim.c",
            runner.repo/"Simulation/Tests/w25q_nor_model.c",
            runner.repo/"Simulation/Renode/Storage/w25q512jv_flash.c",
            runner.repo/"Simulation/Renode/Storage/qspi_nor_validated_storage.c"]
        for index, src in enumerate(extra_sources):
            command=template.copy()
            obj=out/f"application-{index}.o"
            command[command.index("-c")+1]=str(src)
            command[command.index("-o")+1]=str(obj)
            command += ["-I"+str(fw/p) for p in ("CM4/Core/Inc", "CM4/Application/Intercore", "CM7/Application/StateMachine")]
            if application_negative:
                command.append("-DMEMBENCH_INJECT_READ_FAILURE=1")
            compile_jobs.append(command)
            extras.append(obj)
    # Individual compiler invocations are retained in variant_commands.txt.
    (out / "variant_commands.txt").write_text("\n".join(shlex.join(c) for c in compile_jobs)+"\n")
    def compile_one(item):
        index, command = item
        log = out / "compile" / f"{index:04d}.log"
        code, _ = runner.run(command, log, timeout=120, check=False)
        return index, log, code
    failed = False
    with (out / "compile.log").open("w") as log:
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            for index, compile_log, code in pool.map(compile_one, enumerate(compile_jobs)):
                log.write(f"{index:04d}: exit={code}; {compile_log}\n")
                failed |= code != 0
    if failed:
        raise BenchmarkError(f"Diagnostic compilation failed: {out / 'compile.log'}")
    elf = out / f"{core}_diagnostic.elf"
    link = [objects.get(v, v) for v in links[0]]
    link[link.index("-o")+1] = str(elf)
    link = [f"-Wl,-Map={elf.with_suffix('.map')}" if v.startswith("-Wl,-Map=") else v for v in link]
    link += [str(p) for p in extras]+["-Wl,--wrap=xTaskCreate", "-Wl,--wrap=xTaskCreateStatic"]
    if application:
        link += ["-Wl,--wrap="+n for n in ("ft_uart2_dma_start", "ft_uart2_dma_service", "ft_uart2_dma_stop",
            "generate_line_waypoints", "trajectory_m7_take", "trajectory_m7_skip", "_sbrk")]
    runner.run(link, out / "link.log")
    sources=[dict(source=c[c.index("-c")+1], source_sha256=sha256(Path(c[c.index("-c")+1])),
                  object=c[c.index("-o")+1]) for c in compile_jobs]
    return dict(elf=str(elf), elf_sha256=sha256(elf), core=core, production=False,
                changes=changes, instrumentation="bounded task creation/switch/allocation trace; no TCB layout change",
                original_main_sha256=sha256(source), generated_main_sha256=sha256(patched),
                negative=negative, application=application, application_negative=application_negative,
                compiled_sources=sources)
