#!/usr/bin/env python3
"""Build one STM32 core with ARM GCC from the checked-in CubeIDE source graph.

No generated Debug makefiles, Windows SDK paths, staging tree, or host HAL
dependency. Each core/configuration has its own output directory.
"""
import argparse
import concurrent.futures
import os
from pathlib import Path
import re
import shlex
import subprocess
import xml.etree.ElementTree as ET

FW = Path(__file__).resolve().parents[1] / "STM32H745"
REPO = FW.parents[1]


def project_sources(core, config):
    project = FW / core
    graph = ET.parse(project / ".project")
    links = {}
    for link in graph.findall(".//link"):
        name, uri = link.findtext("name"), link.findtext("locationURI")
        match = re.fullmatch(r"PARENT-(\d+)-PROJECT_LOC/(.+)", uri or "")
        if not match:
            raise ValueError(f"Nonportable linked resource: {name}: {uri}")
        target = project.parents[int(match[1]) - 1] / match[2]
        if not target.exists():
            raise ValueError(f"Missing linked resource: {name}: {target}")
        links[name] = target.resolve()
    configurations = ET.parse(project / ".cproject")
    cfg = next(c for c in configurations.findall(".//cconfiguration")
               if c.find(".//configuration").get("name") == config)
    includes, defines = [], []
    for tool in cfg.findall(".//tool"):
        if not tool.get("superClass", "").endswith("tool.c.compiler"):
            continue
        for option in tool.findall("option"):
            values = [v.get("value") for v in option.findall("listOptionValue")]
            if option.get("valueType") == "includePath":
                includes += [(project / config / v).resolve() for v in values]
            elif option.get("valueType") == "definedSymbols":
                defines += values
    sources = {}
    for entry in cfg.findall(".//sourceEntries/entry"):
        if entry.get("excluding"):
            raise ValueError("Source exclusions need explicit build support")
        name = entry.get("name")
        base = project / name
        if base.is_dir():
            for p in base.rglob("*"):
                if p.suffix in (".c", ".s", ".S"):
                    sources[str(p.relative_to(project))] = p.resolve()
        for virtual, target in links.items():
            if virtual == name or virtual.startswith(name + "/"):
                if target.is_dir():
                    for p in target.rglob("*"):
                        if p.suffix in (".c", ".s", ".S"):
                            sources[virtual + "/" + str(p.relative_to(target))] = p
                elif target.suffix in (".c", ".s", ".S"):
                    sources[virtual] = target
    if len(set(sources.values())) != len(sources):
        raise ValueError("A source is selected twice in the core source graph")
    return sources, includes, defines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", choices=["CM7", "CM4"], required=True)
    parser.add_argument("--config", choices=["Debug", "Release"], default="Debug")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1))
    args = parser.parse_args()
    sources, includes, defines = project_sources(args.core, args.config)
    output = (args.output or REPO / "build/stm32" / args.config / args.core).resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = os.environ.get("ARM_NONE_EABI_GCC", "arm-none-eabi-gcc")
    m7 = args.core == "CM7"
    arch = ["-mcpu=cortex-m7" if m7 else "-mcpu=cortex-m4", "-mthumb",
            "-mfpu=fpv5-d16" if m7 else "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    flags = arch + ["-std=gnu11", "-O0" if args.config == "Debug" else "-Os",
                    "-g3", "-ffunction-sections", "-fdata-sections",
                    "-Wall", "-fstack-usage", "--specs=nano.specs"]
    flags += ["-D" + d for d in defines]
    flags += ["-I" + str(p) for p in includes]
    commands = []
    objects = []

    def compile_one(item):
        virtual, source = item
        obj = output / (virtual + ".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        command = [compiler] + flags
        if source.suffix in (".s", ".S"):
            command += ["-x", "assembler-with-cpp"]
        command += ["-c", str(source), "-o", str(obj)]
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        return obj, command, result

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for obj, command, result in pool.map(compile_one, sorted(sources.items())):
            commands.append(shlex.join(command))
            if result.stdout:
                print(result.stdout, end="", flush=True)
            if result.returncode:
                raise SystemExit(result.returncode)
            objects.append(str(obj))
    elf = output / ("STM32H745_" + args.core + ".elf")
    linker = FW / args.core / "STM32H745ZITX_FLASH.ld"
    command = [compiler] + arch + ["-T" + str(linker), "--specs=nosys.specs",
        "--specs=nano.specs", "-static", "-Wl,--gc-sections",
        "-Wl,-Map=" + str(elf.with_suffix(".map")), "-o", str(elf)] + objects + [
        "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"]
    commands.append(shlex.join(command))
    (output / "commands.txt").write_text("\n".join(commands) + "\n")
    subprocess.run(command, check=True)
    subprocess.run([compiler.replace("gcc", "size"), str(elf)], check=True)
    print(f"PASS: {args.core} {args.config}: {len(objects)} translation units -> {elf}")


if __name__ == "__main__":
    main()
