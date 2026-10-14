#!/usr/bin/env python3
"""C3: snapshot a source-locked, NOT-FOR-MOTION dual-core software candidate."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
REQUIRED_DOCS=[
 "Firmware/STAGE3_BATCH_B_SOFTWARE_COMPLETION.md",
 "Firmware/STAGE3_C1_ACCEPTANCE.md",
 "Firmware/STAGE3_C2_ACCEPTANCE.md",
 "Firmware/STAGE3_C3_RELEASE_CANDIDATE.md",
 "Firmware/STAGE3_C3_HARDWARE_HANDOFF.md",
 "Firmware/HARDWARE_ARRIVAL_VERIFICATION_PLAN.md",
]

def sha256(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        while True:
            block=f.read(1024*1024)
            if not block: break
            h.update(block)
    return h.hexdigest()

def current_sha():
    return subprocess.check_output(
        ["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip()

def stamp_check(prep,project):
    context=(prep/"context.txt").read_text().splitlines()
    if len(context)!=2 or context[0]!=current_sha() or \
       Path(context[1]).resolve()!=project:
        raise ValueError("C3 preparation SHA/project differs from current source")
    marker=prep/"rebuild_after_this.marker"
    if not marker.is_file(): raise ValueError("missing C3 build-start marker")
    for core in ("CM7","CM4"):
        elf=project/core/"Debug"/("GP_STM32H745_"+core+".elf")
        if not elf.is_file() or not elf.stat().st_size or \
           elf.stat().st_mtime_ns<=marker.stat().st_mtime_ns:
            raise ValueError(core+" ELF is absent or predates release checkpoint")
    return context[0]

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--project",required=True,type=Path)
    p.add_argument("--prep",required=True,type=Path)
    p.add_argument("--out",required=True,type=Path)
    a=p.parse_args()
    project=a.project.resolve()
    prep=a.prep.resolve()
    out=a.out.resolve()
    if prep.parent != ROOT/".simulation-logs/stage-c3":
        raise ValueError("C3 prep directory must be inside Stage C3 evidence root")
    if not out.is_relative_to(ROOT/".simulation-logs/stage-c3"):
        raise ValueError("C3 release output must remain under Stage C3 evidence root")
    revision=stamp_check(prep,project)

    target=out/"candidate_images"
    docs=out/"documentation"
    target.mkdir(parents=True,exist_ok=True)
    docs.mkdir(parents=True,exist_ok=True)
    files=[]
    missing_optional=[]
    for core in ("CM7","CM4"):
        elf=project/core/"Debug"/("GP_STM32H745_"+core+".elf")
        shutil.copy2(elf,target/elf.name)
        files.append(target/elf.name)
        # Not all CubeIDE configs generate a .map for both cores.
        mp=elf.with_suffix(".map")
        if mp.is_file() and mp.stat().st_size:
            shutil.copy2(mp,target/mp.name)
            files.append(target/mp.name)
        else:
            missing_optional.append(str(mp))
        linker=project/core/"STM32H745IITX_FLASH.ld"
        if not linker.is_file(): raise ValueError("missing "+str(linker))
        dest=target/(core+"_FLASH.ld")
        shutil.copy2(linker,dest)
        files.append(dest)
        cproject=project/core/".cproject"
        if not cproject.is_file(): raise ValueError("missing "+str(cproject))
        dest=target/(core+".cproject")
        shutil.copy2(cproject,dest)
        files.append(dest)
    ioc=project/"GP_STM32H745.ioc"
    if not ioc.is_file(): raise ValueError("missing project .ioc")
    shutil.copy2(ioc,target/ioc.name)
    files.append(target/ioc.name)

    for src in REQUIRED_DOCS:
        source=ROOT/src
        if not source.is_file(): raise ValueError("missing mandatory report "+src)
        dest=docs/source.name
        shutil.copy2(source,dest)
        files.append(dest)

    warning=out/"NOT_FOR_ROBOT_MOTION.md"
    warning.write_text(
        "# C3 software-only candidate — NOT COMMISSIONED\n\n"
        "These ELF binaries were compiled for the independent CubeIDE verifier.\n"
        "They deliberately lack commissioned physical motor/safety and NOR\n"
        "providers. They have NOT run on actual STM32 hardware. Do NOT\n"
        "enable drive power, assume safety-rated behavior, or treat as a\n"
        "production robot firmware release. Hardware H0-H10 sign-off is\n"
        "separate; see STAGE3_C3_HARDWARE_HANDOFF.md.\n"
    )
    files.append(warning)

    audit={
        "category":"PRE-HARDWARE SOFTWARE-CANDIDATE — NOT COMMISSIONED",
        "source_revision":revision,
        "branch":"sara/can-replacement",
        "verified_build_checkpoint_utc":(prep/"start_utc.txt").read_text().strip(),
        "manifest_created_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "project_directory":str(project),
        "limitations":"ELF mtime after marker verifies a new output, NOT independent proof of IDE Clean command",
        "missing_optional_map_files":missing_optional,
        "files":{str(f.relative_to(out)):{"sha256":sha256(f),"bytes":f.stat().st_size}
                 for f in sorted(files)},
    }
    manifest=out/"release_manifest.json"
    manifest.write_text(json.dumps(audit,indent=2,sort_keys=True)+"\n")
    zip_path=out/"stage3_c3_software_candidate.zip"
    with zipfile.ZipFile(zip_path,"w",zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for file in sorted(files+[manifest]):
            z.write(file,arcname=str(file.relative_to(out)))
    print("PASS: C3 source-locked dual-core ELF candidate + manifest SHA256")
    print("CM7 ELF SHA256:",sha256(target/"GP_STM32H745_CM7.elf"))
    print("CM4 ELF SHA256:",sha256(target/"GP_STM32H745_CM4.elf"))
    if missing_optional:
        print("NOTE: map missing (optional), listed in manifest:",", ".join(missing_optional))
    print("ARCHIVE:",zip_path)
    print("WARNING: not authorized for physical robot motion")

if __name__=="__main__":
    try: main()
    except (ValueError,OSError,subprocess.CalledProcessError) as e:
        print("FAIL: C3 firmware evidence:",e,file=sys.stderr)
        sys.exit(1)
