"""Process execution and identities. Every subprocess has a retained log."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading


class BenchmarkError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Runner:
    def __init__(self, repo: Path, out: Path):
        self.repo, self.out = repo, out
        self.commands: list[dict] = []
        self.lock = threading.Lock()

    def run(self, argv, log: Path, timeout=180, env=None, check=True):
        argv = list(map(str, argv))
        log.parent.mkdir(parents=True, exist_ok=True)
        record = dict(argv=argv, cwd=str(self.repo), log=str(log), timeout_seconds=timeout)
        with self.lock:
            self.commands.append(record)
        output, code = "", None
        with log.open("w") as stream:
            stream.write("$ " + shlex.join(argv) + "\n")
            stream.flush()
            try:
                proc = subprocess.Popen(argv, cwd=self.repo, env=env,
                                        stdout=stream, stderr=subprocess.STDOUT,
                                        start_new_session=True)
                try:
                    code = proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                    record["error"] = "timeout"
                    stream.write("\nBENCHMARK TIMEOUT\n")
                except KeyboardInterrupt:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                    raise
            except OSError as exc:
                record["error"] = str(exc)
                stream.write(f"\nBENCHMARK EXEC ERROR: {exc}\n")
        record["exit_code"] = code
        self.save_commands()
        output = log.read_text(errors="replace")
        if check and (code != 0 or "error" in record):
            raise BenchmarkError(f"Command failed ({record.get('error', code)}): {log}")
        return code, output

    def save_commands(self):
        with self.lock:
            (self.out / "commands.json").write_text(json.dumps(self.commands, indent=2) + "\n")


def input_manifest(repo, files):
    manifest = {}
    for name in sorted(set(files)):
        path = repo / name
        if path.is_file() and (path.name in {".project", ".cproject", "CMakeLists.txt"} or path.suffix.lower() in {
            ".c", ".h", ".s", ".ld", ".py", ".sh", ".cmake", ".repl", ".resc",
            ".txt", ".md", ".inc",
        }):
            manifest[name] = sha256(path)
    return manifest


def identity(runner: Runner) -> dict:
    def git(*args):
        return runner.run(["git", *args], runner.out / ("git-" + args[0] + ".log"))[1].split("\n", 1)[1].strip()
    files = git("ls-files", "--cached", "--others", "--exclude-standard").splitlines()
    manifest = input_manifest(runner.repo, files)
    (runner.out / "inputs.json").write_text(json.dumps(manifest, indent=2) + "\n")
    compiler = runner.run(["arm-none-eabi-gcc", "--version"], runner.out / "compiler.log")[1]
    return dict(branch=git("branch", "--show-current"), commit=git("rev-parse", "HEAD"),
                status=git("status", "--porcelain=v1"),
                compiler=compiler.splitlines()[1], input_manifest_sha256=sha256(runner.out / "inputs.json"))
