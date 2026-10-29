#!/usr/bin/env python3
"""Start and supervise ONE owned Windows MATLAB visualizer from WSL.

Readiness means UDP 5005 and the figure initialized, not merely process creation.
No existing MATLAB session is reused, terminated, or modified.
"""
import argparse
import glob
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time


def ps_string(value):
    return "'" + str(value).replace("'", "''") + "'"


def matlab_string(value):
    return "'" + str(value).replace("'", "''") + "'"


def windows_path(path):
    return subprocess.check_output(['wslpath', '-w', str(Path(path).resolve())], text=True).strip()


def discover_matlab():
    selected = os.environ.get('MATLAB_EXE')
    if selected:
        path = Path(selected).expanduser()
        if not path.is_file():
            raise RuntimeError('MATLAB_EXE must name an existing WSL-accessible matlab.exe file')
        return path
    found = shutil.which('matlab.exe')
    if found:
        return Path(found)
    candidates = sorted(glob.glob('/mnt/*/Program Files/MATLAB/R*/bin/matlab.exe'))
    if not candidates:
        raise RuntimeError('Windows MATLAB not found. Set MATLAB_EXE to its WSL path, '
                           'for example /mnt/c/Program Files/MATLAB/R2025b/bin/matlab.exe')
    return Path(candidates[-1])


def start_command(executable, folder, ready, error, stop, pid_file, logfile):
    expression = ("try;addpath(" + matlab_string(folder) + ");robot_windows_start(" +
                  ','.join(matlab_string(p) for p in (ready, error, stop)) +
                  ");exit(0);catch err;disp(getReport(err,'extended'));exit(1);end;")
    # Start-Process accepts a single Windows command-line string. list2cmdline
    # handles Windows argv quoting; the outer string is separately PS-quoted.
    args = subprocess.list2cmdline(['-wait', '-logfile', logfile, '-r', expression])
    script = ("$ErrorActionPreference='Stop';try {"
              "$p=Start-Process -FilePath " + ps_string(executable) +
              ' -ArgumentList ' + ps_string(args) + ' -PassThru;'
              '[IO.File]::WriteAllText(' + ps_string(pid_file) + ',$p.Id.ToString());'
              '$p.WaitForExit();$p.Refresh();if($null -eq $p.ExitCode){exit 1};exit $p.ExitCode'
              '} catch {[IO.File]::WriteAllText(' + ps_string(error) +
              ',$_.Exception.ToString());exit 1}')
    return ['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script]


def stop_owned(process, directory, grace=5, force=False):
    (directory / 'stop').touch()
    try:
        process.wait(timeout=grace)
        if not force:
            return
    except subprocess.TimeoutExpired:
        pass
    pid_file = directory / 'windows.pid'
    if pid_file.is_file():
        pid = pid_file.read_text(encoding='utf-8-sig').strip()
        if pid.isdecimal() and int(pid) > 0:
            # Exact PID obtained from our own Start-Process, never /IM matlab.exe.
            try:
                killed = subprocess.run(['taskkill.exe', '/PID', pid, '/T', '/F'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=10, check=False)
                if killed.returncode:
                    print(f'[MATLAB] cleanup: taskkill returned {killed.returncode} for owned PID {pid}', file=sys.stderr)
            except (OSError, subprocess.SubprocessError) as error:
                print(f'[MATLAB] owned Windows PID {pid} cleanup failed: {error}', file=sys.stderr)
    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def supervise(command, directory, timeout, grace=5):
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    process = None
    try:
        with (directory / 'powershell.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + timeout
            ready = False
            while not stopping:
                error = directory / 'error'
                if error.is_file():
                    raise RuntimeError('Windows MATLAB failed: ' + error.read_text(errors='replace'))
                status = process.poll()
                if status is not None:
                    if status != 0 or not ready:
                        raise RuntimeError(f'Windows MATLAB/PowerShell exited {status} before a healthy shutdown; '
                                           f'see {directory}')
                    return 0
                if not ready and (directory / 'ready').is_file():
                    ready = True
                    print(f'[MATLAB] ready; logs: {directory}', flush=True)
                if not ready and time.monotonic() >= deadline:
                    raise RuntimeError(f'Windows MATLAB startup timed out after {timeout}s. '
                                       f'Check licensing, UDP 5005, toolbox/graphics errors and logs in {directory}')
                time.sleep(0.1)
        return 0
    finally:
        if process is not None:
            status = process.poll()
            if status is None or status != 0:
                stop_owned(process, directory, grace, force=status is not None)
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--output-parent', required=True, type=Path)
    parser.add_argument('--timeout', type=int, default=120)
    ns = parser.parse_args()
    if ns.timeout < 1:
        parser.error('--timeout must be positive')
    if not all(shutil.which(tool) for tool in ('wslpath', 'powershell.exe', 'taskkill.exe')):
        raise RuntimeError('Windows MATLAB autostart requires WSL Windows interop '
                           '(wslpath, powershell.exe, taskkill.exe). Use --no-matlab only to skip it explicitly.')
    executable = discover_matlab()
    ns.output_parent.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix='windows-matlab-', dir=ns.output_parent.resolve()))
    command = start_command(windows_path(executable), windows_path(ns.repo / 'MATLAB'),
                            *(windows_path(directory / name) for name in
                              ('ready', 'error', 'stop', 'windows.pid', 'matlab.log')))
    return supervise(command, directory, ns.timeout)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print('[MATLAB] ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)
