"""Windows process contract tests; native MATLAB is still a WSL laptop check."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('windows_matlab', ROOT/'scripts/windows_matlab.py')
matlab = importlib.util.module_from_spec(spec); spec.loader.exec_module(matlab)


class MatlabContracts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="matlab path 'quoted' ")
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def command(self, body):
        return [sys.executable, '-c', 'from pathlib import Path;import time;d=Path('+repr(str(self.directory))+');'+body]

    def test_windows_and_matlab_quoting(self):
        folder = "C:\\robot's folder\\$x;project"
        argv = matlab.start_command("C:\\Program Files\\MATLAB\\matlab.exe", folder,
                                    'ready', 'error', 'stop', 'pid', 'log')
        self.assertEqual(argv[:4], ['powershell.exe','-NoProfile','-NonInteractive','-Command'])
        self.assertIn("robot''''s folder", argv[4])  # MATLAB quote, then PS quote
        self.assertIn('-wait', argv[4])
        self.assertIn('Start-Process', argv[4])
        self.assertNotIn('/IM', argv[4])

    def test_ready_then_healthy_exit(self):
        command = self.command("(d/'ready').write_text('ready');time.sleep(.25)")
        self.assertEqual(matlab.supervise(command,self.directory,2,grace=.1),0)

    def test_exit_zero_without_handshake_is_failure(self):
        with self.assertRaisesRegex(RuntimeError,'healthy shutdown'):
            matlab.supervise(self.command('pass'),self.directory,2,grace=.1)

    def test_matlab_error_marker(self):
        command=self.command("(d/'error').write_text('License checkout failed');time.sleep(.3)")
        with self.assertRaisesRegex(RuntimeError,'License checkout failed'):
            matlab.supervise(command,self.directory,2,grace=.1)

    def test_timeout_kills_only_owned_windows_pid(self):
        command=self.command("(d/'windows.pid').write_text('12345');time.sleep(20)")
        with mock.patch.object(matlab.subprocess,'run') as run:
            run.return_value.returncode=0
            with self.assertRaisesRegex(RuntimeError,'timed out'):
                matlab.supervise(command,self.directory,.3,grace=.1)
            self.assertEqual(run.call_args.args[0],['taskkill.exe','/PID','12345','/T','/F'])
            self.assertTrue((self.directory/'stop').exists())

    def test_sigterm_requests_graceful_stop(self):
        code="(d/'ready').touch();exec(\"while not (d/'stop').exists(): time.sleep(.02)\")"
        timer=threading.Timer(.4,lambda: os.kill(os.getpid(),signal.SIGTERM))
        timer.start()
        try:
            self.assertEqual(matlab.supervise(self.command(code),self.directory,2,grace=1),0)
        finally:
            timer.cancel();timer.join()
        self.assertTrue((self.directory/'stop').exists())

    def test_missing_explicit_executable_is_error(self):
        with mock.patch.dict(os.environ,{'MATLAB_EXE':str(self.directory/'missing.exe')}):
            with self.assertRaisesRegex(RuntimeError,'existing WSL-accessible'):
                matlab.discover_matlab()


if __name__ == '__main__':
    unittest.main()
