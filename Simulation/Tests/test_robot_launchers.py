"""Execute both launchers with subprocess stubs; inspect exact child environments."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class Launchers(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='robot-launcher-contract-')
        self.root = Path(self.temp.name)
        (self.root/'scripts').mkdir()
        for name in ('run_robot_simulation.sh', 'run_full_can_silkit_simulation.sh'):
            shutil.copy2(ROOT/'scripts'/name, self.root/'scripts'/name)
        (self.root/'Simulation/Robot').mkdir(parents=True)
        shutil.copy2(ROOT/'Simulation/Robot/zero_ft_sender.py', self.root/'Simulation/Robot/zero_ft_sender.py')
        self.build = self.root/'build'; self.build.mkdir()
        self.sdk = self.root/'sdk/bin'; self.sdk.mkdir(parents=True)
        stub = '''#!/usr/bin/env python3
import json,os,time
from pathlib import Path
name=Path(__file__).name
if name in ('robot_simulation','supervisor_silkit_sim'):
 Path(os.environ['CAPTURE']).write_text(json.dumps(dict(os.environ)))
 print('[SILKIT] Controller connected:',flush=True)
elif name=='sil-kit-registry': print('SIL Kit Registry listening',flush=True)
elif name=='avatar_m_silkit_motor_bank': print('AVATAR SIL Kit motor bank ready.',flush=True)
elif name=='can_live_monitor': print('[CAN MONITOR] ready:',flush=True)
while True: time.sleep(.1)
'''
        for name in ('robot_simulation', 'supervisor_silkit_sim', 'avatar_m_silkit_motor_bank',
                     'can_live_monitor', 'supervisor_panel', 'virtual_ft_gui', 'teaching_sim'):
            p = self.build/name; p.write_text(stub); p.chmod(0o755)
        p = self.sdk/'sil-kit-registry'; p.write_text(stub); p.chmod(0o755)
        self.capture = self.root/'environment.json'
        self.env = {**os.environ, 'BUILD_DIR': str(self.build), 'SILKIT_ROOT': str(self.sdk.parent),
                    'SIMULATION_LOG_DIR': str(self.root/'logs'), 'MATLAB_IP': '127.0.0.1',
                    'CAPTURE': str(self.capture)}
        self.env.pop('ROBOT_UNIFIED_MODE', None)

    def tearDown(self):
        self.temp.cleanup()

    def launch(self, unified, *args):
        name = 'run_robot_simulation.sh' if unified else 'run_full_can_silkit_simulation.sh'
        return subprocess.run(['bash', str(self.root/'scripts'/name), '--no-build', '--duration', '1', *args],
                              env=self.env, text=True, capture_output=True, timeout=25)

    def test_legacy_normal_environment_is_exact(self):
        result = self.launch(False, '--no-gui')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        env = json.loads(self.capture.read_text())
        for key in ('HAND_GUIDING_DRY_RUN', 'HAND_GUIDING_SIM_MOTION', 'HAND_GUIDING_VIRTUAL_FT'):
            self.assertEqual(env[key], '0')
        self.assertNotIn('HEADLESS_FT', env)
        self.assertNotIn('SUPERVISOR_TARGET', env)

    def test_legacy_virtual_ft_environment(self):
        result = self.launch(False, '--virtual-ft')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        env = json.loads(self.capture.read_text())
        self.assertEqual(env['HAND_GUIDING_VIRTUAL_FT'], '1')
        self.assertEqual(env['HAND_GUIDING_SIM_MOTION'], '1')

    def test_unified_headless_is_explicit_smoke_and_skips_matlab(self):
        result = self.launch(True, '--headless')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('[SMOKE ONLY]', result.stdout)
        self.assertNotIn('VERIFICATION PASS', result.stdout)
        self.assertEqual(json.loads(self.capture.read_text())['HAND_GUIDING_VIRTUAL_FT'], '1')

    def test_unified_rejects_automatic_matlab_startup(self):
        # Unified launcher must never start a Windows MATLAB process.
        result = self.launch(True, '--matlab')
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn('Automatic MATLAB startup is disabled', result.stderr)
        self.assertFalse(self.capture.exists())

    def test_default_does_not_start_a_second_matlab_instance(self):
        # No helper exists in this fixture: invoking it would fail startup.
        result = self.launch(True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('Independent/manual MATLAB startup', result.stdout)

    def test_explicit_no_matlab(self):
        result = self.launch(True, '--no-matlab')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
