import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('robot_verify',ROOT/'scripts/verify_robot_simulation.py')
verify=importlib.util.module_from_spec(spec);spec.loader.exec_module(verify)


class VerificationContracts(unittest.TestCase):
    def execute(self,failed=None,missing=False):
        calls=[]
        def record(results,output,label,argv,timeout=3600):
            calls.append((label,argv))
            results[label]={'status':'FAIL' if label==failed else 'PASS'}
            return label!=failed
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)
            with mock.patch.object(verify,'record',side_effect=record), mock.patch.object(
                verify,'registered_tests',return_value=set() if missing else verify.REQUIRED_TESTS):
                status=verify.execute(output,output/'build',output/'sdk')
            return status,calls,(output/'summary.json').read_text()

    def test_all_gates_and_unfiltered_ctest(self):
        status,calls,summary=self.execute()
        self.assertEqual(status,0)
        commands=dict(calls)
        self.assertIn('all_ctest',commands)
        self.assertNotIn('-E',commands['all_ctest'])
        self.assertNotIn('-R',commands['all_ctest'])
        self.assertIn('p05_acceptance',commands)
        self.assertIn('"zero_force_smoke_only": false',summary)

    def test_regression_failure_is_nonzero_but_p05_still_runs(self):
        status,calls,summary=self.execute('all_ctest')
        self.assertEqual(status,1)
        self.assertIn('p05_acceptance',dict(calls))
        self.assertIn('"status": "FAIL"',summary)

    def test_missing_required_tests_cannot_pass(self):
        status,calls,summary=self.execute(missing=True)
        self.assertEqual(status,1)
        self.assertNotIn('all_ctest',dict(calls))
        self.assertIn('NOT RUN',summary)

    def test_p05_failure_cannot_pass(self):
        self.assertEqual(self.execute('p05_acceptance')[0],1)

    def test_verify_rejects_smoke_flags(self):
        r=subprocess.run(['bash',str(ROOT/'scripts/run_robot_simulation.sh'),'--verify','--headless'],
                         text=True,capture_output=True)
        self.assertEqual(r.returncode,2,r.stdout+r.stderr)


if __name__=='__main__': unittest.main()
