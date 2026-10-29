"""The installer must never modify dirty source work or overwrite a destination."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('delivery_install',ROOT/'scripts/install_robot_delivery.py')
installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)


class InstallerContracts(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='robot-safe-install-')
        self.root=Path(self.temp.name)
        self.source=self.root/'existing'
        subprocess.run(['git','clone','--quiet','--shared','--branch',installer.BRANCH,
                        str(ROOT),str(self.source)],check=True)
        readme=self.source/'README.md'
        readme.write_text(readme.read_text()+'\nLocal staged work\n')
        subprocess.run(['git','add','README.md'],cwd=self.source,check=True)
        readme.write_text(readme.read_text()+'Local unstaged work\n')
        (self.source/'local-notes.txt').write_text('untracked work\n')
        self.index=(self.source/'.git/index').read_bytes()
        self.contents=readme.read_bytes()
        self.status=subprocess.check_output(['git','status','--porcelain'],cwd=self.source)
        self.patch=self.root/'patch'
        self.patch.write_text('diff --git a/robot-safe-addition.txt b/robot-safe-addition.txt\n'
            'new file mode 100644\n--- /dev/null\n+++ b/robot-safe-addition.txt\n@@ -0,0 +1 @@\n+new implementation\n')

    def tearDown(self):
        self.assertEqual((self.source/'.git/index').read_bytes(),self.index)
        self.assertEqual((self.source/'README.md').read_bytes(),self.contents)
        self.assertEqual((self.source/'local-notes.txt').read_text(),'untracked work\n')
        self.assertEqual(subprocess.check_output(['git','status','--porcelain'],cwd=self.source),self.status)
        self.temp.cleanup()

    def test_dirty_source_is_untouched_and_patch_applied_only_in_new_clone(self):
        target=self.root/'corrected'
        installer.install(self.source,target,self.patch)
        self.assertEqual((target/'robot-safe-addition.txt').read_text(),'new implementation\n')
        self.assertFalse((target/'local-notes.txt').exists())
        self.assertNotIn('Local staged work',(target/'README.md').read_text())

    def test_existing_destination_refused(self):
        target=self.root/'existing-destination';target.mkdir()
        (target/'keep').write_text('keep')
        with self.assertRaisesRegex(ValueError,'already exists'):
            installer.install(self.source,target,self.patch)
        self.assertEqual((target/'keep').read_text(),'keep')

    def test_destination_inside_source_refused(self):
        with self.assertRaisesRegex(ValueError,'outside'):
            installer.install(self.source,self.source/'nested',self.patch)

    def test_conflicting_patch_changes_only_new_clone(self):
        self.patch.write_text('not a patch\n')
        with self.assertRaisesRegex(RuntimeError,'Original checkout is untouched'):
            installer.install(self.source,self.root/'conflict',self.patch)


if __name__=='__main__': unittest.main()
