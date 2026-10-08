"""Keep README installation independent of introductory command examples."""

import contextlib
import io
import json
import os
from pathlib import Path
import re
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import readme_commands


CLONE = 'git clone -b feature/lekiwi-ros2 https://github.com/roboseasy-members/lekiwi.git'
INSTALL = 'sudo apt update\n' + CLONE + '\nvcs import . < lekiwi/lekiwi.repos'
BUILD = ('cd ~/lekiwi_ws\n'
         'colcon build --symlink-install --executor sequential --packages-select ydlidar_sdk\n'
         'ros2 pkg prefix lekiwi_bringup')


class ReadmeInstallationTest(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory(prefix='lekiwi-readme-test-')
        self.addCleanup(directory.cleanup)
        self.home = Path(directory.name)
        self.run = self.start_patch(patch.object(readme_commands.subprocess, 'run'))
        self.start_patch(patch.object(readme_commands.subprocess, 'check_output',
                                     side_effect=['fixture-commit\n', 'amd64\n']))
        self.start_patch(patch.object(readme_commands.Path, 'home', return_value=self.home))
        self.start_patch(patch.object(readme_commands, 'SOURCE', self.home / 'lekiwi_ws/src/lekiwi'))
        self.start_patch(patch.dict(os.environ, SOURCE_COMMIT='fixture-commit', USER='customer'))
        self.blocks = self.start_patch(patch.object(readme_commands, 'blocks'))

    def start_patch(self, patcher):
        value = patcher.start()
        self.addCleanup(patcher.stop)
        return value

    def install(self, blocks):
        self.blocks.return_value = blocks
        with contextlib.redirect_stdout(io.StringIO()):
            readme_commands.install()
        return [call.args[0][2] for call in self.run.call_args_list
                if call.args[0][:2] == ['bash', '-ec']]

    def test_current_readme_executes_install_and_build_only(self):
        readme = Path(__file__).resolve().parents[2] / 'README.md'
        executed = self.install(re.findall(r'```bash\n(.*?)\n```', readme.read_text(), re.S))
        self.assertEqual(len(executed), 2)
        self.assertIn('sudo apt install', executed[0])
        self.assertIn('git clone -b feature/lekiwi-ros2 /opt/lekiwi-source lekiwi', executed[0])
        self.assertIn('vcs import', executed[0])
        self.assertIn('--packages-select ydlidar_sdk', executed[1])
        self.assertIn('--packages-select lekiwi lekiwi_description', executed[1])
        self.assertNotIn('check_robot.sh', '\n'.join(executed))
        report = json.loads((self.home / 'installation.json').read_text())
        self.assertEqual(report['commands'], executed)

    def test_unrelated_examples_and_reordered_blocks_preserve_install_order(self):
        executed = self.install(['nano ./lekiwi-sensor-check/check_robot.sh', BUILD,
                                 'ros2 launch lekiwi_bringup robot.launch.py', INSTALL])
        self.assertEqual(executed, [INSTALL.replace(
            CLONE, 'git clone -b feature/lekiwi-ros2 /opt/lekiwi-source lekiwi'), BUILD])

    def test_missing_install_or_build_is_rejected_before_execution(self):
        for blocks in ([INSTALL], [BUILD]):
            with self.subTest(blocks=blocks), self.assertRaisesRegex(RuntimeError, 'README'):
                self.install(blocks)
            self.run.assert_not_called()

    def test_duplicate_install_or_build_blocks_are_rejected_before_execution(self):
        for blocks in ([INSTALL, INSTALL, BUILD], [INSTALL, BUILD, BUILD]):
            with self.subTest(blocks=blocks), self.assertRaisesRegex(RuntimeError, 'README'):
                self.install(blocks)
            self.run.assert_not_called()

    def test_repeated_clone_in_one_block_is_rejected_before_execution(self):
        with self.assertRaisesRegex(RuntimeError, 'README'):
            self.install([INSTALL + '\n' + CLONE, BUILD])
        self.run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
