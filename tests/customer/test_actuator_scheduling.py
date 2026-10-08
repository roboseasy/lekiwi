"""Check that priority changes target only the known virtual actuator."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import json
import subprocess
import unittest
from unittest.mock import Mock, patch

from actuator_scheduling import prepare_virtual_actuator


class ActuatorSchedulingTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.docker = Mock(return_value=SimpleNamespace(stdout=(
            'PID COMMAND\n101 /usr/bin/python3 /customer-test/pi.py\n'
            '102 /customer-serial-wheels /tmp/state /tmp/port\n')))

    def test_only_actuator_gets_priority_before_readiness_is_published(self):
        with patch('actuator_scheduling.subprocess.run') as run, \
                patch('actuator_scheduling.os.getpriority', return_value=-20):
            prepare_virtual_actuator('virtual-pi', self.docker, self.path, 'run-1')
        self.docker.assert_called_once_with('top', 'virtual-pi', '-eo', 'pid,args',
                                           capture_output=True, text=True)
        run.assert_called_once_with(['sudo', '-n', 'renice', '-n', '-20', '-p', '102'], check=True)
        report = json.loads((self.path / 'actuator-scheduling.json').read_text())
        self.assertEqual((report['host_pid'], report['nice'], report['run']), (102, -20, 'run-1'))

    def test_ambiguous_processes_do_not_change_any_priority(self):
        self.docker.return_value.stdout += '103 /customer-serial-wheels /tmp/other\n'
        with patch('actuator_scheduling.subprocess.run') as run:
            with self.assertRaisesRegex(RuntimeError, 'multiple virtual actuators'):
                prepare_virtual_actuator('virtual-pi', self.docker, self.path, 'run-1')
        run.assert_not_called()

    def test_failed_priority_change_does_not_publish_readiness(self):
        with patch('actuator_scheduling.subprocess.run',
                   side_effect=subprocess.CalledProcessError(1, ['renice'])):
            with self.assertRaises(subprocess.CalledProcessError):
                prepare_virtual_actuator('virtual-pi', self.docker, self.path, 'run-1')
        self.assertFalse((self.path / 'actuator-scheduling.json').exists())

    def test_unverified_priority_does_not_publish_readiness(self):
        with patch('actuator_scheduling.subprocess.run'), \
                patch('actuator_scheduling.os.getpriority', return_value=0):
            with self.assertRaisesRegex(RuntimeError, 'priority was not applied'):
                prepare_virtual_actuator('virtual-pi', self.docker, self.path, 'run-1')
        self.assertFalse((self.path / 'actuator-scheduling.json').exists())

    def test_missing_actuator_times_out_without_changing_priority(self):
        self.docker.return_value.stdout = 'PID COMMAND\n101 /usr/bin/python3 /customer-test/pi.py\n'
        with patch('actuator_scheduling.time.monotonic', side_effect=[0, 11]), \
                patch('actuator_scheduling.subprocess.run') as run:
            with self.assertRaisesRegex(RuntimeError, 'did not appear'):
                prepare_virtual_actuator('virtual-pi', self.docker, self.path, 'run-1')
        run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
