"""Shutdown regression checks; signals and child processes are all mocked."""

import importlib.util
from pathlib import Path
import signal
import subprocess
import unittest
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location(
    'process_cleanup', Path(__file__).resolve().parents[1] / 'integration/process_cleanup.py')
cleanup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cleanup)


class TestProcessCleanup(unittest.TestCase):
    def process(self, args=None):
        return Mock(pid=12345, args=args or ['ros2', 'run', 'mock'],
                    poll=Mock(return_value=None))

    def test_second_timeout_escalates_to_kill(self):
        process = self.process()
        process.wait.side_effect = [subprocess.TimeoutExpired('mock', 12),
                                    subprocess.TimeoutExpired('mock', 5), 0]
        with patch.object(cleanup.os, 'killpg') as kill:
            cleanup.stop_owned_process(process)
        self.assertEqual([call.args[1] for call in kill.call_args_list],
                         [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])
        self.assertEqual([call.kwargs['timeout'] for call in process.wait.call_args_list],
                         [12, 5, 5])

    def test_launch_gets_one_parent_interrupt_before_group_escalation(self):
        process = self.process(['ros2', 'launch', 'mock', 'test.launch.py'])
        process.wait.side_effect = [subprocess.TimeoutExpired('mock', 12), 0]
        with patch.object(cleanup.os, 'killpg') as kill:
            cleanup.stop_owned_process(process)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        kill.assert_called_once_with(process.pid, signal.SIGTERM)

    def test_exit_during_escalation_is_reaped(self):
        process = self.process()
        process.wait.side_effect = [subprocess.TimeoutExpired('mock', 12), 0]
        with patch.object(cleanup.os, 'killpg', side_effect=[None, ProcessLookupError()]):
            cleanup.stop_owned_process(process)
        self.assertEqual(process.wait.call_count, 2)

    def test_one_shutdown_failure_does_not_skip_other_children_or_logs(self):
        for failure in (subprocess.TimeoutExpired('mock', 5), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__):
                first, stuck = self.process(), self.process()
                first_log, stuck_log = Mock(), Mock()
                stuck.wait.side_effect = failure
                with patch.object(cleanup.os, 'killpg'):
                    errors = cleanup.cleanup_processes([(first, first_log), (stuck, stuck_log)])
                self.assertEqual(len(errors), 1)
                first.wait.assert_called_once_with(timeout=12)
                first_log.close.assert_called_once()
                stuck_log.close.assert_called_once()

    def test_log_close_failure_does_not_skip_other_children(self):
        first, second = self.process(), self.process()
        log = Mock()
        log.close.side_effect = OSError('log close failed')
        with patch.object(cleanup.os, 'killpg'):
            errors = cleanup.cleanup_processes([(first, None), (second, log)])
        self.assertEqual(len(errors), 1)
        first.wait.assert_called_once()

    def test_already_reaped_child_is_not_signaled(self):
        process = self.process()
        process.poll.return_value = 0
        with patch.object(cleanup.os, 'killpg') as kill:
            cleanup.stop_owned_process(process)
        kill.assert_not_called()
        process.wait.assert_not_called()


if __name__ == '__main__':
    unittest.main()
