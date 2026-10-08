"""Exercise actual bounded subprocesses; no ROS graph or robot is involved."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest

spec = importlib.util.spec_from_file_location(
    'service_jobs', Path(__file__).resolve().parents[2] / 'tools/diagnostics/test_service_server.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TestJobs(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.worker = self.folder / 'worker.py'
        self.worker.write_text('''import argparse, json, time
p = argparse.ArgumentParser()
p.add_argument('--checks', nargs='+'); p.add_argument('--duration'); p.add_argument('--discovery-timeout')
p.add_argument('--report'); p.add_argument('--camera-indices')
a = p.parse_args()
if a.checks == ['odom']: time.sleep(60)
checks = {name: {'passed': name != 'lidar'} for name in a.checks}
ok = all(check['passed'] for check in checks.values())
with open(a.report, 'w') as f: json.dump({'passed': ok, 'checks': checks, 'camera': a.camera_indices}, f)
raise SystemExit(0 if ok else 1)
''')
        self.manager = module.TestJobs(self.worker, self.folder / 'reports', duration=1.)

    def tearDown(self):
        self.manager.close()
        self.temp.cleanup()

    def finish(self, feature):
        end = time.monotonic() + 5.
        while time.monotonic() < end:
            result = self.manager.status(feature)
            if result['state'] not in module.ACTIVE:
                return result
            time.sleep(.02)
        self.fail('worker did not finish')

    def test_start_status_and_persisted_pass(self):
        self.assertEqual(self.manager.status('cmd_vel')['state'], 'IDLE')
        accepted, job = self.manager.start('cmd_vel')
        self.assertTrue(accepted)
        result = self.finish('cmd_vel')
        self.assertEqual(result['state'], 'PASSED')
        saved = json.loads((Path(job['directory']) / 'status.json').read_text())
        self.assertEqual(saved['state'], 'PASSED')

    def test_worker_failure_is_not_successful_service_completion(self):
        self.manager.start('lidar')
        result = self.finish('lidar')
        self.assertEqual(result['state'], 'FAILED')
        self.assertFalse(result['result']['passed'])

    def test_conflicting_start_is_rejected_without_replacing_active_test(self):
        self.manager.start('odom')
        accepted, result = self.manager.start('teleop')
        self.assertFalse(accepted)
        self.assertEqual(result['error'], 'BUSY')
        self.assertEqual(self.manager.active['feature'], 'odom')

    def test_cancel_stops_own_worker_and_records_cancelled(self):
        self.manager.start('odom')
        process = self.manager.process
        self.assertTrue(self.manager.cancel('odom')[0])
        self.assertEqual(self.finish('odom')['state'], 'CANCELLED')
        self.assertIsNotNone(process.poll())
        report = json.loads(Path(self.manager.status('odom')['report_path']).read_text())
        self.assertFalse(report['passed'])

    def test_wrong_feature_cancel_does_not_stop_another_job(self):
        self.manager.start('odom')
        self.assertFalse(self.manager.cancel('camera')[0])
        self.assertIsNone(self.manager.process.poll())

    def test_deadline_failure_is_distinct_from_user_cancellation(self):
        self.manager.start('odom')
        self.manager.deadline = time.monotonic() - 1
        result = self.finish('odom')
        self.assertEqual(result['state'], 'FAILED')
        self.assertIn('deadline', result['error'])

    def test_camera_selection_and_unique_report_paths(self):
        _, first = self.manager.start('camera0')
        self.assertEqual(self.finish('camera0')['result']['camera'], '0')
        _, second = self.manager.start('camera0')
        self.assertNotEqual(first['report_path'], second['report_path'])
        self.finish('camera0')
        self.assertTrue(Path(first['report_path']).exists())

    def test_invalid_feature_and_duplicate_manager_are_rejected(self):
        self.assertFalse(self.manager.start('arbitrary_shell')[0])
        with self.assertRaises(RuntimeError):
            module.TestJobs(self.worker, self.folder / 'reports')

    def test_crashed_worker_without_report_is_failed(self):
        self.worker.write_text('raise SystemExit(2)\n')
        self.manager.start('cmd_vel')
        result = self.finish('cmd_vel')
        self.assertEqual(result['state'], 'FAILED')
        self.assertIn('report_error', result)
        self.assertIn('결과 파일 없이 종료됨', module.response_message('cmd_vel', 'status', result))

    def test_all_runs_every_check_and_preserves_partial_failure(self):
        accepted, job = self.manager.start('all')
        self.assertTrue(accepted)
        command = job['command']
        self.assertEqual(tuple(command[command.index('--checks') + 1:command.index('--duration')]), module.ALL_CHECKS)
        result = self.finish('all')
        self.assertEqual(result['state'], 'FAILED')
        self.assertEqual(set(result['result']['checks']), set(module.ALL_CHECKS))
        self.assertIn('4/5 통과', module.suite_summary(result))
        messages = module.item_messages(result)
        self.assertTrue(any('[실패] LiDAR:' in message for message in messages))
        self.assertTrue(any('[통과] 카메라' in message for message in messages))

    def test_all_requires_every_item_to_pass_not_just_top_level_flag(self):
        self.worker.write_text(self.worker.read_text().replace("name != 'lidar'", 'True'))
        self.manager.start('all')
        self.assertEqual(self.finish('all')['state'], 'PASSED')
        self.worker.write_text(self.worker.read_text().replace("'checks': checks", "'checks': {}"))
        self.manager.start('all')
        result = self.finish('all')
        self.assertEqual(result['state'], 'FAILED')
        self.assertEqual(len(module.item_messages(result)), 5)
        self.assertTrue(all(message.startswith('[미완료]') for message in module.item_messages(result)))

    def test_all_progress_is_visible_and_cancellation_keeps_completed_items(self):
        self.worker.write_text('''import argparse, json, signal, time
p = argparse.ArgumentParser()
p.add_argument('--checks', nargs='+'); p.add_argument('--duration'); p.add_argument('--discovery-timeout')
p.add_argument('--report')
a = p.parse_args()
with open(a.report, 'w') as f:
    json.dump({'passed': False, 'stage': 'cmd_vel', 'checks': {'teleop': {'passed': True}}}, f)
time.sleep(60)
''')
        self.manager.start('all')
        deadline = time.monotonic() + 5
        while 'result' not in self.manager.status('all') and time.monotonic() < deadline:
            time.sleep(.02)
        progress = self.manager.status('all')
        self.assertEqual(progress['state'], 'RUNNING')
        self.assertEqual(progress['result']['stage'], 'cmd_vel')
        self.assertTrue(module.item_messages(progress)[0].startswith('[통과] teleop'))
        self.assertFalse(self.manager.start('lidar')[0])
        self.assertTrue(self.manager.cancel('all')[0])
        result = self.finish('all')
        self.assertEqual(result['state'], 'CANCELLED')
        self.assertTrue(result['result']['checks']['teleop']['passed'])
        self.assertIn('1/5 통과', module.response_message('all', 'status', result))

    def test_invalid_report_structure_is_a_failed_job(self):
        self.worker.write_text(self.worker.read_text().replace("{'passed': ok, 'checks': checks, 'camera': a.camera_indices}", '[]'))
        self.manager.start('all')
        self.assertEqual(self.finish('all')['state'], 'FAILED')


class TestResponses(unittest.TestCase):
    def test_start_and_idle_explain_next_action_without_claiming_pass(self):
        for state in ('IDLE', 'RUNNING'):
            with self.subTest(state=state):
                message = module.response_message('cmd_vel', 'start', {'state': state})
                self.assertNotIn('[통과]', message)
                self.assertIn('/lekiwi_test/cmd_vel/', message)
        self.assertIn('(mock)', module.response_message('teleop', 'start', {'state': 'RUNNING'}))

    def test_pass_response_is_concise_and_json_mode_preserves_details(self):
        data = {'feature': 'cmd_vel', 'state': 'PASSED', 'report_path': '/tmp/report.json',
                'result': {'passed': True, 'checks': {'cmd_vel': {
                    'passed': True, 'sent': [.04, -.02, .1], 'timeout_stop': True}}}}
        original = json.dumps(data)
        message = module.response_message('cmd_vel', 'status', data)
        self.assertTrue(message.startswith('[통과] cmd_vel:'))
        self.assertIn('mock odom 응답', message)
        self.assertIn('timeout 정지', message)
        self.assertIn('/tmp/report.json', message)
        self.assertNotIn('"sent"', message)
        self.assertEqual(json.loads(module.response_message('cmd_vel', 'status', data, 'json')), data)
        self.assertEqual(json.dumps(data), original)

    def test_worker_error_is_shown_even_without_top_level_error(self):
        data = {'state': 'FAILED', 'result': {'checks': {'cmd_vel': {
            'passed': False, 'error': 'Timeout: cmd_vel subscribers'}}}}
        message = module.response_message('cmd_vel', 'status', data)
        self.assertTrue(message.startswith('[실패]'))
        self.assertIn('/cmd_vel 구독자 연결', message)

    def test_failure_reasons_use_topic_evidence(self):
        cases = [
            ({'endpoints': {'publishers': []}, 'messages': 0}, '발행 노드를 찾지 못함'),
            ({'messages': 0}, '메시지 수신 없음'),
            ({'error': 'Unexpected lidar frame'}, 'lidar_link'),
            ({'maximum_gap_s': .5, 'max_receive_gap_s': .8, 'hz': 30}, '0.800초'),
            ({'maximum_gap_s': .5, 'max_header_gap_s': .9}, 'timestamp 간격'),
            ({'minimum_hz': 10, 'hz': 30, 'window_hz': 4}, '수신율 4Hz'),
            ({'matches_image': False}, '해상도가 다름'),
        ]
        for details, reason in cases:
            with self.subTest(details=details):
                data = {'state': 'FAILED', 'result': {'checks': {'lidar': {'passed': False,
                    'topics': {'/scan': {'passed': False, **details}}}}}}
                message = module.response_message('lidar', 'status', data)
                self.assertTrue(message.startswith('[실패]'))
                self.assertIn('/scan: ', message)
                self.assertIn(reason, message)

    def test_camera_alias_and_partial_failure_identify_selected_camera(self):
        data = {'state': 'PASSED', 'result': {'checks': {'camera': {'passed': True}}}}
        self.assertIn('[통과] 카메라 2: 영상 수신', module.response_message('camera2', 'status', data))
        data = {'state': 'FAILED', 'result': {'checks': {'camera': {'passed': False, 'topics': {
            '/camera_0/image_raw/compressed': {'passed': True},
            '/camera_2/image_raw/compressed': {'passed': False, 'messages': 0}}}}}}
        message = module.response_message('camera', 'status', data)
        self.assertIn('/camera_2/image_raw/compressed: 메시지 수신 없음', message)
        self.assertNotIn('/camera_0/', message)

    def test_deadline_error_is_not_reported_as_user_cancellation(self):
        data = {'state': 'FAILED', 'error': 'Test deadline exceeded',
                'result': {'error': 'signal 2'}}
        self.assertIn('[실패] odom: 검사 제한 시간 초과', module.response_message('odom', 'status', data))
        for state, prefix in [('CANCELLING', '[취소 중]'), ('CANCELLED', '[취소됨]')]:
            self.assertTrue(module.response_message('odom', 'cancel', {'state': state}).startswith(prefix))
        cancelled = {'state': 'CANCELLED', 'result': {'error': 'signal 2', 'checks': {}}}
        message = module.response_message('all', 'status', cancelled)
        self.assertTrue(message.startswith('[취소됨]'))
        self.assertNotIn('signal 2', message)
        self.assertIn('5개 미완료', message)

    def test_busy_and_inactive_cancel_do_not_report_the_requested_test_as_passed(self):
        message = module.response_message('teleop', 'start', {'error': 'BUSY', 'active': 'lidar'})
        self.assertIn('LiDAR 검사 실행 중', message)
        self.assertIn('teleop 검사 시작 거부', message)
        self.assertIn('/lekiwi_test/lidar/status', message)
        message = module.response_message('teleop', 'cancel', {
            'error': 'No running test for this feature', 'state': 'PASSED'})
        self.assertTrue(message.startswith('[거부]'))
        self.assertNotIn('[통과]', message)

    def test_unknown_exception_is_preserved_on_one_line(self):
        message = module.response_message('odom', 'start', {'error': 'Example error\nwith details'})
        self.assertIn('[실패] odom: Example error with details', message)


if __name__ == '__main__':
    unittest.main()
