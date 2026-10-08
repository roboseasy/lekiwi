#!/usr/bin/python3
"""Nonblocking ROS Trigger services for topic checks; never starts hardware runtimes."""

import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


ALL_CHECKS = ('teleop', 'cmd_vel', 'odom', 'lidar', 'camera')
FEATURES = ('all', *ALL_CHECKS, 'camera0', 'camera2')
ACTIVE = ('RUNNING', 'CANCELLING')
LABELS = {'all': '전체', 'teleop': 'teleop', 'cmd_vel': 'cmd_vel', 'odom': 'odom', 'lidar': 'LiDAR',
          'camera': '카메라 0·2', 'camera0': '카메라 0', 'camera2': '카메라 2'}
PASS_SUMMARIES = {
    'teleop': '6방향 키 입력·timeout 정지, mock odom 응답, 실물용 1·2·3단 속도·키 해제 정지 로직 확인 (실물 움직임은 별도)',
    'cmd_vel': '명령 수신, mock odom 응답, timeout 정지 확인',
    'odom': '/odom 수신, 좌표계·위치·속도·시간 갱신 확인 (이동거리 정확도는 별도 확인)',
    'lidar': '/scan·/scan_raw 수신, 유효 거리·좌표계·시간 갱신 확인 (시야·거리 정확도는 별도 확인)',
    'camera': '영상 수신·디코딩, 640×480, camera_info 일치 확인 (카메라 보정 검사는 별도)',
}
ERROR_MESSAGES = {
    'Test deadline exceeded': '검사 제한 시간 초과',
    'Worker ended without a valid report': '검사 프로세스가 유효한 결과 파일 없이 종료됨',
    '/lekiwi_node parameter service unavailable': '/lekiwi_node에 연결할 수 없음. Pi bringup과 ROS 환경을 확인하세요',
    'Parameter query failed': 'base 설정 조회 실패',
    'Another cmd_vel publisher exists': '다른 /cmd_vel 발행 노드가 실행 중입니다. 기존 teleop 또는 명령 발행을 종료하세요',
    'Refusing commands: backend is not mock': '실물 모터 설정이므로 자동 명령 검사를 거부함. mock bringup을 사용하세요',
    'Refusing commands: motor write enabled': '모터 쓰기가 활성화되어 자동 명령 검사를 거부함',
    'Refusing commands: torque enabled': '모터 토크가 활성화되어 자동 명령 검사를 거부함',
    'Expected mock command odometry': 'mock 검사는 odom_source:=command 설정이 필요함',
    'Unexpected lidar frame': '라이다 좌표계가 lidar_link와 다름',
    'No finite ranges within sensor limits': '센서 측정 범위 안에 유효한 거리값이 없음',
    'Compressed image decoding failed': '압축 영상 디코딩 실패',
    'Zero message timestamp': '메시지 timestamp가 0임',
    'Non-increasing message timestamp': '메시지 timestamp가 갱신되지 않거나 역행함',
    'Unexpected odometry frames': 'odom 좌표계가 odom → base_footprint와 다름',
}


def error_summary(error):
    text = ' '.join(str(error).split())
    if text.startswith('Timeout: '):
        target = text.removeprefix('Timeout: ')
        target = {'cmd_vel subscribers': '/cmd_vel 구독자 연결',
                  'mock odom reception': 'mock /odom 수신',
                  'teleop publisher discovery': 'teleop /cmd_vel 발행 노드 연결'}.get(target, target)
        return '제한 시간 안에 확인하지 못함: ' + target
    return ERROR_MESSAGES.get(text, text)


def topic_failure(details):
    """Explain recorded evidence; do not turn missing data into a claimed diagnosis."""
    if details.get('endpoints', {}).get('publishers') == []:
        return '발행 노드를 찾지 못함'
    if details.get('messages') == 0:
        return '메시지 수신 없음'
    if details.get('error'):
        return error_summary(details['error'])
    if details.get('matches_image') is False:
        return 'camera_info와 영상의 좌표계 또는 해상도가 다름'
    maximum = details.get('maximum_gap_s')
    for key, label in (('max_receive_gap_s', '최대 수신 공백'), ('max_header_gap_s', '최대 timestamp 간격')):
        if maximum is not None and details.get(key, 0) > maximum:
            return f'{label} {details[key]:.3f}초 (허용 {maximum:g}초)'
    minimum = details.get('minimum_hz')
    rates = [details[key] for key in ('hz', 'window_hz') if key in details]
    if minimum is not None and rates and min(rates) < minimum:
        return f'수신율 {min(rates):g}Hz (최소 {minimum:g}Hz 필요)'
    if details.get('fresh') is False:
        return '새 메시지 수신이 중단됨'
    return '수신 수·시간 갱신 등 통과 기준 미충족'


def failure_summary(data):
    report = data.get('result', {})
    for error in (data.get('error'), report.get('error')):
        if error:
            return error_summary(error)
    reasons = []
    for check in report.get('checks', {}).values():
        if check.get('error'):
            reasons.append(error_summary(check['error']))
        for topic, details in check.get('topics', {}).items():
            if details.get('passed') is not True:
                reasons.append(f'{topic}: {topic_failure(details)}')
    if reasons:
        return '; '.join(reasons)
    if data.get('exit_code') not in (None, 0):
        return f'검사 프로세스 종료 코드 {data["exit_code"]}. console.log를 확인하세요'
    return '검사 기준 미충족. 상세 결과를 확인하세요'


def item_messages(data):
    """Keep each result visible even if another item failed or the suite was interrupted."""
    report = data.get('result', {})
    checks = report.get('checks', {})
    messages = []
    for name in ALL_CHECKS:
        check = checks.get(name)
        if check is not None:
            if check.get('passed') is True:
                messages.append(f'[통과] {LABELS[name]}: {PASS_SUMMARIES[name]}')
            else:
                reason = failure_summary({'result': {'checks': {name: check}}})
                messages.append(f'[실패] {LABELS[name]}: {reason}')
        elif data.get('state') not in ACTIVE:
            messages.append(f'[미완료] {LABELS[name]}: 검사 결과 없음')
    return messages


def suite_summary(data):
    checks = data.get('result', {}).get('checks', {})
    passed = sum(checks.get(name, {}).get('passed') is True for name in ALL_CHECKS)
    failed = sum(name in checks and checks[name].get('passed') is not True for name in ALL_CHECKS)
    missing = len(ALL_CHECKS) - passed - failed
    return f'{passed}/5 통과, {failed}개 실패, {missing}개 미완료 (주행 명령 검사는 mock 전용)'


def response_message(feature, operation, data, response_format='text'):
    """Keep the full machine response opt-in; default to an actionable one-line status."""
    if response_format == 'json':
        return json.dumps(data, ensure_ascii=False)
    label = LABELS.get(feature, feature)
    state = data.get('state')
    error = data.get('error')
    if error == 'BUSY':
        active = data['active']
        return (f'[진행 중] {LABELS.get(active, active)} 검사 실행 중이므로 {label} 검사 시작 거부. '
                f'/lekiwi_test/{active}/status에서 확인하세요')
    if error == 'No running test for this feature':
        return f'[거부] {label}: 취소할 진행 중인 검사가 없습니다. 현재 상태: {state}'
    if state == 'IDLE':
        return f'[대기] {label}: 아직 실행한 검사가 없습니다. /lekiwi_test/{feature}/start로 시작하세요'
    if state == 'RUNNING':
        action = '시작' if operation == 'start' else '진행 중'
        scope = ' (mock)' if feature in ('teleop', 'cmd_vel') else ''
        if feature == 'all':
            stage = data.get('result', {}).get('stage', 'discovery') or 'finishing'
            stage_label = {'discovery': '노드 연결 확인', 'sensors': 'odom·라이다·카메라 관찰',
                           'finishing': '결과 정리'}.get(stage, stage)
            scope = f' (현재: {stage_label}; {suite_summary(data)})'
        return f'[진행 중] {label} 검사 {action}{scope}. 완료 결과: /lekiwi_test/{feature}/status'
    if state == 'CANCELLING':
        if error:
            return f'[중단 중] {label}: {error_summary(error)}. 종료 처리 중'
        return f'[취소 중] {label} 검사 종료 처리 중'
    if state == 'CANCELLED':
        message = f'[취소됨] {label} 검사 취소 완료'
    elif state == 'PASSED':
        check_name = 'camera' if feature in ('camera0', 'camera2') else feature
        check = data.get('result', {}).get('checks', {}).get(check_name, {})
        summary = PASS_SUMMARIES[check_name] if check_name in PASS_SUMMARIES and check.get('passed') is True else '검사 완료'
        message = f'[통과] {label}: {summary}'
    else:
        message = f'[실패] {label}: {failure_summary(data)}'
    if feature == 'all':
        prefix = {'PASSED': '[통과]', 'CANCELLED': '[취소됨]'}.get(state, '[실패]')
        message = f'{prefix} 전체: {suite_summary(data)}'
        if state == 'FAILED' and (error or data.get('result', {}).get('error') or data.get('report_error')
                                  or data.get('exit_code') not in (None, 0)):
            message += ' | ' + failure_summary(data)
        message += ' | ' + ' | '.join(item_messages(data))
    if data.get('report_path'):
        message += ' | 상세 결과: ' + data['report_path']
    return message


class TestJobs:
    """One owned worker at a time. Status and final reports survive in a run directory."""
    def __init__(self, worker, output_dir, duration=30., discovery_timeout=10.):
        if not all(math.isfinite(v) and 0 < v <= 120 for v in (duration, discovery_timeout)):
            raise ValueError('Test durations must be finite and in (0, 120] seconds')
        self.worker = Path(worker)
        if not self.worker.is_file():
            raise ValueError('Diagnostic worker is missing: ' + str(worker))
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.duration, self.discovery_timeout = duration, discovery_timeout
        self.jobs = {}
        self.active = None
        self.process = self.console = None
        self.cancel_deadline = self.kill_deadline = None
        self.lock = (self.output_dir / ('server-domain-' + os.environ.get('ROS_DOMAIN_ID', '0') + '.lock')).open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise RuntimeError('A test server already owns this output directory and ROS domain')

    def persist(self, job):
        path = Path(job['directory']) / 'status.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(job, indent=2) + '\n')
        temporary.replace(path)

    def start(self, feature):
        self.poll()
        if feature not in FEATURES:
            return False, {'error': 'Unknown test'}
        if self.active is not None:
            return False, {'error': 'BUSY', 'active': self.active['feature'], 'job_id': self.active['job_id']}
        ident = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:10]
        folder = self.output_dir / (feature + '-' + ident)
        folder.mkdir()
        job = {'feature': feature, 'job_id': ident, 'state': 'RUNNING',
               'started_unix': time.time(), 'directory': str(folder),
               'report_path': str(folder / 'report.json'), 'console_path': str(folder / 'console.log'),
               'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
               'scope': 'mock motion tests; passive real sensor/odom checks'}
        check = 'camera' if feature in ('camera0', 'camera2') else feature
        selected = ALL_CHECKS if feature == 'all' else (check,)
        command = [sys.executable, str(self.worker), '--checks', *selected,
                   '--duration', str(self.duration), '--discovery-timeout', str(self.discovery_timeout),
                   '--report', job['report_path']]
        if feature in ('camera0', 'camera2'):
            command += ['--camera-indices', feature[-1]]
        job['command'] = command
        self.console = (folder / 'console.log').open('w')
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self.console,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        except Exception as error:
            self.console.close()
            job.update(state='FAILED', error=str(error), finished_unix=time.time())
            job['result'] = {'passed': False, 'error': str(error), 'checks': {}}
            Path(job['report_path']).write_text(json.dumps(job['result'], indent=2) + '\n')
            self.jobs[feature] = job
            self.persist(job)
            return False, job
        job['pid'] = self.process.pid
        budget = self.duration + (8*self.discovery_timeout + 90. if feature == 'all' else 3*self.discovery_timeout + 60.)
        self.deadline = time.monotonic() + budget
        self.jobs[feature] = self.active = job
        self.cancel_deadline = self.kill_deadline = None
        self.persist(job)
        return True, job.copy()

    def status(self, feature):
        self.poll()
        return self.jobs.get(feature, {'feature': feature, 'state': 'IDLE'}).copy()

    def signal_worker(self, sig):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def cancel(self, feature):
        self.poll()
        if self.active is None or self.active['feature'] != feature:
            return False, {'error': 'No running test for this feature', **self.status(feature)}
        if self.active['state'] == 'RUNNING':
            self.active['state'] = 'CANCELLING'
            self.signal_worker(signal.SIGINT)
            self.cancel_deadline = time.monotonic() + 12.
            self.persist(self.active)
        return True, self.active.copy()

    def poll(self):
        if self.active is None:
            return
        now = time.monotonic()
        code = self.process.poll()
        if code is None:
            if self.active['feature'] == 'all':
                try:
                    report = self.read_report(self.active['report_path'])
                    if self.active.get('result') != report:
                        self.active['result'] = report
                        self.persist(self.active)
                except (OSError, ValueError):
                    pass
            if self.active['state'] == 'RUNNING' and now >= self.deadline:
                self.active.update(state='CANCELLING', error='Test deadline exceeded')
                self.signal_worker(signal.SIGINT)
                self.cancel_deadline = now + 12.
                self.persist(self.active)
            if self.cancel_deadline is not None and now >= self.cancel_deadline:
                self.signal_worker(signal.SIGTERM)
                self.cancel_deadline, self.kill_deadline = None, now + 5.
            if self.kill_deadline is not None and now >= self.kill_deadline:
                self.signal_worker(signal.SIGKILL)
                self.kill_deadline = None
            return
        job = self.active
        # Also stop any descendant left in this worker's private process group.
        self.signal_worker(signal.SIGTERM)
        self.console.close()
        job.update(exit_code=code, finished_unix=time.time())
        try:
            report = self.read_report(job['report_path'])
            job['result'] = report
        except (OSError, ValueError) as error:
            job['report_error'] = str(error)
            report = {'passed': False, 'checks': {}, 'error': 'Worker ended without a valid report',
                      'cancelled': job['state'] == 'CANCELLING' and 'error' not in job}
            job['result'] = report
            Path(job['report_path']).write_text(json.dumps(report, indent=2) + '\n')
        if job['state'] == 'CANCELLING':
            job['state'] = 'FAILED' if 'error' in job else 'CANCELLED'
        else:
            job['state'] = 'PASSED' if code == 0 and report.get('passed') is True else 'FAILED'
            if job['feature'] == 'all' and not all(
                    report.get('checks', {}).get(name, {}).get('passed') is True for name in ALL_CHECKS):
                job['state'] = 'FAILED'
        self.persist(job)
        self.active = self.process = self.console = None

    @staticmethod
    def read_report(path):
        report = json.loads(Path(path).read_text())
        if (not isinstance(report, dict) or not isinstance(report.get('checks', {}), dict)
                or not all(isinstance(check, dict) for check in report.get('checks', {}).values())):
            raise ValueError('Invalid report structure')
        return report

    def close(self):
        if self.active is not None:
            self.cancel(self.active['feature'])
            deadline = time.monotonic() + 19.
            while self.active is not None and time.monotonic() < deadline:
                self.poll()
                time.sleep(.05)
        self.lock.close()


def configure_server(node, log_updates=True, default_response_format='text'):
    """Create the same ROS services for standalone and single-terminal use."""
    from std_srvs.srv import Trigger

    manager = None
    try:
        output = node.declare_parameter('output_dir', str(Path.home() / '.ros/lekiwi-tests')).value
        duration = float(node.declare_parameter('duration', 30.).value)
        discovery = float(node.declare_parameter('discovery_timeout', 10.).value)
        response_format = node.declare_parameter('response_format', default_response_format).value
        if response_format not in ('text', 'json'):
            raise ValueError('response_format must be text or json')
        # Both files are installed together; resolve source symlinks for --symlink-install.
        manager = TestJobs(Path(__file__).resolve().with_name('test_ros_topics.py'), output, duration, discovery)
        # Global names include clients, and by-node queries are ambiguous when
        # another server has our node name. Readiness checks actual server endpoints.
        probe = node.create_client(Trigger, '/lekiwi_test/teleop/status')
        duplicate = probe.wait_for_service(timeout_sec=discovery)
        node.destroy_client(probe)
        if duplicate:
            raise RuntimeError('Another /lekiwi_test service server is already visible')

        announced = {}

        def announce_updates():
            if not log_updates:
                return
            for feature, job in manager.jobs.items():
                version = (job['job_id'], job['state'], job.get('result', {}).get('stage'),
                           tuple(job.get('result', {}).get('checks', {})))
                if announced.get(feature) != version:
                    message = response_message(feature, 'status', job)
                    logger = node.get_logger()
                    if job['state'] == 'FAILED':
                        logger.error(message)
                    else:
                        logger.info(message)
                    announced[feature] = version

        def poll_and_announce():
            manager.poll()
            announce_updates()

        def callback(feature, operation):
            def invoke(request, response):
                try:
                    if operation == 'status':
                        response.success, data = True, manager.status(feature)
                    else:
                        response.success, data = getattr(manager, operation)(feature)
                    response.message = response_message(feature, operation, data, response_format)
                    announce_updates()
                except Exception as error:
                    response.success = False
                    response.message = response_message(feature, operation, {'error': str(error)}, response_format)
                return response
            return invoke

        for feature in FEATURES:
            for operation in ('start', 'status', 'cancel'):
                node.create_service(Trigger, f'/lekiwi_test/{feature}/{operation}', callback(feature, operation))
        node.create_timer(.1, poll_and_announce)
        node.get_logger().info('Test services ready: /lekiwi_test/<feature>/{start,status,cancel}')
        return manager
    except BaseException:
        if manager is not None:
            manager.close()
        raise


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('lekiwi_test_server')
    manager = None
    handlers = {}
    try:
        manager = configure_server(node)
        def interrupted(signum, frame):
            raise KeyboardInterrupt()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, interrupted)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        if manager is not None:
            manager.close()
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
