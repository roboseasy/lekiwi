#!/usr/bin/python3
"""Run the local ROS test server and call its all services from one terminal."""

import json
import signal
import time

from test_service_server import ACTIVE, configure_server, failure_summary, item_messages, response_message, suite_summary


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    server = rclpy.create_node('lekiwi_test_server')
    client_node = rclpy.create_node('lekiwi_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    manager = None
    started = False
    handlers = {}

    def interrupted(signum, frame):
        raise KeyboardInterrupt()

    def request(client):
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=10.)
        if not future.done() or future.result() is None:
            raise RuntimeError('검사 서비스 응답 시간 초과')
        response = future.result()
        data = json.loads(response.message)
        if not response.success:
            raise RuntimeError(response_message('all', 'start', data))
        return data

    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, interrupted)
        print('[준비] 검사 서버 시작 중. 기본 연결 확인 시간은 10초입니다.', flush=True)
        manager = configure_server(server, log_updates=False, default_response_format='json')
        if server.get_parameter('response_format').value != 'json':
            raise ValueError('test_all.py는 response_format:=json을 사용합니다')
        start = client_node.create_client(Trigger, '/lekiwi_test/all/start')
        status = client_node.create_client(Trigger, '/lekiwi_test/all/status')
        if not start.wait_for_service(timeout_sec=10.) or not status.wait_for_service(timeout_sec=10.):
            raise RuntimeError('전체 검사 서비스를 찾지 못함')
        data = request(start)
        started = True
        print(response_message('all', 'start', data), flush=True)
        printed = set()
        stage = None
        while True:
            data = request(status)
            current = data.get('result', {}).get('stage')
            if current and current != stage:
                label = {'discovery': '노드 연결 확인', 'teleop': 'teleop 키 입력 검사',
                         'cmd_vel': 'cmd_vel 명령 검사', 'sensors': 'odom·라이다·카메라 관찰'}.get(current, current)
                print('[진행 중] ' + label, flush=True)
                stage = current
            for message in item_messages(data):
                if message not in printed:
                    print(message, flush=True)
                    printed.add(message)
            if data['state'] not in ACTIVE:
                prefix = '[통과]' if data['state'] == 'PASSED' else '[취소됨]' if data['state'] == 'CANCELLED' else '[실패]'
                print(f'{prefix} 전체: {suite_summary(data)}', flush=True)
                if (data.get('error') or data.get('result', {}).get('error')
                        or (data['state'] == 'FAILED' and data.get('exit_code') not in (None, 0))):
                    print('원인: ' + failure_summary(data), flush=True)
                print('상세 결과: ' + data['report_path'], flush=True)
                return 0 if data['state'] == 'PASSED' else 1
            # Keep both service nodes and the worker watchdog responsive while waiting.
            until = time.monotonic() + .5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        print('[취소 중] 전체 검사 종료 처리 중', flush=True)
        return 130
    except Exception as error:
        print('[실패] ' + str(error), flush=True)
        if manager is None:
            print('기존 PC 검사 서버가 실행 중이면 Ctrl-C로 종료한 뒤 다시 실행하세요.', flush=True)
        return 1
    finally:
        if manager is not None:
            manager.close()
            if started and manager.status('all')['state'] == 'CANCELLED':
                print(response_message('all', 'status', manager.status('all')), flush=True)
        executor.shutdown()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
