#!/usr/bin/python3
"""Create a private per-device Wi-Fi configuration; do not apply networking."""

import getpass
import os
from pathlib import Path
import subprocess
import sys

import yaml


def wifi_config(interface, ssid, password, renderer):
    if not ssid or len(ssid.encode('utf-8')) > 32 or '\x00' in ssid:
        raise ValueError('Wi-Fi 이름이 올바르지 않습니다.')
    valid_key = (8 <= len(password) <= 63 and password.isascii()) or (
        len(password) == 64 and all(char in '0123456789abcdefABCDEF' for char in password))
    if not valid_key:
        raise ValueError('Wi-Fi 암호는 ASCII 8~63자 또는 64자리 16진수여야 합니다.')
    wifi = {'renderer': renderer, 'dhcp4': True, 'optional': True,
            'access-points': {ssid: {'password': password}}}
    if renderer == 'NetworkManager':
        wifi['networkmanager'] = {'name': 'lekiwi-validation-wifi',
                                 'passthrough': {'connection.autoconnect-priority': '999'}}
    return {'network': {'version': 2, 'wifis': {interface: wifi}}}


def write_config(target, config):
    descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        stream.write(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))


def main():
    target = Path('/etc/netplan/90-lekiwi-wifi.yaml')
    if os.geteuid() != 0:
        sys.exit('sudo로 실행하세요.')
    if target.exists():
        print('기존 Wi-Fi 설정 파일을 유지합니다. 다음 시험 적용 단계로 진행하세요.')
        return
    interfaces = [entry.parent.name for entry in Path('/sys/class/net').glob('*/wireless')]
    if len(interfaces) != 1:
        sys.exit('무선 장치를 하나로 식별하지 못했습니다. Wi-Fi 장애 안내를 확인하세요.')
    renderer = 'NetworkManager' if subprocess.run(
        ['systemctl', 'is-active', '--quiet', 'NetworkManager']).returncode == 0 else 'networkd'
    with open('/dev/tty', 'r', encoding='utf-8') as terminal_input, \
            open('/dev/tty', 'w', encoding='utf-8') as terminal_output:
        terminal_output.write('노트북과 같은 Wi-Fi 이름(SSID): ')
        terminal_output.flush()
        ssid = terminal_input.readline().rstrip('\r\n')
        password = getpass.getpass('Wi-Fi 암호: ', stream=terminal_output)
    try:
        write_config(target, wifi_config(interfaces[0], ssid, password, renderer))
    except (OSError, ValueError) as error:
        sys.exit(str(error))
    print(f'Wi-Fi 설정 준비 완료: {interfaces[0]} / {renderer}')
    print('암호는 화면에 출력하지 않습니다. 아직 네트워크를 변경하지 않았습니다.')


if __name__ == '__main__':
    main()
