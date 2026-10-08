#!/usr/bin/python3
"""Prepare PC/Pi ROS packages from the laptop checkout, without starting hardware."""

import argparse
import hashlib
import ipaddress
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parent.parent
PACKAGES = ('lekiwi', 'lekiwi_description', 'lekiwi_bringup', 'lekiwi_node',
            'lekiwi_sensors', 'lekiwi_teleop', 'lekiwi_cartographer', 'lekiwi_navigation2')
HELPERS = ('tools/prepare_ros.py', 'tools/install_ros.sh', 'tools/ros_env.sh',
           'tools/configure_wifi.py', 'tools/validation.sh', 'tools/save_map.sh')
SOURCE_NAMES = (*PACKAGES, 'lekiwi.repos', *HELPERS)
# Exact bytes of the repository-owned helper shipped in commit 0f87b84.
# Any other differing helper (including user edits) is preserved and refused.
RELEASED_HELPERS = {
    'tools/prepare_ros.py': {'57a595a64e2132148a3a58ce0e2db439b9cc98d2cc1304b58f7fc76fe6cee775'},
    'tools/install_ros.sh': {'03612e84bf98566f6ddde542b281049d7b598ff198eb4208e5cda5362704068c'},
}


def source_files(root, names=SOURCE_NAMES):
    files = {}
    for name in names:
        entry = root / name
        if not entry.exists():
            raise RuntimeError(f'소스 파일이 없습니다: {entry}')
        for path in sorted(entry.rglob('*')) if entry.is_dir() else [entry]:
            relative = path.relative_to(root)
            if '__pycache__' in relative.parts or '.pytest_cache' in relative.parts or path.suffix in ('.pyc', '.pyo'):
                continue
            if path.is_symlink():
                raise RuntimeError(f'전송 소스에 심볼릭 링크가 있습니다: {relative}')
            if path.is_file():
                files[relative.as_posix()] = path
    return files


def source_hash(root):
    digest = hashlib.sha256()
    for name, path in sorted(source_files(root).items()):
        digest.update(name.encode() + b'\0')
        digest.update(str(path.stat().st_mode & 0o111).encode() + b'\0')
        digest.update(path.read_bytes())
    return digest.hexdigest()


def robot_target(root):
    content = (root / 'lekiwi-sensor-check/check_robot.sh').read_text()
    settings = {}
    for key in ('ROBOT_IP', 'ROBOT_USER'):
        matches = re.findall(r'^' + key + r'="([^"\n]+)"\s*$', content, re.M)
        if len(matches) != 1:
            raise RuntimeError(f'check_robot.sh의 {key}="..." 설정을 확인하세요.')
        settings[key] = matches[0]
    address = ipaddress.ip_address(settings['ROBOT_IP'])
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*', settings['ROBOT_USER']):
        raise RuntimeError('Pi 계정명을 확인하세요.')
    return settings['ROBOT_USER'], str(address)


def make_bundle(root, destination):
    with tarfile.open(destination, 'w:gz') as archive:
        for name, path in source_files(root).items():
            archive.add(path, arcname=name, recursive=False)


def deploy_bundle(bundle, destination):
    """Copy fresh sources; preserve and refuse any differing existing sources."""
    with tempfile.TemporaryDirectory(prefix='lekiwi-source-') as temporary:
        unpacked = Path(temporary)
        with tarfile.open(bundle, 'r:gz') as archive:
            for member in archive.getmembers():
                path = PurePosixPath(member.name)
                if path.is_absolute() or '..' in path.parts or not member.isfile():
                    raise RuntimeError('소스 압축 파일의 경로·파일 형식이 올바르지 않습니다.')
            archive.extractall(unpacked, filter='data')
        incoming = source_files(unpacked)
        if destination.is_symlink():
            raise RuntimeError('기존 Pi 소스가 심볼릭 링크입니다. 보존하고 중단합니다.')
        if destination.exists():
            old = source_files(destination, (*PACKAGES, 'lekiwi.repos'))
            new = source_files(unpacked, (*PACKAGES, 'lekiwi.repos'))
            if old.keys() != new.keys() or any(old[name].read_bytes() != new[name].read_bytes() for name in old):
                raise RuntimeError('기존 Pi ROS 소스가 다릅니다. 덮어쓰지 않고 중단합니다. ros_setup_troubleshooting.md를 확인하세요.')
            updates = set()
            for name, path in incoming.items():
                current = destination / name
                if any(parent.is_symlink() for parent in current.parents if parent != destination.parent):
                    raise RuntimeError(f'기존 소스 경로에 심볼릭 링크가 있습니다: {current}')
                if current.is_symlink():
                    raise RuntimeError(f'기존 파일을 보존하고 중단합니다: {current}')
                if current.exists() and current.read_bytes() != path.read_bytes():
                    digest = hashlib.sha256(current.read_bytes()).hexdigest() if current.is_file() else ''
                    if digest not in RELEASED_HELPERS.get(name, set()):
                        raise RuntimeError(f'기존 파일을 보존하고 중단합니다: {current}')
                    updates.add(name)
            for name, path in incoming.items():
                current = destination / name
                if not current.exists() or name in updates:
                    current.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(path, current)
            print('Pi의 같은 ROS 소스를 유지하고 검증된 준비 도구를 적용했습니다.', flush=True)
        else:
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(unpacked, destination)
            print('Pi ROS 소스 준비: ~/lekiwi_ws/src/lekiwi', flush=True)


def prepare_pi(root):
    user, address = robot_target(root)
    target = f'{user}@{address}'
    scp_target = f'{user}@[{address}]' if ':' in address else target
    print(f'[1/2] Pi ROS 준비: {target}', flush=True)
    with tempfile.TemporaryDirectory(prefix='lekiwi-ros-ssh-') as temporary:
        directory = Path(temporary)
        bundle = directory / 'source.tar.gz'
        make_bundle(root, bundle)
        options = ['-o', f'ControlPath={directory / "control"}', '-o', 'ConnectTimeout=10',
                   '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3']
        ssh = ['ssh', *options]
        connected = False
        try:
            subprocess.run([*ssh, '-M', '-N', '-f', '-o', 'ControlPersist=120', target], check=True)
            connected = True
            subprocess.run([*ssh, target, 'test -r /opt/ros/jazzy/setup.bash'], check=True)
            remote = subprocess.check_output([*ssh, target, 'mktemp -d /tmp/lekiwi-ros.XXXXXXXX'], text=True).strip()
            if not re.fullmatch(r'/tmp/lekiwi-ros\.[a-zA-Z0-9]+', remote):
                raise RuntimeError('Pi 임시 경로를 확인하지 못했습니다.')
            try:
                subprocess.run(['scp', *options, str(bundle), str(root / 'tools/prepare_ros.py'),
                                f'{scp_target}:{remote}/'], check=True)
                deploy = shlex.join(['/usr/bin/python3', f'{remote}/prepare_ros.py', '--deploy', f'{remote}/source.tar.gz'])
                install = 'bash --noprofile --norc "$HOME/lekiwi_ws/src/lekiwi/tools/install_ros.sh" pi'
                command = ('mkdir -p "$HOME/.cache" && flock -n "$HOME/.cache/lekiwi-ros-prepare.lock" '
                           'bash --noprofile --norc -c ' + shlex.quote(deploy + ' && ' + install))
                subprocess.run([*ssh, '-tt', target, command], check=True)
            finally:
                subprocess.run([*ssh, target, 'rm -rf -- ' + shlex.quote(remote)], check=False)
        finally:
            if connected:
                subprocess.run([*ssh, '-O', 'exit', target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def main():
    parser = argparse.ArgumentParser(description='노트북에서 Pi 소스 전송·ROS 빌드와 노트북 ROS 준비를 한 번에 수행합니다.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--target', action='store_true', help='센서 스크립트의 SSH 접속 대상만 출력')
    group.add_argument('--source-hash', action='store_true', help=argparse.SUPPRESS)
    group.add_argument('--deploy', type=Path, help=argparse.SUPPRESS)
    group.add_argument('--pc-only', action='store_true', help='노트북 준비만 수행')
    group.add_argument('--pi-only', action='store_true', help='Pi 준비만 수행')
    args = parser.parse_args()
    try:
        if args.target:
            user, address = robot_target(ROOT)
            print(f'{user}@{address}')
        elif args.source_hash:
            print(source_hash(ROOT))
        elif args.deploy:
            deploy_bundle(args.deploy, Path.home() / 'lekiwi_ws/src/lekiwi')
        else:
            if not args.pi_only and not Path('/opt/ros/jazzy/setup.bash').is_file():
                raise RuntimeError('노트북에 ROS 2 Jazzy를 먼저 설치하세요.')
            if not args.pc_only:
                prepare_pi(ROOT)
            if not args.pi_only:
                print('[2/2] 노트북 ROS 준비', flush=True)
                subprocess.run(['bash', '--noprofile', '--norc', str(ROOT / 'tools/install_ros.sh'), 'pc'], check=True)
            print('ROS 준비 완료. Wi-Fi 연결 확인으로 진행하세요.', flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, tarfile.TarError) as error:
        print(f'ROS 준비 중단: {error}\n오류가 해결되기 전에는 다음 단계로 넘어가지 마세요.', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('ROS 준비를 중단했습니다. 다음 실행에서 준비 상태를 다시 확인합니다.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
