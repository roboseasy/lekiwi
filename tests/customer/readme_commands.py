"""Execute the published customer commands; keep simulation substitutions explicit."""

import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


SOURCE = Path.home() / 'lekiwi_ws/src/lekiwi'


def blocks(path=SOURCE / 'README.md'):
    return re.findall(r'```bash\n(.*?)\n```', path.read_text(), re.S)


def command_starting(prefix):
    matches = [block for block in blocks() if block.startswith(prefix)]
    if len(matches) != 1:
        raise RuntimeError(f'expected exactly one README block starting with {prefix!r}')
    return matches[0]


def block_containing(readme_blocks, marker):
    matches = [block for block in readme_blocks if marker in block]
    if len(matches) != 1 or matches[0].count(marker) != 1:
        raise RuntimeError(f'expected exactly one README command containing {marker!r}')
    return matches[0]


def ros_command(block):
    return shlex.split(os.path.expandvars(block.replace('\\\n', ' ')))


def apply_ros_environment():
    if os.environ.get('LEKIWI_CUSTOMER_ROS_ENV') == 'ready':
        return
    block = next(block for block in blocks() if 'export ROS_DOMAIN_ID=' in block)
    result = subprocess.run(['bash', '-ec', block + '\nenv -0'],
                            capture_output=True, check=True)
    for entry in result.stdout.split(b'\0'):
        if b'=' in entry:
            key, value = entry.split(b'=', 1)
            os.environ[os.fsdecode(key)] = os.fsdecode(value)
    assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    assert os.environ['FASTDDS_BUILTIN_TRANSPORTS'] == 'UDPv4'
    # Python and the shared-library loader need these paths at process startup.
    os.execve(sys.executable, [sys.executable, *sys.argv],
              dict(os.environ, LEKIWI_CUSTOMER_ROS_ENV='ready'))


def install():
    source = Path('/opt/lekiwi-source')
    readme_blocks = blocks(source / 'README.md')
    # Authentication/access is assumed for a customer of this private repository.
    # Clone the identical checked-out Git repository, without embedding credentials.
    clone = 'git clone -b feature/lekiwi-ros2 https://github.com/roboseasy-members/lekiwi.git'
    install_block = block_containing(readme_blocks, clone).replace(
        clone, 'git clone -b feature/lekiwi-ros2 /opt/lekiwi-source lekiwi')
    build_block = block_containing(readme_blocks, '--packages-select ydlidar_sdk')
    for block in (install_block, build_block):
        print('[README commands]\n' + block, flush=True)
        subprocess.run(['bash', '-ec', block], check=True)
    commit = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip()
    assert commit == os.environ['SOURCE_COMMIT'], (commit, os.environ['SOURCE_COMMIT'])
    subprocess.run(['sudo', 'usermod', '-aG', 'dialout,video', os.environ['USER']], check=True)
    evidence = Path.home() / 'installation.json'
    evidence.write_text(json.dumps({
        'commit': commit, 'architecture': subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip(),
        'commands': [install_block, build_block],
        'substitutions': ['Git clone transport uses the same local checkout; repository access is assumed.',
                          'USB/I2C hardware is replaced at runtime; hardware permissions are not validated.'],
    }, indent=2) + '\n')


if __name__ == '__main__':
    install()
