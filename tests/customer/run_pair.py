"""Run remote customer images on separate network stacks; never attach host devices."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

from actuator_scheduling import prepare_virtual_actuator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arch', choices=['amd64', 'arm64'],
                        help='Run both roles natively; cross-architecture QEMU cannot validate 5 ms serial deadlines.')
    args = parser.parse_args()
    evidence = Path('evidence').resolve()
    for role in ('pc', 'pi'):
        directory = evidence / role
        directory.mkdir(parents=True, exist_ok=True)
        directory.chmod(0o777)
    prefix = 'lekiwi-customer-' + os.environ.get('GITHUB_RUN_ID', str(os.getpid()))
    network = prefix + '-lan'
    containers = []
    roles = [('pi', args.arch or 'arm64'), ('pc', args.arch or 'amd64')]
    role_architectures = dict(roles)
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 2:
        raise RuntimeError('PC/Pi isolation requires at least two available CPUs')
    split = max(1, len(cpus) // 2)
    role_cpus = {'pi': cpus[:split], 'pc': cpus[split:]}
    harness = Path(__file__).resolve().parent
    binary = Path(tempfile.mkdtemp(prefix='lekiwi-customer-actuator-')) / 'serial-wheels'
    subprocess.run(['g++', '-std=c++17', '-O2', '-static', '-pthread', '-Wall', '-Wextra', '-Werror',
                    str(harness / 'serial_wheels.cpp'), '-o', str(binary)], check=True)

    def docker(*args, **kwargs):
        return subprocess.run(['docker', *args], check=True, **kwargs)

    docker('network', 'create', network)
    try:
        for role, arch in roles:
            name = prefix + '-' + role
            docker('run', '-d', '--init', '--name', name, '--hostname', role,
                   '--platform', 'linux/' + arch, '--network', network,
                   '--cpuset-cpus', ','.join(map(str, role_cpus[role])),
                   '--cap-drop', 'ALL', '-v', f'{evidence / role}:/evidence',
                   '-v', f'{harness}:/customer-test:ro',
                   '-v', f'{binary}:/customer-serial-wheels:ro',
                   '-e', 'LEKIWI_CUSTOMER_PC_ARCH=' + role_architectures['pc'],
                   '-e', 'LEKIWI_CUSTOMER_PI_ARCH=' + role_architectures['pi'],
                   '-e', 'LEKIWI_CUSTOMER_EXECUTION=' + ('native' if args.arch else 'cross-architecture QEMU'),
                   '-e', 'LEKIWI_CUSTOMER_ACTUATOR_RUN=' + prefix,
                   'lekiwi-customer:' + arch, '/usr/bin/python3',
                   f'/customer-test/{role}.py')
            containers.append(name)
            if role == 'pi':
                prepare_virtual_actuator(name, docker, evidence / role, prefix)
        metadata = []
        for name in containers:
            item = json.loads(docker('inspect', name, capture_output=True, text=True).stdout)[0]
            metadata.append({
                'name': name, 'image': item['Config']['Image'],
                'hostname': item['Config']['Hostname'],
                'networks': item['NetworkSettings']['Networks'],
                'devices': item['HostConfig']['Devices'],
                'privileged': item['HostConfig']['Privileged'],
                'cpus': item['HostConfig']['CpusetCpus'],
            })
        (evidence / 'network.json').write_text(json.dumps(metadata, indent=2) + '\n')
        addresses = [item['networks'][network]['IPAddress'] for item in metadata]
        assert len(set(addresses)) == 2 and all(addresses), addresses
        assert all(not item['devices'] and not item['privileged'] for item in metadata)
        result = docker('wait', containers[1], capture_output=True, text=True, timeout=720)
        exit_code = int(result.stdout.strip())
        if exit_code:
            raise RuntimeError(f'customer PC validation exited {exit_code}')
        summary = json.loads((evidence / 'pc/result.json').read_text())
        assert summary['success']
    finally:
        for name in reversed(containers):
            with (evidence / (name + '.log')).open('w') as log:
                subprocess.run(['docker', 'logs', name], stdout=log, stderr=subprocess.STDOUT)
            subprocess.run(['docker', 'stop', '--time', '25', name], check=False)
            subprocess.run(['docker', 'rm', name], check=False)
        subprocess.run(['docker', 'network', 'rm', network], check=False)
    wheels = json.loads((evidence / 'pi/wheels-final.json').read_text())
    assert not any(wheels['torque'].values()) and not wheels['error'], wheels
    assert wheels['packets']['nonzero_speed'] > 0 and wheels['packets']['read'] > 0, wheels
    installations = {role: json.loads((evidence / role / 'installation.json').read_text())
                     for role in ('pc', 'pi')}
    assert installations['pc']['architecture'] == role_architectures['pc']
    assert installations['pi']['architecture'] == role_architectures['pi']
    assert len({item['commit'] for item in installations.values()}) == 1
    summary['network_addresses'] = addresses
    summary['test_harness_commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    summary['final_virtual_motor_registers'] = wheels
    summary['installations'] = installations
    (evidence / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
