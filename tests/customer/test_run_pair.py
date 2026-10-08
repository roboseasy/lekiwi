"""Preserve live network evidence before the PC container exits."""

import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import run_pair


class PairNetworkEvidenceTest(unittest.TestCase):
    def test_successful_pc_exit_does_not_erase_network_evidence(self):
        with TemporaryDirectory(prefix='lekiwi-pair-test-') as directory:
            root = Path(directory)
            for role in ('pc', 'pi'):
                target = root / 'evidence' / role
                target.mkdir(parents=True)
                (target / 'installation.json').write_text(json.dumps(
                    {'architecture': 'amd64', 'commit': 'installed-commit'}))
            (root / 'evidence/pc/result.json').write_text('{"success": true}')
            (root / 'evidence/pi/wheels-final.json').write_text(json.dumps(
                {'torque': {'8': False, '9': False, '7': False}, 'error': None,
                 'packets': {'nonzero_speed': 1, 'read': 3}}))
            exited = False
            inspections = []

            def execute(command, **_kwargs):
                nonlocal exited
                if command[:2] == ['docker', 'wait']:
                    exited = True
                    return SimpleNamespace(stdout='0\n')
                if command[:2] == ['docker', 'inspect']:
                    inspections.append(exited)
                    name = command[2]
                    role = name.rsplit('-', 1)[1]
                    ip = '' if exited and role == 'pc' else '172.18.0.' + ('2' if role == 'pi' else '3')
                    data = {'Config': {'Image': 'lekiwi-customer:amd64', 'Hostname': role},
                            'NetworkSettings': {'Networks': {name.rsplit('-', 1)[0] + '-lan':
                                                            {'IPAddress': ip}}},
                            'HostConfig': {'Devices': [], 'Privileged': False, 'CpusetCpus': '0'}}
                    return SimpleNamespace(stdout=json.dumps([data]))
                return SimpleNamespace(stdout='')

            original = Path.cwd()
            try:
                os.chdir(root)
                with patch.object(sys, 'argv', ['run_pair', '--arch', 'amd64']), \
                        patch.object(run_pair.os, 'sched_getaffinity', return_value={0, 1}), \
                        patch.object(run_pair.tempfile, 'mkdtemp', return_value=str(root / 'binary')), \
                        patch.object(run_pair.subprocess, 'run', side_effect=execute), \
                        patch.object(run_pair.subprocess, 'check_output', return_value='harness-commit\n'), \
                        patch.object(run_pair, 'prepare_virtual_actuator'):
                    run_pair.main()
            finally:
                os.chdir(original)
            self.assertEqual(inspections, [False, False])
            report = json.loads((root / 'evidence/result.json').read_text())
            self.assertEqual(report['network_addresses'], ['172.18.0.2', '172.18.0.3'])


if __name__ == '__main__':
    unittest.main()
