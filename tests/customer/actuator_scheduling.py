"""Give only the virtual motor process priority over the simulated Pi workload."""

import json
import os
from pathlib import Path
import subprocess
import time


def prepare_virtual_actuator(container, docker, evidence, run_name):
    deadline = time.monotonic() + 10
    while True:
        listing = docker('top', container, '-eo', 'pid,args',
                         capture_output=True, text=True).stdout
        processes = [line.split() for line in listing.splitlines()[1:]]
        candidates = [int(fields[0]) for fields in processes
                      if len(fields) > 1 and fields[1] == '/customer-serial-wheels']
        if len(candidates) > 1:
            raise RuntimeError('multiple virtual actuators in the Pi container')
        if candidates:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('virtual actuator did not appear in the Pi container')
        time.sleep(0.02)
    pid = candidates[0]
    # Disposable CI runners already provide sudo. Do not grant additional
    # capabilities to either container or change the ROS driver's deadlines.
    subprocess.run(['sudo', '-n', 'renice', '-n', '-20', '-p', str(pid)], check=True)
    priority = os.getpriority(os.PRIO_PROCESS, pid)
    if priority != -20:
        raise RuntimeError(f'virtual actuator priority was not applied: {priority}')
    report = {'run': run_name, 'host_pid': pid, 'nice': priority,
              'scope': 'virtual actuator only; ROS processes retain their priorities'}
    path = Path(evidence) / 'actuator-scheduling.json'
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)
