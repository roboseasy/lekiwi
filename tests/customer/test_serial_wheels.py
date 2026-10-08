"""A blocked snapshot disk must not block virtual motor packets."""

import json
import os
from pathlib import Path
import select
import signal
import stat
import subprocess
from tempfile import TemporaryDirectory
import time
import unittest


class SerialSnapshotIsolationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = TemporaryDirectory(prefix='lekiwi-serial-test-')
        cls.addClassCleanup(cls.build.cleanup)
        cls.binary = Path(cls.build.name) / 'serial-wheels'
        subprocess.run(['g++', '-std=c++17', '-O2', '-static', '-pthread',
                        '-Wall', '-Wextra', '-Werror',
                        str(Path(__file__).with_name('serial_wheels.cpp')),
                        '-o', str(cls.binary)], check=True)

    def test_blocked_snapshot_does_not_delay_reply_and_final_state_is_flushed(self):
        with TemporaryDirectory(prefix='lekiwi-serial-state-') as directory:
            state = Path(directory) / 'state.json'
            temporary = Path(str(state) + '.tmp')
            port = Path(directory) / 'motor'
            process = subprocess.Popen(
                [str(self.binary), str(state), str(port), '8', '9', '7',
                 '1', '1', '1', '.03', '.1', '651.8986', '0', '0,0'],
                start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            serial = release = None
            try:
                deadline = time.monotonic() + 3
                while not state.exists():
                    self.assertIsNone(process.poll())
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.005)
                serial = os.open(port, os.O_RDWR | os.O_NOCTTY)
                os.mkfifo(temporary)
                time.sleep(.08)  # The next snapshot blocks opening the FIFO.
                packet = [255, 255, 8, 4, 2, 40, 1]
                packet.append((~sum(packet[2:])) & 255)
                started = time.monotonic()
                os.write(serial, bytes(packet))
                readable = select.select([serial], [], [], .05)[0]
                round_trip_ms = (time.monotonic() - started) * 1000
                self.assertTrue(readable, 'snapshot I/O blocked a motor response')
                self.assertEqual(os.read(serial, 7), bytes([255, 255, 8, 3, 0, 0, 244]))
                print(json.dumps({'blocked_snapshot_reply_ms': round_trip_ms}), flush=True)
                release = os.open(temporary, os.O_RDONLY | os.O_NONBLOCK)
                select.select([release], [], [], 1)
                os.read(release, 4096)
                deadline = time.monotonic() + 2
                while True:
                    # rename can briefly make state itself a FIFO; never block on it.
                    snapshot_fd = os.open(state, os.O_RDONLY | os.O_NONBLOCK)
                    try:
                        current = (json.loads(os.read(snapshot_fd, 4096))
                                   if stat.S_ISREG(os.fstat(snapshot_fd).st_mode) else {})
                    finally:
                        os.close(snapshot_fd)
                    if current.get('serial_timing', {}).get('max_snapshot_write_us', 0) > 5000:
                        break
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.005)
            finally:
                # Release blocked I/O even when running this regression on the old code.
                if release is None and temporary.exists() and stat.S_ISFIFO(temporary.stat().st_mode):
                    release = os.open(temporary, os.O_RDONLY | os.O_NONBLOCK)
                    select.select([release], [], [], 1)
                    os.read(release, 4096)
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    stdout, stderr = process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()
                    raise
                finally:
                    if serial is not None:
                        os.close(serial)
                    if release is not None:
                        os.close(release)
            self.assertEqual(process.returncode, 0, (stdout, stderr))
            final = json.loads(state.read_text())
            self.assertEqual(final['packets']['read'], 1)
            self.assertFalse(any(final['torque'].values()))
            self.assertGreater(final['serial_timing']['max_snapshot_write_us'], 5000)


if __name__ == '__main__':
    unittest.main()
