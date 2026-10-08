"""Verify virtual-desktop key delivery before attempting the customer ROS journey."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from gui_input import focus_tk_window, key_event, visible_windows


def child(events):
    import tkinter as tk
    root = tk.Tk(className='lekiwi_guarded_base_teleop')
    root.title('LeKiwi 텔레옵 · 키를 누르는 동안만 이동')
    root.geometry('460x200')
    tk.Label(root, text='W/S · Space · Esc').pack(pady=30)

    def record(kind, event):
        with events.open('a') as output:
            output.write(json.dumps({'kind': kind, 'key': event.keysym}) + '\n')
        if kind == 'press' and event.keysym == 'Escape':
            root.quit()

    root.bind('<KeyPress>', lambda event: record('press', event))
    root.bind('<KeyRelease>', lambda event: record('release', event))
    root.update()
    root.focus_force()
    root.mainloop()
    root.destroy()


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--child':
        child(Path(sys.argv[2]))
        return
    evidence = Path('evidence')
    evidence.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='lekiwi-gui-input-') as directory:
        events = Path(directory) / 'events.jsonl'
        environment = dict(os.environ, DISPLAY=':99')
        display = subprocess.Popen(['Xvfb', ':99', '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   start_new_session=True)
        application = None
        try:
            deadline = time.monotonic() + 10
            while not Path('/tmp/.X11-unix/X99').exists():
                assert display.poll() is None and time.monotonic() < deadline
                time.sleep(0.02)
            os.environ['DISPLAY'] = ':99'
            application = subprocess.Popen([sys.executable, __file__, '--child', str(events)],
                                           env=environment, start_new_session=True)
            while not visible_windows('^LeKiwi '):
                assert application.poll() is None and time.monotonic() < deadline
                time.sleep(0.02)
            client, children = focus_tk_window(visible_windows('^LeKiwi ')[0])
            (evidence / 'gui-window.txt').write_text(children)
            time.sleep(0.2)
            key_event(client, 'w', pressed=True)
            time.sleep(0.3)
            key_event(client, 'w', pressed=False)
            time.sleep(0.2)
            records = [json.loads(line) for line in events.read_text().splitlines()] if events.exists() else []
            assert {'kind': 'press', 'key': 'w'} in records, records
            assert {'kind': 'release', 'key': 'w'} in records, records
            key_event(client, 'Escape', pressed=True)
            application.wait(timeout=3)
            assert application.returncode == 0
            (evidence / 'gui-input.json').write_text(json.dumps(records, indent=2) + '\n')
            print('Visible Tk client receives press/release and Esc: passed')
        finally:
            with (evidence / 'gui-tree.txt').open('w') as log:
                subprocess.run(['xwininfo', '-root', '-tree'], stdout=log, check=False)
            for process in (application, display):
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)


if __name__ == '__main__':
    main()
