"""X11 input for the actual Tk client, separate from its window-manager wrapper."""

import re
import subprocess


def visible_windows(pattern):
    result = subprocess.run(['xdotool', 'search', '--onlyvisible', '--name', pattern],
                            capture_output=True, text=True)
    return result.stdout.splitlines() if result.returncode == 0 else []


def focus_tk_window(window):
    children = subprocess.check_output(['xwininfo', '-id', window, '-children'], text=True)
    child_ids = re.findall(r'^\s+(0x[0-9a-fA-F]+)\s', children, re.M)
    client = child_ids[0] if child_ids else window
    subprocess.run(['xdotool', 'windowfocus', '--sync', client], check=True)
    subprocess.run(['xdotool', 'mousemove', '--window', window, '120', '100', 'click', '1'], check=True)
    return client, children


def key_event(client, key, *, pressed):
    subprocess.run(['xdotool', 'keydown' if pressed else 'keyup', '--window', client, key], check=True)
