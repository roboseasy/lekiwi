"""Bounded shutdown for child process groups owned by mock integration checks."""

import os
import signal
import subprocess


def stop_owned_process(process, *, interrupt_timeout=12):
    """Only accept children started with start_new_session=True by the caller.

    ROS launch forwards SIGINT to its children itself. Signal its parent first
    to avoid duplicate interrupts; escalate to its owned group if needed.
    """
    if process.poll() is not None:
        return
    is_launch = (isinstance(process.args, (list, tuple))
                 and len(process.args) > 1 and process.args[1] == 'launch')
    for sig, timeout in ((signal.SIGINT, interrupt_timeout),
                         (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            if sig == signal.SIGINT and is_launch:
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass  # It may exit between poll() and signaling; still reap it.
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            if sig == signal.SIGKILL:
                raise


def cleanup_processes(processes, *, interrupt_timeout=12):
    """Attempt every child and log close, returning failures for the caller."""
    errors = []
    for process, log in reversed(processes):
        try:
            stop_owned_process(process, interrupt_timeout=interrupt_timeout)
        except BaseException as error:
            errors.append(error)
        finally:
            if log is not None:
                try:
                    log.close()
                except BaseException as error:
                    errors.append(error)
    return errors
