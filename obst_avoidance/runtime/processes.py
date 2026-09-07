"""Cleanup for simulator groups explicitly created with start_new_session=True."""
import os
import signal
import time


def stop_process_group(process, grace_s=5.0):
    """Stop the owned group, including descendants orphaned by shell wrappers.

    The caller must have created this process with start_new_session=True.
    Waiting for only the ROS launch parent misses Gazebo/MAVProxy children.
    """
    if process.pid == os.getpgrp():
        raise ValueError('Refusing to signal our own process group')
    for sig, wait_s in ((signal.SIGINT, grace_s), (signal.SIGTERM, grace_s),
                        (signal.SIGKILL, 1.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.poll()
            return
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            process.poll()  # reap our direct child even if grandchildren remain
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    process.wait(timeout=1)
