"""Exercise cleanup of a child that outlives its ROS-like wrapper."""
import os
from pathlib import Path
import subprocess
import sys
import time

from obst_avoidance.runtime.processes import stop_process_group


def test_cleanup_reaches_descendant_after_wrapper_exits(tmp_path):
    pid_file = tmp_path / 'child.pid'
    child_code = ('import os, signal, time; from pathlib import Path; '
                  'signal.signal(signal.SIGINT, signal.SIG_IGN); '
                  f'Path({str(pid_file)!r}).write_text(str(os.getpid())); '
                  'time.sleep(30)')
    wrapper_code = f'import subprocess, sys; subprocess.Popen([sys.executable, "-c", {child_code!r}])'
    wrapper = subprocess.Popen([sys.executable, '-c', wrapper_code], start_new_session=True)
    try:
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert pid_file.exists()
        child_pid = int(pid_file.read_text())
        wrapper.wait(timeout=5)
        assert os.getpgid(child_pid) == wrapper.pid
        stop_process_group(wrapper, grace_s=0.1)
        stat = Path(f'/proc/{child_pid}/stat')
        assert not stat.exists() or stat.read_text().split(') ')[1].startswith('Z ')
    finally:
        stop_process_group(wrapper, grace_s=0.1)
