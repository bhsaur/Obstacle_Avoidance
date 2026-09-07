"""Startup timeout regressions without initializing ROS or opening sockets."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from obst_avoidance.frame_source import sim


def test_missing_clock_is_bounded(monkeypatch):
    times = iter([0, 0, 2])
    monkeypatch.setattr(sim, 'time', SimpleNamespace(monotonic=lambda: next(times), sleep=lambda _: None))
    fake = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
    with pytest.raises(TimeoutError, match='/clock'):
        sim.SimFrameSource._assert_initial_offset(fake, timeout_s=1)


def test_missing_heartbeat_is_not_ignored(monkeypatch):
    mav = Mock()
    mav.wait_heartbeat.return_value = None
    monkeypatch.setattr(sim.mavutil, 'mavlink_connection', lambda _: mav)
    fake = SimpleNamespace()
    with pytest.raises(TimeoutError, match='heartbeat'):
        sim.SimFrameSource._connect_mavlink(fake, 'udp:127.0.0.1:14551', 1, 1)
    mav.mav.param_set_send.assert_not_called()
