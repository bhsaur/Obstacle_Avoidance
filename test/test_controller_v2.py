"""Regression cases from the controller review; baseline remains selectable."""
import math

import numpy as np
import pytest

from obst_avoidance.control import SectorController, SectorControllerConfig, ControlState
from obst_avoidance.perception.types import SectorBelief


def make_belief(winner):
    scores = np.full(5, 0.1)
    scores[winner] = 1.0
    return SectorBelief(scores, np.ones(5, dtype=bool), 1.0, 'cheap', 0.0)


def controller():
    return SectorController(SectorControllerConfig(robust_hysteresis=True))


def held():
    return ControlState(0.0, 1.0, 'cheap', 0.0)


def test_alternating_challengers_do_not_combine_confirmation():
    c, s = controller(), held()
    for t, winner in [(0.04, 1), (0.08, 3), (0.12, 1), (0.16, 3), (0.20, 1)]:
        cmd, s = c.step(make_belief(winner), 0.0, 0.0, t, s)
        assert cmd.target_sector == 2
    cmd, s = c.step(make_belief(1), 0.0, 0.0, 0.33, s)
    assert cmd.target_sector == 1


def test_slow_first_observation_does_not_inherit_unobserved_time():
    cmd, s = controller().step(make_belief(1), 0.0, 0.0, 1.25, held())
    assert cmd.target_sector == 2
    assert s.challenger_since == 1.25


def test_unseen_held_direction_is_replaced_by_observed_direction():
    s = ControlState(2.0, 1.0, 'cheap', 0.0)
    cmd, s = controller().step(make_belief(0), 0.0, 0.0, 0.04, s)
    assert abs(s.target_world_bearing) < math.radians(40)
    assert cmd.telemetry.is_new_commitment


@pytest.mark.parametrize('t', [0.0, 0.04])
def test_capture_clock_reset_restarts_challenge(t):
    c = controller()
    _, s = c.step(make_belief(1), 0.0, 0.0, 0.04, held())
    cmd, s = c.step(make_belief(1), 0.0, 0.0, t, s)
    assert cmd.target_sector == 2
    assert s.challenger_since == t
