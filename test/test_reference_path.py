"""Unit tests for the goal-heading providers (Step CTRL-COMPARE).
Pure logic, no sim. Verifies heading/frame signs and cross-track recovery
from either side -- the safety-critical properties of the variant."""
import math

import pytest

from obst_avoidance.control.reference_path import WorldBearingGoal, ReferencePathGoal, _wrap_to_pi


# ---- baseline: world-bearing provider matches the old _goal_heading_body ----
def test_world_bearing_matches_negative_yaw_for_zero_bearing():
    g = WorldBearingGoal(goal_world_bearing_rad=0.0)
    for yaw in (-1.0, -0.3, 0.0, 0.4, 1.2):
        info = g.goal_heading((5.0, 0.0), yaw)
        assert info.goal_heading_body == pytest.approx(_wrap_to_pi(-yaw))
        assert info.cross_track_error is None


def test_world_bearing_is_position_independent():
    g = WorldBearingGoal(0.0)
    a = g.goal_heading((0.0, -5.0), 0.0).goal_heading_body
    b = g.goal_heading((99.0, 5.0), 0.0).goal_heading_body
    assert a == pytest.approx(b)  # baseline ignores where you are -> no recovery


# ---- reference path: on the line, aligned -> steer straight ----
def test_on_path_aligned_gives_zero_goal_heading():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)   # +x route at y=0
    info = g.goal_heading((10.0, 0.0), 0.0)               # on the line, facing +x
    assert info.goal_heading_body == pytest.approx(0.0, abs=1e-9)
    assert info.cross_track_error == pytest.approx(0.0, abs=1e-9)


# ---- cross-track SIGN + RECOVERY from either side (the key safety property) ----
def test_recovery_from_right_of_path_steers_left():
    # route +x at y=0; vehicle BELOW the line (y<0 = right of +x heading),
    # facing +x. It must steer LEFT (positive goal_heading, ENU CCW) to recover.
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    info = g.goal_heading((10.0, -1.0), 0.0)
    assert info.goal_heading_body > 0.0, "right of path must steer left (+)"
    assert info.cross_track_error < 0.0, "vehicle right of A->B => negative cte"


def test_recovery_from_left_of_path_steers_right():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    info = g.goal_heading((10.0, 1.0), 0.0)               # above line = left of +x
    assert info.goal_heading_body < 0.0, "left of path must steer right (-)"
    assert info.cross_track_error > 0.0, "vehicle left of A->B => positive cte"


def test_recovery_symmetric_magnitude():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    left = g.goal_heading((10.0, 1.0), 0.0).goal_heading_body
    right = g.goal_heading((10.0, -1.0), 0.0).goal_heading_body
    assert left == pytest.approx(-right, abs=1e-9)


def test_larger_offset_demands_sharper_correction():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    near = abs(g.goal_heading((10.0, -0.5), 0.0).goal_heading_body)
    far = abs(g.goal_heading((10.0, -2.5), 0.0).goal_heading_body)
    assert far > near, "further off the path => larger heading correction"


# ---- route not along +x: bearing accounts for the true tangent + yaw ----
def test_diagonal_route_bearing():
    # route heading 45deg; vehicle at A facing 0 -> goal ~ +45deg in body frame
    g = ReferencePathGoal(0, 0, 30, 30, lookahead_m=3.0)
    info = g.goal_heading((0.0, 0.0), 0.0)
    assert info.goal_heading_body == pytest.approx(math.radians(45), abs=1e-6)
    # facing along the route -> zero body heading
    info2 = g.goal_heading((0.0, 0.0), math.radians(45))
    assert info2.goal_heading_body == pytest.approx(0.0, abs=1e-6)


# ---- yaw composition: rotating the body rotates the goal bearing opposite ----
def test_goal_heading_tracks_yaw():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    on_line = (10.0, 0.0)
    base = g.goal_heading(on_line, 0.0).goal_heading_body       # ~0
    turned = g.goal_heading(on_line, 0.5).goal_heading_body     # yawed +0.5 (left)
    # goal is still world +x, so in body frame it now appears to the RIGHT
    assert turned == pytest.approx(base - 0.5, abs=1e-6)


# ---- missing odom: falls back to tangent-following, never crashes ----
def test_missing_position_follows_tangent():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    info = g.goal_heading(None, 0.0)
    assert info.goal_heading_body == pytest.approx(0.0, abs=1e-9)  # tangent is +x
    assert info.cross_track_error is None


# ---- past the end: lookahead clamps to B, no overshoot past goal ----
def test_lookahead_clamps_at_endpoint():
    g = ReferencePathGoal(0, 0, 30, 0, lookahead_m=3.0)
    info = g.goal_heading((29.0, 0.0), 0.0)
    assert info.lookahead_xy[0] == pytest.approx(30.0, abs=1e-9)  # clamped to B.x


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))


@pytest.mark.parametrize('index', range(5))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_route_parameters_rejected(index, value):
    args = [0.0, 0.0, 30.0, 0.0, 3.0]
    args[index] = value
    with pytest.raises(ValueError, match='finite'):
        ReferencePathGoal(*args)
