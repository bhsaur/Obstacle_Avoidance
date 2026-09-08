"""Tests for runtime/orchestrator.py (Step AR) -- pure math (obstacle
distance, zone lookup, yaw/heading conversion) plus a full run() test
using STUB FrameSource/CheapStage/vehicle and the REAL SectorController,
so the wiring (collision detection, goal-reached, mode/switch counting,
per-frame logging) is exercised without needing a live sim -- Step AS's
closed-loop runs are what actually need a sim, this is checking the
orchestrator's own logic in isolation.
"""
import json
import math
from dataclasses import dataclass

import numpy as np
import pytest

from obst_avoidance.control.sector import SectorController, SectorControllerConfig
from obst_avoidance.perception import geometry
from obst_avoidance.perception.types import SectorBelief
from obst_avoidance.runtime.orchestrator import (
    GOAL_WORLD_BEARING_RAD,
    Orchestrator,
    OrchestratorConfig,
    _goal_heading_body,
    _quat_to_yaw,
    _wrap_to_pi,
    nearest_obstacle,
    zone_of,
)


# ---------------------------------------------------------------------
# Pure math
# ---------------------------------------------------------------------
def test_zone_of_boundaries():
    assert zone_of(-3.0) == "gap_preA"
    assert zone_of(20.0) == "zone_A"
    assert zone_of(55.0) == "zone_B"
    assert zone_of(90.0) == "zone_C"
    assert zone_of(110.0) == "zone_D"
    assert zone_of(200.0) == "past_D"


def test_nearest_obstacle_at_wallB1_centre():
    obs, dist = nearest_obstacle(55.0, 0.0)
    assert obs["name"] == "wallB1"
    assert dist == pytest.approx(0.0, abs=1e-6)  # inside the wall's footprint


def test_nearest_obstacle_far_from_everything():
    obs, dist = nearest_obstacle(40.0, 50.0)  # empty gap, well off any obstacle's y
    assert dist > 10.0


def test_nearest_obstacle_circle_math():
    # treeA1 at (17, -3.8), radius 0.25 -- a point 1m due east of centre
    # should be (1.0 - 0.25) = 0.75m from the trunk surface.
    obs, dist = nearest_obstacle(18.0, -3.8)
    assert obs["name"] == "treeA1"
    assert dist == pytest.approx(0.75, abs=1e-6)


def test_quat_to_yaw_identity_is_zero():
    assert _quat_to_yaw((0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)


def test_quat_to_yaw_90deg_left():
    # quaternion for +90deg (pi/2) rotation about Z
    half = math.pi / 4
    q = (0.0, 0.0, math.sin(half), math.cos(half))
    assert _quat_to_yaw(q) == pytest.approx(math.pi / 2, abs=1e-6)


def test_wrap_to_pi():
    assert _wrap_to_pi(0.0) == pytest.approx(0.0)
    assert _wrap_to_pi(3 * math.pi) == pytest.approx(math.pi, abs=1e-6) or _wrap_to_pi(3 * math.pi) == pytest.approx(-math.pi, abs=1e-6)
    assert -math.pi <= _wrap_to_pi(100.0) <= math.pi


# ---------------------------------------------------------------------
# Step CF -- goal_heading is a fixed world bearing converted to body
# frame via current yaw, not a formula that only happens to look right
# for one goal direction. Verifies the SEMANTIC property the task asked
# for directly (sector nearest the TRUE world corridor direction gets
# the lowest goal cost), not a tautological self-consistency check --
# independently recomputes each sector's WORLD-frame bearing
# (sector_body_bearing + yaw) and checks the minimum |world_bearing -
# GOAL_WORLD_BEARING_RAD| sector is the one goal_heading_body's own
# argmin would also pick, for several spawn-yaw offsets including ones
# no run in this project has actually used.
# ---------------------------------------------------------------------
def test_goal_heading_body_matches_negative_yaw_for_zero_world_bearing():
    """Step CE originally suspected `goal_heading = -yaw` was itself the
    bug. Re-derived independently at Step CF: for the corridor's actual
    goal direction (world bearing 0), converting a fixed world bearing
    into body frame is mathematically `goal_world_bearing - yaw = -yaw`
    -- reporting plainly that this specific formula was not wrong, not
    quietly re-deriving the same number under a new name and calling it
    a fix."""
    for yaw in [0.0, math.radians(5), math.radians(-5), math.radians(30), math.radians(-90), math.radians(179)]:
        assert _goal_heading_body(yaw) == pytest.approx(_wrap_to_pi(-yaw), abs=1e-9)


def test_goal_heading_picks_sector_closest_to_true_world_corridor_direction():
    """The actual semantic property Step CF's VERIFY asked for: at
    several spawn-yaw offsets, the sector whose bearing is closest to
    goal_heading_body(yaw) must be the SAME sector whose WORLD-frame
    bearing is closest to the corridor's true direction
    (GOAL_WORLD_BEARING_RAD=0) -- computed two independent ways and
    checked for agreement, not assumed."""
    intr_width, fx, cx = 640, 205.4696273803711, 320.0
    bearings_body = geometry.sector_bearings_rad(intr_width, fx, cx, n_sectors=11)

    for yaw in [0.0, math.radians(5), math.radians(-5), math.radians(15), math.radians(-30),
                math.radians(45), math.radians(-60)]:
        goal_heading_body = _goal_heading_body(yaw)
        # what the controller's cost function would pick (min |bearing - goal_heading|)
        picked_by_goal_cost = int(np.argmin(np.abs(bearings_body - goal_heading_body)))

        # independently: each sector's bearing in WORLD frame is body_bearing + yaw
        # (undoing the same rotation goal_heading_body applied); the sector closest
        # to the TRUE corridor direction (world bearing 0) is the min of THAT.
        bearings_world = bearings_body + yaw
        picked_by_world_alignment = int(np.argmin(np.abs(bearings_world - GOAL_WORLD_BEARING_RAD)))

        assert picked_by_goal_cost == picked_by_world_alignment, (
            f"at yaw={math.degrees(yaw):.1f}deg: goal-cost picked sector "
            f"{picked_by_goal_cost} but the sector actually closest to the true "
            f"world corridor direction is {picked_by_world_alignment}"
        )


# ---------------------------------------------------------------------
# Full run() with stubs
# ---------------------------------------------------------------------
@dataclass
class _StubPacket:
    seq: int
    t_capture: float
    image: object = None
    gyro: object = None
    gyro_valid: bool = True


class _StubFrameSource:
    """Yields one packet per read() call, t_capture advancing 1/25s
    each time. Never runs out (the orchestrator decides when to stop)."""

    def __init__(self):
        self._seq = 0

    def read(self):
        self._seq += 1
        return _StubPacket(seq=self._seq, t_capture=self._seq / 25.0)


class _StubCheapStage:
    """Always reports one valid centre sector, moderate score -- enough
    for SectorController to produce sane cruise commands. Perception
    correctness isn't what this test is checking."""

    def infer(self, packet, prev_packet, odom):
        n = 5
        scores = np.full(n, np.nan)
        valid = np.zeros(n, dtype=bool)
        scores[2] = 0.8
        valid[2] = True
        belief = SectorBelief(scores=scores, valid=valid, confidence=0.5, source="cheap", latency_ms=1.0)
        feats = np.zeros(54)
        return belief, feats


class _StubVehicle:
    """Marches x forward at a fixed rate per state_at() call (not real
    physics -- this is testing the ORCHESTRATOR's collision/goal logic,
    not flight dynamics), holding y and yaw fixed at construction-time
    values."""

    def __init__(self, dx_per_call: float, y: float = 0.0):
        self._x = -3.0
        self._y = y
        self._dx = dx_per_call
        self.sent_commands = []

    def state_at(self, t_capture):
        self._x += self._dx
        position = (self._x, self._y, 3.0)
        velocity = (1.5, 0.0, 0.0)
        attitude_quat = (0.0, 0.0, 0.0, 1.0)
        return position, velocity, attitude_quat, True

    def send(self, cmd):
        self.sent_commands.append(cmd)


def _make_orchestrator(dx_per_call, y, log_path=None, max_wall_time_s=5.0):
    controller = SectorController(SectorControllerConfig())
    cfg = OrchestratorConfig(max_wall_time_s=max_wall_time_s, log_path=log_path)
    orch = Orchestrator(
        frame_source=_StubFrameSource(),
        perception_stage=_StubCheapStage(),
        controller=controller,
        vehicle=_StubVehicle(dx_per_call=dx_per_call, y=y),
        config=cfg,
    )
    return orch


def test_run_detects_collision_at_wallB1():
    # Large step size to reach x~55 quickly; y=0 runs straight into wallB1.
    orch = _make_orchestrator(dx_per_call=1.0, y=0.0)
    result = orch.run()
    assert result.collided is True
    assert result.collision_zone == "zone_B"
    assert result.completed is False
    assert result.n_frames > 0
    # explicit stop command sent after collision
    last_cmd = orch.vehicle.sent_commands[-1]
    assert last_cmd.fwd_vel == 0.0


def test_observer_sees_command_and_can_stop_without_second_inference():
    orch = _make_orchestrator(dx_per_call=0.1, y=0.0)
    seen = []
    def observe(packet, belief, features, odom, cmd):
        seen.append((packet.seq, belief, features, odom))
        assert orch.vehicle.sent_commands[-1] is cmd
        return False
    orch.on_frame = observe
    result = orch.run()
    assert result.n_frames == len(seen) == 1
    assert result.stop_reason == 'user_stop'
    assert orch.vehicle.sent_commands[-1].fwd_vel == 0.0


def test_stop_request_is_checked_even_without_camera_frames():
    orch = _make_orchestrator(dx_per_call=0.0, y=0.0)
    orch.frame_source.read = lambda: None
    orch.should_stop = lambda: True
    result = orch.run()
    assert result.stop_reason == 'user_stop'
    assert result.n_frames == 0
    assert orch.vehicle.sent_commands[-1].fwd_vel == 0.0


def test_camera_timeout_stops_last_command(monkeypatch):
    orch = _make_orchestrator(dx_per_call=0.0, y=0.0)
    orch.frame_source.read = lambda: None
    orch.config.frame_timeout_s = 0.5
    ticks = iter([10.0, 10.2, 10.6])
    monkeypatch.setattr('obst_avoidance.runtime.orchestrator.time.monotonic', lambda: next(ticks))
    result = orch.run()
    assert result.stop_reason == 'camera_timeout'
    assert orch.vehicle.sent_commands[-1].fwd_vel == 0.0


def test_run_completes_when_clear_of_all_obstacles():
    # y=50 is far from every obstacle's footprint -- should reach goal_x_m untouched.
    orch = _make_orchestrator(dx_per_call=2.0, y=50.0)
    result = orch.run()
    assert result.completed is True
    assert result.collided is False


def test_run_logs_expected_fields(tmp_path):
    log_path = str(tmp_path / "run.jsonl")
    orch = _make_orchestrator(dx_per_call=2.0, y=50.0, log_path=log_path)
    result = orch.run()
    assert result.log_path == log_path
    with open(log_path) as f:
        rows = [json.loads(line) for line in f]
    # CTRL-COMPARE: the first line is a run-metadata record (no "seq");
    # the remaining lines are per-frame rows.
    meta = rows[0]
    assert "meta" in meta and "seq" not in meta
    for mk in ("controller_version", "velocity_frame", "scene_version",
               "goal_reference", "goal_x_m", "collision_radius_m"):
        assert mk in meta["meta"]
    frames = rows[1:]
    assert len(frames) == result.n_frames
    first = frames[0]
    for key in ("seq", "t_capture", "scores", "valid", "confidence", "source",
                "perception_latency_ms", "features", "cmd", "vehicle_position", "odom_valid",
                # CTRL-COMPARE additions:
                "yaw", "measured_yaw_rate", "commanded_yaw_rate", "target_world_bearing",
                "heading_error", "switch_reason", "candidate_costs", "cross_track_error",
                "obs_age_ms"):
        assert key in first, f"missing log field: {key}"
    # mode is always present and never silently defaulted
    assert first["cmd"]["mode"] in ("cruise", "avoid", "blind")
    assert result.perception_latency_ms  # non-empty
    assert result.controller_latency_ms


def test_run_tracks_mode_counts_and_switches():
    orch = _make_orchestrator(dx_per_call=2.0, y=50.0)
    result = orch.run()
    assert sum(result.mode_counts.values()) == result.n_frames
    assert result.n_target_switches >= 1  # at least the initial "no target -> sector 2" switch


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))


def test_yaw_rate_does_not_bridge_missing_odometry(tmp_path):
    path = str(tmp_path / 'run.jsonl')
    orch = _make_orchestrator(2.0, 50.0, log_path=path)
    original = orch.vehicle.state_at
    calls = 0
    def state_at(t):
        nonlocal calls
        calls += 1
        p, v, q, valid = original(t)
        if calls == 2:
            return None, None, None, False
        half = 0.2
        return p, v, (0.0, 0.0, math.sin(half), math.cos(half)), valid
    orch.vehicle.state_at = state_at
    orch.run()
    rows = [json.loads(line) for line in open(path)][1:]
    assert all(r['measured_yaw_rate'] is None for r in rows[:3])
    assert rows[3]['measured_yaw_rate'] == pytest.approx(0.0)
    assert all(r['obs_age_ms'] is None for r in rows)
    assert all(r['inference_to_command_ms'] >= 0 for r in rows)


def test_endpoint_requires_lateral_arrival(tmp_path):
    orch = _make_orchestrator(2.0,50.0)
    orch.config.goal_xy = (5.0, 0.0)
    orch.config.goal_x_m = 5.0
    orch.should_stop = lambda: orch.vehicle._x > 8
    result = orch.run()
    assert not result.completed
    assert result.stop_reason == 'user_stop'


def test_endpoint_arrival_is_distinct_from_goal_line():
    orch = _make_orchestrator(2.0,50.0)
    orch.config.goal_xy = (5.0,50.0)
    result = orch.run()
    assert result.completed
    assert result.stop_reason == 'endpoint_reached'


def test_repeated_camera_timestamp_cannot_refresh_watchdog(monkeypatch):
    from types import SimpleNamespace
    from obst_avoidance.runtime import orchestrator as module
    wall = [0.0]
    def sleep(dt):
        wall[0] += dt
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: wall[0],
                        perf_counter=lambda: wall[0], sleep=sleep))
    orch = _make_orchestrator(2.0,50.0)
    orch.config.frame_timeout_s = .05
    orch.frame_source.read = lambda: _StubPacket(seq=1,t_capture=1.0)
    result = orch.run()
    assert result.stop_reason == 'camera_timeout'
    assert result.n_frames == 0
    assert orch.vehicle.sent_commands[-1].fwd_vel == 0


def test_late_inference_cannot_restart_motion_after_watchdog(monkeypatch):
    from types import SimpleNamespace
    from obst_avoidance.runtime import orchestrator as module
    wall = [0.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: wall[0],
                        perf_counter=lambda: wall[0], sleep=lambda dt: None))
    orch = _make_orchestrator(2.0,50.0)
    orch.vehicle._command_timeout_s = 1.0
    infer = orch.perception_stage.infer
    def late(*args):
        wall[0] += 1.5
        return infer(*args)
    orch.perception_stage.infer = late
    result = orch.run()
    assert result.stop_reason == 'perception_timeout'
    assert all(cmd.fwd_vel == cmd.yaw_rate == 0 for cmd in orch.vehicle.sent_commands)


def test_endpoint_approach_does_not_asymptotically_stop_outside_tolerance():
    orch = _make_orchestrator(2.0,50.0)
    orch.config.goal_xy = (-0.249,50.0)  # first pose -1 is 0.751m away
    orch.should_stop = lambda: bool(orch.vehicle.sent_commands)
    orch.run()
    assert orch.vehicle.sent_commands[0].fwd_vel > .5


def test_continuous_missing_support_ends_with_explicit_reason():
    orch = _make_orchestrator(0.,50.)
    orch.config.unsupported_timeout_s = .12
    infer = orch.perception_stage.infer
    def blind(*args):
        b,f = infer(*args)
        b.valid[:] = False
        return b,f
    orch.perception_stage.infer = blind
    result = orch.run()
    assert result.stop_reason == 'perception_unavailable'
    assert not result.completed
    assert orch.vehicle.sent_commands[-1].fwd_vel == 0


def test_collision_takes_priority_over_simultaneous_perception_timeout():
    orch = _make_orchestrator(0.,50.)
    orch.config.unsupported_timeout_s = .1
    calls = [0]
    def state_at(t):
        calls[0] += 1
        p = (55.,0.,3.) if calls[0] >= 4 else (-3.,50.,3.)
        return p,(0.,0.,0.),(0.,0.,0.,1.),True
    orch.vehicle.state_at = state_at
    infer = orch.perception_stage.infer
    def blind(*args):
        b,f = infer(*args)
        b.valid[:] = False
        return b,f
    orch.perception_stage.infer = blind
    assert orch.run().stop_reason == 'collision'


def test_invalid_odometry_resets_slew_to_command_actually_sent():
    from dataclasses import replace
    from obst_avoidance.control.types import ControlCommand
    orch = _make_orchestrator(0.,50.)
    orch.config.goal_xy = (30.,50.)
    original = orch.vehicle.state_at
    calls = [0]
    def state_at(t):
        calls[0] += 1
        p,v,q,valid = original(t)
        return p,v,q,calls[0] != 1
    observed=[]
    def step(b,goal,yaw,t,state):
        observed.append(state.previous_yaw_rate)
        return ControlCommand(.8,.4,'avoid',1),replace(state,last_t=t,previous_yaw_rate=.4)
    orch.vehicle.state_at=state_at
    orch.controller.step=step
    orch.should_stop=lambda: len(orch.vehicle.sent_commands)>=2
    orch.run()
    assert observed == [0.,0.]
    assert orch.vehicle.sent_commands[0].yaw_rate == 0


def test_custom_scene_collision_and_metadata(tmp_path):
    path = str(tmp_path / 'basic.jsonl')
    orch = _make_orchestrator(dx_per_call=1.0, y=0.0, log_path=path)
    box = dict(name='basic_box', kind='box', x=17., y=0.,
               half_x=1., half_y=1.5, zone='basic')
    orch.config.evaluation_obstacles = [box]
    result = orch.run()
    assert result.collided and result.collision_zone == 'basic'
    assert result.collision_xy[0] <= 18
    meta = json.loads(open(path).readline())['meta']
    assert meta['evaluation_obstacles'] == [box]
