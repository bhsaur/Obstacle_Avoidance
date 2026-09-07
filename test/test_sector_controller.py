"""Unit tests for SectorController (Step AO) -- pure logic, no sim, no
FrameSource. Constructs SectorBelief directly to exercise each
documented behavior: candidate masking, hysteresis (including the
source-switch bypass), mode selection, and the constant-speed policy.
"""
import math

import numpy as np
import pytest

from obst_avoidance.control import ControlState, SectorController, SectorControllerConfig
from obst_avoidance.perception.types import SectorBelief

N = 5


def belief(scores, valid, source="cheap", confidence=1.0):
    return SectorBelief(
        scores=np.array(scores, dtype=float),
        valid=np.array(valid, dtype=bool),
        confidence=confidence,
        source=source,
        latency_ms=0.0,
    )


def fresh_state():
    return ControlState(target_world_bearing=None, seconds_on_target=0.0, last_source=None, last_t=None)


def make_controller(**overrides):
    cfg = SectorControllerConfig(n_sectors=N, horizontal_fov_rad=math.radians(80.0), **overrides)
    return SectorController(cfg), cfg


# ---------------------------------------------------------------------
# Candidate masking
# ---------------------------------------------------------------------
def test_invalid_sector_never_selected_even_with_best_raw_score():
    """An invalid sector must be excluded from candidacy entirely, not
    scored zero -- give it the best possible score (high tau) and
    confirm it is still never chosen."""
    ctrl, _ = make_controller()
    b = belief(scores=[1.0, 1.0, 1.0, 1.0, 100.0], valid=[True, True, True, True, False])
    cmd, state = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector != 4
    assert cmd.target_sector in (0, 1, 2, 3)


def test_all_invalid_is_blind():
    ctrl, cfg = make_controller()
    b = belief(scores=[0.0] * N, valid=[False] * N)
    cmd, state = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.mode == "blind"
    assert cmd.target_sector is None
    assert cmd.yaw_rate == 0.0
    assert cmd.fwd_vel == cfg.fwd_vel  # constant speed, not decelerated, by default
    assert state.target_world_bearing is None


def test_blind_mode_decelerate_ablation_flag():
    ctrl, cfg = make_controller(decelerate_when_blind=True, blind_fwd_vel=0.5)
    b = belief(scores=[0.0] * N, valid=[False] * N)
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.mode == "blind"
    assert cmd.fwd_vel == 0.5


# ---------------------------------------------------------------------
# Selection / obstacle-dead-centre sanity (no interpolation)
# ---------------------------------------------------------------------
def test_obstacle_dead_centre_does_not_steer_straight_ahead():
    """Centre sector dangerous, both flanks clear -- argmin must pick a
    SIDE sector, never sector 2 (the classic potential-field failure a
    score-weighted centroid would produce)."""
    ctrl, _ = make_controller()
    b = belief(scores=[10.0, 10.0, 0.1, 10.0, 10.0], valid=[True] * N)
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector != 2
    assert cmd.mode == "avoid"  # target != centre


# ---------------------------------------------------------------------
# Hysteresis (Step CQ: time-based, hysteresis_min_seconds, evaluated
# against t_capture -- NOT wall clock, NOT a raw frame count. All tests
# below use a 25Hz-equivalent dt=0.04s between steps unless testing rate
# independence specifically, so hysteresis_min_seconds=0.12 (the Step CQ
# converted default, 3 frames at CheapStage's ~25Hz) reproduces the
# exact same "3 confirming frames" behaviour these tests exercised
# before the conversion -- see sector.py's own module comments for why.
# ---------------------------------------------------------------------
def test_hysteresis_blocks_single_frame_switch():
    """A challenger that beats the current target by the margin on only
    ONE frame (0.04s at 25Hz, well under min_seconds=0.12) must not
    cause an immediate switch."""
    ctrl, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.15)
    b_initial = belief(scores=[5.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    cmd, state = ctrl.step(b_initial, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector

    # sector 0 now costs much less (much higher score) -- one frame only
    b_challenge = belief(scores=[50.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    cmd2, state2 = ctrl.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=0.04, state=state)
    assert cmd2.target_sector == initial_target  # not switched yet
    # seconds_on_target uses a sign convention (positive=dwelling,
    # negative=in-progress challenge streak magnitude, see sector.py) --
    # this is dt=0.04s into a fresh challenge streak, so -0.04, not a
    # continuation of the prior dwell time.
    assert state2.seconds_on_target == pytest.approx(-0.04)


def test_hysteresis_switches_after_min_consecutive_frames():
    ctrl, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.15)
    b_initial = belief(scores=[5.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    cmd, state = ctrl.step(b_initial, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector

    b_challenge = belief(scores=[50.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    t = 0.0
    for _ in range(2):  # frames at t=0.04, 0.08 -- cumulative 0.08s, not enough yet
        t += 0.04
        cmd, state = ctrl.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=t, state=state)
        assert cmd.target_sector == initial_target
    t += 0.04  # t=0.12 -- cumulative streak reaches hysteresis_min_seconds, switches
    cmd, state = ctrl.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=t, state=state)
    assert cmd.target_sector == 0
    assert state.seconds_on_target == pytest.approx(0.04)  # dwell restarts on the new
                                                              # target, this frame's own dt


def test_hysteresis_margin_not_met_never_switches():
    """A challenger that's better but NOT by the margin must never
    switch, no matter how many frames pass."""
    ctrl, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.5)
    b_initial = belief(scores=[5.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    cmd, state = ctrl.step(b_initial, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector

    b_tiny_edge = belief(scores=[5.05, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    t = 0.0
    for _ in range(10):
        t += 0.04
        cmd, state = ctrl.step(b_tiny_edge, goal_heading=0.0, yaw=0.0, t_capture=t, state=state)
        assert cmd.target_sector == initial_target


def test_hysteresis_bypass_on_source_switch():
    """A change in belief.source must allow an immediate switch, even
    with zero consecutive challenge frames."""
    ctrl, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.15)
    b_cheap = belief(scores=[5.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N, source="cheap")
    cmd, state = ctrl.step(b_cheap, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector

    # heavy stage disagrees, on the very first frame it's consulted
    b_heavy = belief(scores=[50.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N, source="heavy")
    cmd2, state2 = ctrl.step(b_heavy, goal_heading=0.0, yaw=0.0, t_capture=1.2, state=state)
    assert cmd2.target_sector == 0
    assert cmd2.target_sector != initial_target or initial_target == 0
    assert state2.last_source == "heavy"


def test_target_no_longer_candidate_forces_immediate_reselection():
    """If the previous target sector's own valid flag flips to False,
    hysteresis must not keep the controller waiting around for a
    now-nonexistent sector."""
    ctrl, _ = make_controller()
    b1 = belief(scores=[1.0, 1.0, 5.0, 1.0, 1.0], valid=[True] * N)
    cmd, state = ctrl.step(b1, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector == 2

    b2 = belief(scores=[1.0, 1.0, 5.0, 1.0, 1.0], valid=[True, True, False, True, True])
    cmd2, state2 = ctrl.step(b2, goal_heading=0.0, yaw=0.0, t_capture=0.04, state=state)
    assert cmd2.target_sector != 2
    assert state2.seconds_on_target == pytest.approx(0.04)  # this frame's own dt, see sector.py


def test_hysteresis_rate_independent_25hz_vs_0_8hz():
    """Step CQ's VERIFY: identical debounce behaviour at 25Hz and 0.8Hz
    input rates. hysteresis_min_seconds=0.12 (3 frames at 25Hz,
    converted). At 25Hz (dt=0.04s), a lone challenging frame must NOT
    switch (0.04s < 0.12s) -- noise rejection preserved. At 0.8Hz
    (dt=1.25s), a SINGLE challenging frame already spans MORE real time
    than the threshold and must switch immediately -- this is the whole
    point of Step CQ: the old frame-COUNT rule forced HeavyStage to wait
    for 3 raw frames (~3.75s) to confirm the exact same 0.12s of real
    danger a 25Hz source confirms in 3 frames (0.12s). The RULE (confirm
    for >= hysteresis_min_seconds of sim time) is identical at both
    rates; only the number of samples needed to satisfy it differs,
    which is the correct, rate-independent behaviour."""
    b_initial = belief(scores=[5.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)
    b_challenge = belief(scores=[50.0, 5.0, 5.0, 5.0, 5.0], valid=[True] * N)

    # -- 25Hz: one challenging frame is NOT enough --
    ctrl_fast, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.15)
    cmd, state = ctrl_fast.step(b_initial, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector
    cmd, state = ctrl_fast.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=0.04, state=state)
    assert cmd.target_sector == initial_target, "25Hz: single 0.04s frame must not switch"
    # ... but 3 of them (cumulative 0.12s) do, exactly as the pre-CQ frame-count rule did
    cmd, state = ctrl_fast.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=0.08, state=state)
    cmd, state = ctrl_fast.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=0.12, state=state)
    assert cmd.target_sector == 0, "25Hz: three consecutive 0.04s frames (0.12s) must switch"

    # -- 0.8Hz: one challenging frame ALREADY spans more than 0.12s and must switch immediately --
    ctrl_slow, _ = make_controller(hysteresis_min_seconds=0.12, hysteresis_margin=0.15)
    cmd, state = ctrl_slow.step(b_initial, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    initial_target = cmd.target_sector
    cmd, state = ctrl_slow.step(b_challenge, goal_heading=0.0, yaw=0.0, t_capture=1.25, state=state)
    assert cmd.target_sector == 0, (
        "0.8Hz: a single 1.25s-spaced frame already exceeds hysteresis_min_seconds and must "
        "switch on its own -- forcing a 2nd/3rd HeavyStage frame here would reproduce the exact "
        "rate-unfairness Step CQ exists to remove"
    )


# ---------------------------------------------------------------------
# Mode
# ---------------------------------------------------------------------
def test_cruise_when_centre_clear_and_tau_above_crit():
    ctrl, _ = make_controller(tau_crit_score=0.1)
    b = belief(scores=[0.9, 0.9, 0.9, 0.9, 0.9], valid=[True] * N)  # all well clear, [0,1] scale
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector == 2  # centre wins on a flat/clear field
    assert cmd.mode == "cruise"


def test_avoid_when_min_tau_below_crit_even_if_target_is_centre():
    ctrl, _ = make_controller(tau_crit_score=0.1)
    b = belief(scores=[0.9, 0.9, 0.9, 0.05, 0.9], valid=[True] * N)  # sector 3 dangerous (below crit), not the target
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector == 2
    assert cmd.mode == "avoid"  # min tau (1.0) below tau_crit (3.0)


# ---------------------------------------------------------------------
# Command generation
# ---------------------------------------------------------------------
def test_fwd_vel_constant_across_all_modes():
    ctrl, cfg = make_controller()
    for scores, valid in [
        ([10.0] * N, [True] * N),          # cruise
        ([10.0, 10.0, 0.1, 10.0, 10.0], [True] * N),  # avoid
        ([0.0] * N, [False] * N),          # blind
    ]:
        cmd, _ = ctrl.step(belief(scores, valid), goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
        assert cmd.fwd_vel == cfg.fwd_vel


def test_yaw_rate_clamped_to_max():
    ctrl, cfg = make_controller(k_yaw=10.0, yaw_rate_max=0.8)  # exaggerated gain to force saturation
    b = belief(scores=[10.0, 1.0, 1.0, 1.0, 1.0], valid=[True] * N)  # far-left sector wins
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert abs(cmd.yaw_rate) <= cfg.yaw_rate_max + 1e-9


def test_left_sector_yields_positive_yaw_rate_enu_convention():
    """Sector 0 (far left) must produce POSITIVE yaw_rate (ENU: turn
    left) -- see module docstring's sign-convention note."""
    ctrl, _ = make_controller()
    b = belief(scores=[10.0, 1.0, 1.0, 1.0, 1.0], valid=[True] * N)
    cmd, _ = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=fresh_state())
    assert cmd.target_sector == 0
    assert cmd.yaw_rate > 0


# ---------------------------------------------------------------------
# Step CM -- open-loop yaw bug fix (Bug 1: yaw_rate must converge as
# the vehicle turns; Bug 2: hysteresis must hold a world bearing, not a
# stale sector index). Both tests hold hysteresis_margin very large so
# a genuinely separate interaction cannot muddy what's being checked
# here: cost[i] is computed from BODY-FRAME-FIXED sector bearings and
# does not depend on yaw at all, so as yaw sweeps, the sector the held
# world bearing currently projects into can drift away from
# best_candidate's OWN sector well before real convergence -- with a
# small hysteresis_margin this can trigger a challenge that re-anchors
# the world bearing to "best_candidate's body bearing + CURRENT yaw", a
# MOVING target that would fight convergence. Not fixed here (outside
# Bug 1/2's stated scope, and Step CG's score-stretch can produce large
# enough gaps to trigger it for real) -- neutralised in these two tests
# so they isolate the specific property each Bug's fix claims, and
# flagged in the README as a real, found-but-unaddressed interaction
# for Step CN/CO's closed-loop verification to watch for.
# ---------------------------------------------------------------------
def test_yaw_rate_converges_as_vehicle_turns_toward_target():
    """Bug 1's VERIFY: with a fixed desired direction (a static belief,
    sector 0 clearly best) and the drone progressively rotating toward
    it (yaw increasing frame to frame, belief unchanged), the commanded
    yaw_rate must decrease monotonically and reach ~0 once aligned --
    not stay constant forever, which was Step CL's actual failure."""
    ctrl, cfg = make_controller(hysteresis_margin=1e6, hysteresis_min_seconds=1e6)
    b = belief(scores=[10.0, 1.0, 1.0, 1.0, 1.0], valid=[True] * N)  # sector 0 clearly best, static
    from obst_avoidance.control.sector import _sector_bearing
    sector0_body_bearing = _sector_bearing(0, cfg)  # exact value, not a hand-rounded approximation

    sector0_bearing = None
    state = fresh_state()
    yaw_rates = []
    yaws = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, sector0_body_bearing]  # sweeps up to full alignment
    for i, yaw in enumerate(yaws):
        cmd, state = ctrl.step(b, goal_heading=0.0, yaw=yaw, t_capture=float(i), state=state)
        if sector0_bearing is None:
            sector0_bearing = state.target_world_bearing  # committed once, at yaw=0
        yaw_rates.append(cmd.yaw_rate)

    # strictly decreasing (the vehicle is turning INTO the target, never away)
    for a, b_ in zip(yaw_rates, yaw_rates[1:]):
        assert b_ < a, f"yaw_rate did not decrease: {yaw_rates}"
    # ~0 once yaw has caught up to the committed world bearing
    assert yaw_rates[-1] == pytest.approx(0.0, abs=1e-6)


def test_committed_world_bearing_stable_while_sector_index_drifts():
    """Bug 2's VERIFY: with the drone rotating and the scene static,
    the COMMITTED WORLD BEARING must stay constant (it's the same
    physical direction throughout) while the SECTOR INDEX currently
    containing it changes (a rotating camera sweeps different sectors
    across the same world direction) -- the exact distinction Step CL
    found missing: sector 4 at frame 0 and sector 4 at frame 9 were
    silently different world directions."""
    ctrl, cfg = make_controller(hysteresis_margin=1e6, hysteresis_min_seconds=1e6)
    b = belief(scores=[10.0, 1.0, 1.0, 1.0, 1.0], valid=[True] * N)  # sector 0 clearly best, static

    state = fresh_state()
    cmd0, state = ctrl.step(b, goal_heading=0.0, yaw=0.0, t_capture=0.0, state=state)
    committed_bearing = state.target_world_bearing
    assert cmd0.target_sector == 0

    reported_sectors = [cmd0.target_sector]
    for i, yaw in enumerate([0.15, 0.3, 0.45, 0.5585]):
        cmd, state = ctrl.step(b, goal_heading=0.0, yaw=yaw, t_capture=float(i + 1), state=state)
        # the WORLD bearing does not move -- it's the same commitment throughout
        assert state.target_world_bearing == pytest.approx(committed_bearing, abs=1e-9)
        reported_sectors.append(cmd.target_sector)

    # the SECTOR INDEX containing that fixed world direction changes as the
    # camera rotates past it (0 -> ... -> 2, the centre, once fully aligned)
    assert reported_sectors[0] != reported_sectors[-1]
    assert reported_sectors[-1] == 2  # fully aligned = centre sector


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
