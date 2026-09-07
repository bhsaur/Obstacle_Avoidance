"""Shared sector controller with an unchanged default baseline and explicit
experimental recovery options. Historical baseline rationale follows below;
see docs/handoffs/2026-09-07-codex-controller-final.md for v3/v4 policy changes.

Used IDENTICALLY by both CheapStage and (once it exists) HeavyStage --
this module never inspects belief.source for anything except the
hysteresis bypass (see step 3 below), and never branches on it. That's
the one exception, and it exists so that switching TO a better answer
is never suppressed by the previous, worse one's momentum -- not so the
controller can special-case either stage. No perception logic lives
here; this module only ever reads the SectorBelief/feature contract,
never recomputes anything from raw pixels/flow.

SCORE UNITS (Step AP, resolved): belief.scores is [0, 1] traversability,
higher = safer, on the SAME scale regardless of source -- see
SectorBelief's docstring for the comparability contract this module
relies on. The cost formula below (w_tau * (1 - scores[i])) is written
assuming that bound: (1 - scores[i]) always lands in [0, 1], so w_tau
and w_goal (bounded by ~pi, the max possible angular difference) are
now on genuinely comparable scales, and w_tau=1.0/w_goal=0.5 means what
it looks like it means -- roughly a 2:1 weighting, not a tau-dominated
one. tau_crit_score (config, starts at 0.1) is compared directly
against belief.scores in that same [0,1] space -- 0.1 is the DIRECT
equivalent of the original "3.0 seconds" cutoff under CheapStage's
current linear tau_to_score() mapping (score = tau / TAU_CAP_S = 3/30 =
0.1), preserving the original tuning intent while fixing the units.
This module does not normalize anything itself -- doing so here would
put perception logic (knowledge of what raw tau means) in control/,
which the task constraints explicitly forbid. It consumes belief.scores
exactly as given, and trusts the producer to have honored the contract
(CheapStage asserts it does, at the point it builds each SectorBelief).

YAW SIGN CONVENTION: theta (bearing, radians) uses the ENU/mavros
convention record_pass.py's setpoint_velocity topic already uses --
POSITIVE = counter-clockwise from above = turning LEFT (see README:
"+0.6 rad/s commanded yaw rate (mavros/ROS convention) shows up as
about -0.6 rad/s in the raw MAVLink ATTITUDE.yawspeed", i.e. mavros
angular.z follows standard ENU right-hand-rule about +Z). Sector 0 (far
left in the image) therefore has the LARGEST POSITIVE theta; the last
sector (far right) has the most negative. This lets yaw_rate = K_yaw *
theta_target be published to mavros with NO sign flip -- chosen
deliberately so Step AQ's VehicleInterface doesn't need to guess or
re-derive a sign convention. NOT independently verified against a real
mavros session in this module -- flagged for confirmation when Step AQ
actually wires a MavrosVehicle and sends a live setpoint.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

from .types import ControlCommand, ControlState, ControlTelemetry


@dataclass
class SectorControllerConfig:
    """All tunables. Nothing in sector.py should be a bare literal --
    if you're about to write a number in step(), it belongs here
    instead."""

    n_sectors: int = 5
    horizontal_fov_rad: float = math.radians(80.0)  # FALLBACK ONLY (Step BJ) -- feeds
                                                       # _sector_bearing()'s uniform-angular-split
                                                       # approximation, used only when
                                                       # sector_bearings_rad below is not supplied.
                                                       # Real callers must supply sector_bearings_rad
                                                       # instead; this field (and the approximation
                                                       # it drives) exists so synthetic unit tests
                                                       # that only check qualitative sign/ordering
                                                       # don't need real camera intrinsics.
    sector_bearings_rad: tuple = ()  # PREFERRED (Step BJ): per-sector bearings computed
                                       # trigonometrically from real intrinsics via
                                       # geometry.sector_bearings_rad(width, fx, cx, n_sectors) --
                                       # pixel columns are NOT equal-angle under a pinhole
                                       # projection (Step BG), so a uniform width/n_sectors split
                                       # (the horizontal_fov_rad fallback above) makes every
                                       # theta_i systematically wrong. Length must equal n_sectors
                                       # when supplied.
    w_tau: float = 1.0
    w_goal: float = 0.5
    hysteresis_margin: float = 0.15
    hysteresis_min_seconds: float = 0.12  # Step CQ (replaces hysteresis_min_frames=3):
                                            # 3 frames at CheapStage's ~25Hz = 3/25 = 0.12s.
                                            # A frame-COUNT threshold silently assumed the
                                            # rate of whatever perception stage feeds it --
                                            # under HeavyStage's ~0.8Hz it meant waiting
                                            # 3.75 REAL SECONDS of confirmation inside a
                                            # ~2s danger window (Step CP's heavy_co_run3
                                            # diagnosis), not a bug, just an unfair, silent
                                            # rate assumption baked into a bare "3". This is
                                            # the CONVERTED value, not retuned -- see
                                            # step()'s hysteresis block and README's Step CQ.
    tau_crit_score: float = 0.1  # [0,1] score-space threshold -- see module docstring's SCORE
                                   # UNITS note. Equivalent to the original "3.0 seconds" cutoff
                                   # under CheapStage's tau_to_score() mapping (3/TAU_CAP_S=30 = 0.1);
                                   # re-derive if that mapping ever changes.
    k_yaw: float = 1.0
    yaw_rate_max: float = 0.8  # rad/s
    fwd_vel: float = 0.8       # m/s, CONSTANT across cruise/avoid/blind -- see module docstring
                                # below. Changed from 1.5 (Step BJ/BF, cheap AND heavy shared it
                                # via this single default) to 0.8 in Step BK: at 1.5 m/s the new
                                # Zone B box's lead-time arithmetic gave only 0.433m of achievable
                                # avoidance clearance against 0.5m needed (README's Step BK) --
                                # short even in the best case, before this obstacle can be flown
                                # against at all. 0.8 m/s gives 0.733m (+0.233m margin). This is
                                # a GLOBAL, shared value -- both arms fly at the same speed, kept
                                # as one number in one place rather than a per-script override, so
                                # the two arms stay a controlled comparison at the NEW speed (Step
                                # AS's flow-only baseline is being re-run at 0.8 m/s specifically
                                # to re-establish that comparability, not left stale at 1.5m/s).
                                # and decelerate_when_blind below for the one sanctioned exception.
    decelerate_when_blind: bool = False  # ablation flag, NOT the default -- see SPEED POLICY
    blind_fwd_vel: float = 0.5           # only used if decelerate_when_blind=True

    reorient_to_goal: bool = False  # v5: never drive while the route lies outside view
    opening_steering: bool = False  # v4: pursue goal within connected admitted sectors
    yaw_accel_max: float = 1.0  # v4 command slew, rad/s^2; experimental bound
    goal_recovery: bool = False  # v3: continuous goal correction and uncertainty speed
    min_ttc_s: float = 2.0  # v3 experimental admission threshold
    min_forward_depth_m: float = 1.0  # camera-z distance, not airframe clearance
    confidence_full_speed: float = 0.25  # experimental coverage threshold, not a probability
    robust_hysteresis: bool = False  # opt-in v2; baseline remains reproducible

    def __post_init__(self):
        if self.opening_steering and (not self.goal_recovery or not math.isfinite(self.yaw_accel_max) or self.yaw_accel_max <= 0):
            raise ValueError("opening steering requires goal recovery and a positive finite yaw acceleration")
        if self.goal_recovery and not all(math.isfinite(v) and v > 0 for v in
                (self.confidence_full_speed, self.min_ttc_s, self.min_forward_depth_m)):
            raise ValueError("v3 admission thresholds must be finite and positive")
        # Step CN finding: n_sectors does NOT auto-track sector_bearings_rad's
        # length, and every closed-loop run before this check existed silently
        # got away with passing an 11-element sector_bearings_rad alongside the
        # n_sectors=5 dataclass default -- candidates/hysteresis/_sector_bearing
        # all index by cfg.n_sectors, not len(sector_bearings_rad), so this
        # restricted every prior run's candidate set to the LEFT HALF of the
        # image only, regardless of score (target_sector never exceeded 4 in
        # any logged run). Fail loud here rather than silently mis-selecting.
        if self.sector_bearings_rad and len(self.sector_bearings_rad) != self.n_sectors:
            raise ValueError(
                f"SectorControllerConfig.n_sectors ({self.n_sectors}) does not match "
                f"len(sector_bearings_rad) ({len(self.sector_bearings_rad)}) -- pass "
                f"n_sectors=len(bearings) explicitly, don't rely on the n_sectors default."
            )


def _sector_bearing(sector: int, config: SectorControllerConfig) -> float:
    """Bearing (rad, ENU/mavros sign convention -- see module docstring)
    of sector `sector`'s centre, relative to current heading. Assumes a
    rigid, forward-looking camera aligned with body heading (established
    for this airframe -- see frame_source/sim.py's gimbal docstring: the
    gimbal is rigid, not stabilizing, so the camera IS the body's
    forward-facing view).

    Reads config.sector_bearings_rad if supplied (Step BJ; PREFERRED --
    the real, trigonometrically-correct bearings from
    geometry.sector_bearings_rad()). Falls back to a uniform
    horizontal_fov_rad/n_sectors split only when it isn't -- an
    APPROXIMATION that treats pixel columns as equal-angle, which they
    are not under a pinhole projection (Step BG); see
    SectorControllerConfig's docstring for when this fallback is
    acceptable (synthetic unit tests only)."""
    if config.sector_bearings_rad:
        return config.sector_bearings_rad[sector]
    n = config.n_sectors
    fov = config.horizontal_fov_rad
    # sector 0 = far left = most positive theta (ENU convention, see module docstring)
    return fov / 2.0 - (sector + 0.5) * (fov / n)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _nearest_sector_for_bearing(bearing: float, cfg: "SectorControllerConfig") -> int:
    """Which sector currently contains a given BODY-FRAME bearing --
    the sector whose own centre bearing is closest to it (sector
    bearings are monotonic across the image, Step BJ, so nearest-centre
    is the correct membership test given only a list of centres, not
    raw pixel boundaries, is available in control/). Used by Step CM to
    turn a projected world bearing back into a sector index each step,
    for logging (ControlCommand.target_sector) and for candidate-
    membership/hysteresis checks -- never to derive yaw_rate, which
    Step CM computes straight from the bearing itself."""
    n = cfg.n_sectors
    return min(range(n), key=lambda i: abs(_wrap_to_pi(_sector_bearing(i, cfg) - bearing)))


class SectorController:
    """VFH-style sector selection + command generation. See module
    docstring for the score-units caveat and yaw sign convention --
    both load-bearing, neither obvious from the code alone."""

    def __init__(self, config: SectorControllerConfig):
        """config.sector_bearings_rad should be set from the camera's
        real intrinsics via geometry.sector_bearings_rad(width, fx, cx,
        n_sectors) (Step BJ; matching how CheapStage/geometry.py already
        treat intrinsics as fixed for a session) -- the
        horizontal_fov_rad dataclass default is only a fallback
        approximation, not a verified value for this airframe's camera.
        Passing an intrinsics dict here instead of pre-derived bearings
        was deliberately avoided: deriving bearings from fx/cx/width is
        a one-line perception-adjacent computation that belongs at the
        call site (which already has intrinsics for CheapStage/HeavyStage),
        not duplicated inside control/."""
        self.config = config

    def step(self, belief, goal_heading: float, yaw: float, t_capture: float, state: ControlState):
        """belief: SectorBelief (scores, valid, confidence, source --
        see perception/types.py). goal_heading: bearing (rad, same ENU
        sign convention as theta, see module docstring) from CURRENT
        heading to the goal -- computed by the CALLER from vehicle yaw
        and goal position, not by this module. yaw (Step CM): vehicle's
        current WORLD-frame yaw (rad, same ENU convention), needed to
        project a WORLD-frame committed target (state.target_world_bearing)
        into the CURRENT camera frame each step, and to convert a
        freshly-adopted target's body-frame bearing into a world bearing
        for storage. t_capture (Step CQ, NEW): this frame's packet
        capture time (sim time -- see types.py's module docstring, NEVER
        wall clock), used ONLY to compute dt = t_capture - state.last_t
        for hysteresis's now-time-based debounce (ControlState.
        seconds_on_target's own docstring has the full rationale and
        arithmetic) -- not used anywhere else in this module. state:
        ControlState, caller-owned (see types.py's docstring for why).

        Returns (ControlCommand, ControlState) -- the new state, not a
        mutation of the one passed in."""
        cfg = self.config
        n = cfg.n_sectors
        centre = n // 2

        dt = max(0.0, t_capture - state.last_t) if state.last_t is not None else 0.0

        if cfg.reorient_to_goal:
            bearings = [_sector_bearing(i, cfg) for i in range(n)]
            view_limit = min(abs(min(bearings)), abs(max(bearings)))
            # Enter outside observed centres; exit in central half of view.
            # Odometry, not pixel texture, supplies heading for stationary yaw.
            limit = view_limit * 0.5 if state.reorienting else view_limit
            if abs(goal_heading) > limit:
                desired = _clamp(cfg.k_yaw * goal_heading, -cfg.yaw_rate_max, cfg.yaw_rate_max)
                previous = state.previous_yaw_rate if dt > 0 else 0.0
                yaw_rate = _clamp(desired, previous - cfg.yaw_accel_max * dt,
                                 previous + cfg.yaw_accel_max * dt)
                target_world = _wrap_to_pi(yaw + goal_heading)
                cmd = ControlCommand(0.0, yaw_rate, "avoid", None,
                    ControlTelemetry({}, None, None, target_world, goal_heading,
                                     "reorient_to_goal", True, None))
                return cmd, ControlState(target_world, 0.0, belief.source, t_capture,
                                         previous_yaw_rate=yaw_rate, reorienting=True)
            if state.reorienting:
                # The old avoidance choice was made while facing elsewhere.
                # Reacquire from the current image, preserving yaw slew only.
                state = ControlState(None, 0.0, belief.source, state.last_t,
                                     previous_yaw_rate=state.previous_yaw_rate)

        candidates = [i for i in range(n) if belief.valid[i]]
        if cfg.goal_recovery:
            candidates = [i for i in candidates if math.isfinite(belief.scores[i])]
            # Admission uses physical estimates, never the min-max ranking.
            # Stages expose only the quantity they can estimate; neither field
            # present means no physical support for forward travel in v3.
            def admitted(i):
                evidence = False
                for values, threshold in ((belief.ttc_s, cfg.min_ttc_s),
                                          (belief.forward_depth_m, cfg.min_forward_depth_m)):
                    if values is not None:
                        value = float(values[i])
                        if not math.isfinite(value) or value <= threshold:
                            return False
                        evidence = True
                return evidence
            candidates = [i for i in candidates if admitted(i)]
            if not math.isfinite(belief.confidence) or belief.confidence <= 0:
                candidates = []

        if not candidates:
            # BLIND: no sector is valid. Steering only -- hold heading
            # (yaw_rate=0), never interpolate/guess a direction from
            # sectors we've explicitly marked "don't know". fwd_vel
            # stays constant per SPEED POLICY unless the ablation flag
            # is set. No sensible bearing to keep holding either --
            # same "nothing to stay committed to" reasoning as the old
            # target_sector=None reset.
            fwd_vel = (0.0 if cfg.goal_recovery else
                       cfg.blind_fwd_vel if cfg.decelerate_when_blind else cfg.fwd_vel)
            cmd = ControlCommand(
                fwd_vel=fwd_vel, yaw_rate=0.0, mode="blind", target_sector=None,
                telemetry=ControlTelemetry(
                    costs={}, best_candidate=None, current_target=None,
                    target_world_bearing=None, heading_error=None,
                    switch_reason="blind_no_valid_sectors", is_new_commitment=False,
                    min_score_valid=None),
            )
            new_state = ControlState(
                target_world_bearing=None,
                seconds_on_target=0.0,
                last_source=belief.source,
                last_t=t_capture,  # Step CQ: MUST advance even while blind, so dt is
                                    # correct on the next real frame -- previously carried
                                    # `state.last_t` through unchanged, harmless before
                                    # dt existed at all, now would poison the first
                                    # post-blind dt with a stale timestamp.
            )
            return cmd, new_state

        cost = {}
        for i in candidates:
            theta_i = _sector_bearing(i, cfg)
            cost[i] = cfg.w_tau * (1.0 - belief.scores[i]) + cfg.w_goal * abs(theta_i - goal_heading)

        best_candidate = min(candidates, key=lambda i: cost[i])

        source_switched = state.last_source is not None and belief.source != state.last_source

        # Step CM: project the held WORLD bearing into the CURRENT camera frame, then
        # find which sector it now falls in -- this is "current_target" for candidacy/
        # hysteresis purposes, recomputed fresh every step, never a stale stored index.
        if state.target_world_bearing is None:
            current_target = None
        else:
            theta_current = _wrap_to_pi(state.target_world_bearing - yaw)
            current_target = _nearest_sector_for_bearing(theta_current, cfg)

        # is_new_commitment (Step CM): set explicitly per branch, not reconstructed
        # after the fact -- True exactly when target_world_bearing must be
        # (re)derived from this sector's CURRENT body-frame bearing; False when the
        # PREVIOUSLY held world bearing should simply carry through unchanged. That
        # "carry through unchanged while yaw changes" is the entire fix: it is what
        # makes theta_target_body shrink toward zero as the vehicle turns, instead of
        # reissuing a constant command forever (Step CL's failure).
        is_new_commitment = False
        challenger_sector = None
        challenger_since = None
        if cfg.robust_hysteresis and current_target is not None:
            # Conservative support interval: never clamp an unseen held bearing
            # onto the edge sector. Real camera edges extend beyond the centres.
            bearings = [_sector_bearing(i, cfg) for i in range(n)]
            if not min(bearings) <= theta_current <= max(bearings):
                current_target = None

        if current_target is None or source_switched or current_target not in candidates:
            # No prior target, source just changed (bypass hysteresis --
            # see module docstring: never suppress switching TO a better
            # answer because the previous, worse one had momentum), or
            # the sector the held bearing currently projects into is no
            # longer even a candidate (its own valid flag flipped) -- in
            # all three cases there is nothing sensible to stay
            # committed to, so adopt the best candidate immediately.
            # seconds_on_target=dt, NOT 0.0: this frame itself counts as
            # the first (partial) dwell frame on the new target -- using
            # 0.0 here would silently discard this frame's own dt on the
            # VERY NEXT reinforcing step (0.0 is not "> 0", so that next
            # step would look like ANOTHER fresh start and restart at
            # its own dt alone, under-counting true elapsed dwell time
            # by one frame-interval every single commitment). Mirrors
            # the old frame-counting code's "frames_on_target=1" (this
            # frame counts) exactly, dt substituted for the literal 1.
            target = best_candidate
            seconds_on_target = dt
            is_new_commitment = True
            switch_reason = ("adopt_no_prior" if current_target is None
                             else "adopt_source_switch" if source_switched
                             else "adopt_target_lost")
        elif (cfg.goal_recovery and best_candidate != current_target
              and belief.scores[best_candidate] >= belief.scores[current_target]
              and abs(_wrap_to_pi(_sector_bearing(best_candidate, cfg) - goal_heading))
                  < abs(_wrap_to_pi(_sector_bearing(current_target, cfg) - goal_heading))):
            # Route recovery does not need the obstacle-side switch margin when
            # the new sector has no worse ranking. Do not call it clearance.
            target = best_candidate
            seconds_on_target = dt
            is_new_commitment = True
            switch_reason = "goal_recovery"
        elif best_candidate == current_target:
            # Current target still wins -- reinforce. Step CQ: each
            # qualifying (dwelling) frame contributes ITS OWN dt, so a
            # fresh dwell (state.seconds_on_target not already positive
            # -- e.g. arriving straight out of an in-progress-but-
            # unsuccessful challenge, previous value negative or the
            # neutral 0.0) starts at dt, not 0 -- "this frame has been
            # dwelling for its own duration so far", exactly mirroring
            # frame-counting's old "1" starting point, seconds instead
            # of frames. A challenge that lapses because the challenger
            # stopped winning is a fresh dwell, not a continuation of
            # anything.
            target = current_target
            seconds_on_target = state.seconds_on_target + dt if state.seconds_on_target > 0 else dt
            switch_reason = "hold_reinforce"
        else:
            # A different sector currently costs less. Debounce: only
            # switch once a challenger has beaten the current target by
            # hysteresis_margin for a CUMULATIVE hysteresis_min_seconds
            # of SIM TIME (Step CQ -- replaces a raw frame count that
            # silently assumed CheapStage's ~25Hz rate; see
            # ControlState.seconds_on_target's docstring for why a bare
            # frame count penalized HeavyStage's much slower rate).
            #
            # seconds_on_target is reused for this streak, SAME sign
            # convention as before (just seconds instead of frames):
            # positive = elapsed seconds dwelling on the current target;
            # negative = elapsed seconds of an in-progress challenge
            # (abs value). A fresh challenge (previous value not already
            # negative) starts its streak at THIS frame's own dt, not
            # 0.0 -- 0.0 would be indistinguishable from "no challenge
            # in progress" on the NEXT step's `< 0` check (there is no
            # sign-carrying "negative zero" to lean on), and it also
            # mirrors frame-counting's old "streak starts at 1, not 0"
            # exactly, seconds instead of frames. Without this reset-to-
            # a-fresh-streak behavior, a challenge starting right after
            # a long stable dwell would inherit the dwell's elapsed time
            # as its own starting point and could trigger an instant
            # switch on the very first challenging frame -- exactly the
            # single-frame-noise case hysteresis_min_seconds exists to
            # prevent (caught by test_hysteresis_blocks_single_frame_switch).
            #
            # NOTE: this streak counts consecutive frames where *some*
            # sector beats the target by the margin, not necessarily the
            # SAME sector each frame -- ControlState (Step AN) carries
            # only one counter, not a challenger identity, so a
            # genuinely oscillating multi-way tie could reach the
            # threshold via different challengers on different frames
            # (confirmed happening for real in Step CP's heavy_co_run3
            # diagnosis: the winning challenger flipped sector 7 ->
            # sector 9 between its two confirming frames). Accepted as a
            # documented simplification, same as before Step CQ.
            #
            # NOISE-REJECTION TRADE-OFF (Step CQ, worth stating plainly):
            # at a slow perception rate whose OWN frame interval already
            # exceeds hysteresis_min_seconds (HeavyStage's ~1.25s vs.
            # 0.12s here), a SINGLE noisy/spurious challenging frame can
            # now trigger an immediate switch, where before it needed 3
            # confirming HeavyStage frames (~3.75s) to do so. This is
            # the direct, unavoidable consequence of fixing the rate
            # unfairness this step exists to fix, not an oversight --
            # converting the debounce to real time necessarily means a
            # slow source gets less built-in noise suppression per
            # sample than a fast one did under the old frame-count rule.
            if cost[best_candidate] < cost[current_target] - cfg.hysteresis_margin:
                if cfg.robust_hysteresis:
                    # Evidence starts when THIS challenger is first observed,
                    # not during the preceding frame interval. A changed winner
                    # or a non-increasing capture clock restarts confirmation.
                    challenger_sector = best_candidate
                    continuing = (state.challenger_sector == best_candidate
                                  and state.challenger_since is not None
                                  and state.last_t is not None
                                  and t_capture > state.last_t)
                    challenger_since = state.challenger_since if continuing else t_capture
                    streak_seconds = t_capture - challenger_since
                elif state.seconds_on_target < 0:
                    streak_seconds = -state.seconds_on_target + dt
                else:
                    streak_seconds = dt
                if streak_seconds >= cfg.hysteresis_min_seconds:
                    # Switch completes -- same "this frame counts, use dt
                    # not 0.0" reasoning as the bypass-adopt branch above;
                    # the just-elapsed challenge streak itself is now
                    # irrelevant (a NEW dwell starts on the new target).
                    target = best_candidate
                    seconds_on_target = dt
                    is_new_commitment = True
                    switch_reason = "switch_hysteresis_met"
                else:
                    target = current_target
                    seconds_on_target = -streak_seconds
                    switch_reason = "hold_challenge_pending"
            else:
                # Challenger exists but never even cleared the margin --
                # genuinely dormant, discard any prior streak entirely
                # (0.0, not dt: unlike a commitment, nothing here just
                # started that the next frame should build on).
                target = current_target
                seconds_on_target = 0.0
                switch_reason = "hold_no_challenger"

        # Step CM: only a GENUINE new commitment (re)derives the world bearing from
        # this sector's CURRENT body-frame bearing + yaw. Every other case (reinforcing
        # an existing target, or a challenge still in progress) leaves
        # target_world_bearing UNCHANGED -- see is_new_commitment's own comment above
        # for why that is the entire fix.
        if is_new_commitment:
            target_world_bearing = _wrap_to_pi(_sector_bearing(target, cfg) + yaw)
        else:
            target_world_bearing = state.target_world_bearing

        if cfg.goal_recovery:
            # Steer continuously within the selected angular cell. Clamp at
            # centre midpoints; outer cells stop at their observed centres.
            theta = _sector_bearing(target, cfg)
            lo = ((theta + _sector_bearing(target + 1, cfg)) / 2
                  if target + 1 < n else theta)
            hi = ((theta + _sector_bearing(target - 1, cfg)) / 2
                  if target > 0 else theta)
            target_world_bearing = _wrap_to_pi(yaw + _clamp(goal_heading, lo, hi))
        if cfg.opening_steering:
            # An admitted opening is contiguous; never interpolate across an
            # unknown/rejected sector. Goal steering within it need not chase
            # each min-max score fluctuation between its component sectors.
            left = right = target
            while left > 0 and left - 1 in candidates:
                left -= 1
            while right + 1 < n and right + 1 in candidates:
                right += 1
            desired = _clamp(goal_heading, _sector_bearing(right, cfg), _sector_bearing(left, cfg))
            target = min(range(left, right + 1), key=lambda i: abs(_sector_bearing(i, cfg) - desired))
            target_world_bearing = _wrap_to_pi(yaw + desired)
        theta_target_body = _wrap_to_pi(target_world_bearing - yaw)
        yaw_rate = _clamp(cfg.k_yaw * theta_target_body, -cfg.yaw_rate_max, cfg.yaw_rate_max)

        min_score_valid = min(belief.scores[i] for i in candidates)
        if min_score_valid < cfg.tau_crit_score or target != centre:
            mode = "avoid"
        else:
            mode = "cruise"

        fwd_vel = cfg.fwd_vel
        if cfg.goal_recovery:
            coverage = _clamp(belief.confidence / cfg.confidence_full_speed, 0.0, 1.0)
            alignment = max(0.0, math.cos(theta_target_body))
            # Turn in place if the goal lies behind the forward camera.
            if abs(goal_heading) >= math.pi / 2:
                alignment = 0.0
            fwd_vel *= coverage * alignment
        if cfg.opening_steering:
            requested_yaw_rate = yaw_rate
            # A capture-clock reset starts from rest. Blind stops are immediate;
            # normal changes are bounded in sim time, independent of frame rate.
            previous = state.previous_yaw_rate if dt > 0 else 0.0
            change = cfg.yaw_accel_max * dt
            yaw_rate = _clamp(yaw_rate, previous - change, previous + change)
            if yaw_rate * requested_yaw_rate < 0:
                fwd_vel = 0.0  # no forward drive while still turning the old way
        cmd = ControlCommand(
            fwd_vel=fwd_vel, yaw_rate=yaw_rate, mode=mode, target_sector=target,
            telemetry=ControlTelemetry(
                costs=dict(cost), best_candidate=best_candidate, current_target=current_target,
                target_world_bearing=target_world_bearing, heading_error=theta_target_body,
                switch_reason=switch_reason, is_new_commitment=is_new_commitment,
                min_score_valid=float(min_score_valid)),
        )
        new_state = ControlState(
            target_world_bearing=target_world_bearing,
            seconds_on_target=seconds_on_target,
            last_source=belief.source,
            last_t=t_capture,
            challenger_sector=challenger_sector if not is_new_commitment else None,
            challenger_since=challenger_since if not is_new_commitment else None,
            previous_yaw_rate=yaw_rate if cfg.opening_steering else 0.0,
        )
        return cmd, new_state
