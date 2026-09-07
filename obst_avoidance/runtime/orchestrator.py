"""Orchestrator (Step AR) -- wires FrameSource -> CheapStage ->
SectorController -> VehicleInterface into the flow-only closed-loop arm,
logs everything needed for the results tables, and detects collision /
goal-reached so a run has a defined end.

Not part of control/ or perception/ -- this module is allowed to know
about both (it's the glue), but neither of THEM is allowed to know
about this: CheapStage has no idea a controller exists, and
SectorController has no idea it's flying a corridor with obstacles at
particular x,y coordinates. That separation is what makes the
collision-rate number this produces a measurement of PERCEPTION quality
(Step AS), not something baked into the controller or the perception
stage.

Wall-clock IS used here (time.perf_counter() for latency measurement,
time.time() for the run's overall safety timeout) -- this is
orchestration/experiment-control code, not the deterministic per-frame
control path Step AN's "no wall clock" rule governs (that rule is
scoped to control/types.py and control/sector.py specifically, and
SectorController.step() itself still takes no wall-clock input here).
"""
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np

from ..control.sector import SectorController, SectorControllerConfig
from ..control.types import ControlCommand, ControlState
from ..control.reference_path import WorldBearingGoal
from ..perception.cheap import CheapStage
from ..perception.features import feature_names

# ---------------------------------------------------------------------
# Ground-truth obstacle table, from worlds/env_zones.sdf, read directly
# (grep'd for <pose>/<radius>/<size> per model) rather than guessed or
# remembered -- see README's per-zone analysis for the same source.
# Only the geometry that actually intersects the flight altitude
# (z=3m, this project's standard -- see Step W) is included: tree
# canopies are spheres well above 3m (e.g. treeA1's canopy spans
# z=3.7-7.3) and are NOT collidable at cruise altitude, only trunks
# are. Box/wall yaw rotations (a few tenths of a radian on some
# obstacles) are IGNORED -- axis-aligned bounding half-extents are used
# instead, per this whole project's "uninnovative, don't over-engineer"
# discipline. kind is "circle" (trunks) or "box" (everything else);
# half_x/half_y are box half-extents, radius is circle radius.
OBSTACLES = [
    # Zone A (textured, off-centreline)
    dict(name="treeA1", x=17.0, y=-3.8, kind="circle", radius=0.25, zone="zone_A"),
    dict(name="treeA2", x=20.0, y=3.8, kind="circle", radius=0.25, zone="zone_A"),
    dict(name="boxA1", x=23.0, y=-2.8, kind="box", half_x=0.75, half_y=0.75, zone="zone_A"),
    dict(name="treeA3", x=24.0, y=4.2, kind="circle", radius=0.22, zone="zone_A"),
    # Zone B (textureless box, centreline -- Step BG replaced the original
    # 16m wall with a 1.0m-wide box; Step BT/BU resized to 1.2m x 2.5m to
    # work around the relative-depth checkpoint's aggregation limits,
    # then Step BW reverted back to 1.0m x 1.5m once the checkpoint
    # itself was switched to metric, removing the problem that resize
    # existed to solve; see worlds/env_zones.sdf's comments)
    dict(name="wallB1", x=55.0, y=0.0, kind="box", half_x=0.2, half_y=0.5, zone="zone_B"),
    dict(name="boxB1", x=58.0, y=-6.0, kind="box", half_x=1.0, half_y=1.0, zone="zone_B"),
    # Zone C (textured, mixed -- boxC2 is on the centreline)
    dict(name="treeC1", x=86.0, y=-1.5, kind="circle", radius=0.25, zone="zone_C"),
    dict(name="treeC2", x=87.0, y=1.5, kind="circle", radius=0.25, zone="zone_C"),
    dict(name="boxC1", x=89.0, y=-3.0, kind="box", half_x=0.75, half_y=0.75, zone="zone_C"),
    dict(name="boxC2", x=89.0, y=0.0, kind="box", half_x=0.75, half_y=0.75, zone="zone_C"),
    dict(name="treeC3", x=91.0, y=2.5, kind="circle", radius=0.22, zone="zone_C"),
    dict(name="boxC3", x=93.0, y=-1.0, kind="box", half_x=0.65, half_y=0.65, zone="zone_C"),
    dict(name="treeC4", x=94.0, y=1.0, kind="circle", radius=0.25, zone="zone_C"),
    # Zone D (narrow textureless + wide textureless, both centreline)
    dict(name="narrowD1", x=108.0, y=0.0, kind="box", half_x=0.2, half_y=1.75, zone="zone_D"),
    dict(name="treeD1", x=116.0, y=-3.5, kind="circle", radius=0.25, zone="zone_D"),
    dict(name="boxD1", x=119.0, y=2.0, kind="box", half_x=0.9, half_y=0.9, zone="zone_D"),
    dict(name="wallD1", x=122.0, y=0.0, kind="box", half_x=0.2, half_y=5.0, zone="zone_D"),
]

# Zone boundaries -- SDF nominal x, EXCEPT B/D widened to their actual
# pre-collision approach footprint (the wall/narrowD1's own collision
# surface intercepts flight before the nominal centre-x -- see README's
# "CheapStage Phase 2" per-zone analysis, Step AL, for the measurement
# behind this adjustment).
ZONES = [
    ("gap_preA", -10.0, 17.0),
    ("zone_A", 17.0, 24.0),
    ("gap_AB", 24.0, 50.0),
    ("zone_B", 50.0, 58.0),
    ("gap_BC", 58.0, 86.0),
    ("zone_C", 86.0, 94.0),
    ("gap_CD", 94.0, 104.0),
    ("zone_D", 104.0, 122.0),
    ("past_D", 122.0, 1e9),
]

COLLISION_RADIUS_M = 0.35  # placeholder airframe half-width for COLLISION specifically -- smaller
                            # and a DIFFERENT concept than perception/cheap.py's CONE_R_SAFE_M (a
                            # broader "this matters" awareness radius, not a contact radius). NOT
                            # verified against this workspace's actual Iris model dimensions, same
                            # caveat as CONE_R_SAFE_M.
GOAL_X_M = 125.0  # a few metres past wallD1 (x=122) -- "reached the end of the corridor"

# Per-zone segment presets (Step AX) -- start a few metres before the
# zone, goal a few metres past it (or the corridor's end for the last
# zone), so a segment run's collision/completion outcome reflects THAT
# zone specifically rather than the whole remaining corridor. start_x/
# start_y are what the LAUNCH's spawn_x/spawn_y arguments should be set
# to (a sim-spawn concern, outside what OrchestratorConfig itself
# controls); goal_x_m is what OrchestratorConfig.goal_x_m should be set
# to for that run. Both live together here so the two concerns can't
# silently drift apart (e.g. spawning at zone C's start but still
# carrying the whole-corridor goal_x_m from a prior run's config).
ZONE_SEGMENTS = {
    "zone_A": dict(start_x=10.0, start_y=0.0, goal_x_m=30.0),
    "zone_B": dict(start_x=45.0, start_y=0.0, goal_x_m=65.0),
    "zone_C": dict(start_x=80.0, start_y=0.0, goal_x_m=100.0),
    "zone_D": dict(start_x=100.0, start_y=0.0, goal_x_m=GOAL_X_M),
}

# Collision-course start-y jitter (Step BN). Step BL/BM found that
# Step AS's y~Uniform(-1.5,1.5) spawn jitter -- sized for the ORIGINAL
# 16m Zone B wall, which nothing in that range could miss -- made
# "never on a collision course" the default outcome against the 1.0m
# Zone B box that replaced it (Step BG): 4 of 5 heavy runs and all 5
# tracked flow runs cleared the box purely because their y-jitter
# already had them 0.8-1.5m clear of it before ever reaching x=55, not
# because either stage detected-then-avoided anything. A metric that
# mostly measures start-position luck is not a metric.
#
# j = 0.5 * (obstacle_half_y + r_air) -- HALF the true "any straight-line
# y in this range physically contacts the obstacle+airframe" limit
# (obstacle_half_y + r_air), so every drawn y_start keeps real margin
# below the absolute worst case, not just barely inside it. r_air is
# COLLISION_RADIUS_M (0.35m) -- deliberately NOT perception/cheap.py's
# CONE_R_SAFE_M (0.6m): that constant is a soft planning/awareness
# margin, not a contact radius, and this jitter needs to guarantee an
# actual, physical, ground-truth-detectable collision absent any
# avoidance, which is exactly what COLLISION_RADIUS_M already means
# elsewhere in this same file.
#
# Computed per zone from that zone's own CANONICAL CENTRELINE positive
# case (the one obstacle each zone's own comments in OBSTACLES/the SDF
# already single out as "the" gate-should-fire test for that zone) --
# NOT every obstacle in the zone. Zone A has no such single centreline
# target (its trees/box are deliberately off-centreline clutter, a
# different kind of test -- general avoidance amid obstacles the
# vehicle isn't aimed at, not "does the gate fire for one specific
# obstacle"), so it's intentionally absent here rather than forced.
ZONE_COLLISION_JITTER_Y_M = {
    "zone_B": 0.5 * (0.5 + COLLISION_RADIUS_M),   # wallB1 (box), half_y=0.5 (Step BW reverted BT/BU) -> j=0.425
    "zone_C": 0.5 * (0.75 + COLLISION_RADIUS_M),  # boxC2 (box), half_y=0.75 -> j=0.550
    "zone_D": 0.5 * (1.75 + COLLISION_RADIUS_M),  # narrowD1 (box), half_y=1.75 -> j=1.050
}


def zone_of(x: float) -> str:
    for name, lo, hi in ZONES:
        if lo <= x < hi:
            return name
    return "unknown"


def _distance_to_obstacle(x: float, y: float, obstacle: dict) -> float:
    """2D distance from (x,y) to an obstacle's footprint (ignoring z --
    see OBSTACLES' docstring on why only flight-altitude-intersecting
    geometry is modelled at all). 0.0 (not negative) if already inside
    a box footprint -- collision detection only needs "how close", not
    penetration depth."""
    if obstacle["kind"] == "circle":
        d = math.hypot(x - obstacle["x"], y - obstacle["y"])
        return max(d - obstacle["radius"], 0.0)
    dx = max(abs(x - obstacle["x"]) - obstacle["half_x"], 0.0)
    dy = max(abs(y - obstacle["y"]) - obstacle["half_y"], 0.0)
    return math.hypot(dx, dy)


def nearest_obstacle(x: float, y: float):
    """Returns (obstacle_dict, distance) for the closest obstacle to
    (x, y), or (None, inf) if OBSTACLES is empty (it never is)."""
    best_obs, best_d = None, float("inf")
    for obs in OBSTACLES:
        d = _distance_to_obstacle(x, y, obs)
        if d < best_d:
            best_obs, best_d = obs, d
    return best_obs, best_d


def _quat_to_yaw(q) -> float:
    """ENU yaw from a (x,y,z,w) quaternion -- standard atan2 form,
    positive = counter-clockwise from above, matching
    control/sector.py's documented theta sign convention exactly (so
    goal_heading below composes with it with no extra sign flip)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


GOAL_WORLD_BEARING_RAD = 0.0  # the corridor's fixed direction, world ENU heading -- +x,
                               # matching record_pass.py's fly_forward convention. A CONSTANT,
                               # not derived from anything about the vehicle -- see
                               # _goal_heading_body()'s docstring (Step CF).


def _goal_heading_body(yaw: float, goal_world_bearing_rad: float = GOAL_WORLD_BEARING_RAD) -> float:
    """Bearing to the goal, in BODY frame (same theta convention as
    control/sector.py's sector bearings), given the vehicle's current
    world-frame yaw. `goal_heading_body = goal_world_bearing - yaw` --
    the goal is a FIXED WORLD DIRECTION (down the corridor); converting
    a fixed world bearing into body frame at the current instant
    NECESSARILY uses current yaw, every step, for any goal direction
    that isn't "wherever I happen to be pointed" -- that is not what
    was wrong here (Step CE originally suspected `-yaw` alone was the
    bug; re-derived and unit-tested explicitly at Step CF and found
    numerically IDENTICAL to `_wrap_to_pi(-yaw)` for
    `goal_world_bearing_rad=0`, the value used everywhere in this
    codebase so far -- reporting that plainly rather than claiming a
    math fix that isn't there). What Step CF actually changes: the
    goal is now a NAMED CONSTANT (`GOAL_WORLD_BEARING_RAD`) instead of
    an inline literal `0` folded into a sign flip, and this function is
    independently unit-testable (test_orchestrator.py) against several
    spawn-yaw offsets, verifying the sector closest to the TRUE world
    corridor direction gets the lowest goal-cost at each -- a
    regression check that did not exist before, whether or not the
    number it protects changed."""
    return _wrap_to_pi(goal_world_bearing_rad - yaw)


@dataclass
class OrchestratorConfig:
    goal_x_m: float = GOAL_X_M
    collision_radius_m: float = COLLISION_RADIUS_M
    max_wall_time_s: float = 300.0  # overall run safety cap -- wall clock is fine HERE, see module docstring
    log_path: Optional[str] = None  # JSONL, one row per frame; None = no logging
    frame_timeout_s: Optional[float] = None  # optional wall-time camera watchdog
    # CTRL-COMPARE: goal-heading source. None => WorldBearingGoal (baseline,
    # fixed world +x corridor -- identical to _goal_heading_body). A
    # ReferencePathGoal (or any object with .goal_heading(position, yaw) ->
    # GoalInfo) selects the path-tracking variant. The SectorController is
    # unchanged either way; only where goal_heading comes from differs.
    goal_provider: object = None
    controller_version: str = "baseline_world_bearing"
    velocity_frame: str = "body_forward->world_ENU/LOCAL_NED"  # post velocity-frame-fix
    scene_version: str = ""          # caller fills (e.g. world file id) -- for run metadata
    goal_xy: Optional[tuple] = None  # opt-in endpoint arrival criterion
    goal_tolerance_m: float = 0.75
    unsupported_timeout_s: Optional[float] = None  # continuous blind capture time

    def __post_init__(self):
        if self.unsupported_timeout_s is not None and (not math.isfinite(self.unsupported_timeout_s) or self.unsupported_timeout_s <= 0):
            raise ValueError("unsupported_timeout_s must be finite and positive")
        if not math.isfinite(self.goal_tolerance_m) or self.goal_tolerance_m <= 0:
            raise ValueError("goal_tolerance_m must be finite and positive")
        if self.goal_xy is not None and (len(self.goal_xy) != 2 or
                not all(math.isfinite(v) for v in self.goal_xy)):
            raise ValueError("goal_xy must contain two finite coordinates")



@dataclass
class RunResult:
    collided: bool
    collision_zone: Optional[str]
    collision_xy: Optional[tuple]
    completed: bool  # reached goal_x_m
    n_frames: int
    duration_s: float  # sim time (last t_capture - first t_capture), NOT wall clock
    mode_counts: dict  # {"cruise": n, "avoid": n, "blind": n}
    n_target_switches: int
    perception_latency_ms: List[float] = field(default_factory=list)  # whichever stage drove
                                                                        # this run -- CheapStage
                                                                        # (Step AS) or HeavyStage
                                                                        # (Step AW), same field
    controller_latency_ms: List[float] = field(default_factory=list)
    log_path: Optional[str] = None
    stop_reason: str = ""


class HeavyStageAdapter:
    """Wraps a HeavyStage instance so Orchestrator can call it with the
    SAME 3-argument signature CheapStage uses (packet, prev_packet,
    odom) -> (belief, feature_vector), even though HeavyStage itself
    genuinely only needs `packet` (monocular, instantaneous -- no flow
    between frames, so no prev_packet/odom dependency, and it says so
    honestly in its own infer() signature rather than accepting unused
    parameters just to fake uniformity). This adapter is orchestration
    GLUE (runtime/, allowed to know about both perception and control),
    not perception logic living where it shouldn't -- HeavyStage itself
    is untouched. Returns an EMPTY feature vector (HeavyStage doesn't
    produce CheapStage's 54-element, tau/flow-specific feature set;
    fabricating placeholder values for features that don't apply to a
    depth-based stage would violate the same "never invent a number"
    discipline as everywhere else in this codebase)."""

    def __init__(self, heavy_stage):
        self._heavy = heavy_stage

    def infer(self, packet, prev_packet, odom):
        belief = self._heavy.infer(packet)
        return belief, np.array([])


class Orchestrator:
    """Ties FrameSource -> CheapStage (or HeavyStage, via
    HeavyStageAdapter) -> SectorController -> VehicleInterface together
    for one closed-loop run. One instance per run -- not reused across
    runs (CheapStage/SectorController's own internal EMA/hysteresis
    state should start fresh each time, see
    Step AS's paired-seed determinism requirement)."""

    def __init__(self, frame_source, perception_stage, controller: SectorController,
                 vehicle, config: OrchestratorConfig, on_frame=None, should_stop=None):
        """perception_stage: anything with .infer(packet, prev_packet,
        odom) -> (SectorBelief, feature_vector) -- a CheapStage directly
        (Step AS), or a HeavyStage wrapped in HeavyStageAdapter
        (Step AW). Duck-typed deliberately (no shared base class) --
        see HeavyStageAdapter's docstring for why HeavyStage itself
        isn't forced into CheapStage's flow-specific signature."""
        self.frame_source = frame_source
        self.perception_stage = perception_stage
        self.controller = controller
        self.vehicle = vehicle
        self.config = config
        self._feature_names = feature_names()
        # Optional observation/cancellation hooks share the inference result;
        # a GUI must not create another source on the exclusive MAVLink port.
        self.on_frame = on_frame
        self.should_stop = should_stop

    def run(self) -> RunResult:
        cfg = self.config
        log_f = open(cfg.log_path, "w") if cfg.log_path else None

        ctrl_state = ControlState(target_world_bearing=None, seconds_on_target=0.0, last_source=None, last_t=None)
        # CTRL-COMPARE: goal-heading provider (baseline world-bearing unless a
        # variant was configured). Constructed once per run.
        goal_provider = cfg.goal_provider or WorldBearingGoal(GOAL_WORLD_BEARING_RAD)
        # run metadata as the first log line (plotter skips rows without "seq").
        if log_f is not None:
            try:
                ref = goal_provider.goal_heading(None, 0.0).reference
            except Exception:
                ref = {}
            log_f.write(json.dumps({"meta": {
                "controller_version": cfg.controller_version,
                "velocity_frame": cfg.velocity_frame,
                "scene_version": cfg.scene_version,
                "goal_reference": ref,
                "goal_x_m": cfg.goal_x_m,
                "completion_criterion": "endpoint_radius" if cfg.goal_xy is not None else "goal_x_crossing",
                "goal_xy": cfg.goal_xy, "goal_tolerance_m": cfg.goal_tolerance_m,
                "command_timeout_s": getattr(self.vehicle, "_command_timeout_s", None),
                "unsupported_timeout_s": cfg.unsupported_timeout_s,
                "target_switch_count_semantics": "world bearing updates; includes continuous goal trim",
                "yaw_rate_source": "finite difference of valid odometry yaw over capture time",
                "obs_age_ms_semantics": "unavailable; inference_to_command_ms is wall processing latency",
                "collision_radius_m": cfg.collision_radius_m,
            }}) + "\n")
        mode_counts = {"cruise": 0, "avoid": 0, "blind": 0}
        n_switches = 0
        prev_target_world_bearing = None
        unsupported_since = None
        prev_yaw = None            # for finite-difference measured yaw rate
        prev_yaw_t = None
        perception_latencies: List[float] = []
        controller_latencies: List[float] = []

        collided = False
        collision_zone = None
        collision_xy = None
        completed = False

        prev_packet = None
        t_first = None
        t_last = None
        n_frames = 0
        t_wall_start = time.monotonic()
        last_frame_wall = t_wall_start
        stop_reason = "wall_timeout"

        try:
            while True:
                if self.should_stop is not None and self.should_stop():
                    stop_reason = "user_stop"
                    break
                now_wall = time.monotonic()
                if now_wall - t_wall_start > cfg.max_wall_time_s:
                    break  # safety cap -- did not collide or complete within the wall-clock budget

                packet = self.frame_source.read()
                if packet is not None and prev_packet is not None and packet.t_capture <= prev_packet.t_capture:
                    packet = None  # repeated/backwards timestamps are not fresh observations
                if packet is None:
                    if cfg.frame_timeout_s is not None and now_wall - last_frame_wall > cfg.frame_timeout_s:
                        stop_reason = "camera_timeout"
                        break
                    # No new frame since last read() -- SimFrameSource is
                    # designed to be polled from a controlled-cadence
                    # caller, not spun on with no delay (see its own
                    # docstring's "Freshness / continuous spinning"
                    # section); a short sleep avoids burning a full CPU
                    # core on this branch while waiting for the next
                    # frame, matching the polling cadence used elsewhere
                    # in this codebase (e.g. wait_for_intrinsics()).
                    time.sleep(0.005)
                    continue
                last_frame_wall = now_wall
                if prev_packet is None:
                    prev_packet = packet
                    continue

                odom = self.vehicle.state_at(packet.t_capture)
                position, velocity, attitude_quat, odom_valid = odom

                t0 = time.perf_counter()
                belief, feats = self.perception_stage.infer(packet, prev_packet, odom)
                perception_latencies.append((time.perf_counter() - t0) * 1000.0)

                ginfo = None
                if position is not None and attitude_quat is not None:
                    yaw = _quat_to_yaw(attitude_quat)
                    # CTRL-COMPARE: goal_heading from the selected provider (baseline
                    # world-bearing, or path-tracking lookahead). Position is passed so a
                    # reference-path provider can compute cross-track recovery; the
                    # baseline ignores it and reproduces _goal_heading_body exactly.
                    ginfo = goal_provider.goal_heading(position, yaw)
                    goal_heading = ginfo.goal_heading_body
                else:
                    yaw = 0.0            # no odometry yet -- assume aligned, corrected next frame
                    goal_heading = 0.0   # Step CM: yaw is now also passed to the controller below

                # measured yaw rate: finite difference of yaw over sim time (frame-agnostic,
                # no dependence on gyro-frame conventions). NaN until a prior sample exists.
                meas_yaw_rate = float("nan")
                yaw_valid = bool(odom_valid and position is not None
                                 and attitude_quat is not None and math.isfinite(yaw))
                if yaw_valid and prev_yaw is not None and prev_yaw_t is not None:
                    dt_yaw = packet.t_capture - prev_yaw_t
                    if dt_yaw > 0:
                        meas_yaw_rate = _wrap_to_pi(yaw - prev_yaw) / dt_yaw
                prev_yaw = yaw if yaw_valid else None
                prev_yaw_t = packet.t_capture if yaw_valid else None

                t1 = time.perf_counter()
                cmd, ctrl_state = self.controller.step(belief, goal_heading, yaw, packet.t_capture, ctrl_state)
                controller_latencies.append((time.perf_counter() - t1) * 1000.0)

                processing_limit = getattr(self.vehicle, "_command_timeout_s", None)
                if processing_limit is not None and time.perf_counter() - t0 >= processing_limit:
                    # The republisher has already braked during the stall. Do
                    # not restart motion using a result that arrived too late.
                    stop_reason = "perception_timeout"
                    break
                if cfg.goal_xy is not None:
                    if not odom_valid:
                        cmd = ControlCommand(0.0, 0.0, "blind", None, cmd.telemetry)
                        ctrl_state.previous_yaw_rate = 0.0  # slew resumes from the command actually sent
                    elif position is not None:
                        distance = math.hypot(position[0] - cfg.goal_xy[0], position[1] - cfg.goal_xy[1])
                        cmd.fwd_vel = 0.0 if distance <= cfg.goal_tolerance_m else min(cmd.fwd_vel, distance)
                self.vehicle.send(cmd)
                inference_to_command_ms = (time.perf_counter() - t0) * 1000.0

                mode_counts[cmd.mode] = mode_counts.get(cmd.mode, 0) + 1
                # Step CM: count GENUINE re-commitments (target_world_bearing changing),
                # not cmd.target_sector -- that field now reports which sector the HELD
                # bearing currently projects into, which drifts on its own (e.g.
                # sector4->sector3->...->5) as the vehicle turns toward an UNCHANGED
                # commitment. Counting that drift as a "switch" would count convergence,
                # not decisions -- exactly the confusion Step CL's diagnosis flagged
                # (comparing a stale, meaningless-in-a-rotating-frame index).
                if ctrl_state.target_world_bearing != prev_target_world_bearing:
                    n_switches += 1
                    prev_target_world_bearing = ctrl_state.target_world_bearing

                if t_first is None:
                    t_first = packet.t_capture
                t_last = packet.t_capture
                n_frames += 1

                if log_f is not None:
                    tel = cmd.telemetry
                    row = {
                        "seq": packet.seq,
                        "t_capture": packet.t_capture,
                        "scores": belief.scores.tolist(),
                        "valid": belief.valid.tolist(),
                        "confidence": belief.confidence,
                        "ttc_s": belief.ttc_s.tolist() if belief.ttc_s is not None else None,
                        "forward_depth_m": belief.forward_depth_m.tolist() if belief.forward_depth_m is not None else None,
                        "source": belief.source,
                        "perception_latency_ms": belief.latency_ms,
                        "features": dict(zip(self._feature_names, feats.tolist())),
                        "cmd": {"fwd_vel": cmd.fwd_vel, "yaw_rate": cmd.yaw_rate,
                                "mode": cmd.mode, "target_sector": cmd.target_sector},
                        "vehicle_position": position,
                        "vehicle_velocity": velocity,
                        "odom_valid": odom_valid,
                        # CTRL-COMPARE: control diagnostics + reference tracking.
                        "yaw": (yaw if yaw_valid else None),
                        "measured_yaw_rate": (None if math.isnan(meas_yaw_rate) else meas_yaw_rate),
                        "commanded_yaw_rate": cmd.yaw_rate,   # signed; == cmd.yaw_rate, kept explicit
                        "target_world_bearing": ctrl_state.target_world_bearing,
                        "heading_error": (tel.heading_error if tel else None),
                        "switch_reason": (tel.switch_reason if tel else None),
                        "candidate_costs": ({str(k): v for k, v in tel.costs.items()} if tel else {}),
                        "min_score_valid": (tel.min_score_valid if tel else None),
                        "cross_track_error": (ginfo.cross_track_error if ginfo else None),
                        "lookahead_xy": (list(ginfo.lookahead_xy) if (ginfo and ginfo.lookahead_xy) else None),
                        "obs_age_ms": None,  # capture-to-command age unavailable across these clocks
                        "inference_to_command_ms": inference_to_command_ms,
                    }
                    log_f.write(json.dumps(row) + "\n")

                if cmd.mode == "blind":
                    if unsupported_since is None:
                        unsupported_since = packet.t_capture
                else:
                    unsupported_since = None

                keep_running = True
                if self.on_frame is not None:
                    keep_running = self.on_frame(packet, belief, feats, odom, cmd) is not False

                if position is not None:
                    x, y = position[0], position[1]
                    obs, dist = nearest_obstacle(x, y)
                    if dist <= cfg.collision_radius_m:
                        collided = True
                        collision_zone = obs["zone"]
                        collision_xy = (x, y)
                        stop_reason = "collision"
                        break
                    arrived = (math.hypot(x - cfg.goal_xy[0], y - cfg.goal_xy[1]) <= cfg.goal_tolerance_m
                               if cfg.goal_xy is not None else x >= cfg.goal_x_m)
                    if arrived:
                        completed = True
                        stop_reason = "endpoint_reached" if cfg.goal_xy is not None else "goal_reached"
                        break

                if (cfg.unsupported_timeout_s is not None and unsupported_since is not None
                        and packet.t_capture - unsupported_since >= cfg.unsupported_timeout_s):
                    stop_reason = "perception_unavailable"
                    break
                if not keep_running:
                    stop_reason = "user_stop"
                    break

                prev_packet = packet
        finally:
            # Explicit stop -- do not leave the vehicle flying forward on
            # whatever the last command was (into the thing it just hit,
            # in the collision case). The 15Hz republish timer would
            # otherwise keep sending the pre-collision command forever.
            self.vehicle.send(ControlCommand(fwd_vel=0.0, yaw_rate=0.0, mode="cruise", target_sector=None))
            if log_f is not None:
                log_f.close()

        return RunResult(
            collided=collided,
            collision_zone=collision_zone,
            collision_xy=collision_xy,
            completed=completed,
            n_frames=n_frames,
            duration_s=(t_last - t_first) if (t_first is not None and t_last is not None) else 0.0,
            mode_counts=mode_counts,
            n_target_switches=n_switches,
            perception_latency_ms=perception_latencies,
            controller_latency_ms=controller_latencies,
            log_path=cfg.log_path,
            stop_reason=stop_reason,
        )
