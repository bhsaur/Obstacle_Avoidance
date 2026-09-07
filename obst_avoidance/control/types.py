"""Control-layer types, shared by SectorController and (once it exists)
whatever calls it -- VehicleInterface, the orchestrator, offline replay
tooling. No perception logic here (see obst_avoidance/perception/ for
that); this module only describes what a controller step consumes and
produces.

NO WALL CLOCK anywhere in this module or in control/sector.py. All
timing derives from packet.t_capture (sim time), never time.time() or
similar -- RTF has been measured between 0.15 and 1.25 across sessions
(see README), so wall-clock-based timing would make controller
behaviour differ between runs at different RTF, making evaluation arms
non-comparable. ControlState.last_t holds a t_capture value, not wall
time.

STATE IS PASSED IN AND RETURNED, never held as instance attributes on
SectorController. Two reasons: (1) paired-seed determinism for
evaluation -- two runs given the same belief sequence and the same
initial ControlState must produce identical commands, which isn't
guaranteed if state lives on a stateful object that might carry hidden
history; (2) the ability to replay a logged belief sequence offline
(no simulator running) and reproduce commands exactly, by threading
ControlState through step() calls the same way the orchestrator does
live -- controller debugging without a sim in the loop.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class ControlCommand:
    """One controller output. fwd_vel and yaw_rate are body-frame
    setpoints (fwd_vel: +x, forward; yaw_rate: rad/s, matching the
    ENU/mavros convention record_pass.py's velocity setpoints already
    use -- NOT the body-frame-NED gyro convention frame_source uses;
    see frame_source/sim.py's docstring for that distinction, it does
    not apply here)."""

    fwd_vel: float
    yaw_rate: float
    mode: str  # "cruise" | "avoid" | "blind"
    target_sector: Optional[int]
    telemetry: Optional["ControlTelemetry"] = None  # CTRL-COMPARE: diagnostic-only, see below.
                                                     # Default None keeps back-compat and the
                                                     # paired-seed determinism contract (a field
                                                     # that never affects fwd_vel/yaw_rate/mode).


@dataclass
class ControlTelemetry:
    """CTRL-COMPARE: per-step controller diagnostics for logging/plotting.
    NOT an input to any decision and never consumed by VehicleInterface --
    purely so the orchestrator can log WHY the controller did what it did
    (candidate costs, which target won, why a switch did/didn't happen,
    the heading error that produced yaw_rate). Keeping it here rather than
    recomputing downstream means the plot/log reflect the controller's
    ACTUAL internal quantities, not a re-derivation that could drift."""

    costs: dict                      # sector index -> VFH cost this frame (valid candidates only)
    best_candidate: Optional[int]    # argmin-cost candidate this frame
    current_target: Optional[int]    # sector the held world bearing projects into now
    target_world_bearing: Optional[float]
    heading_error: Optional[float]   # theta_target_body = wrap(target_world_bearing - yaw)
    switch_reason: str               # why target was (not) changed -- see sector.py branches
    is_new_commitment: bool          # True iff the world bearing was (re)derived this frame
    min_score_valid: Optional[float] # min score over valid sectors (drives avoid/cruise mode)


@dataclass
class ControlState:
    """Carried across SectorController.step() calls by the CALLER
    (orchestrator or replay tooling) -- never held internally by the
    controller itself. See module docstring for why.

    target_world_bearing (Step CM, replaces the old target_sector field):
    the committed target, as a WORLD-FRAME bearing (rad, same ENU
    convention as everything else in control/), not a sector INDEX.
    Step CL found sector-index latching was silently broken in a
    rotating frame: sector 4 at frame 0 and sector 4 at frame 9 are
    DIFFERENT world directions once the vehicle has yawed between them
    (sectors are fixed slices of the CURRENT camera image, not stable
    world references) -- so a "held sector index" compared frame to
    frame was comparing against a moving target that happened to share
    a label, both for yaw-rate computation (which then never converged,
    reissuing the same command every frame -- Step CL's actual failure)
    and for hysteresis (which was therefore never comparing like
    against like -- the actual cause of `switches=1` in every closed-
    loop run through Step CL). A world bearing has no such problem: it
    means the same physical direction regardless of how much the
    vehicle has rotated since it was set, and SectorController projects
    it into the current camera frame fresh every step (sector.py's own
    docstring) rather than assuming a stale index still means anything."""

    target_world_bearing: Optional[float]

    seconds_on_target: float
    """Step CQ (replaces the old frames_on_target COUNT): elapsed sim
    TIME (seconds, derived from consecutive packet.t_capture values,
    never wall clock), not a frame count. Step CP found HeavyStage's
    one collision run (heavy_co_run3, Step CO) never switched away from
    a collapsing centre score for a completely mundane reason: no bug,
    just hysteresis_min_frames=3 -- a THRESHOLD PICKED FOR CHEAP-STAGE'S
    ~25Hz RATE -- silently meant "wait for 3 REAL SECONDS of confirmation"
    under CheapStage (0.12s) but "wait for 3.75 REAL SECONDS" under
    HeavyStage's ~0.8Hz rate, inside a danger window that was itself only
    ~2s wide. Same bug class as n_sectors (Step CN) and the world-bearing
    fix (Step CM): a control-layer constant silently assuming a fact
    about whichever perception stage happens to be feeding it.

    SIGN CONVENTION (unchanged in spirit from the old frames_on_target,
    only the unit changed from frames to seconds -- see sector.py's
    step() for the exact arithmetic): positive = elapsed seconds
    dwelling on the current (winning) target; negative = elapsed
    seconds of an in-progress, not-yet-committed challenge (magnitude);
    exactly 0.0 = neutral -- no streak in either direction (a fresh
    commitment, a blind reset, or a challenger that exists but hasn't
    cleared hysteresis_margin at all). Each qualifying frame contributes
    ITS OWN dt (this step's t_capture minus state.last_t) to whichever
    streak is in progress, so "3 frames at 25Hz" and
    hysteresis_min_seconds=0.12 (Step CQ's converted default) reproduce
    IDENTICAL behaviour to the old code at CheapStage's rate, while a
    single HeavyStage frame -- whose own dt alone already exceeds
    0.12s -- can now correctly complete a challenge in ONE frame instead
    of being forced to wait for three. That is Step CQ's entire point,
    not an oversight: see sector.py's module docstring for the
    noise-rejection trade-off this implies at slow perception rates.
    """

    last_source: Optional[str]  # belief.source from the previous step -- for the
                                 # hysteresis bypass on source switch, see sector.py
    last_t: Optional[float]     # previous step's packet.t_capture (sim time, NOT wall
                                 # clock) -- Step CQ: now genuinely READ every step (dt =
                                 # t_capture - last_t) to compute seconds_on_target, not
                                 # merely carried through unused as it was through Step CM.

    # Used only by the opt-in robust hysteresis variant.
    challenger_sector: Optional[int] = None
    challenger_since: Optional[float] = None

    previous_yaw_rate: float = 0.0  # v4 only; zero after a blind stop

    reorienting: bool = False  # v5: stationary turn until goal is inside central FOV
