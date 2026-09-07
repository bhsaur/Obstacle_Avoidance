"""Shared trajectory geometry -- sectoring, focus of expansion, miss
distance, and cone weighting. Used by CheapStage today; HeavyStage must
use this SAME module, not its own copy. If the two stages reasoned about
trajectory geometry differently, a disagreement between them would stop
measuring perception quality (the thing the learned gate is supposed to
learn from) and start measuring an architectural asymmetry this codebase
introduced -- silently invalidating the whole "learning to defer"
premise. Keep every trajectory assumption in exactly one place.

All angles/rates/velocities here are in the same conventions the rest
of the package uses: gyro is body-frame NED (see frame_source/sim.py),
"forward" is +x (matching record_pass.py's fly_forward convention and
flow_explore.py's confirmed camera-Z=forward=body-X mapping, Step U).

ASYMMETRY REMOVED (Step BY). miss_distance()/cone_weight() were
CheapStage-only through Steps AZ-BS: DA-V2's RELATIVE-depth checkpoint
had no fixed scale (a scale/shift fit on one obstacle failed held-out
on another, Step AZ, R^2=-7.98), so HeavyStage had no metric quantity
to offer these functions, and passing anything through them would have
fabricated a distance it couldn't actually supply. Step BW switched
HeavyStage to a METRIC checkpoint instead
(depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf) -- its
output IS a real-world distance Z by construction, so the asymmetry's
root cause is gone. HeavyStage now computes miss directly as
`miss_distance_from_z(x_norm, y_norm, z) = z * sqrt(x_norm^2+y_norm^2)`
(added below) rather than through miss_distance()'s tau/vz route --
CheapStage only has tau (time-to-contact) and needs vz to recover an
implied Z (Z=vz*tau); HeavyStage already HAS Z directly, so the
detour through a time quantity it doesn't measure would be needless
indirection, not extra rigor. Both routes feed the SAME shared
cone_weight() below -- that's what "shared, not reimplemented" means
here: the two stages measure fundamentally different things (flow-
derived time vs. monocular-depth-derived distance) and each converts
what it actually has into a miss distance its own way, but every
consumer downstream of "here is a miss distance in metres" is one
function, used by both, not two divergent implementations of the same
idea.
"""
import numpy as np

N_SECTORS = 11  # vertical strips, left-to-right (Step BR, raised from 5). A 1.0m Zone B box
                 # needs finer angular resolution to be LOCATED, not just detected -- at 5
                 # sectors the centre sector alone spans 34.6deg (Step BG), wide enough that a
                 # small near obstacle is a small fraction of the sector's own pixels and gets
                 # diluted by whatever aggregation runs over it (Step BP/BQ). Zone C's 1.5-2m
                 # gaps have the same problem from the other direction: flow-only spent 92.4%
                 # of its Zone C frames in "avoid" mode and still crashed (Step AX) -- it was
                 # screaming danger continuously without ever having enough angular resolution
                 # to tell WHICH direction was actually clear. Both stages derive n_sectors from
                 # THIS constant (CheapStage, HeavyStage both default to geometry.N_SECTORS) --
                 # changing it here changes it for both, deliberately (Step BR: "do not
                 # special-case heavy"). control/sector.py's SectorControllerConfig.n_sectors
                 # is NOT wired to this constant (a separate hardcoded 5) -- known, not fixed
                 # here, since this change is scoped to offline perception analysis (Step BP-BS),
                 # not the closed-loop controller.


def sector_index(x_px: np.ndarray, width: float, n_sectors: int = N_SECTORS) -> np.ndarray:
    """Assigns each point's pixel x-coordinate to a vertical sector index
    in [0, n_sectors). Sector 0 = far left, n_sectors-1 = far right; for
    the CURRENT default of 11 sectors (Step BR), sector 5 is centre. The
    "sector 2 is centre" / "sectors 1-3 (centre)" convention referenced
    elsewhere in this codebase (tools/ttc_sectors.py, older README
    sections) describes the PRE-Step-BR 5-sector default -- callers
    comparing against those must pass n_sectors=5 explicitly, not assume
    the module default still matches."""
    idx = (x_px / width * n_sectors).astype(int)
    return np.clip(idx, 0, n_sectors - 1)


def sector_bearings_rad(width: float, fx: float, cx: float,
                         n_sectors: int = N_SECTORS) -> np.ndarray:
    """Bearing (rad) of each sector's pixel-column CENTRE, computed
    TRIGONOMETRICALLY from the camera's actual intrinsics -- NOT a
    uniform angular split (Step BJ). sector_index() above bins by pixel
    COLUMN, which is correct -- that's how the image itself divides --
    but under a pinhole projection, equal-width pixel columns are NOT
    equal-angle: theta=atan((cx-px)/fx) is nonlinear in px, so the
    centre sector subtends a substantially WIDER angle than an
    equal-pixel-width outer sector. Concretely, for this project's
    camera (fx=205.4696, cx=320, width=640, Step BG): sector boundaries
    are [-57.30, -43.06, -17.30, 17.30, 43.06, 57.30] deg -- the centre
    sector is 34.6deg wide, not the naive equal-share estimate of
    HFOV/n_sectors=~23deg, while the outermost sectors are only
    14.24deg each. Every per-sector bearing used anywhere downstream
    (SectorController's goal-heading cost term, yaw command generation)
    must come from THIS function, not a linear width/n_sectors split --
    using the linear split makes every theta_i systematically wrong.

    Sign convention: POSITIVE = left of image centre = ENU/mavros
    counter-clockwise-from-above (matches control/sector.py's module
    docstring: sector 0, far left, has the largest POSITIVE bearing).
    Pixel column increases RIGHTWARD, so this is (cx - px_centre)/fx,
    the negation of the naive atan((px_centre-cx)/fx) -- getting this
    sign backwards would silently invert every steering decision."""
    px_centres = (np.arange(n_sectors) + 0.5) * width / n_sectors
    return np.arctan((cx - px_centres) / fx)


def foe(cx: float, cy: float) -> tuple:
    """Focus of expansion, in pixel coordinates. Assumed to be the image
    centre (cx, cy) -- valid for level forward flight with a rigid,
    forward-looking camera (confirmed for this airframe: Step B/the
    gimbal docstring in frame_source/sim.py -- the gimbal is rigid, not
    stabilizing, so the camera rotates 1:1 with the airframe and the FOE
    doesn't wander independently of body attitude). Does NOT estimate
    FOE from ego-motion/vanishing-point tracking -- if the flight path
    ever includes sustained non-level or non-forward motion (e.g. a
    lateral avoidance maneuver in progress), this assumption degrades
    and an online FOE estimate would be needed instead. Not attempted
    here; flagged for whoever integrates this with the controller."""
    return cx, cy


def normalized_coords(x_px: np.ndarray, y_px: np.ndarray, cx: float, cy: float,
                       fx: float, fy: float) -> tuple:
    """Pixel -> normalized image-plane coordinates, SAME normalization
    flow_explore.py's rotational_flow() uses ((px-cx)/fx, (py-cy)/fy) --
    composes directly with that code's outputs."""
    return (x_px - cx) / fx, (y_px - cy) / fy


def miss_distance(x_norm: np.ndarray, y_norm: np.ndarray, tau: np.ndarray,
                   vz: float) -> np.ndarray:
    """miss = |vz * tau| * sqrt(x_norm^2 + y_norm^2) -- the radial
    (world-frame) offset a tracked point's underlying obstacle would
    pass the camera's forward path at, at the estimated time of closest
    approach, assuming constant velocity.

    Derivation, matching the literal spec "miss = x * Tz * tau": for a
    world point at depth Z with lateral offset X from the optical axis,
    normalized image coordinate x_norm = X/Z. If Tz (vz here -- the
    ACTUAL forward/closing velocity, see below) is roughly constant and
    tau = Z/Tz is this point's own time-to-contact, then at time tau the
    point has closed the full depth Z, and its lateral offset X = x_norm
    * Z = x_norm * (Tz * tau) is exactly the miss distance for the
    horizontal axis. The same derivation applies unchanged to the
    vertical axis (y_norm, Tz, tau) since nothing in it is
    horizontal-specific -- only the horizontal case was spelled out in
    the original spec. This function combines both into one radial
    distance because the cone weight below needs a single scalar "how
    far off the flight-path axis is this point's obstacle," not two
    independent axis-wise numbers; a point offset diagonally from the
    path is not less dangerous than one offset the same total amount
    along a single axis.

    vz is Tz: the ACTUAL forward velocity component (m/s) from
    INTERPOLATED live odometry (see Step V's odom buffering in
    record_pass.FlightControl.state_at()) -- deliberately NOT
    cmd_linear_x, which is what was commanded, not what the vehicle
    actually did. Using cmd_linear_x here would silently substitute
    intent for measurement in exactly the way Step V was built to stop
    happening for position/velocity/attitude.
    """
    return np.abs(vz * tau) * np.sqrt(x_norm ** 2 + y_norm ** 2)


def miss_distance_from_z(x_norm: np.ndarray, y_norm: np.ndarray, z: np.ndarray) -> np.ndarray:
    """miss = |z| * sqrt(x_norm^2 + y_norm^2) -- the HeavyStage route to
    the same miss-distance quantity miss_distance() above computes for
    CheapStage (Step BY). For a world point at depth Z with normalized
    image coordinate x_norm=X/Z, the lateral offset is X = x_norm * Z
    directly -- no time-to-contact or velocity needed, unlike
    miss_distance()'s tau/vz route, because a METRIC depth checkpoint
    (Step BW) already supplies Z directly; recovering it via Z=vz*tau
    the way CheapStage must would be needless indirection through a
    quantity (time) this stage doesn't measure. Feeds the same
    cone_weight() below as CheapStage's route -- see module docstring.

    NOT CURRENTLY CALLED FROM HeavyStage.infer() -- deliberately, not an
    oversight. Step BX measured this checkpoint's absolute accuracy on
    `boxB1_approach` and found MAE=4.66m against known ground-truth
    range, with predicted values 2.5-4.8x the true distance and that
    ratio GROWING closer in (2.54x @4m, 3.31x @3m, 4.82x @2m) -- not a
    fixable constant scale/shift, more consistent with a real VKITTI-
    to-synthetic-Gazebo domain gap than a calibration bug. The
    CORRELATION survives this (+0.722, Step BX) because the systematic
    overprediction preserves RANKING even though the absolute metres are
    wrong -- exactly the property HeavyStage's sector-selection actually
    needs (it ranks sectors, it never consumes a metre value directly).
    But `miss_distance_from_z()`'s whole output is a metre value, not a
    rank -- feeding these Z's through it would produce a miss distance
    with the same 2.5-4.8x error baked in, and cone_weight() would then
    be trusting an "airframe-radius-scale" comparison (r_safe, itself a
    real metres constant) against a distance that isn't actually
    trustworthy at that scale. Use this function once the accuracy gap
    is closed (a fixed calibration fit might work here specifically,
    unlike Step AZ's failed attempt on the relative checkpoint, since
    this error looks more like a smooth range-dependent bias than an
    unscaled affine-invariant output -- not verified, just plausible)."""
    return np.abs(z) * np.sqrt(x_norm ** 2 + y_norm ** 2)


def cone_weight(miss: np.ndarray, r_safe: float) -> np.ndarray:
    """w = 1 / (1 + (miss/r_safe)^2) -- soft weight in (0, 1], 1 at
    miss=0 (dead ahead on the flight path), 0.5 at miss=r_safe, falling
    off smoothly beyond that. r_safe is a safety-radius scale in
    METERS (same units as miss, which is world-frame via vz*tau) --
    roughly "how far off-path can an obstacle be and still matter,"
    e.g. the airframe's own half-width plus margin. Callers must supply
    a documented value for their own airframe/margin choice; no default
    is baked in here since guessing an unverified physical constant is
    worse than requiring the caller to state it explicitly."""
    return 1.0 / (1.0 + (miss / r_safe) ** 2)


def normalize_scores_to_span(scores: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Stretch a belief's VALID scores to fill [0,1] via min-max ACROSS
    SECTORS WITHIN THIS ONE FRAME (Step CG) -- explicitly NOT across
    time/frames, which is the normalization that collapsed before
    (Step AY: per-frame-across-TIME min-max let an obstacle filling the
    view look "typical" once every recent frame was similarly close).
    This is a different axis entirely: SectorController compares scores
    ACROSS SECTORS within one belief to pick a target, so what matters
    for that decision is the SPREAD across sectors at one instant, not
    across time.

    Step CE's root cause: HeavyStage's metric checkpoint (Step BW)
    produces genuine, correctly-signed sector-to-sector variation, but
    a narrow one (~0.44-0.53 typical) -- smaller than
    SectorController's fixed hysteresis_margin (0.15) plus the fixed
    goal-heading cost gap between adjacent sectors, so a real signal
    could never numerically clear the bar needed to change the selected
    sector. This is Step AP's comparability contract failing in
    practice: CheapStage and HeavyStage are both bounded [0,1] but were
    NOT on comparable scales (CheapStage typically spans much of that
    range; HeavyStage compressed to a tenth of it). Stretching is a
    monotonic affine map -- it preserves rank order exactly, it does
    not invent a difference that wasn't there, it makes an
    ALREADY-PRESENT one usable by a margin-based decision.

    Degenerate case (all valid scores identical, span=0): returned
    UNCHANGED, not stretched to some arbitrary spread -- that is a
    genuinely uniform reading (no sector looks better than any other),
    which is real information, not a normalization artifact to smooth
    over. Applied identically to BOTH stages (not just HeavyStage) so
    neither one gets an unequal, one-sided rescue -- see cheap.py and
    heavy.py's own infer() for where this is called."""
    valid_scores = scores[valid]
    if valid_scores.size == 0:
        return scores
    lo, hi = float(valid_scores.min()), float(valid_scores.max())
    if hi <= lo:
        return scores
    out = scores.copy()
    out[valid] = (scores[valid] - lo) / (hi - lo)
    return out
