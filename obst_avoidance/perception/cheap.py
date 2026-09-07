"""CheapStage -- Stage 1 of the two-stage adaptive perception pipeline:
Lucas-Kanade optical flow, gyro-based de-rotation, per-sector
time-to-contact. Fast and cheap, and -- per Step R2 -- blind in
specific, DOCUMENTED ways (untextured obstacles produce zero trackable
points; small-point-count sectors are noisy, not just "less confident")
that the learned gate is meant to detect and defer past to HeavyStage,
not that this stage should try to paper over with a fabricated number.
See types.SectorBelief's docstring for why `valid` exists at all.

track_pair() and rotational_flow() live here as the canonical,
validated implementations, moved from tools/flow_explore.py (which now
imports them from here instead of defining its own copy) per Step U's
diagnostic-phase validation:
  - rotational_flow()'s mapping (wx_cam=pitch, wy_cam=yaw, wz_cam=roll)
    is CONFIRMED, not assumed: the Step U roll_pitch_wiggle sweep showed
    wx_cam=pitch beating wx_cam=roll by 66.4% lower median residual.
    Do not re-derive it.
  - track_pair()'s three-stage cleaning (vignette y>100, LK-error
    top-quartile rejection, 3x-median magnitude rejection) is validated
    against the yaw_only/wall_approach recordings documented in
    flow_explore.py's module docstring. Do not reimplement it -- the
    tracking/rejection LOGIC below is unchanged from that validated
    version. The only difference is the return value is EXTENDED (err
    and n_initial, previously computed internally and discarded) so
    CheapStage/features.py can build the LK-residual and
    fraction-of-features-lost features from data track_pair() already
    had, not from a second re-derivation of it.
"""
import time

import cv2
import numpy as np

from . import geometry
from .contours import ContourLoomingChannel
from .features import extract_features
from .types import SectorBelief

GF_PARAMS = dict(maxCorners=300, qualityLevel=0.01, minDistance=7, blockSize=7)
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
VIGNETTE_Y_MIN = 100  # rotor-blur/gimbal-housing vignette, see track_pair docstring

# -- CheapStage tuning constants, all documented individually below --
# MIN_POINTS_PER_SECTOR: set from measured per-frame per-sector survival
# (Steps Y-AB, see README's "CheapStage Phase 2 -- point-count diagnosis"
# section for the full tables), NOT from R2's aggregate-over-~200-frames
# numbers the previous value (20) was actually based on -- that
# threshold never cleared on real per-frame data (0% valid-rate on every
# wall_approach recording). Measured on the CURRENT shipped pipeline
# (with the LK-error-quartile filter, estimator (a)) on wall_approach_1p0
# -- the hardest available recording, since its centre sector is
# genuinely textureless and structurally always empty regardless of this
# threshold: valid-rate holds at 88.0% down to min_points=5, drops to
# 56.7% at 10, and hits exactly 0.0% at 15. 10 is chosen as the largest
# value that still clears the majority of frames on the hardest
# recording, while requiring double-digit points for the per-sector
# median to mean something (a median of 1-4 points, which is what
# thresholds below ~5 would accept, isn't meaningfully "robust" no
# matter what the estimator is). Steps AA/AB separately found that a
# direction-agnostic tau estimator and dropping the quartile-rejection
# stage both raise achievable valid-rate further (and, for the cleaning
# change specifically, may even fix a wrong-signed distance/tau
# correlation at low point counts) -- NOT adopted here: only this one
# constant is in scope for this change, per the task that produced it.
MIN_POINTS_PER_SECTOR = 10
# TAU_ESTIMATOR: which per-point tau formula + per-sector aggregator to
# use. Set from Steps AG-AI (see README's "CheapStage Phase 2 -- tau
# estimator fix"), after AF found the previous default ("divergence")
# fails its own acceptance test on a textured approach -- tau spiked to
# its highest recorded value (19.4s, EMA-smoothed) at the closest
# distance to a real, trackable obstacle, moments before collision.
# AH found why: as an obstacle looms, it becomes too close/low-contrast
# to track at all, and the survivors in its sector become a FEW points
# on unrelated nearby objects/background -- a median over that mix is
# dominated by whichever is more numerous, not by the nearest thing.
#   "divergence" (previous default): per-point tau = r / radial_rate
#     (radial_rate = flow projected onto the outward-radial direction;
#     points with radial_rate <= 0 are DISCARDED, not clamped or abs()'d
#     -- see infer()), aggregated per sector by MEDIAN.
#   "percentile" (current default): per-point tau = r / |flow
#     magnitude| (direction-agnostic, no point excluded by a sign
#     check), aggregated per sector by the LOW PERCENTILE
#     TAU_PERCENTILE, not the median -- collision risk is set by the
#     nearest thing in a direction, not the typical thing, and a low
#     quantile stays robust to a sector mixing one close object with
#     several distant background points in a way a median cannot.
# VERIFY on the textured approach (treeA2, 244 pre-collision frame
# pairs, min_points=10): divergence+median correlation(distance,
# min_tau) = -0.061 (wrong-signed, unstable -- WORSENS as min_points
# rises: 0.390 -> -0.061 from min_points=1 to 10); percentile+20th
# correlation = +0.462 (correctly signed, IMPROVES as min_points rises:
# 0.413 -> 0.462). Max recorded tau over the whole approach: 465s
# (divergence, one wild outlier) vs. 5.3s (percentile) -- the dangerous
# late spike is gone, not just damped.
# NOT a universal fix: on wall_approach and narrow_approach, percentile
# does NOT flip the sign (still negative on both) -- because per Step
# AE, those recordings' target-facing sector rarely or never supplies
# min_tau at all; no per-sector aggregator can fix a sector that has no
# real signal in the first place. percentile fixes the AGGREGATION
# failure mode (median diluted by background), not the SECTOR-SELECTION
# failure mode (AE) -- those are different problems.
TAU_ESTIMATOR = "percentile"  # "percentile" | "divergence"
# Step DH -- world-geometry row band, DEFAULT OFF. Same derivation
# HeavyStage uses (Step CJ): a symmetric pixel band around the horizon
# (cy) tall enough to contain an obstacle of half-height
# OBSTACLE_HALF_HEIGHT_M at the closest range of interest
# ROW_BAND_MIN_RANGE_M -- half_band_px = fy * half_height / min_range.
# CheapStage today only crops the top VIGNETTE_Y_MIN rows; this optional
# band additionally crops sky ABOVE the band and near-ground BELOW it,
# where DH.1 found ~43% of tracked points sit and ~61% of the danger-
# setting 20th-pct-tau tail is ground-plane. Left OFF until DH.3 shows
# it improves tau-vs-distance correlation more than the point loss costs
# (masking costs points, and cheap is already point-starved).
ROW_BAND_OBSTACLE_HALF_HEIGHT_M = 0.75  # matches HeavyStage.OBSTACLE_HALF_HEIGHT_M (Zone B 1.5m box)
ROW_BAND_MIN_RANGE_M = 2.0              # matches HeavyStage.ROW_BAND_MIN_RANGE_M -> band rows 162-318
TAU_PERCENTILE = 20  # verified at exactly this value -- see the module docstring's
                      # note: not swept/tuned to produce a better-looking correlation.
TAU_CAP_S = 30.0            # clip before EMA/scores so "wide open" sectors don't carry an
                             # unbounded (or near-infinite, e.g. yaw_only) tau downstream.
EMA_ALPHA = 0.5              # short EMA, no particular tuning behind this value yet.
CONE_R_SAFE_M = 0.6          # placeholder safety radius (airframe half-width + margin, m) --
                             # NOT verified against this workspace's actual Iris model
                             # dimensions. Update once a real spec is available.
CONE_IN_THRESHOLD = 0.5      # cone_weight > this => "in cone" for cone_min_tau. 0.5 is the
                             # weight function's own half-max point (miss == r_safe).
NOMINAL_CONE_AREA_FRACTION = 0.3  # assumed fraction of frame area the flight-path cone
                                   # nominally covers, for cone_density_ratio. A documented
                                   # placeholder (the cone has no fixed pixel-space shape --
                                   # each point's own miss depends on its own tau -- see
                                   # geometry.miss_distance), not empirically derived.


def tau_to_score(tau_s, tau_cap_s=TAU_CAP_S):
    """Step AP's comparability contract: SectorBelief.scores MUST be
    [0,1] traversability, higher=safer, on the SAME scale regardless of
    source (cheap or heavy) -- see SectorBelief's docstring. CheapStage's
    internal tau (seconds; EMA-smoothed, capped at tau_cap_s -- see
    TAU_CAP_S) is converted to that scale by the simplest thing that
    satisfies the contract: linear, score = min(tau / tau_cap_s, 1.0).
    tau=0 (about to collide) -> score=0; tau>=tau_cap_s (wide open) ->
    score=1. Deliberately NOT a saturating/log/sigmoid mapping that
    would give more resolution near tau=0 -- HeavyStage doesn't exist
    yet to compare against, so a cleverer curve now would be tuning
    against nothing. NaN (invalid) passes through NaN, unchanged --
    normalization must never turn "don't know" into a number.

    This is the ONLY place tau in seconds becomes a score. Everything
    else in this module (feature vector, EMA state, TAU_CAP_S itself)
    stays in seconds -- only SectorBelief.scores, the value the
    controller and any future gate actually compare across sources,
    goes through this conversion."""
    return np.minimum(tau_s / tau_cap_s, 1.0)


def track_pair(gray0, gray1):
    """Returns (p0, p1, err, n_initial), or None if too few points
    survive at any stage. p0/p1: Nx2 float32 pixel coords, the inlier
    pairs surviving all three cleaning stages (unchanged logic from the
    validated version in flow_explore.py's history). err: length-N LK
    tracking error (px), carried through the SAME masks as p0/p1 (the
    original track_pair discarded this after the error-percentile
    filter -- kept here, nothing about which points are kept/rejected
    changed). n_initial: raw corner count from goodFeaturesToTrack,
    before any rejection -- denominator for fraction-of-features-lost."""
    pts0 = cv2.goodFeaturesToTrack(gray0, **GF_PARAMS)
    if pts0 is None or len(pts0) < 10:
        return None
    n_initial = len(pts0)
    pts1, st, err = cv2.calcOpticalFlowPyrLK(gray0, gray1, pts0, None, **LK_PARAMS)
    st = st.reshape(-1).astype(bool)
    p0 = pts0.reshape(-1, 2)[st]
    p1 = pts1.reshape(-1, 2)[st]
    err = err.reshape(-1)[st]

    keep = p0[:, 1] > VIGNETTE_Y_MIN
    p0, p1, err = p0[keep], p1[keep], err[keep]
    if len(p0) < 12:
        return None

    err_thresh = np.percentile(err, 75)
    good = err <= err_thresh
    p0, p1, err = p0[good], p1[good], err[good]
    if len(p0) < 8:
        return None

    mags = np.linalg.norm(p1 - p0, axis=1)
    med = np.median(mags)
    inlier = mags < max(3 * med, 10.0)
    p0, p1, err = p0[inlier], p1[inlier], err[inlier]
    if len(p0) < 6:
        return None
    return p0, p1, err, n_initial


def rotational_flow(pts_px, gyro_ned, fx, fy, cx, cy):
    """Step U-confirmed mapping (wx_cam=pitch, wy_cam=yaw, wz_cam=roll,
    all +signs) -- see module docstring. pts_px: Nx2 pixel coords.
    gyro_ned: (rollspeed, pitchspeed, yawspeed), body-frame NED, rad/s.
    Returns Nx2 flow RATE (px/s) from rotation alone -- multiply by dt
    for a displacement."""
    roll, pitch, yaw = gyro_ned
    wx, wy, wz = pitch, yaw, roll
    x = (pts_px[:, 0] - cx) / fx
    y = (pts_px[:, 1] - cy) / fy
    u = x * y * wx - (1 + x ** 2) * wy + y * wz
    v = (1 + y ** 2) * wx - x * y * wy - x * wz
    return np.stack([u * fx, v * fy], axis=1)


def _gradient_energy_per_sector(gray, n_sectors):
    """Mean Sobel gradient magnitude per vertical sector, over the WHOLE
    image region for that sector -- independent of how many corners
    goodFeaturesToTrack found there (a flat, high-corner-count-defeating
    surface can still have strong edges; a busy sky can have neither).
    Sector boundaries match geometry.sector_index() exactly (equal
    n_sectors-way pixel-column split)."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = np.sqrt(gx ** 2 + gy ** 2)
    width = gray.shape[1]
    # Integer pixel x belongs to floor(x / width * n_sectors): its next
    # sector starts at ceil(boundary), not floor(boundary).
    bounds = np.ceil(np.linspace(0, width, n_sectors + 1)).astype(int)
    energy = np.full(n_sectors, np.nan)
    for s in range(n_sectors):
        region = grad_mag[:, bounds[s]:bounds[s + 1]]
        if region.size:
            energy[s] = float(np.mean(region))
    return energy


class CheapStage:
    """Stage 1: LK flow + gyro de-rotation + per-sector TTC + trajectory
    geometry (geometry.py) + feature extraction (features.py). Stateful
    across infer() calls (EMA-smoothed tau per sector, previous
    sector/global counts and tau for temporal derivatives) -- one
    instance per flight/stream, not safe to share across independent
    ones."""

    def __init__(self, intrinsics: dict, n_sectors: int = geometry.N_SECTORS,
                 min_points_per_sector: int = MIN_POINTS_PER_SECTOR,
                 tau_cap_s: float = TAU_CAP_S, ema_alpha: float = EMA_ALPHA,
                 cone_r_safe_m: float = CONE_R_SAFE_M,
                 estimator: str = TAU_ESTIMATOR, tau_percentile: float = TAU_PERCENTILE,
                 use_row_band: bool = False,
                 row_band_obstacle_half_height_m: float = ROW_BAND_OBSTACLE_HALF_HEIGHT_M,
                 row_band_min_range_m: float = ROW_BAND_MIN_RANGE_M,
                 use_contours: bool = True):
        """intrinsics: {fx, fy, cx, cy, width, height} -- the decoded
        form SimFrameSource.intrinsics / LogFrameSource.intrinsics
        already produce. Static for a session (a camera doesn't
        re-calibrate mid-flight), so passed once here rather than per
        infer() call -- matching how the rest of this package treats
        intrinsics (see frame_source's wait_for_intrinsics).

        estimator: "percentile" (default, see TAU_ESTIMATOR's docstring
        for the VERIFY numbers behind this default) or "divergence" --
        kept available for comparison, per the task that replaced it as
        the default. tau_percentile: only used when estimator=
        "percentile"; verified at exactly 20, not swept."""
        if estimator not in ("percentile", "divergence"):
            raise ValueError(f"estimator must be 'percentile' or 'divergence', got {estimator!r}")
        self.fx = intrinsics["fx"]
        self.fy = intrinsics["fy"]
        self.cx = intrinsics["cx"]
        self.cy = intrinsics["cy"]
        self.width = intrinsics["width"]
        self.height = intrinsics["height"]
        self.n_sectors = n_sectors
        self.min_points_per_sector = min_points_per_sector
        self.estimator = estimator
        self.tau_percentile = tau_percentile
        self.tau_cap_s = tau_cap_s
        self.ema_alpha = ema_alpha
        self.cone_r_safe_m = cone_r_safe_m

        # Step DH -- optional world-geometry row band (default OFF, byte-identical
        # to the pre-DH pipeline when off). Same geometry as HeavyStage's Step CJ
        # band: half_band_px = fy * obstacle_half_height / min_range, band =
        # [cy - half_band_px, cy + half_band_px]. For this camera (fy=205.47,
        # cy=240) with 0.75 m half-height at 2.0 m this is rows 162-318, identical
        # to heavy. Applied as an ADD-ON filter on tracked points in infer() --
        # the LK path (track_pair) is untouched.
        self.use_row_band = use_row_band
        half_band_px = self.fy * row_band_obstacle_half_height_m / row_band_min_range_m
        self.row_band_lo = max(0, int(self.cy - half_band_px))
        self.row_band_hi = min(int(self.height), int(self.cy + half_band_px) + 1)

        # Step DI/DJ -- contour/looming channel, a SECOND TTC source merged per
        # sector (see contours.py). Default ON (it is the new CheapStage
        # capability); use_contours=False makes the belief byte-identical to the
        # pre-DI LK-only pipeline AND skips the cv2 detection cost (for the Step DL
        # latency comparison). Holds cross-frame region tracks -> one per instance.
        self.use_contours = use_contours
        self._contours = ContourLoomingChannel(intrinsics, n_sectors) if use_contours else None
        self._smoothed_merged_tau = [None] * n_sectors  # separate EMA state for the MERGED belief,
                                                          # so the LK-only tau_sector_* features keep
                                                          # their exact pre-DI meaning

        self._smoothed_tau = [None] * n_sectors  # frozen at last valid EMA value while a sector is invalid
        self._last_sector_count = None   # np.ndarray, previous call's per-sector point counts
        self._last_sector_tau = None     # np.ndarray, previous call's RAW (pre-smoothing) per-sector tau, NaN where invalid
        self._last_global_count = None
        self._last_t = None
        self.last_tracks = None  # diagnostic view of this inference's LK tracks

    def reset(self):
        """Discard temporal history when starting a new stream/sim-time epoch."""
        self._smoothed_tau = [None] * self.n_sectors
        self._smoothed_merged_tau = [None] * self.n_sectors
        self._last_sector_count = None
        self._last_sector_tau = None
        self._last_global_count = None
        self._last_t = None
        self.last_tracks = None
        if self._contours is not None:
            self._contours.reset()

    def infer(self, packet, prev_packet, odom):
        """packet, prev_packet: FramePacket -- flow computed
        prev_packet -> packet (older -> newer), matching track_pair's
        (gray_a, gray_b) convention used elsewhere in this codebase.
        odom: (position, velocity, attitude_quat, odom_valid), the SAME
        shape record_pass.FlightControl.state_at() returns, interpolated
        to packet.t_capture -- for offline replay (see VERIFY), the
        equivalent tuple built from a packets.jsonl row's
        position/velocity/attitude_quat/odom_valid fields. velocity[0]
        (forward = body/world +x, see geometry.py) is used as Tz for
        miss-distance; cmd_linear_x is never read here -- only actual
        odometry, per Step V.

        Returns (SectorBelief, feature_vector: np.ndarray). Never
        raises on "nothing tracked this frame" -- degrades to an
        all-invalid SectorBelief and a feature vector full of the
        documented NaN/0.0 placeholders instead (see features.py)."""
        t0 = time.perf_counter()
        _, velocity, _, odom_valid = odom
        vz = float(velocity[0]) if velocity is not None else 0.0

        n = self.n_sectors
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
        dt = packet.t_capture - prev_packet.t_capture
        if not np.isfinite(packet.t_capture) or not np.isfinite(prev_packet.t_capture):
            raise ValueError("frame timestamps must be finite")
        # Repeated/reversed pairs have no measurable flow. Reset history so
        # a simulator reset cannot mix old EMA/contour state into a new epoch.
        if dt <= 0 or (self._last_t is not None and packet.t_capture <= self._last_t):
            self.reset()
        state_dt = packet.t_capture - self._last_t if self._last_t is not None else None
        self.last_tracks = None

        gray_b = cv2.cvtColor(packet.image, cv2.COLOR_RGB2GRAY)
        grad_energy = _gradient_energy_per_sector(gray_b, n)

        tracked = None
        if dt > 0:
            gray_a = cv2.cvtColor(prev_packet.image, cv2.COLOR_RGB2GRAY)
            tracked = track_pair(gray_a, gray_b)

        sector_tau = np.full(n, np.nan)
        sector_valid = np.zeros(n, dtype=bool)
        sector_count = np.zeros(n, dtype=float)
        sector_flow_mag = np.full(n, np.nan)
        sector_ground_frac = np.full(n, np.nan)
        cone_min_tau = np.nan
        cone_effective_count = 0.0
        cone_density_ratio = np.nan
        lk_residual = np.nan
        frac_lost = np.nan
        global_count = 0.0

        if tracked is not None:
            p0, p1, err, n_initial = tracked
            # Step DH -- optional world-geometry row band, applied here as an
            # ADD-ON filter on the tracked points (LK/track_pair itself untouched).
            # Keeps only points whose row falls in the obstacle-height band around
            # the horizon; crops sky above and near-ground below. n_initial is left
            # as the raw goodFeaturesToTrack count so frac_features_lost honestly
            # reflects the band's cost. Off by default -> this block is a no-op.
            if self.use_row_band:
                band = (p0[:, 1] >= self.row_band_lo) & (p0[:, 1] < self.row_band_hi)
                p0, p1, err = p0[band], p1[band], err[band]
            self.last_tracks = (p0.copy(), p1.copy())
            global_count = float(len(p0))
            frac_lost = 1.0 - (global_count / n_initial) if n_initial > 0 else np.nan
            lk_residual = float(np.median(err)) if len(err) else np.nan

            raw_flow = p1 - p0
            gyro_avg = (prev_packet.gyro + packet.gyro) / 2.0
            rot_flow = rotational_flow(p0, gyro_avg, fx, fy, cx, cy) * dt
            resid_flow = raw_flow - rot_flow
            flow_mag = np.linalg.norm(resid_flow, axis=1)

            # Per-point TTC: radial expansion rate from the FOE (image
            # centre, geometry.foe -- valid for level forward flight,
            # see that function's docstring). TTC is scale-invariant
            # between pixel and normalized coordinates (both r and
            # radial_rate scale by the same factor), so computing this
            # in pixel space -- as tools/ttc_sectors.py already
            # validated -- gives the identical tau a normalized-space
            # computation would.
            foe_x, foe_y = geometry.foe(cx, cy)
            r_vec = p0 - np.array([foe_x, foe_y])
            r_px = np.linalg.norm(r_vec, axis=1)
            point_tau = np.full(len(p0), np.nan)
            if self.estimator == "divergence":
                # Previous default. Per-point tau = r / radial_rate,
                # radial_rate = flow projected onto the outward-radial
                # direction. Points with radial_rate <= 0 are
                # DISCARDED (not clamped, not abs()'d) -- see
                # TAU_ESTIMATOR's module-level docstring for why this
                # was replaced: it fails its own acceptance test on a
                # textured approach (tau spikes at closest range).
                r_px_safe = np.where(r_px > 1e-6, r_px, 1e-6)
                radial_rate = np.sum(resid_flow * (r_vec / r_px_safe[:, None]), axis=1) / dt
                expanding = radial_rate > 1e-6
                point_tau[expanding] = r_px[expanding] / radial_rate[expanding]
            else:
                # Current default (Step AI). Per-point tau = r / |flow
                # magnitude| -- direction-agnostic, no point excluded
                # by a sign check (only exactly-stationary points,
                # magnitude<=0, are undefined).
                mag = np.linalg.norm(resid_flow, axis=1) / dt
                moving = mag > 1e-6
                point_tau[moving] = r_px[moving] / mag[moving]

            # Miss distance / cone weight: a SEPARATE, world-frame (m)
            # computation from point_tau above -- needs normalized
            # coordinates and Tz (geometry.miss_distance), not pixel
            # radial distance.
            x_norm, y_norm = geometry.normalized_coords(p0[:, 0], p0[:, 1], cx, cy, fx, fy)
            point_miss = geometry.miss_distance(x_norm, y_norm, point_tau, vz)
            point_cone_w = geometry.cone_weight(point_miss, self.cone_r_safe_m)
            point_cone_w = np.nan_to_num(point_cone_w, nan=0.0)  # undefined tau -> not trusted into the cone

            cone_mask = point_cone_w > CONE_IN_THRESHOLD
            if np.any(cone_mask):
                in_cone_tau = point_tau[cone_mask]
                in_cone_tau = in_cone_tau[~np.isnan(in_cone_tau)]
                if len(in_cone_tau):
                    cone_min_tau = float(np.min(in_cone_tau))
            cone_effective_count = float(np.sum(point_cone_w))
            if global_count > 0:
                full_frame_density = global_count / (self.width * self.height)
                cone_pixel_area = NOMINAL_CONE_AREA_FRACTION * self.width * self.height
                cone_density = cone_effective_count / cone_pixel_area
                cone_density_ratio = cone_density / full_frame_density if full_frame_density > 0 else np.nan

            sec_idx = geometry.sector_index(p0[:, 0], self.width, n)
            ground_mask = p0[:, 1] >= cy
            for s in range(n):
                in_sector = sec_idx == s
                n_s = int(np.sum(in_sector))
                sector_count[s] = float(n_s)
                if n_s > 0:
                    sector_flow_mag[s] = float(np.mean(flow_mag[in_sector]))
                    sector_ground_frac[s] = float(np.mean(ground_mask[in_sector]))
                    valid_tau = point_tau[in_sector]
                    valid_tau = valid_tau[~np.isnan(valid_tau)]
                    if n_s >= self.min_points_per_sector and len(valid_tau) > 0:
                        sector_valid[s] = True
                        if self.estimator == "divergence":
                            sector_tau[s] = float(np.median(valid_tau))
                        else:
                            # Low percentile, not median -- collision risk is set
                            # by the nearest thing in the sector, not the typical
                            # thing. See TAU_ESTIMATOR's docstring for the VERIFY
                            # numbers behind this choice.
                            sector_tau[s] = float(np.percentile(valid_tau, self.tau_percentile))

        # -- EMA smoothing (tau-capped) on tau per sector; frozen while invalid --
        smoothed_tau = np.full(n, np.nan)
        for s in range(n):
            if sector_valid[s]:
                capped = min(sector_tau[s], self.tau_cap_s)
                if self._smoothed_tau[s] is None:
                    self._smoothed_tau[s] = capped
                else:
                    self._smoothed_tau[s] = (
                        self.ema_alpha * capped + (1 - self.ema_alpha) * self._smoothed_tau[s]
                    )
                smoothed_tau[s] = self._smoothed_tau[s]
            # else: leave self._smoothed_tau[s] untouched (frozen), this frame's output stays NaN/invalid.

        # -- aggregate stats over sectors (raw, pre-smoothing tau, matching features.py's docs) --
        valid_taus = sector_tau[sector_valid]
        min_tau = float(np.min(valid_taus)) if len(valid_taus) else np.nan
        tau_variance = float(np.var(valid_taus)) if len(valid_taus) >= 2 else np.nan

        centre_idx = list(range(1, n - 1)) if n >= 3 else list(range(n))
        centre_valid = sector_valid[centre_idx]
        centre_taus = sector_tau[centre_idx][centre_valid]
        centre_mean_tau = float(np.mean(centre_taus)) if len(centre_taus) else np.nan
        centre_min_tau = float(np.min(centre_taus)) if len(centre_taus) else np.nan

        # -- temporal derivatives vs. the previous call --
        sector_count_rate = np.zeros(n)
        sector_tau_rate = np.full(n, np.nan)
        global_count_rate = 0.0
        if self._last_sector_count is not None and state_dt is not None and state_dt > 0:
            sector_count_rate = (sector_count - self._last_sector_count) / state_dt
            sector_tau_rate = (sector_tau - self._last_sector_tau) / state_dt
        if self._last_global_count is not None and state_dt is not None and state_dt > 0:
            global_count_rate = (global_count - self._last_global_count) / state_dt

        self._last_sector_count = sector_count.copy()
        self._last_sector_tau = sector_tau.copy()
        self._last_global_count = global_count
        self._last_t = packet.t_capture

        # -- confidence: fraction of valid sectors, de-rated if gyro/odom weren't fresh --
        # ---- Step DI/DJ: contour/looming channel + per-sector merge ----
        # The LK arrays above (sector_tau, sector_valid, smoothed_tau) are now
        # FINAL and UNTOUCHED -- they still feed the LK-only tau_sector_*/
        # valid_sector_* features with their exact pre-DI meaning. The merge below
        # is a NEW downstream combiner producing the BELIEF (and the new contour
        # features); it does not modify anything the LK path computed.
        c_tau_area = np.full(n, np.nan)
        c_region_count = np.zeros(n)
        c_total_area = np.zeros(n)
        c_largest_growth = np.full(n, np.nan)
        if self.use_contours and dt > 0:
            regions = self._contours.update(gray_b, packet.t_capture)
            agg = self._contours.aggregate_per_sector(regions)
            c_tau_area = agg["tau_area"]
            c_region_count = agg["region_count"]
            c_total_area = agg["total_area"]
            c_largest_growth = agg["largest_growth"]

        merged_valid = np.zeros(n, dtype=bool)
        merged_raw_tau = np.full(n, np.nan)
        ttc_source = np.zeros(n)          # 0 neither, 1 LK-only, 2 contour-only, 3 both
        tau_agreement = np.full(n, np.nan)
        # Step DN -- DETECTION-ONLY merge. DM proved the contour AREA is sound
        # (smooth, monotonic, matches the 1/d^2 geometric prediction) but its
        # aggregated tau_area is unreliable because the region TRACKER swaps
        # identity near closest approach and drops detection when the obstacle
        # exceeds the max-area filter -- an identity problem window-fitting cannot
        # fix (README Step DM). So tau_area does NOT enter SectorBelief.scores or
        # .valid at all. The belief is LK-ONLY (identical to the pre-DI pipeline).
        # A contour present in a sector with NO LK support is flagged
        # "detected, unmeasured" via ttc_source=2 and leaves that sector
        # valid=False -- the controller then avoids it as UNKNOWN, not because of
        # an unreliable number. All contour statistics still flow to the feature
        # vector for the gate (a 3.54%-false-looming signal belongs in the gate's
        # features, not the control loop -- Step DN).
        for s in range(n):
            lk_ok = bool(sector_valid[s])
            contour_present = c_region_count[s] > 0
            contour_tau_ok = not np.isnan(c_tau_area[s])
            # belief validity/scores: LK-only (contour never contributes)
            merged_valid[s] = lk_ok
            merged_raw_tau[s] = float(sector_tau[s]) if lk_ok else np.nan
            # source flag (informational, for the gate): 0 neither, 1 LK-only,
            # 2 contour-only == "detected, unmeasured" (valid stays False),
            # 3 both present (belief still LK-measured; contour corroborates).
            if lk_ok and contour_present:
                ttc_source[s] = 3.0
            elif lk_ok:
                ttc_source[s] = 1.0
            elif contour_present:
                ttc_source[s] = 2.0
            # agreement is still recorded where BOTH an LK tau and a contour tau
            # exist -- disagreement is an unreliability signal the gate can learn
            # from, even though the contour number never touches control.
            if lk_ok and contour_tau_ok:
                lo = min(float(sector_tau[s]), float(c_tau_area[s]))
                hi = max(float(sector_tau[s]), float(c_tau_area[s]))
                tau_agreement[s] = lo / hi if hi > 0 else np.nan

        # EMA-smooth the belief tau, in its OWN state. Step DN: merged_raw_tau is
        # now LK-only, so this tracks the LK smoothing identically and the belief
        # is byte-identical to the pre-DI pipeline whether contours are on or off
        # (contours are detection/features-only after DN, never in the belief).
        smoothed_merged = np.full(n, np.nan)
        for s in range(n):
            if merged_valid[s]:
                capped = min(merged_raw_tau[s], self.tau_cap_s)
                if self._smoothed_merged_tau[s] is None:
                    self._smoothed_merged_tau[s] = capped
                else:
                    self._smoothed_merged_tau[s] = (
                        self.ema_alpha * capped + (1 - self.ema_alpha) * self._smoothed_merged_tau[s]
                    )
                smoothed_merged[s] = self._smoothed_merged_tau[s]

        confidence = float(np.mean(merged_valid)) if n > 0 else 0.0
        if (not packet.gyro_valid) or (not prev_packet.gyro_valid) or (not odom_valid):
            confidence *= 0.5

        latency_ms = (time.perf_counter() - t0) * 1000.0

        # Step AP: scores must be [0,1] traversability, not raw seconds -- see
        # tau_to_score(). Belief is built from the MERGED (LK+contour) tau/valid
        # (Step DJ); the feature vector's tau_sector_* entries stay LK-only.
        scores = tau_to_score(smoothed_merged, self.tau_cap_s)
        assert np.all((np.isnan(scores)) | ((scores >= 0.0) & (scores <= 1.0))), (
            "SectorBelief.scores violated the [0,1] comparability contract -- "
            f"got {scores}"
        )
        # Step CG -- per-frame across-sectors stretch, symmetric with HeavyStage.
        scores = geometry.normalize_scores_to_span(scores, merged_valid)

        belief = SectorBelief(
            scores=scores,
            valid=merged_valid,
            confidence=confidence,
            source="cheap",
            latency_ms=latency_ms,
        )

        computed = {}
        for s in range(n):
            computed[f"tau_sector_{s}"] = smoothed_tau[s] if sector_valid[s] else np.nan
            computed[f"valid_sector_{s}"] = 1.0 if sector_valid[s] else 0.0
            computed[f"count_sector_{s}"] = sector_count[s]
            computed[f"flow_mag_sector_{s}"] = sector_flow_mag[s]
            computed[f"ground_frac_sector_{s}"] = sector_ground_frac[s]
            computed[f"count_rate_sector_{s}"] = sector_count_rate[s]
            computed[f"tau_rate_sector_{s}"] = sector_tau_rate[s]
            computed[f"grad_energy_sector_{s}"] = grad_energy[s]
        computed["min_tau"] = min_tau
        computed["tau_variance"] = tau_variance
        computed["centre_mean_tau"] = centre_mean_tau
        computed["centre_min_tau"] = centre_min_tau
        computed["cone_min_tau"] = cone_min_tau
        computed["cone_effective_count"] = cone_effective_count
        computed["cone_density_ratio"] = cone_density_ratio
        computed["global_feature_count"] = global_count
        computed["global_feature_count_rate"] = global_count_rate
        computed["across_sector_tau_variance"] = tau_variance
        computed["lk_residual"] = lk_residual
        computed["frac_features_lost"] = frac_lost
        computed["gyro_valid"] = 1.0 if packet.gyro_valid else 0.0
        computed["odom_valid"] = 1.0 if odom_valid else 0.0
        # Step DI/DJ -- contour/looming channel features (NaN/0.0 where the channel
        # is off or found no region in a sector). See features.py's spec entries.
        for s in range(n):
            computed[f"tau_area_sector_{s}"] = c_tau_area[s]
            computed[f"contour_region_count_sector_{s}"] = c_region_count[s]
            computed[f"contour_total_area_sector_{s}"] = c_total_area[s]
            computed[f"contour_growth_sector_{s}"] = c_largest_growth[s]
            computed[f"ttc_source_sector_{s}"] = ttc_source[s]
            computed[f"tau_agreement_sector_{s}"] = tau_agreement[s]

        feature_vector = extract_features(computed, n)
        return belief, feature_vector
