#!/usr/bin/env python3
"""Diagnoses WHY per-frame per-sector tracked-point counts are too low
for CheapStage.MIN_POINTS_PER_SECTOR=20 to ever clear (see README's
"CheapStage Phase 2" finding: per-frame counts peak at 14 on
wall_approach). THROWAWAY DIAGNOSTIC, not production code -- does not
modify obst_avoidance/perception/cheap.py's track_pair() or
CheapStage's production behavior; everything here is a parameterized
CLONE used only for measurement.

Four --check modes, matching the four diagnostic steps this was built
for:
  cascade     Step Y -- per-stage point-count survival, per sector, per
              recording ("zone"). Identifies which cleaning stage
              removes the most.
  sweep       Step Z -- detector-parameter sweep (maxCorners,
              minDistance, qualityLevel), reporting per-sector counts
              AND real CheapStage.infer() wall-clock latency for each
              combination (temporarily monkeypatches
              obst_avoidance.perception.cheap.GF_PARAMS for the latency
              measurement only -- restored after each combination, never
              left changed).
  estimators  Step AA -- compares two per-sector tau estimators against
              ground-truth distance on a wall_approach recording:
              (a) CURRENT: CheapStage's actual per-point signed radial
                  tau (r/radial_rate, radial_rate = the flow component
                  along the outward-radial direction, NaN wherever a
                  point isn't expanding), median over the sector.
              (b) ALTERNATIVE: per-point tau = r / |flow magnitude|
                  (direction-agnostic -- no expansion-sign requirement,
                  so no point is ever excluded by that check), median
                  over the sector.
  cleaning    Step AB -- compares valid-rate and tau-vs-distance
              behavior WITH vs. WITHOUT the unconditional LK-error
              top-quartile rejection stage.

Only the FINAL step -- setting MIN_POINTS_PER_SECTOR from what this
diagnostic shows the CURRENT shipped pipeline actually produces -- is
allowed to touch cheap.py, and only that one constant. Nothing here
changes the estimator or the cleaning stages in production; Steps AA/AB
report a comparison, they don't authorize swapping the shipped approach.

Usage:
  tools/diagnose_tracking.py --check cascade
  tools/diagnose_tracking.py --check sweep
  tools/diagnose_tracking.py --check estimators --recording wall_approach_1p0 [--out-dir DIR]
  tools/diagnose_tracking.py --check cleaning --recording wall_approach_1p0 [--out-dir DIR]
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import geometry
from obst_avoidance.perception.cheap import (
    GF_PARAMS as PROD_GF_PARAMS,
    LK_PARAMS,
    VIGNETTE_Y_MIN,
)

RECORDINGS_DIR = Path("/home/saurabh/ardu_ws/recordings")
ALL_RECORDINGS = [
    "odom_sync_check1",   # yaw_only
    "empty_gap1",
    "roll_pitch_wiggle1",
    "wall_approach_1p0",
    "wall_approach_1p5",
    "wall_approach_2p0",
]
WALL_X = 55.0


def load_frames(name):
    source = LogFrameSource(str(RECORDINGS_DIR / name))
    intr = source.intrinsics
    frames = []
    while True:
        p = source.read()
        if p is None:
            break
        frames.append(p)
    return frames, intr


# ---------------------------------------------------------------------
# Shared instrumented tracker: same logic/thresholds as
# perception/cheap.py's track_pair(), parameterized (gf_params) and
# returning EVERY stage's surviving points, not just the final one.
# skip_error_quartile: Step AB toggle.
# ---------------------------------------------------------------------
def track_pair_instrumented(gray0, gray1, gf_params, skip_error_quartile=False,
                             vignette_y_min=VIGNETTE_Y_MIN):
    stages = {}  # stage_name -> Nx2 array of p0 positions surviving that stage
    pts0 = cv2.goodFeaturesToTrack(gray0, **gf_params)
    if pts0 is None:
        empty = np.empty((0, 2))
        for name in ("initial", "after_lk_status", "after_vignette",
                     "after_lk_error_quartile", "after_magnitude"):
            stages[name] = empty
        return stages, None

    stages["initial"] = pts0.reshape(-1, 2)
    pts1, st, err = cv2.calcOpticalFlowPyrLK(gray0, gray1, pts0, None, **LK_PARAMS)
    st = st.reshape(-1).astype(bool)
    p0 = pts0.reshape(-1, 2)[st]
    p1 = pts1.reshape(-1, 2)[st]
    err = err.reshape(-1)[st]
    stages["after_lk_status"] = p0.copy()

    keep = p0[:, 1] > vignette_y_min
    p0, p1, err = p0[keep], p1[keep], err[keep]
    stages["after_vignette"] = p0.copy()

    if len(p0) == 0:
        stages["after_lk_error_quartile"] = p0.copy()
        stages["after_magnitude"] = p0.copy()
        return stages, None

    if skip_error_quartile:
        stages["after_lk_error_quartile"] = p0.copy()  # unchanged -- stage skipped
    else:
        err_thresh = np.percentile(err, 75)
        good = err <= err_thresh
        p0, p1, err = p0[good], p1[good], err[good]
        stages["after_lk_error_quartile"] = p0.copy()
        if len(p0) == 0:
            stages["after_magnitude"] = p0.copy()
            return stages, None

    mags = np.linalg.norm(p1 - p0, axis=1)
    med = np.median(mags)
    inlier = mags < max(3 * med, 10.0)
    p0, p1, err = p0[inlier], p1[inlier], err[inlier]
    stages["after_magnitude"] = p0.copy()

    if len(p0) == 0:
        return stages, None
    return stages, (p0, p1, err)


STAGE_NAMES = ["initial", "after_lk_status", "after_vignette",
               "after_lk_error_quartile", "after_magnitude"]


# ---------------------------------------------------------------------
# Step Y -- survival cascade
# ---------------------------------------------------------------------
def check_cascade(limit):
    for name in ALL_RECORDINGS:
        frames, intr = load_frames(name)
        width = intr["width"]
        n_sectors = geometry.N_SECTORS
        # counts[stage][sector] accumulated over frames, plus a whole-frame total per stage
        stage_sector_counts = {s: np.zeros(n_sectors) for s in STAGE_NAMES}
        stage_totals = {s: 0 for s in STAGE_NAMES}
        n_pairs = 0
        for i in range(min(limit, len(frames) - 1)):
            gray_a = cv2.cvtColor(frames[i].image, cv2.COLOR_RGB2GRAY)
            gray_b = cv2.cvtColor(frames[i + 1].image, cv2.COLOR_RGB2GRAY)
            stages, _ = track_pair_instrumented(gray_a, gray_b, PROD_GF_PARAMS)
            n_pairs += 1
            for s in STAGE_NAMES:
                pts = stages[s]
                stage_totals[s] += len(pts)
                if len(pts):
                    sec = geometry.sector_index(pts[:, 0], width, n_sectors)
                    for sec_i in range(n_sectors):
                        stage_sector_counts[s][sec_i] += int(np.sum(sec == sec_i))

        print(f"\n=== {name} ({n_pairs} frame pairs) ===")
        header = "stage".ljust(26) + "".join(f"sec{s}".rjust(9) for s in range(n_sectors)) + "  total".rjust(9)
        print(header)
        prev_total = None
        biggest_drop_stage, biggest_drop_amount = None, -1
        for s in STAGE_NAMES:
            row = s.ljust(26)
            row += "".join(f"{stage_sector_counts[s][sec_i]/n_pairs:9.2f}" for sec_i in range(n_sectors))
            mean_total = stage_totals[s] / n_pairs
            row += f"{mean_total:9.2f}"
            print(row)
            if prev_total is not None:
                drop = prev_total - mean_total
                if drop > biggest_drop_amount:
                    biggest_drop_amount = drop
                    biggest_drop_stage = s
            prev_total = mean_total
        print(f"  biggest single-stage drop: '{biggest_drop_stage}' "
              f"(-{biggest_drop_amount:.2f} points/frame mean)")


# ---------------------------------------------------------------------
# Step Z -- detector budget sweep
# ---------------------------------------------------------------------
def check_sweep(limit):
    from obst_avoidance.perception import cheap as cheap_module
    from obst_avoidance.perception.cheap import CheapStage

    name = "wall_approach_1p0"
    frames, intr = load_frames(name)
    width = intr["width"]
    n_sectors = geometry.N_SECTORS

    max_corners_opts = [300, 600, 1000]
    min_distance_opts = [7, 4, 2]
    quality_opts = [0.01, 0.005]

    print(f"{'maxCorners':>11}{'minDist':>9}{'quality':>9}"
          + "".join(f"  sec{s}_mean".rjust(11) for s in range(n_sectors))
          + f"{'total_mean':>13}{'latency_ms_mean':>18}{'latency_ms_max':>16}")

    original_gf_params = dict(cheap_module.GF_PARAMS)
    try:
        for max_corners in max_corners_opts:
            for min_distance in min_distance_opts:
                for quality in quality_opts:
                    gf_params = dict(maxCorners=max_corners, qualityLevel=quality,
                                      minDistance=min_distance, blockSize=7)

                    # -- per-sector counts, via the instrumented tracker (no production code touched) --
                    sector_counts = np.zeros(n_sectors)
                    n_pairs = min(limit, len(frames) - 1)
                    for i in range(n_pairs):
                        gray_a = cv2.cvtColor(frames[i].image, cv2.COLOR_RGB2GRAY)
                        gray_b = cv2.cvtColor(frames[i + 1].image, cv2.COLOR_RGB2GRAY)
                        _, final = track_pair_instrumented(gray_a, gray_b, gf_params)
                        if final is not None:
                            p0 = final[0]
                            sec = geometry.sector_index(p0[:, 0], width, n_sectors)
                            for sec_i in range(n_sectors):
                                sector_counts[sec_i] += int(np.sum(sec == sec_i))
                    sector_means = sector_counts / n_pairs

                    # -- real CheapStage.infer() latency, GF_PARAMS temporarily patched --
                    cheap_module.GF_PARAMS = gf_params
                    stage = CheapStage(intr)
                    latencies = []
                    for i in range(n_pairs):
                        odom = (None, (1.0, 0.0, 0.0), None, True)
                        belief, _ = stage.infer(frames[i + 1], frames[i], odom)
                        latencies.append(belief.latency_ms)
                    cheap_module.GF_PARAMS = original_gf_params  # restore immediately

                    lat = np.array(latencies)
                    row = f"{max_corners:>11}{min_distance:>9}{quality:>9.3f}"
                    row += "".join(f"{sector_means[s]:11.2f}" for s in range(n_sectors))
                    row += f"{sector_means.sum():13.2f}{lat.mean():18.3f}{lat.max():16.3f}"
                    print(row)
    finally:
        cheap_module.GF_PARAMS = original_gf_params


# ---------------------------------------------------------------------
# Step AA -- estimator comparison
# ---------------------------------------------------------------------
def _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points):
    """(a) CURRENT: exactly CheapStage's own per-point signed-radial tau,
    median over sector, gated on min_points AND >=1 defined tau."""
    r_vec = p0 - np.array([cx, cy])
    r = np.linalg.norm(r_vec, axis=1)
    r_safe = np.where(r > 1e-6, r, 1e-6)
    radial_rate = np.sum(resid_flow * (r_vec / r_safe[:, None]), axis=1) / dt
    point_tau = np.full(len(p0), np.nan)
    expanding = radial_rate > 1e-6
    point_tau[expanding] = r[expanding] / radial_rate[expanding]

    sector_tau = np.full(n_sectors, np.nan)
    sector_valid = np.zeros(n_sectors, dtype=bool)
    for s in range(n_sectors):
        in_sector = sec_idx == s
        n_s = int(np.sum(in_sector))
        vals = point_tau[in_sector]
        vals = vals[~np.isnan(vals)]
        if n_s >= min_points and len(vals) > 0:
            sector_valid[s] = True
            sector_tau[s] = float(np.median(vals))
    return sector_tau, sector_valid


def _alternative_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points):
    """(b) ALTERNATIVE: tau = r / |flow magnitude|, direction-agnostic --
    no expansion-sign requirement, so no point is excluded by that
    check (only div-by-zero on exactly-stationary points is guarded)."""
    r_vec = p0 - np.array([cx, cy])
    r = np.linalg.norm(r_vec, axis=1)
    mag = np.linalg.norm(resid_flow, axis=1) / dt
    point_tau = np.full(len(p0), np.nan)
    moving = mag > 1e-6
    point_tau[moving] = r[moving] / mag[moving]

    sector_tau = np.full(n_sectors, np.nan)
    sector_valid = np.zeros(n_sectors, dtype=bool)
    for s in range(n_sectors):
        in_sector = sec_idx == s
        n_s = int(np.sum(in_sector))
        vals = point_tau[in_sector]
        vals = vals[~np.isnan(vals)]
        if n_s >= min_points and len(vals) > 0:
            sector_valid[s] = True
            sector_tau[s] = float(np.median(vals))
    return sector_tau, sector_valid


def _percentile_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points,
                                      percentile=20):
    """(c) Step AI: SAME per-point formula as (b) (tau = r / |flow
    magnitude|, direction-agnostic), but aggregated per sector by a low
    PERCENTILE (default 20th) instead of the median. Rationale:
    collision risk is set by the nearest thing in a direction, not the
    typical thing -- a sector with one close object and many distant
    background points has a median dominated by background; a low
    quantile is the physically correct aggregator here. percentile is
    a parameter for auditability, NOT something to sweep for a
    better-looking correlation -- the task specified 20 as the value
    to test, not to tune."""
    r_vec = p0 - np.array([cx, cy])
    r = np.linalg.norm(r_vec, axis=1)
    mag = np.linalg.norm(resid_flow, axis=1) / dt
    point_tau = np.full(len(p0), np.nan)
    moving = mag > 1e-6
    point_tau[moving] = r[moving] / mag[moving]

    sector_tau = np.full(n_sectors, np.nan)
    sector_valid = np.zeros(n_sectors, dtype=bool)
    for s in range(n_sectors):
        in_sector = sec_idx == s
        n_s = int(np.sum(in_sector))
        vals = point_tau[in_sector]
        vals = vals[~np.isnan(vals)]
        if n_s >= min_points and len(vals) > 0:
            sector_valid[s] = True
            sector_tau[s] = float(np.percentile(vals, percentile))
    return sector_tau, sector_valid


def _flow_for_pair(p_a, p_b, fx, fy, cx, cy, gf_params=PROD_GF_PARAMS, skip_error_quartile=False,
                    vignette_y_min=VIGNETTE_Y_MIN):
    from obst_avoidance.perception.cheap import rotational_flow
    gray_a = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2GRAY)
    gray_b = cv2.cvtColor(p_b.image, cv2.COLOR_RGB2GRAY)
    dt = p_b.t_capture - p_a.t_capture
    if dt <= 0:
        return None
    _, final = track_pair_instrumented(gray_a, gray_b, gf_params, skip_error_quartile, vignette_y_min)
    if final is None:
        return None
    p0, p1, err = final
    raw_flow = p1 - p0
    gyro_avg = (p_a.gyro + p_b.gyro) / 2.0
    rot_flow = rotational_flow(p0, gyro_avg, fx, fy, cx, cy) * dt
    resid_flow = raw_flow - rot_flow
    return p0, resid_flow, dt


def check_estimators(recording, limit, min_points, out_dir):
    """min_points here is used ONLY as the representative threshold for
    the distance-vs-tau plot. The valid-rate TABLE sweeps several
    thresholds (including the shipped 20), because at 20 neither
    estimator ever clears the gate on this data (see README) -- fixing
    min_points=20 would mask exactly the "more robust at low point
    counts" difference Step AA asks about. The sweep shows that
    difference where it's actually visible."""
    frames, intr = load_frames(recording)
    fx, fy, cx, cy, width = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"]
    n_sectors = geometry.N_SECTORS
    n_pairs = min(limit, len(frames) - 1)

    import json
    odom_by_seq = {}
    with open(RECORDINGS_DIR / recording / "packets.jsonl") as f:
        for line in f:
            row = json.loads(line)
            odom_by_seq[row["seq"]] = row.get("position")

    # Precompute flow/positions once; re-derive tau per threshold cheaply.
    pairs = []
    for i in range(n_pairs):
        result = _flow_for_pair(frames[i], frames[i + 1], fx, fy, cx, cy)
        pos = odom_by_seq.get(frames[i + 1].seq)
        if result is None or pos is None:
            continue
        p0, resid_flow, dt = result
        sec_idx = geometry.sector_index(p0[:, 0], width, n_sectors)
        pairs.append((p0, resid_flow, dt, sec_idx, WALL_X - pos[0]))

    print(f"\n=== estimators on {recording} ({n_pairs} frame pairs attempted, "
          f"{len(pairs)} produced tracked points) ===")
    threshold_sweep = sorted(set([1, 3, 5, 10, 15, 20, min_points]))
    print(f"{'min_points':>11}{'(a) valid%':>12}{'(a) corr':>10}{'(b) valid%':>12}{'(b) corr':>10}")
    plot_data = None
    for mp in threshold_sweep:
        dists_a, taus_a, valid_a = [], [], 0
        dists_b, taus_b, valid_b = [], [], 0
        for p0, resid_flow, dt, sec_idx, dist in pairs:
            tau_a, valid_mask_a = _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, mp)
            tau_b, valid_mask_b = _alternative_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, mp)
            if np.any(valid_mask_a):
                dists_a.append(dist); taus_a.append(float(np.min(tau_a[valid_mask_a]))); valid_a += 1
            if np.any(valid_mask_b):
                dists_b.append(dist); taus_b.append(float(np.min(tau_b[valid_mask_b]))); valid_b += 1
        corr_a = float(np.corrcoef(dists_a, taus_a)[0, 1]) if len(dists_a) >= 3 else float("nan")
        corr_b = float(np.corrcoef(dists_b, taus_b)[0, 1]) if len(dists_b) >= 3 else float("nan")
        n_total = max(len(pairs), 1)
        print(f"{mp:>11}{valid_a/n_total*100:>12.1f}{corr_a:>10.3f}{valid_b/n_total*100:>12.1f}{corr_b:>10.3f}")
        if mp == min_points:
            plot_data = {"a": (dists_a, taus_a), "b": (dists_b, taus_b)}

    if out_dir and plot_data:
        dists_a, min_taus_a = plot_data["a"]
        dists_b, min_taus_b = plot_data["b"]
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7, 5))
        if dists_a:
            ax.scatter(dists_a, min_taus_a, s=14, alpha=0.6, label=f"(a) current, n={len(dists_a)}")
        if dists_b:
            ax.scatter(dists_b, min_taus_b, s=14, alpha=0.6, label=f"(b) r/|u|, n={len(dists_b)}")
        ax.set_xlabel("distance to wall (m)")
        ax.set_ylabel("min_tau (s)")
        ax.set_title(f"Step AA: estimator comparison -- {recording}")
        ax.legend()
        ax.invert_xaxis()
        out_path = Path(out_dir) / f"step_aa_estimators_{recording}.png"
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"plot saved: {out_path}")


# ---------------------------------------------------------------------
# Step AB -- revisit cleaning stages
# ---------------------------------------------------------------------
def check_cleaning(recording, limit, min_points, out_dir):
    """Same multi-threshold reasoning as check_estimators: at
    min_points=20 nothing clears the gate regardless of cleaning
    config, which would hide exactly the effect being tested. Sweeps
    thresholds for the valid-rate/correlation table; uses `min_points`
    (default from --min-points) only for the representative plot."""
    frames, intr = load_frames(recording)
    fx, fy, cx, cy, width = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"]
    n_sectors = geometry.N_SECTORS
    import json
    odom_by_seq = {}
    with open(RECORDINGS_DIR / recording / "packets.jsonl") as f:
        for line in f:
            row = json.loads(line)
            odom_by_seq[row["seq"]] = row.get("position")

    n_pairs = min(limit, len(frames) - 1)
    configs = [("with_quartile_filter (current)", False), ("without_quartile_filter", True)]

    # Precompute flow/positions once per config; re-derive tau per threshold cheaply.
    pairs_by_config = {}
    for label, skip in configs:
        pairs = []
        for i in range(n_pairs):
            result = _flow_for_pair(frames[i], frames[i + 1], fx, fy, cx, cy, skip_error_quartile=skip)
            pos = odom_by_seq.get(frames[i + 1].seq)
            if result is None or pos is None:
                continue
            p0, resid_flow, dt = result
            sec_idx = geometry.sector_index(p0[:, 0], width, n_sectors)
            pairs.append((p0, resid_flow, dt, sec_idx, WALL_X - pos[0]))
        pairs_by_config[label] = pairs
        print(f"{label}: {len(pairs)}/{n_pairs} pairs produced tracked points at all")

    threshold_sweep = sorted(set([1, 3, 5, 10, 15, 20, min_points]))
    print(f"\n{'min_points':>11}" + "".join(f"{label + ' valid%':>28}{label + ' corr':>16}" for label, _ in configs))
    plot_data = {}
    for mp in threshold_sweep:
        row = f"{mp:>11}"
        for label, _ in configs:
            dists, min_taus, valid = [], [], 0
            for p0, resid_flow, dt, sec_idx, dist in pairs_by_config[label]:
                tau, valid_mask = _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, mp)
                if np.any(valid_mask):
                    dists.append(dist); min_taus.append(float(np.min(tau[valid_mask]))); valid += 1
            n_total = max(len(pairs_by_config[label]), 1)
            corr = float(np.corrcoef(dists, min_taus)[0, 1]) if len(dists) >= 3 else float("nan")
            row += f"{valid/n_total*100:>28.1f}{corr:>16.3f}"
            if mp == min_points:
                plot_data[label] = (dists, min_taus, valid)
        print(row)

    results = {label: plot_data[label] for label, _ in configs} if plot_data else {}

    if out_dir and results:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7, 5))
        for label, (dists, min_taus, _) in results.items():
            if dists:
                ax.scatter(dists, min_taus, s=14, alpha=0.6, label=label)
        ax.set_xlabel("distance to wall (m)")
        ax.set_ylabel("min_tau (s)")
        ax.set_title(f"Step AB: cleaning-stage comparison -- {recording}")
        ax.legend()
        ax.invert_xaxis()
        out_path = Path(out_dir) / f"step_ab_cleaning_{recording}.png"
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"plot saved: {out_path}")


# ---------------------------------------------------------------------
# Step AC -- what does the vignette filter actually remove?
# ---------------------------------------------------------------------
def check_vignette(recording, limit, min_points, out_dir):
    """Saves annotated frames (kept vs. vignette-rejected points) for
    visual inspection, then sweeps the vignette threshold itself
    (100 [current], 50, disabled) and reports the distance-vs-tau
    correlation at each -- everything ELSE (quartile filter, estimator)
    stays exactly the shipped pipeline; only vignette_y_min varies."""
    frames, intr = load_frames(recording)
    fx, fy, cx, cy, width = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"]
    n_sectors = geometry.N_SECTORS
    n_pairs = min(limit, len(frames) - 1)

    if out_dir:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        sample_indices = [n_pairs // 4, n_pairs // 2, (3 * n_pairs) // 4]
        for idx in sample_indices:
            p_a, p_b = frames[idx], frames[idx + 1]
            gray_a = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2GRAY)
            gray_b = cv2.cvtColor(p_b.image, cv2.COLOR_RGB2GRAY)
            pts0 = cv2.goodFeaturesToTrack(gray_a, **PROD_GF_PARAMS)
            img = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2BGR).copy()
            if pts0 is not None:
                pts1, st, err = cv2.calcOpticalFlowPyrLK(gray_a, gray_b, pts0, None, **LK_PARAMS)
                st = st.reshape(-1).astype(bool)
                p0 = pts0.reshape(-1, 2)[st]
                for (x, y) in p0:
                    kept = y > VIGNETTE_Y_MIN
                    color = (0, 255, 0) if kept else (0, 0, 255)  # BGR: green=kept, red=rejected
                    cv2.circle(img, (int(x), int(y)), 3, color, -1)
                cv2.line(img, (0, VIGNETTE_Y_MIN), (img.shape[1], VIGNETTE_Y_MIN), (255, 255, 0), 1)
            n_kept = int(np.sum(pts0.reshape(-1, 2)[:, 1] > VIGNETTE_Y_MIN)) if pts0 is not None else 0
            n_total = len(pts0) if pts0 is not None else 0
            cv2.putText(img, f"green=kept({n_kept}) red=rejected({n_total-n_kept}) cyan=y=100",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            out_path = Path(out_dir) / f"step_ac_vignette_{recording}_frame{idx}.png"
            cv2.imwrite(str(out_path), img)
            print(f"saved: {out_path} ({n_kept} kept, {n_total-n_kept} rejected of {n_total} raw corners)")

    import json
    odom_by_seq = {}
    with open(RECORDINGS_DIR / recording / "packets.jsonl") as f:
        for line in f:
            row = json.loads(line)
            odom_by_seq[row["seq"]] = row.get("position")

    print(f"\n=== vignette threshold sweep on {recording} (quartile filter KEPT, "
          f"estimator (a) unchanged -- only vignette_y_min varies) ===")
    for vy in [100, 50, -1]:  # -1 == disabled (every point has y > -1)
        pairs = []
        for i in range(n_pairs):
            result = _flow_for_pair(frames[i], frames[i + 1], fx, fy, cx, cy, vignette_y_min=vy)
            pos = odom_by_seq.get(frames[i + 1].seq)
            if result is None or pos is None:
                continue
            p0, resid_flow, dt = result
            sec_idx = geometry.sector_index(p0[:, 0], width, n_sectors)
            pairs.append((p0, resid_flow, dt, sec_idx, WALL_X - pos[0]))
        label = "disabled" if vy == -1 else f"y>{vy}"
        dists, min_taus, valid = [], [], 0
        for p0, resid_flow, dt, sec_idx, dist in pairs:
            tau, valid_mask = _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points)
            if np.any(valid_mask):
                dists.append(dist); min_taus.append(float(np.min(tau[valid_mask]))); valid += 1
        corr = float(np.corrcoef(dists, min_taus)[0, 1]) if len(dists) >= 3 else float("nan")
        n_total = max(len(pairs), 1)
        print(f"  vignette={label:>10}: {valid}/{n_total} valid ({valid/n_total*100:.1f}%), "
              f"correlation(distance, min_tau) = {corr:.3f} (n={len(dists)})")


# ---------------------------------------------------------------------
# Step AD -- above-horizon only (main hypothesis)
# ---------------------------------------------------------------------
def _above_horizon_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points):
    """Same as the current (a) estimator, but points with row >= cy
    (below the horizon -- plausibly ground) are excluded before binning
    into sectors at all, not just down-weighted."""
    above = p0[:, 1] < cy
    r_vec = p0 - np.array([cx, cy])
    r = np.linalg.norm(r_vec, axis=1)
    r_safe = np.where(r > 1e-6, r, 1e-6)
    radial_rate = np.sum(resid_flow * (r_vec / r_safe[:, None]), axis=1) / dt
    point_tau = np.full(len(p0), np.nan)
    expanding = radial_rate > 1e-6
    point_tau[expanding] = r[expanding] / radial_rate[expanding]

    sector_tau = np.full(n_sectors, np.nan)
    sector_valid = np.zeros(n_sectors, dtype=bool)
    for s in range(n_sectors):
        in_sector = (sec_idx == s) & above
        n_s = int(np.sum(in_sector))
        vals = point_tau[in_sector]
        vals = vals[~np.isnan(vals)]
        if n_s >= min_points and len(vals) > 0:
            sector_valid[s] = True
            sector_tau[s] = float(np.median(vals))
    return sector_tau, sector_valid


def check_above_horizon(recording, limit, min_points, out_dir):
    frames, intr = load_frames(recording)
    fx, fy, cx, cy, width = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"]
    n_sectors = geometry.N_SECTORS
    n_pairs = min(limit, len(frames) - 1)

    import json
    odom_by_seq = {}
    with open(RECORDINGS_DIR / recording / "packets.jsonl") as f:
        for line in f:
            row = json.loads(line)
            odom_by_seq[row["seq"]] = row.get("position")

    pairs = []
    for i in range(n_pairs):
        result = _flow_for_pair(frames[i], frames[i + 1], fx, fy, cx, cy)
        pos = odom_by_seq.get(frames[i + 1].seq)
        if result is None or pos is None:
            continue
        p0, resid_flow, dt = result
        sec_idx = geometry.sector_index(p0[:, 0], width, n_sectors)
        pairs.append((p0, resid_flow, dt, sec_idx, WALL_X - pos[0]))

    print(f"\n=== above-horizon-only vs. all points on {recording} "
          f"({len(pairs)}/{n_pairs} pairs produced tracked points) ===")
    threshold_sweep = sorted(set([1, 3, 5, 10, 15, 20, min_points]))
    print(f"{'min_points':>11}{'all valid%':>12}{'all corr':>10}{'above-horizon valid%':>22}{'above-horizon corr':>20}")
    plot_data = None
    for mp in threshold_sweep:
        dists_all, taus_all, valid_all = [], [], 0
        dists_ah, taus_ah, valid_ah = [], [], 0
        for p0, resid_flow, dt, sec_idx, dist in pairs:
            tau_all, mask_all = _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, mp)
            tau_ah, mask_ah = _above_horizon_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, mp)
            if np.any(mask_all):
                dists_all.append(dist); taus_all.append(float(np.min(tau_all[mask_all]))); valid_all += 1
            if np.any(mask_ah):
                dists_ah.append(dist); taus_ah.append(float(np.min(tau_ah[mask_ah]))); valid_ah += 1
        corr_all = float(np.corrcoef(dists_all, taus_all)[0, 1]) if len(dists_all) >= 3 else float("nan")
        corr_ah = float(np.corrcoef(dists_ah, taus_ah)[0, 1]) if len(dists_ah) >= 3 else float("nan")
        n_total = max(len(pairs), 1)
        print(f"{mp:>11}{valid_all/n_total*100:>12.1f}{corr_all:>10.3f}"
              f"{valid_ah/n_total*100:>22.1f}{corr_ah:>20.3f}")
        if mp == min_points:
            plot_data = {"all": (dists_all, taus_all), "above_horizon": (dists_ah, taus_ah)}

    if out_dir and plot_data:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7, 5))
        da, ta = plot_data["all"]
        dh, th = plot_data["above_horizon"]
        if da:
            ax.scatter(da, ta, s=14, alpha=0.6, label=f"all points, n={len(da)}")
        if dh:
            ax.scatter(dh, th, s=14, alpha=0.6, label=f"above-horizon only, n={len(dh)}")
        ax.set_xlabel("distance to wall (m)")
        ax.set_ylabel("min_tau (s)")
        ax.set_title(f"Step AD: above-horizon vs. all -- {recording}")
        ax.legend()
        ax.invert_xaxis()
        out_path = Path(out_dir) / f"step_ad_above_horizon_{recording}.png"
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"plot saved: {out_path}")


# ---------------------------------------------------------------------
# Step AE -- is wall_approach validatable at all?
# ---------------------------------------------------------------------
def check_sector_source(recording, limit, min_points):
    """For every frame where CheapStage's CURRENT shipped estimator
    produces a valid min_tau, records WHICH sector supplied it."""
    frames, intr = load_frames(recording)
    fx, fy, cx, cy, width = intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["width"]
    n_sectors = geometry.N_SECTORS
    n_pairs = min(limit, len(frames) - 1)

    winner_counts = np.zeros(n_sectors, dtype=int)
    n_valid_frames = 0
    for i in range(n_pairs):
        result = _flow_for_pair(frames[i], frames[i + 1], fx, fy, cx, cy)
        if result is None:
            continue
        p0, resid_flow, dt = result
        sec_idx = geometry.sector_index(p0[:, 0], width, n_sectors)
        tau, valid_mask = _current_estimator_sector_tau(p0, resid_flow, dt, sec_idx, n_sectors, cx, cy, min_points)
        if np.any(valid_mask):
            n_valid_frames += 1
            candidate_tau = np.where(valid_mask, tau, np.inf)
            winner = int(np.argmin(candidate_tau))
            winner_counts[winner] += 1

    print(f"\n=== which sector supplies min_tau on {recording} "
          f"(min_points={min_points}, {n_valid_frames}/{n_pairs} frames had >=1 valid sector) ===")
    for s in range(n_sectors):
        pct = winner_counts[s] / max(n_valid_frames, 1) * 100
        print(f"  sector {s}: {winner_counts[s]}/{n_valid_frames} frames ({pct:.1f}%)")
    if winner_counts[2] == 0:
        print("  sector 2 (the one pointed at the wall) NEVER supplies min_tau on this "
              "recording -- min_tau is always coming from sectors looking at scenery "
              "that recedes past the airframe, not the obstacle ahead. wall_approach "
              "CANNOT validate obstacle response with this stage as shipped: a metric "
              "that never measures the actual obstacle can't be evidence the pipeline "
              "detects obstacles.")
    else:
        print(f"  sector 2 supplied min_tau on {winner_counts[2]} frame(s) -- "
              f"wall_approach CAN partially validate obstacle response.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", required=True,
                         choices=["cascade", "sweep", "estimators", "cleaning",
                                  "vignette", "above_horizon", "sector_source"])
    parser.add_argument("--recording", type=str, default="wall_approach_1p0")
    parser.add_argument("--limit", type=int, default=1000)
    parser.add_argument("--min-points", type=int, default=10,
                         help="min points/sector to call it valid (default matches the "
                              "current shipped MIN_POINTS_PER_SECTOR)")
    parser.add_argument("--out-dir", type=str, default=None)
    args = parser.parse_args()

    if args.check == "cascade":
        check_cascade(args.limit)
    elif args.check == "sweep":
        check_sweep(args.limit)
    elif args.check == "estimators":
        check_estimators(args.recording, args.limit, args.min_points, args.out_dir)
    elif args.check == "cleaning":
        check_cleaning(args.recording, args.limit, args.min_points, args.out_dir)
    elif args.check == "vignette":
        check_vignette(args.recording, args.limit, args.min_points, args.out_dir)
    elif args.check == "above_horizon":
        check_above_horizon(args.recording, args.limit, args.min_points, args.out_dir)
    elif args.check == "sector_source":
        check_sector_source(args.recording, args.limit, args.min_points)


if __name__ == "__main__":
    main()
