#!/usr/bin/env python3
"""Per-sector time-to-contact (TTC) diagnostic -- THROWAWAY DIAGNOSTIC,
not production code. Driven by LogFrameSource, reusing flow_explore.py's
tracking/outlier-rejection and the CONFIRMED rotational-flow mapping
(see Step U) to de-rotate before computing TTC, so results reflect
translational (obstacle-relevant) flow only.

Purpose (Step R2): does including ground-plane points in a sector's TTC
estimate produce a misleadingly urgent (low) number even when there is
no real obstacle above the horizon in that sector -- "ground TTC
contamination"? Answered by comparing, per sector, TTC computed from ALL
tracked points against TTC computed from ABOVE-HORIZON points only
(pixel row < cy, i.e. the upper half of the frame -- see module
docstring's horizon convention). Run on an empty-gap recording (no real
obstacle -- any low TTC here IS contamination by definition) and a
wall_approach recording (a real obstacle spans both above and below the
horizon, so the question there is whether ground inflates urgency/
distorts which sector looks worst, not whether it invents a signal from
nothing).

TTC per point: assumes forward translation with the focus of expansion
(FOE) at the image center (cx, cy) -- justified by the project's own
prior finding (flow_explore.py's wall_approach VERIFY: median
cos(residual_dir, radially-outward-from-center) = 0.99, i.e. residual
flow really is radially outward from center on a forward approach).
For a tracked point p0 with residual (de-rotated, translational) flow
vector f over dt:
    r_vec = p0 - (cx, cy)
    r     = |r_vec|
    radial_rate = dot(f, r_vec / r) / dt   (px/s, outward-positive)
    TTC   = r / radial_rate                (s; undefined if not expanding)
Points with radial_rate <= 0 (not expanding away from FOE -- noise, or
genuinely receding) are excluded, same as the standard formulation: TTC
is only meaningful for expanding flow.

5 sectors, image split into vertical strips left-to-right (sector 0 =
far left .. sector 4 = far right), matching the "5 sectors, centre vs
outer" language in README's CheapStage feature-extractor requirements.

Usage:
  tools/ttc_sectors.py --recording <dir> [--limit N] [--stride N]
"""
import argparse

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource

# track_pair/rotational_flow: canonical implementations, moved to
# obst_avoidance.perception.cheap for the CheapStage build -- imported
# from there directly (not from flow_explore.py, which now just
# re-exports the same objects) so TTC is computed from the same
# de-rotated translational flow the production pipeline uses.
from obst_avoidance.perception.cheap import rotational_flow, track_pair  # noqa: E402

N_SECTORS = 5


def sector_index(x_px, width, n_sectors=N_SECTORS):
    idx = (x_px / width * n_sectors).astype(int)
    return np.clip(idx, 0, n_sectors - 1)


def compute_frame_ttc(p_a, p_b, fx, fy, cx, cy):
    """Returns (p0, ttc, above_horizon_mask) for one frame pair, or None
    if tracking failed. ttc is NaN for points not expanding (excluded
    from any median)."""
    gray_a = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2GRAY)
    gray_b = cv2.cvtColor(p_b.image, cv2.COLOR_RGB2GRAY)
    tracked = track_pair(gray_a, gray_b)
    if tracked is None:
        return None
    p0, p1, _err, _n_initial = tracked
    dt = p_b.t_capture - p_a.t_capture
    if dt <= 0:
        return None

    raw_flow = p1 - p0
    gyro_avg = (p_a.gyro + p_b.gyro) / 2.0
    rot_flow = rotational_flow(p0, gyro_avg, fx, fy, cx, cy) * dt
    resid_flow = raw_flow - rot_flow

    r_vec = p0 - np.array([cx, cy])
    r = np.linalg.norm(r_vec, axis=1)
    r_safe = np.where(r > 1e-6, r, 1e-6)
    radial_rate = np.sum(resid_flow * (r_vec / r_safe[:, None]), axis=1) / dt

    ttc = np.full_like(r, np.nan)
    expanding = radial_rate > 1e-6
    ttc[expanding] = r[expanding] / radial_rate[expanding]

    above_horizon = p0[:, 1] < cy
    return p0, ttc, above_horizon


def run(recording, limit, stride):
    source = LogFrameSource(recording)
    intr = source.intrinsics
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    width = intr["width"]

    frames = []
    while True:
        p = source.read()
        if p is None:
            break
        if p.gyro_valid:
            frames.append(p)
    print(f"loaded {len(frames)} gyro_valid frames from {recording} "
          f"(cx={cx:.1f}, cy={cy:.1f}, width={width})")

    all_pts = {s: [] for s in range(N_SECTORS)}
    horizon_pts = {s: [] for s in range(N_SECTORS)}
    n_done = 0
    for i in range(0, len(frames) - stride, stride):
        if n_done >= limit:
            break
        result = compute_frame_ttc(frames[i], frames[i + stride], fx, fy, cx, cy)
        if result is None:
            continue
        p0, ttc, above_horizon = result
        sec = sector_index(p0[:, 0], width)
        for s in range(N_SECTORS):
            in_sector = sec == s
            all_pts[s].append(ttc[in_sector])
            horizon_pts[s].append(ttc[in_sector & above_horizon])
        n_done += 1

    print(f"frame pairs processed: {n_done}\n")
    print(f"{'sector':<8}{'all-pts TTC (s)':<20}{'above-horizon TTC (s)':<24}{'n_all':<8}{'n_horizon':<10}")
    for s in range(N_SECTORS):
        all_arr = np.concatenate(all_pts[s]) if all_pts[s] else np.array([])
        hor_arr = np.concatenate(horizon_pts[s]) if horizon_pts[s] else np.array([])
        all_valid = all_arr[~np.isnan(all_arr)] if all_arr.size else all_arr
        hor_valid = hor_arr[~np.isnan(hor_arr)] if hor_arr.size else hor_arr
        all_med = np.median(all_valid) if all_valid.size else float("nan")
        hor_med = np.median(hor_valid) if hor_valid.size else float("nan")
        print(f"{s:<8}{all_med:<20.2f}{hor_med:<24.2f}{all_valid.size:<8}{hor_valid.size:<10}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recording", type=str, required=True)
    parser.add_argument("--limit", type=int, default=200, help="max frame pairs to process")
    parser.add_argument("--stride", type=int, default=1, help="frame stride between pairs")
    args = parser.parse_args()
    run(args.recording, args.limit, args.stride)


if __name__ == "__main__":
    main()
