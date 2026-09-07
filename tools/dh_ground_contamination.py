"""Step DH.1 -- measure ground-plane contamination of CheapStage's
tracked points, on existing recordings. OFFLINE.

For each recording, per sector, over all frames:
  (a) fraction of tracked points that fall BELOW the horizon row (row
      >= cy=240) -- these sit on the ground plane / world-plane edges,
      not necessarily on an obstacle;
  (b) of the points in the LOW-TAU TAIL that actually SETS the sector's
      danger reading (per-point tau <= the sector's own 20th-percentile
      tau, exactly the aggregator CheapStage uses), what fraction are
      those below-horizon points -- i.e. is the ground DRIVING the
      danger score, not just present in the frame.

Replicates CheapStage's point-tau pipeline using the SAME module
functions (track_pair, rotational_flow, geometry.foe/sector_index) so
the numbers match what CheapStage actually computes; the LK path itself
is untouched. odom/vz not needed here -- the 'percentile' estimator's
per-point tau = r/|resid_flow| does not use forward velocity.
"""
import argparse
import glob
import os

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception.cheap import (track_pair, rotational_flow,
                                             VIGNETTE_Y_MIN, TAU_PERCENTILE, MIN_POINTS_PER_SECTOR)
from obst_avoidance.perception import geometry

N = geometry.N_SECTORS


def analyse(recording, tau_pct=TAU_PERCENTILE):
    src = LogFrameSource(recording)
    intr = src.intrinsics
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    w = intr["width"]

    # accumulators per sector
    n_pts = np.zeros(N)          # total tracked points
    n_ground = np.zeros(N)       # below-horizon points
    tail_pts = np.zeros(N)       # points in the low-tau tail (tau <= sector 20th pct)
    tail_ground = np.zeros(N)    # of those, below-horizon

    prev = None
    while True:
        pkt = src.read()
        if pkt is None:
            break
        if prev is None:
            prev = pkt
            continue
        dt = pkt.t_capture - prev.t_capture
        if dt <= 0:
            prev = pkt
            continue
        ga = cv2.cvtColor(prev.image, cv2.COLOR_RGB2GRAY)
        gb = cv2.cvtColor(pkt.image, cv2.COLOR_RGB2GRAY)
        tracked = track_pair(ga, gb)
        if tracked is None:
            prev = pkt
            continue
        p0, p1, err, _ = tracked
        raw_flow = p1 - p0
        gyro_avg = (prev.gyro + pkt.gyro) / 2.0
        resid = raw_flow - rotational_flow(p0, gyro_avg, fx, fy, cx, cy) * dt
        foe_x, foe_y = geometry.foe(cx, cy)
        r_vec = p0 - np.array([foe_x, foe_y])
        r_px = np.linalg.norm(r_vec, axis=1)
        mag = np.linalg.norm(resid, axis=1) / dt
        point_tau = np.full(len(p0), np.nan)
        moving = mag > 1e-6
        point_tau[moving] = r_px[moving] / mag[moving]

        sec = geometry.sector_index(p0[:, 0], w, N)
        ground = p0[:, 1] >= cy

        for s in range(N):
            insec = sec == s
            ns = int(insec.sum())
            if ns == 0:
                continue
            n_pts[s] += ns
            n_ground[s] += int(ground[insec].sum())
            taus = point_tau[insec]
            valid = ~np.isnan(taus)
            if ns >= MIN_POINTS_PER_SECTOR and valid.sum() > 0:
                thr = np.percentile(taus[valid], tau_pct)
                tail = valid & (point_tau <= thr) if False else None
                # low-tau tail among in-sector valid points
                tail_mask = insec & (~np.isnan(point_tau)) & (point_tau <= thr)
                tail_pts[s] += int(tail_mask.sum())
                tail_ground[s] += int((tail_mask & ground).sum())
    src.close()

    with np.errstate(invalid="ignore", divide="ignore"):
        gfrac = np.where(n_pts > 0, n_ground / n_pts, np.nan)
        tail_gfrac = np.where(tail_pts > 0, tail_ground / tail_pts, np.nan)
    return gfrac, tail_gfrac, n_pts, tail_pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recordings", nargs="*", help="recording dirs (default: a representative set)")
    args = ap.parse_args()
    base = "/home/saurabh/ardu_ws/recordings"
    recs = args.recordings or [
        os.path.join(base, r) for r in
        ["boxB1_approach", "textured_approach_treeA2", "narrow_approach_D1",
         "wall_approach_1p0", "corridor_segment1", "corridor_segment2",
         "corridor_segment3", "corridor_segment4"]
    ]
    print(f"horizon cy=240; below-horizon = ground-plane candidate. n_sectors={N}, tau_pct={TAU_PERCENTILE}\n")
    all_g, all_tg, all_np, all_tp = [], [], [], []
    for rec in recs:
        name = os.path.basename(rec)
        gfrac, tail_gfrac, npts, tpts = analyse(rec)
        all_g.append(gfrac * npts); all_np.append(npts)
        all_tg.append(np.nan_to_num(tail_gfrac) * tpts); all_tp.append(tpts)
        # weighted overall for this recording
        ov_g = np.nansum(gfrac * npts) / np.nansum(npts) if np.nansum(npts) else np.nan
        ov_tg = np.nansum(tail_gfrac * tpts) / np.nansum(tpts) if np.nansum(tpts) else np.nan
        print(f"=== {name} ===")
        print(f"  overall: {100*ov_g:.1f}% of tracked points below horizon; "
              f"{100*ov_tg:.1f}% of the 20th-pct-tau tail is ground")
        print(f"  {'sec':>3} {'below-horiz%':>12} {'tail-ground%':>13} {'pts':>8} {'tailpts':>8}")
        for s in range(N):
            print(f"  {s:>3} {100*gfrac[s]:>11.1f} {100*tail_gfrac[s]:>12.1f} "
                  f"{int(npts[s]):>8} {int(tpts[s]):>8}")
        print()

    # grand totals
    G = np.nansum(np.array(all_g), axis=0); NP = np.nansum(np.array(all_np), axis=0)
    TG = np.nansum(np.array(all_tg), axis=0); TP = np.nansum(np.array(all_tp), axis=0)
    print("=== ALL RECORDINGS COMBINED (point-weighted) ===")
    ov_g = np.nansum(G) / np.nansum(NP); ov_tg = np.nansum(TG) / np.nansum(TP)
    print(f"  overall: {100*ov_g:.1f}% points below horizon; {100*ov_tg:.1f}% of tau-tail is ground")
    print(f"  {'sec':>3} {'below-horiz%':>12} {'tail-ground%':>13}")
    for s in range(N):
        g = 100*G[s]/NP[s] if NP[s] else float('nan')
        tg = 100*TG[s]/TP[s] if TP[s] else float('nan')
        print(f"  {s:>3} {g:>11.1f} {tg:>12.1f}")


if __name__ == "__main__":
    main()
