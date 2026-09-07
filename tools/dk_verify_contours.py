"""Step DK -- verify the contour/looming channel on the diagnostic
recordings. OFFLINE. Runs CheapStage with use_contours False vs True and
reports the four DK checks.
"""
import json
import math
from pathlib import Path

import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import CheapStage, geometry
from obst_avoidance.perception.features import feature_names

N = geometry.N_SECTORS
names = feature_names(N)
VALID = [names.index(f"valid_sector_{i}") for i in range(N)]
SRC = [names.index(f"ttc_source_sector_{i}") for i in range(N)]
TAUA = [names.index(f"tau_area_sector_{i}") for i in range(N)]
MINTAU = names.index("min_tau")

OBST = {"boxB1_approach": (58.0, -6.0), "textured_approach_treeA2": (20.0, 3.8)}
ZONES = [("zone_A", 10, 30), ("zone_B", 40, 70), ("zone_C", 75, 100), ("zone_D", 100, 140)]


def load_odom(rec):
    o = {}
    for l in open(Path(rec) / "packets.jsonl"):
        l = l.strip()
        if l:
            d = json.loads(l)
            o[d["seq"]] = (d.get("position"), d.get("velocity"), d.get("attitude_quat"), d.get("odom_valid", True))
    return o


def run(rec, use_contours):
    src = LogFrameSource(rec)
    intr = src.intrinsics
    odom = load_odom(rec)
    c = CheapStage(intr, use_contours=use_contours)
    prev = None
    rows = []
    while True:
        p = src.read()
        if p is None:
            break
        if prev is None:
            prev = p; continue
        b, fv = c.infer(p, prev, odom.get(p.seq, (None, None, None, False)))
        pos = odom.get(p.seq, (None,))[0]
        rows.append((p.seq, pos, b.valid.copy(), fv[MINTAU], fv[np.array(SRC)].copy(), fv[np.array(TAUA)].copy()))
        prev = p
    src.close()
    return rows


def dist_to(pos, obs):
    return math.hypot(pos[0] - obs[0], pos[1] - obs[1]) if pos else float("nan")


def check_boxB1():
    print("=" * 60)
    print("DK.1 boxB1_approach -- does contour detect the untextured box?")
    print("=" * 60)
    rec = "/home/saurabh/ardu_ws/recordings/boxB1_approach"
    obs = OBST["boxB1_approach"]
    for use in (False, True):
        rows = run(rec, use)
        vr = np.mean([r[2].sum() for r in rows]) / N
        # contour-valid = source 2 or 3 anywhere
        cont_dists = [dist_to(r[1], obs) for r in rows if r[1] and ((r[4] == 2) | (r[4] == 3)).any()]
        merged_valid_frames = sum(1 for r in rows if r[2].any())
        tag = "ON " if use else "OFF"
        line = f"[contours {tag}] belief valid-rate={100*vr:.1f}%  frames w/ any valid sector={merged_valid_frames}/{len(rows)}"
        if use and cont_dists:
            line += f"  contour onset range: {max(cont_dists):.1f}m -> {min(cont_dists):.1f}m"
        print(line)
        # correlation(distance, min_tau) among valid
        d = np.array([dist_to(r[1], obs) for r in rows])
        mt = np.array([r[3] for r in rows])
        ok = ~np.isnan(mt) & ~np.isnan(d)
        if ok.sum() >= 2:
            print(f"                corr(distance, min_tau) = {np.corrcoef(d[ok], mt[ok])[0,1]:+.3f}  (n={ok.sum()})")


def check_textured():
    print("\n" + "=" * 60)
    print("DK.2 textured_approach_treeA2 -- merged tau must NOT degrade LK")
    print("=" * 60)
    rec = "/home/saurabh/ardu_ws/recordings/textured_approach_treeA2"
    obs = OBST["textured_approach_treeA2"]
    for use in (False, True):
        rows = run(rec, use)
        d = np.array([dist_to(r[1], obs) for r in rows])
        mt = np.array([r[3] for r in rows])
        ok = ~np.isnan(mt) & ~np.isnan(d)
        corr = np.corrcoef(d[ok], mt[ok])[0, 1] if ok.sum() >= 2 else float("nan")
        vr = np.mean([r[2].sum() for r in rows]) / N
        tag = "ON " if use else "OFF"
        print(f"[contours {tag}] corr(distance, min_tau)={corr:+.3f}  valid-rate={100*vr:.1f}%  (n={ok.sum()})")
    print("  (ON corr should stay ~>= OFF corr: LK path preserved, contour only adds)")


def check_rotation_only():
    print("\n" + "=" * 60)
    print("DK.3 odom_sync_check1 (rotation-in-place, ~0 translation) -- yaw_only stand-in")
    print("=" * 60)
    rec = "/home/saurabh/ardu_ws/recordings/odom_sync_check1"
    rows = run(rec, True)
    total_sectors = len(rows) * N
    finite_tau_area = sum(int(np.isfinite(r[5]).sum()) for r in rows)
    print(f"  frames={len(rows)}  sector-slots={total_sectors}")
    print(f"  sector-slots with a FINITE tau_area: {finite_tau_area} "
          f"({100*finite_tau_area/total_sectors:.2f}%)")
    print("  (must be ~0: pure rotation = zero looming; any finite low tau_area = false looming)")


def check_corridor():
    print("\n" + "=" * 60)
    print("DK.4 corridor -- per-zone valid-rate OFF vs ON + dominant source")
    print("=" * 60)
    recs = [f"/home/saurabh/ardu_ws/recordings/corridor_segment{i}" for i in (1, 2, 3, 4)]
    for use in (False, True):
        # accumulate per zone
        zvalid = {z[0]: [0, 0] for z in ZONES}   # [valid_sector_count, total_slots]
        zsrc = {z[0]: [0, 0, 0, 0] for z in ZONES}  # source 0,1,2,3 counts
        for rec in recs:
            for r in run(rec, use):
                pos = r[1]
                if not pos:
                    continue
                x = pos[0]
                for zname, lo, hi in ZONES:
                    if lo <= x < hi:
                        zvalid[zname][0] += int(r[2].sum())
                        zvalid[zname][1] += N
                        for sc in r[4]:
                            zsrc[zname][int(sc)] += 1
                        break
        tag = "ON " if use else "OFF"
        print(f"\n[contours {tag}]")
        print(f"  {'zone':<8} {'valid-rate':>11} {'src:LK-only':>12} {'contour-only':>13} {'both':>7}")
        for zname, _, _ in ZONES:
            vc, tot = zvalid[zname]
            if tot == 0:
                continue
            s = zsrc[zname]
            print(f"  {zname:<8} {100*vc/tot:>10.1f}% {s[1]:>12} {s[2]:>13} {s[3]:>7}")


if __name__ == "__main__":
    check_boxB1()
    check_textured()
    check_rotation_only()
    check_corridor()
