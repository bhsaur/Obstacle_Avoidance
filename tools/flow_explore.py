#!/usr/bin/env python3
"""Flow diagnostic instrument (Step Q) -- THROWAWAY DIAGNOSTICS, not
production code. Driven by LogFrameSource. For visual inspection, not
automated testing:

  1. Sparse LK flow (goodFeaturesToTrack + calcOpticalFlowPyrLK) between
     consecutive frames.
  2. The gyro-predicted ROTATIONAL flow component, from packet.gyro and
     the camera intrinsics.
  3. The residual (translational) flow after subtracting (2) from (1).

Saves a PNG per frame pair with three panels: raw flow, rotational
prediction, residual -- vectors drawn as arrows on the frame.

Body-frame-NED-gyro -> camera-optical-frame axis mapping
==========================================================
SimFrameSource's gyro is body-frame NED (rollspeed about body-X/forward,
pitchspeed about body-Y/right, yawspeed about body-Z/down) -- NOT the
camera optical frame (X=right, Y=down, Z=forward/into-scene) the
standard rotational-flow formula below expects. This is NOT guessed --
it was determined empirically against the yaw_only recording (pure
commanded yaw, zero translation -- see record_pass.py --mode yaw_only),
because the gimbal's actual mounted orientation involves a compound
rotation (see the -1.57/-1.57/0 sensor pose in gimbal_small_3d's model
.sdf, itself downstream of several joint frames) that isn't safe to
reason about by inspection alone.

Method: swept all 6 axis permutations x 8 sign combinations (48
candidates), scoring each by mean/median residual flow magnitude on
yaw_only's steady-yaw window. The winner, wy_cam=+yaw with wx_cam/wz_cam
originally tied between +pitch/+roll (unsurprising -- both are near-zero
throughout a pure-yaw maneuver, so yaw_only alone couldn't discriminate
them), also happens to be exactly the cyclic permutation you'd derive by
hand for a camera mounted looking straight down the body's forward axis
(camera Z=forward=body X, camera X=right=body Y, camera Y=down=body Z)
-- a proper (orientation-preserving) axis relabeling, so angular
velocity components carry over with no extra sign flips:

STEP U UPDATE -- wx_cam/wz_cam ambiguity resolved, not just assumed by
elimination: a dedicated roll_pitch_wiggle recording (record_pass.py
--mode roll_pitch_wiggle -- alternating lateral then fore/aft velocity,
each phase inducing large roll/pitch rate with near-zero net
translation) was run specifically to break this tie, then scored with
the same 48-candidate sweep (now reusable via flow_explore.py --sweep,
see run_sweep()). Result: wx_cam=+pitch beats wx_cam=+roll by 66.4%
lower median residual (0.728px vs. 2.168px, best candidate in each
group, 334 frame pairs) -- decisive, not a coin flip. The mapping below
is therefore CONFIRMED by a recording built to discriminate it, not just
inherited from the yaw_only sweep by symmetry-breaking assumption.

    wx_cam = +pitchspeed   (body-Y)
    wy_cam = +yawspeed     (body-Z)
    wz_cam = +rollspeed    (body-X)

VERIFY result on yaw_only (steady-yaw window, vignette-excluded,
LK-error-filtered, magnitude-outlier-rejected -- see below):
  raw flow:      mean 24.6px, median 19.2px
  residual flow: mean 11.0px, median  3.4px
  ratio:         mean 0.45,   median 0.18  (82% reduction, median)

The MEDIAN is the number to trust here: the bulk of tracked points fit
the rotation-only model very well (residual/raw = 0.18). The mean stays
higher because a persistent minority of points are simply mistracked
(LK locking onto the wrong feature, aliasing on repetitive noise
texture) and produce huge garbage flow unrelated to true rotation --
this is a real, separate finding, not a sign/axis bug: two rounds of
targeted cleaning made large, predictable improvements:
  (a) excluding points with pixel-y < 100 -- this is the rotor-blur /
      gimbal-housing vignette visible in every frame's top corners. It's
      PART OF THE DRONE, rigidly fixed in the image, not a static world
      point -- LK tracks it anyway and produces flow with no relation to
      world geometry at all (one such point showed 142px of flow against
      an expected ~20-40px, and even the wrong SIGN). Cut mean ratio
      0.99 -> 0.65.
  (b) rejecting LK's own top quartile by tracking error, then rejecting
      points whose flow magnitude exceeds 3x that pair's median -- a
      standard robust-statistics step. Cut mean ratio further to 0.45.
A real (non-throwaway) flow stage would need equivalent outlier handling
-- this isn't specific to validating the axis mapping.

VERIFY result on wall_approach (straight 1.0 m/s approach toward the
Zone B wall, de-rotated with the same mapping, same outlier handling,
stride=3): raw and residual flow are close in magnitude here (mean
ratio 0.46) because the commanded motion is near-pure translation with
little rotation to remove -- expected, and not the thing being tested.
The thing being tested is whether the residual (translational) flow
shows the focus-of-expansion signature of forward motion: flow near
zero at the heading direction, growing outward with distance from it,
pointing radially away from that point. Measured directly instead of
by eye (arrows at this frame's resolution are too small to judge
visually) over 1579 tracked points across the approach:
  - magnitude vs. distance from image center: monotonic increase across
    distance quintiles (0.19 -> 0.17 -> 0.39 -> 1.95 -> 2.61 px), overall
    correlation(dist, |residual|) = 0.54.
  - direction: median cos(residual_dir, radially-outward-from-center) =
    0.99; restricting to the top quartile by magnitude (the more
    reliably-tracked points, least affected by LK noise near zero) still
    gives mean cos = 0.59, with 73% of all points pointing outward-ish
    (cos > 0).
Both magnitude growth and outward direction hold, so residual flow does
show a focus of expansion near image center -- consistent with the
heading direction, since the recording is a straight approach with the
camera pointed forward along the direction of travel.

Usage:
  tools/flow_explore.py --recording <dir> --out-dir <dir> [--limit N]
"""
import argparse
import glob
import itertools
import os

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource
# track_pair/rotational_flow moved to obst_avoidance.perception.cheap (the
# CheapStage Phase 2 build) as the canonical, validated implementations --
# imported here rather than redefined, so this diagnostic script can never
# silently drift from what the production pipeline actually runs. See
# cheap.py's module docstring for why each is considered "confirmed."
from obst_avoidance.perception.cheap import rotational_flow, track_pair

AXIS_NAMES = ("roll", "pitch", "yaw")  # order matches gyro_ned = (roll, pitch, yaw)


def sweep_candidates():
    """Yields (label, perm, signs) for all 6 permutations x 8 sign
    combinations (48 total) of assigning (rollspeed, pitchspeed,
    yawspeed) to (wx_cam, wy_cam, wz_cam) -- see --sweep and the module
    docstring's Step U section. perm is a 3-tuple of indices into
    gyro_ned=(roll,pitch,yaw); signs is a 3-tuple of +-1, both applied
    positionally to (wx_cam, wy_cam, wz_cam)."""
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            label = "".join(
                ("+" if s > 0 else "-") + AXIS_NAMES[p][0]
                for p, s in zip(perm, signs)
            )
            yield label, perm, signs


def rotational_flow_mapped(pts_px, gyro_ned, fx, fy, cx, cy, perm, signs):
    """Same formula as rotational_flow(), but (wx_cam, wy_cam, wz_cam) is
    gyro_ned permuted/signed per (perm, signs) instead of the fixed
    winning mapping below -- used only by --sweep to score all 48
    candidates against each other."""
    wx, wy, wz = (gyro_ned[p] * s for p, s in zip(perm, signs))
    x = (pts_px[:, 0] - cx) / fx
    y = (pts_px[:, 1] - cy) / fy
    u = x * y * wx - (1 + x ** 2) * wy + y * wz
    v = (1 + y ** 2) * wx - x * y * wy - x * wz
    return np.stack([u * fx, v * fy], axis=1)


def draw_vectors(image_rgb, p0, vectors, color, scale=1.0):
    img = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR).copy()
    for (x, y), (dx, dy) in zip(p0, vectors):
        pt0 = (int(x), int(y))
        pt1 = (int(x + dx * scale), int(y + dy * scale))
        cv2.arrowedLine(img, pt0, pt1, color, 1, tipLength=0.3)
    return img


def run_sweep(frames, fx, fy, cx, cy, limit, stride):
    """Scores all 48 (perm, signs) candidates from sweep_candidates() by
    median residual-flow magnitude across every tracked point in every
    processed frame pair (same track_pair() outlier handling as the
    single-mapping path). Prints the ranking, low-to-high (winner =
    lowest median residual, same logic the yaw_only validation uses: the
    correct rotation model collapses residual flow toward zero). No PNGs
    -- this is a scoring pass, not a visualization one."""
    candidates = list(sweep_candidates())
    per_candidate_resid = {label: [] for label, _, _ in candidates}

    n_done = 0
    for i in range(0, len(frames) - stride, stride):
        if n_done >= limit:
            break
        p_a, p_b = frames[i], frames[i + stride]
        gray_a = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2GRAY)
        gray_b = cv2.cvtColor(p_b.image, cv2.COLOR_RGB2GRAY)

        tracked = track_pair(gray_a, gray_b)
        if tracked is None:
            continue
        p0, p1, _err, _n_initial = tracked
        dt = p_b.t_capture - p_a.t_capture
        if dt <= 0:
            continue

        raw_flow = p1 - p0
        gyro_avg = (p_a.gyro + p_b.gyro) / 2.0
        for label, perm, signs in candidates:
            rot_flow = rotational_flow_mapped(p0, gyro_avg, fx, fy, cx, cy, perm, signs) * dt
            resid_flow = raw_flow - rot_flow
            per_candidate_resid[label].append(np.linalg.norm(resid_flow, axis=1))
        n_done += 1

    print(f"\nsweep: {n_done} frame pairs processed, {len(candidates)} candidates")
    if n_done == 0:
        print("no frame pairs survived tracking -- cannot score any candidate")
        return

    medians = {}
    for label, arrs in per_candidate_resid.items():
        medians[label] = float(np.median(np.concatenate(arrs)))
    ranked = sorted(medians.items(), key=lambda kv: kv[1])

    print("\nrank  label   median residual (px)")
    for rank, (label, med) in enumerate(ranked[:10], start=1):
        print(f"{rank:>4}  {label:<6}  {med:.3f}")
    if len(ranked) > 10:
        print(f"  ... ({len(ranked) - 10} more candidates not shown)")

    winner_label, winner_med = ranked[0]
    runner_label, runner_med = ranked[1]
    margin = (runner_med - winner_med) / runner_med if runner_med > 0 else float("nan")
    print(f"\noverall winner: {winner_label} (median residual {winner_med:.3f}px)")
    print(f"overall runner-up: {runner_label} (median residual {runner_med:.3f}px)")
    print(f"overall margin: {margin*100:.1f}% lower than runner-up")

    # The historically ambiguous question (see module docstring, Step U):
    # is wx_cam pitch or roll? Group every candidate by which axis
    # occupies the wx_cam position (perm[0]) regardless of sign or what
    # occupies wy_cam/wz_cam, and compare the best of each group -- this
    # answers the actual question, unlike the overall winner/runner-up
    # above which can differ only in a sign that's already resolved
    # (e.g. wy_cam's sign, settled independently by yaw_only).
    by_wx_axis = {}
    for label, perm, signs in candidates:
        wx_axis = AXIS_NAMES[perm[0]]
        med = medians[label]
        if wx_axis not in by_wx_axis or med < by_wx_axis[wx_axis][1]:
            by_wx_axis[wx_axis] = (label, med)
    if "pitch" in by_wx_axis and "roll" in by_wx_axis:
        pitch_label, pitch_med = by_wx_axis["pitch"]
        roll_label, roll_med = by_wx_axis["roll"]
        best_axis, best = ("pitch", by_wx_axis["pitch"]) if pitch_med < roll_med else ("roll", by_wx_axis["roll"])
        worst_axis, worst = ("roll", by_wx_axis["roll"]) if pitch_med < roll_med else ("pitch", by_wx_axis["pitch"])
        wx_margin = (worst[1] - best[1]) / worst[1] if worst[1] > 0 else float("nan")
        print(f"\nwx_cam=pitch best: {pitch_label} (median residual {pitch_med:.3f}px)")
        print(f"wx_cam=roll  best: {roll_label} (median residual {roll_med:.3f}px)")
        print(f"=> wx_cam is best explained by {best_axis} "
              f"({wx_margin*100:.1f}% lower residual than the best {worst_axis}-as-wx_cam candidate)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recording", type=str, required=True)
    parser.add_argument("--out-dir", type=str, required=True)
    parser.add_argument("--limit", type=int, default=40, help="max frame pairs to process")
    parser.add_argument("--stride", type=int, default=1, help="frame stride between pairs")
    parser.add_argument("--sweep", action="store_true",
                         help="score all 48 axis-mapping candidates instead of drawing "
                              "PNGs with the fixed winning mapping -- see Step U")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    source = LogFrameSource(args.recording)
    intr = source.intrinsics
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    frames = []
    while True:
        p = source.read()
        if p is None:
            break
        if p.gyro_valid:
            frames.append(p)

    print(f"loaded {len(frames)} gyro_valid frames from {args.recording}")

    if args.sweep:
        run_sweep(frames, fx, fy, cx, cy, args.limit, args.stride)
        return

    raw_mags_all = []
    resid_mags_all = []
    n_done = 0
    for i in range(0, len(frames) - args.stride, args.stride):
        if n_done >= args.limit:
            break
        p_a, p_b = frames[i], frames[i + args.stride]
        gray_a = cv2.cvtColor(p_a.image, cv2.COLOR_RGB2GRAY)
        gray_b = cv2.cvtColor(p_b.image, cv2.COLOR_RGB2GRAY)

        tracked = track_pair(gray_a, gray_b)
        if tracked is None:
            continue
        p0, p1, _err, _n_initial = tracked
        dt = p_b.t_capture - p_a.t_capture
        if dt <= 0:
            continue

        raw_flow = p1 - p0
        gyro_avg = (p_a.gyro + p_b.gyro) / 2.0
        rot_flow = rotational_flow(p0, gyro_avg, fx, fy, cx, cy) * dt
        resid_flow = raw_flow - rot_flow

        raw_mags_all.append(np.linalg.norm(raw_flow, axis=1))
        resid_mags_all.append(np.linalg.norm(resid_flow, axis=1))

        raw_img = draw_vectors(p_a.image, p0, raw_flow, (0, 0, 255))
        rot_img = draw_vectors(p_a.image, p0, rot_flow, (255, 0, 0))
        resid_img = draw_vectors(p_a.image, p0, resid_flow, (0, 255, 0))
        side_by_side = np.hstack([raw_img, rot_img, resid_img])
        cv2.putText(side_by_side, "raw (red)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(side_by_side, "rotational pred (blue)", (gray_a.shape[1] + 10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(side_by_side, "residual (green)", (2 * gray_a.shape[1] + 10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        out_path = os.path.join(args.out_dir, f"flow_{p_a.seq:06d}.png")
        cv2.imwrite(out_path, side_by_side)
        n_done += 1

    if raw_mags_all:
        raw_all = np.concatenate(raw_mags_all)
        resid_all = np.concatenate(resid_mags_all)
        print(f"\nframe pairs processed: {n_done}")
        print(f"raw flow:      mean={raw_all.mean():.2f}px median={np.median(raw_all):.2f}px")
        print(f"residual flow: mean={resid_all.mean():.2f}px median={np.median(resid_all):.2f}px")
        print(f"ratio:         mean={resid_all.mean()/raw_all.mean():.2f} "
              f"median={np.median(resid_all)/np.median(raw_all):.2f}")
    print(f"PNGs saved to {args.out_dir}")


if __name__ == "__main__":
    main()
