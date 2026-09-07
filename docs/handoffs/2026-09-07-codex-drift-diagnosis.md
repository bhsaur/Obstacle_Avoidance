# CTRL-DIAGNOSE — drift mechanism and next perception work

Codex, baseline 87698e7. User reported the flight still looked bad and asked
how to make it better. This task diagnoses the failure, fixes one concrete
heading mechanism in opt-in pathtrack_v5, and measures the cached depth model.
Prior variants/default remain unchanged; peer review pending.

## What was actually wrong

On v4's 739 control frames, projecting the reference lookahead using the known
camera calibration (640x480, fx~205.4696, cx=320) put the goal inside sector-
centre support on just 79 frames; its sector was valid/admitted on only 10.
Of 356 forward-command frames, 348 lacked support in the goal sector. Median
number of valid sectors was ONE (quartiles 1,1,2; maximum 5). Counts per sector:
[7,11,42,149,267,310,147,28,37,13,2].

Using a rounded 1-radian FOV threshold: goal outside view on 659 frames,
forward commanded on 282, yaw pointed away from goal on 125, and |yaw|<0.05
on 420. Exact sector-centre support gives 660 outside frames; do not mix these
thresholds. The camera was facing away from the return direction while the
reactive controller kept choosing among whichever textured sectors it could see.

At x10..15, average commanded speed .598m/s; x15..20 .245; x20..25 .100.
The slowdown was tied to lost support, not evidence that a low-level PID gain
was the main cause. Raw camera inspection shows sparse corners concentrated
on vegetation/object edges. A valid TTC estimate is not proof of a navigable
opening; distant textured obstacles can be selected while untextured space
stays unknown. Lowering the point-count threshold or declaring unknown clear
would remove evidence checks without establishing a route.

## Implemented mechanism change

New optional reorient_to_goal state (demo option pathtrack_v5): when the route
heading leaves the supported camera-centre range, stop translation and yaw
using odometry toward the route. Exit only when the goal is in the central
half of view, avoiding boundary chatter. Then reacquire the avoidance target
from current perception; no flow support still means no forward travel.
Yaw slew and common watchdog remain. This is stationary observation-direction
control, not obstacle-free-space inference or a blind forward escape.

Tests cover both rotation signs, zero translation during reorientation,
convergence with no flow, hysteretic exit, and continuing to require perception
after facing the route. Full relevant offline suite: 125 passed, 18 dependency
deprecation warnings. Same suite listed in controller-final handoff, with the
new tests in test_controller_recovery.py. Python syntax/diff checks passed.

Added raw_camera.avi (unannotated MJPG, lossy) beside the existing overlay video.
video_frames.jsonl maps frame index to seq/t_capture/phase for both videos.
This is suitable for image inspection/model probes, not byte-exact replay of
original pixels. Both use fixed 25 fps; consult timestamps for true timing.
After flight, display labels were clarified to REORIENT, and future config.json
now records actual camera intrinsics. Those metadata/display changes were
syntax checked, not reflown. Source hashes preserve the flown version.

## Live outcome — improved route heading, still a failed mission

Workspace eval_results/ctrl_compare_zone_A_Yzulxn/: trajectory.png, metrics.json,
source_hashes.json, pathtrack_v5/ recordings/index/logs/config/result.
Command: CTRL_COMPARE_ARMS='pathtrack_v5' CTRL_COMPARE_WALL_S=150
bash tools/run_ctrl_compare.sh zone_A -3.8 (host execution).

488 frames, 48.543 simulated seconds; 54 reorientation frames. Maximum lateral
route deviation 1.872 m, final lateral deviation 1.128 m (v4 final 6.756 m).
Stopped at (17.791, -2.672), endpoint error 12.261 m: less forward progress than v4,
NOT a navigation success. End reason perception_unavailable after sustained
lack of admitted sectors. Estimated minimum margin +0.763m under the existing
sampled 2D footprint evaluator. Landed/disarmed, cleanup errors empty.
Separate single trials do not establish general improvement or swept safety.

## Real offline metric-depth probe

Cached Depth-Anything-V2-Metric-Outdoor-Small-hf, network disabled; no model
was downloaded. CPU, torch 2.8.0+cu128, 2 torch threads, 1 warmup then 3 clean
flight frames. Load 6.497 s; measured inference 2094.7,2168.7,1939.1ms. All 11
sectors returned finite depth estimates; this is NOT an accuracy evaluation.
Probe used the existing camera calibration estimate (explicit in output).
Artifacts: depth_probe.json and depth_probe_script.py in the run root.
Run from package with PYTHONPATH=. HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1;
the preserved script documents this exact probe and writes the same JSON path,
so copy it and change output before a repeat to preserve the original.

A synchronous drop-in would exceed the current 1 s command-expiry budget. Raising
that timeout just to keep old forward commands alive would conceal stale data.
Next: benchmark an explicit device/resolution, validate metric depth against
rendered-object/depth ground truth OFFLINE, then implement a stop-and-observe
or asynchronous depth scheduler with timestamp/pose freshness checks and
footprint-aware admission. Preserve cheap-stage features and compare against
an always-depth reference before claiming learned gating benefits. Do not
assume finite model predictions imply calibrated safety, particularly in this
synthetic visual domain. No depth-based flight or gate was implemented here.

Final host audit: no simulator/MAVROS/MAVProxy processes or UDP14550/14551/9002/9003 listeners remained. User media playback was left alone.
