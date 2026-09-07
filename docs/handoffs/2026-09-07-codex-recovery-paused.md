# CTRL-RECOVERY — paused at user request

User explicitly requested stopping and cleaning up on 2026-09-07.
Changes remain UNCOMMITTED on baseline c0e074f. Do not overwrite them.
Edit lock released for resumption. No simulator/autonomous_demo/comparison/
MAVROS/MAVProxy processes remained in the final host process inspection;
UDP ports 14550/14551/9002/9003 were free. Both flights landed and disarmed.

Implemented experimental pathtrack_v3: continuous within-sector goal steering,
route correction toward equally/better ranked sectors, physical-estimate
admission (LK magnitude-flow TTC proxy >2s or metric camera-z depth >1m),
confidence/alignment speed scaling and zero forward/yaw when no admitted
sectors. These are provisional thresholds, not proven safety margins.
Added optional raw ttc_s/forward_depth_m fields without changing normalized
scores/features. v3 is NOT promoted to default.

Added 1s command expiry on steady-clock republisher (all live controllers),
nonfinite command braking, duplicate/backwards capture timestamp watchdog,
optional endpoint arrival criterion (0.75m), and common endpoint flag for
comparisons. Runtime metadata records policy and world SHA256. Runner now
executes Python from package root to avoid stale installed copies. Rebuilt
workspace package with colcon --symlink-install successfully.

Validation: final offline suite 116 passed, 18 installed Matplotlib/pyparsing
warnings. Suite files: test_controller_recovery, test_controller_v2,
test_reference_path, test_sector_controller, test_orchestrator,
test_score_normalization, test_vehicle_lifecycle, test_owned_processes,
test_cheap_stage, test_live_viewer, test_sim_startup, test_trajectory_review,
test_physical_belief (all under test/, .py). Run from package using
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/obst-mpl python3 -m pytest -q
-p no:cacheprovider followed by those files. No depth model downloaded;
metric contract test uses a stub pipeline.

## Real matched run evidence

Workspace eval_results/ctrl_compare_zone_A_t2V5Xx/ contains both camera.avi
recordings, frames.jsonl, result.json, simulator logs, config files,
source_hashes.json, metrics.json, trajectory.png and timing.png.
Command: CTRL_COMPARE_ARMS='pathtrack pathtrack_v3' CTRL_COMPARE_WALL_S=150
bash tools/run_ctrl_compare.sh zone_A -3.8 (host execution required).
Both arms used endpoint criterion, same spawn/world and lookahead. This is
an entire controller policy comparison, not a single-variable steering test.

- pathtrack: collision after 20.493 sim seconds, 226 frames; minimum estimated
  margin -0.0449m, endpoint error 10.3104m, command sign reversals 23.
- v3: wall_timeout after 73.656 sim seconds /150 wall seconds, 765 frames;
  minimum estimated margin +1.3780m, endpoint error 9.2019m, command sign
  reversals 179. Blind fraction 36.6%. Stopped near (23.216,2.417) after losing
  usable flow. It did NOT recover the path or reach the endpoint. More command
  reversals, even allowing for longer duration: do not call it smoother.

This exposes the optical-flow stop/restart problem and noisy steering.
No-collision in this single stopped run is not mission success or general
safety evidence. Geometry is the existing sampled 2D approximation, not swept
clearance. LK magnitude-flow time proxy is not calibrated closing TTC.

Initial attempt eval_results/ctrl_compare_zone_A_AtTa2a/ failed before launch:
old installed demo did not recognize --endpoint-goal. Preserved its log.
Rebuild + running from source fixed this; no simulator was started by that attempt.

After the completed flights, two additional runtime corrections were unit
checked but not flown: reject an inference result arriving after the command
expiry budget, and avoid asymptotic braking just outside endpoint tolerance.
Neither flight approached its endpoint; recorded max processing latencies
were 201ms/178ms, below 1s. Input validation/type annotations also updated.
The source_hashes.json describes the flown source, not these later corrections.

## Resume

Read this handoff and actual git diff, claim lock, review changes before commit.
Remaining: inspect noisy v3 switching and loss-of-flow recovery; assess depth
fallback or another actual measurement rather than fabricating free space or
resuming blind forward motion. Keep v3 experimental. Final review/commit still
pending. Do not restart simulator merely to resume context.
