# CTRL-FINAL — implementation and validation

Codex resumed the user-paused CTRL-RECOVERY edits on baseline c0e074f.
Includes the previous uncommitted controller/runtime changes; see
2026-09-07-codex-recovery-paused.md for their original tests and flights.
Original recordings and rejected outcomes remain intact. Peer review pending.

## Scope and versions

Baseline controller remains default. 10,000 sequential randomized command
and state comparisons against c0e074f match (NumPy seed 49, changing scores,
validity, goal/yaw, cheap/heavy source). This establishes controller decision
compatibility, NOT unchanged platform behavior: every live MavrosVehicle now
has a 1s command expiry on a steady-clock timer. New inference results arriving
after that budget are rejected rather than restarting movement after a stall.
Repeated/backwards camera capture timestamps cannot refresh the watchdog.

pathtrack_v3 adds physical-estimate admission and confidence/alignment speed,
continuous goal steering, and endpoint arrival within 0.75m. Raw LK magnitude-
flow time proxy and metric camera-z depth are exposed separately from the
unchanged normalized scores and feature vectors. Thresholds (2s, 1m,
confidence coverage 0.25) are provisional engineering choices, not calibrated
safety margins. Contour TTC is not used. Metric depth is not airframe clearance.
Missing estimates remain unknown; no simulator geometry enters decisions.

pathtrack_v4 additionally pursues the goal within contiguous admitted sectors,
never across unknown/rejected gaps, and limits ordinary yaw-command change to
1 rad/s^2 using capture time. Blind braking remains immediate. Translation
stops while a slew-limited command still turns opposite to the requested turn.
Five continuous seconds of unsupported perception ends with
perception_unavailable, then the demo lands; it does not resume blind forward
motion. This is bounded failure handling, not recovery of missing measurements.

All path variants are explicit CLI options. The comparison runner uses fresh
output directories, current source, common endpoint criteria, and owned group
cleanup; it neither deletes old runs nor kills unrelated simulator processes.

## Why v4

Of v3's 179 recorded command sign reversals, 107 occurred at adopt_target_lost,
32 at hold_challenge_pending, 18 at goal_recovery, 18 at adopt_no_prior, and
4 in other branches. Sparse changing support dominated the issue.
With the exact same recorded beliefs/poses fed to the new controller, reversals
fell from 179 to 36 (ignore |commanded yaw rate| <=0.05). This is fixed-input
replay, not a simulated counterfactual trajectory or a flight safety claim.

## Checks

120 offline tests passed before the v4 trial; 18 installed Matplotlib/pyparsing
deprecation warnings. Package modules parse; runner bash syntax and git diff
whitespace checks pass. Suite: test_controller_recovery, test_controller_v2,
test_reference_path, test_sector_controller, test_orchestrator,
test_score_normalization, test_vehicle_lifecycle, test_owned_processes,
test_cheap_stage, test_live_viewer, test_sim_startup, test_trajectory_review,
test_physical_belief (all test/*.py). Use PYTHONDONTWRITEBYTECODE=1
MPLCONFIGDIR=/tmp/obst-mpl python3 -m pytest -q -p no:cacheprovider with those files.

New coverage includes no interpolation across unknown sectors, insensitive
steering to rankings within one opening, bounded reversal and translation
hold, explicit perception failure, endpoint arrival and non-asymptotic approach,
expiry without inference, late-result rejection, duplicate-frame timeout,
LK/depth physical-value preservation without downloading a model.

## Scope limits

A cached metric depth model exists, but an online depth fallback is NOT wired
or latency/quality validated here. This task does not establish the project's
learned perception gate. Completing a mission after flow loss requires a real
usable recovery measurement or perception improvement; safe termination is
not autonomous navigation success. v3/v4 remain experimental.

Raw measurements, noisy command counts, and model-free 2D collision estimates
must not be represented as validated swept-volume safety. Seed=0 in existing
demo config is historical metadata, not proof of deterministic simulator/RNG
seeding. These runs share scene/spawn settings, not proven paired randomness.

## Completed v4 simulation and final review

Artifacts: workspace `eval_results/ctrl_compare_zone_A_HUaaci/`:
`pathtrack_v4/camera.avi`, frames.jsonl, simulator.log, result.json, config.json;
root source_hashes.json, metrics.json, trajectory.png and timing.png.
Command: CTRL_COMPARE_ARMS='pathtrack_v4' CTRL_COMPARE_WALL_S=150
bash tools/run_ctrl_compare.sh zone_A -3.8 (host execution).

v4: wall_timeout, 739 frames /75.636 simulated seconds /150 wall seconds.
38 command sign reversals, blind fraction 26.93%, path length 16.417m,
CTE RMS 5.260m, endpoint error 8.892m, estimated minimum margin +0.693m.
Final logged XY (24.219,2.956). Landed/disarmed; cleanup errors empty.
No simulator/MAVROS/MAVProxy processes or UDP 14550/14551/9002/9003 sockets
remained in the final host inspection. Camera and plots saved; no live display
left running. Initial host approval check timed out, succeeded on its allowed
retry; no permission blocker remains.

Relative to the earlier v3 run (179 reversals over 73.656s), v4 has fewer
command reversals but LOWER minimum estimated margin (v3 +1.378m).
Neither completed its mission. These are separate single trials, not a
statistically established improvement. No collision detection is not proof
of safety. The five-second unsupported timeout did not fire because support
returned intermittently; the independent 150s wall cap ended the trial.

Post-flight final review prioritized collision/endpoint classification over
simultaneous unsupported-perception termination, and synchronized slew state
with a zero command sent for invalid odometry. Both have focused regressions;
the latter condition occurred in zero logged v4 frames. Also removed the
misleading seed=0 default in future metadata (now null, seed_applied=false).
These final review changes were unit-tested, not reflown; source_hashes.json
retains exactly the flown source. Baseline explanatory docstring updated.

Final validation: 121-test suite passed with 18 dependency deprecation warnings;
one additional odometry/slew regression also passed with the orchestrator
suite, totaling 122 distinct passing offline tests. Live depth inference,
learned gating, multi-seed flight evaluation and second-assistant review were
not performed. No v3/v4 default promotion. Commit includes the entire preserved
CTRL-RECOVERY change plus this reviewed iteration; no recordings are staged.

