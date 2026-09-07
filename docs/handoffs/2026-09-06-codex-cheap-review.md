# CHEAP-001 — CheapStage review and camera viewer

Date: 2026-09-06. Author: Codex. Peer review pending.
User priority: review CheapStage correctness and show camera sectors/scores;
then stop all project simulator processes and restart after a crash.

## Corrections

- Count/TTC derivatives now divide by time since the previous inference,
  rather than the current image-pair interval. These differ when calls skip
  image pairs. Ordinary consecutive-frame outputs are preserved.
- Reversed/repeated timestamps clear temporal history. Invalid image pairs
  do not enter contour fitting; contour updates also guard their own time
  history. Nonfinite timestamps are rejected.
- Gradient feature columns now agree exactly with the shared sector-index
  function at integer pixel boundaries (ceil, not floor, of boundaries).
- Confidence accounts for stale gyro on either image in the pair.
- Viewer uses the actual inference's tracked points instead of running LK
  a second time. It renders 11 sector columns, bottom scores, TTC, counts,
  validity, and a legend explaining relative scores/unknown sectors.
- Default viewer is monitoring only, with recorded replay/headless export.
  `tools/live_viewer.py` now delegates to the installed `cheap_viewer` entry
  point; its old automatic-flight CLI is replaced by monitoring options.
- Offline frame-source imports no longer eagerly import ROS/MAVLink.
- SimFrameSource checks missing heartbeat, bounds startup clock waiting, and
  cleans up threads/socket after connection failure; close is idempotent.
- The old verification tool mistakenly interpreted normalized scores as TTC
  seconds during yaw checks. It now reads TTC features and honors replay limits.
- Closing the Qt viewer with its X button is handled as a clean exit. The
  combined launch shuts down owned children when the viewer exits; Gazebo
  exit also requests launch shutdown.

## Validation

47 tests initially passed across `test_cheap_stage`, `test_live_viewer`,
`test_sim_startup`, `test_sector_controller`, `test_orchestrator`, and
`test_score_normalization`, using `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest
-q -p no:cacheprovider` with their `test/*.py` paths.
After the live Qt-close regression was found, the viewer test group passed
3 tests (including that new regression). Total distinct passing tests: 48.

Colcon package build and ROS launch argument discovery succeeded. Replay
processed/exported 220 recorded frame pairs. Comparing old/new CheapStage on
150 tree-approach pairs found identical belief scores/masks and identical
non-gradient features. Two confidences changed because previous-frame gyro
validity is now considered. Gradient feature differences are intentional.

Artifacts:
`/home/saurabh/ardu_ws/eval_results/cheap_review_20260906_0gYo8b/`
contains `textured_approach.avi`, `sector_preview.png`,
`baseline_comparison.json`, and the first live launch/restart logs.

The first live run processed over 200 frames. MAVROS independently aborted
with `std::future_error: Promise already satisfied` after autopilot-version
request timeouts. The camera continued. Closing the viewer subsequently
exposed the Qt receiver error fixed above. Gazebo exited, leaving other
launch children running until the user requested full process cleanup.
All identified simulator groups and three stale `gz/tf -> tf` relays were
stopped and process cleanup was verified before restarting.

## Scope and limits

This is a correctness pass and visualization, not a claim that optical flow
is perfect or that autonomous avoidance is validated. Textureless regions,
low-motion noise, the fixed forward-FOE assumption, terrain contamination,
and unreliable contour TTC remain. Percentile thresholds, the LK-only belief,
11 sectors, and the 168-feature layout were retained.

Gradient feature values and derivative behavior on skipped calls changed;
future trained artifacts must record this implementation revision. No model
was trained and no old AUC was re-measured. The pre-existing historical
54/84 feature-loader mismatch is still GATE-001, outside this user priority.

Before-copies are in
`.collaboration/backups/e75c78d66de5156fdb6b6aca0650b195/`.
Changes include perception cheap/contours, frame_source sim/imports, the
verification/viewer tools, new `obst_avoidance/live_viewer.py`, new viewer
launch, setup entry point, three focused test files, and project documents.

Next reviewer: inspect timestamp-reset semantics, feature compatibility notes,
and live monitoring/cleanup. MAVROS' startup failure is an external stack
issue observed here, not a corrected MAVROS implementation.
