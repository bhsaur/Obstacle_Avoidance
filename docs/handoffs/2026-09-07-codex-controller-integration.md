# Controller integration and Claude comparison review

Task CTRL-INTEGRATE; Codex. Claude's CTRL-COMPARE handoff was complete and
its edit lock available before claiming. Token/before-copies:
`.collaboration/backups/29e6e4807d09ae47e05728edb86e7a26/`.

## Accepted and corrected

Claude's goal provider, signed diagnostics, and trajectory comparison are useful.
Baseline commands AND states matched its pre-change `sector.py.before` across
10,000 sequential inputs (NumPy seed 49; random scores, validity, goals/yaws,
and cheap/heavy switches). Existing baseline remains the default.

New opt-in `--controller pathtrack_v2` uses the same path goal, speed,
perception and gains as `pathtrack`, with two mechanism changes:
- Confirm one challenger continuously starting at its first observed capture
  time. Different sector winners and duplicate/backwards clocks restart it.
- Replace held bearings outside the range of sector centres instead of
  clamping unsupported directions onto edge sectors. This conservatively
  excludes the camera's outer half-sectors, not an exact FOV-edge calculation.
Sector identity confirmation can restart as yaw moves a direction between
sectors. These are explicit experimental tradeoffs; no flight benefit claimed.

Logging leaves unavailable capture-to-command age null and separately records
`inference_to_command_ms` in wall time. Invalid/missing odometry interrupts
finite-difference yaw-rate measurement. Metadata identifies goal completion
as crossing x, not endpoint arrival. Reference parameters reject NaN/inf.

Plotter fixes: a direct `frames.jsonl` now resolves its sibling result rather
than trying to parse the JSONL as result JSON. Missing modes retain positions
as `unknown`; goal completion labels now say `goal_x_crossed`.

Runner uses fresh directories, stops on failures/occupied resources, and relies
on each demo's owned-group cleanup. Removed deletion of old runs and global
process-name kills. Demo attempts landing after any takeoff attempt, including
an arm acknowledgement preceding telemetry, and isolates GUI/ROS cleanup errors
so simulator cleanup is still attempted. Cleanup errors produce nonzero exit.

## Independent check of Claude's existing recordings

Workspace `eval_results/cmp_{baseline,pathtrack}_zone_A/frames.jsonl`:

| Arm | Frames | Path m | Minimum estimated margin m | Endpoint distance m | CTE RMS m |
|---|---:|---:|---:|---:|---:|
| baseline | 304 | 22.1406 | 0.2939 | 6.5976 | 5.0645 |
| pathtrack | 292 | 21.9771 | 0.6389 | 6.1504 | 5.0482 |

Endpoint = (30,-3.8). Both crossed x=30; neither reached that endpoint.
The larger margin is a single recorded comparison, not established improvement.
The blocked straight route requires some detour, but does NOT establish that
5 m RMS deviation is unavoidable. The baseline CTE can and should be evaluated
against the same route even though its controller lacks that objective.
Geometry remains the current static 2D obstacle approximation with radius
0.35 m, sampled at logged positions; not swept collision proof or ground-truth
perception. Original recordings and Claude handoff remain intact.

## Validation

From package directory:
`PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/obst-mpl python3 -m pytest -q -p no:cacheprovider test/test_controller_v2.py test/test_reference_path.py test/test_sector_controller.py test/test_orchestrator.py test/test_score_normalization.py test/test_vehicle_lifecycle.py test/test_owned_processes.py test/test_cheap_stage.py test/test_live_viewer.py test/test_sim_startup.py test/test_trajectory_review.py`

100 passed; 18 installed Matplotlib/pyparsing deprecation warnings.
`bash -n tools/run_ctrl_compare.sh` passed. Randomized baseline check above passed.
No simulator launched or flight rerun in this integration. Live frame identity,
GUI lifecycle fault injection, and v2 matched closed-loop trials remain unrun.

## Remaining controller work

This is a bounded mechanism correction, not a complete controller redesign.
Continuous goal recovery/deadband removal, independent command expiry,
uncertainty/speed policy, and an absolute urgency/admission contract remain
open from CTRL-REVIEW. Normalized rankings cannot establish absolute clearance.
Do not promote v2 to default based on unit tests. For the next simulation use
`CTRL_COMPARE_ARMS='pathtrack pathtrack_v2' bash tools/run_ctrl_compare.sh zone_A -3.8`
under the project lock with exclusive simulator resources, then inspect both
full trajectories, blind fractions, margins and endpoint errors.

## Version control

Workspace `.git` is an empty read-only directory; it was not modified.
A new package-local repository records an initial snapshot including existing
project source, Claude's comparison work and these corrections. Generated
builds, logs, recordings and collaboration backups are excluded. This is not
reconstructed historical Git history. No Git user identity was configured;
the commit uses the explicit automation identity `Codex <codex@localhost>`.
Codex additions await peer review; Claude comparison has been reviewed above.
