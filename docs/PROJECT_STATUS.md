# Current project status

Current: CTRL-FINAL implementation complete, peer review pending. Experimental
pathtrack_v4 reduces fixed-input steering reversals and adds bounded failure
handling; it did NOT complete the live endpoint mission. 122 distinct offline
tests passed. Latest run landed/disarmed; host simulator processes/ports clean.
See [final controller handoff](handoffs/2026-09-07-codex-controller-final.md).
The baseline remains default. Depth fallback and mission reliability remain
open; this is not completion of the adaptive-gating research project.

Latest update 2026-09-07: Claude controller comparison reviewed; opt-in
`pathtrack_v2` corrects challenger confirmation and unsupported held bearings.
Baseline preserved. 100 offline tests pass; v2 flight validation pending.
Both recorded Claude runs crossed the goal line but ended more than 6 m
from the route endpoint. See [integration handoff](handoffs/2026-09-07-codex-controller-integration.md).
A package-local Git repository now tracks source; the workspace root `.git`
remains untouched. Older status entries below are historical.

Last reconciled: 2026-09-06, by Codex, against source, the README research log,
local artifacts, and the supplied project reference. This is a status snapshot,
not a new replication of the research experiments.

Latest user priority: CHEAP-001, reviewing CheapStage and showing its camera
sectors/scores. The [CheapStage handoff](handoffs/2026-09-06-codex-cheap-review.md)
records timing/sector-feature corrections, 48 distinct passing tests, replay
comparison, and the live demo. `cheap_viewer.launch.py` now starts the existing
zone simulator plus a monitoring camera display. No flight was commanded.
The startup-clock hang described in the initial setup audit below has since
been bounded, with failure cleanup; see that handoff for the new behavior.
The user's repeated Ubuntu crash dialog was traced by core dump to MAVROS
2.14.0's duplicate-command-ACK handling. A matching-version workspace overlay
now passes a 25-request duplicate-ACK test and live cold-start monitoring;
see the [MAVROS crash handoff](handoffs/2026-09-06-codex-mavros-crash.md).
Source `install/setup.bash` to select the patched MAVROS. Claude review is
pending; active simulator ownership is recorded in the task board.

## Objective

Adaptive perception gating for monocular drone obstacle avoidance: run the
cheap cue continuously and invoke metric depth only when needed. The decisive
experiment is learned gating versus periodic depth at a matched measured
compute budget, using the same controller, worlds, speed, and paired seeds.
The six-arm closed-loop comparison is still pending; offline classifier AUC
does not establish collision reduction or a publishable novelty claim.

## What exists

| Area | Current evidence/state |
|---|---|
| Simulation and capture | `launch/env_zones.launch.py`, `worlds/env_zones.sdf`, `frame_source/sim.py`, and `record_pass.py` exist. Camera monitoring has now been run; no flight commanded. MAVROS startup-crash correction is in SIM-001. |
| Replay | `obst_avoidance/frame_source/log.py` provides `LogFrameSource`. The current orchestrator does not distinguish replay EOF from temporary no-frame availability. |
| Cheap perception | `perception/cheap.py`, `contours.py`, `geometry.py`, and `features.py`; default 11 sectors and 168 features. LK drives the belief; contours supply detection/features only. |
| Heavy perception | `perception/heavy.py`; metric outdoor DA-V2 Small, geometry-derived row band, and within-frame sector-score normalization. Device selection/benchmarking remain to be made explicit. |
| Shared control | `control/sector.py` and `platform/vehicle.py`; world-bearing targets, time-based hysteresis, decoupled command republishing. |
| Runtime | `runtime/orchestrator.py` accepts one perception stage. `tools/run_heavy_only_eval.py` supplies a heavy adapter. Cheap -> gate -> optional heavy scheduling is not implemented. |
| Offline gate research | `tools/gate_dataset.py`, `gate_train.py`, `gate_validate.py`, `gate_dataset_recordings.py`, `dp_dr_gate_eval.py`; historical and recording-based experiments exist. Historical dataset loading currently has schema drift. |
| Six runtime gates/config harness | No `obst_avoidance/gating/` package or unified six-arm configuration/harness yet. Prior single-arm evaluation tools and step-specific batch scripts exist. |
| Collaboration | Shared instructions, task board, handoff convention, and advisory edit-lock helper added. Actual Claude acknowledgement/review is pending. |

## Corrections to the supplied reference

The original is preserved byte-for-byte at
[reference/project_complete_reference.md](reference/project_complete_reference.md). Source was
`/home/saurabh/Downloads/project_complete_reference(1).md`;
SHA256 `3c65759ac719cb1f2186d47a7d3f3b94b35ca92deeb6e320155ed52d7a247e09`.
Read it for the research rationale and findings, with these updates:

1. **The contour merge is resolved.** README Steps DM/DN/DO and current
   `cheap.py` make contours detection/features-only. `tau_area` does not modify
   `SectorBelief.scores` or `.valid`. The reference's MIN merge and unresolved
   choice in sections 4 and 10 are obsolete.
2. **A 102-vs-168 comparison already ran.** README Steps DP/DQ/DR report
   matched recording-data pooled LORO AUC 0.979 versus 0.966, respectively.
   The added contour features did not improve generalization on that dataset.
   This is not comparable to the old 0.900 result: the dataset and labels
   differ, with approximately 68.9% positives and positional-only AUC 0.849.
3. **The old 0.900 result used 54 features / 5 sectors.** README Step CS and
   `gate_dataset.py` explicitly excluded 102-feature / 11-sector logs. The
   reference calls this result 102-feature in several places and 54-feature
   in another; retain the schema supported by the original experiment record.
4. **Historical extraction is currently broken.** The old loader accepts
   54-key logs but calls today's `extract_features(..., 5)`, which expects
   84 values including contour fields. Reproduced on an actual accepted log:
   `KeyError: 'tau_area_sector_0'`. Fix compatibility explicitly; padding old
   data or silently changing its definition would invalidate reproduction.
5. **A runtime learned gate is still missing.** Having trained offline
   classifiers does not mean there is a deployable gate artifact/interface,
   validated schema contract, or scheduler. Historical `p*=0.125`, rate 26.5%,
   and periodic `k=4` are reference operating points, not current matched
   budget guarantees (25% and 26.5% are already different rates).
6. **Heavy scores still have a normalization step.** The metric checkpoint
   removed the old relative-depth calibration problem, but current `heavy.py`
   calls `geometry.normalize_scores_to_span` across sector scores within each
   frame (Step CG). The reference's blanket "no normalisation code" statement
   does not describe the current implementation.

## Open issues and limits

- Prior CPU depth measurements (~1256 ms) and closed-loop failures are
  historical measurements on the documented setup. Measure the actual target
  GPU; do not assume a latency or safety gain from a different device.
- The old gate data contains only collision-ending runs. New image-logged
  closed-loop data needs clear/successful approaches as well as failures,
  whole-run splits, leakage checks, and a held-out-zone evaluation.
- Recording-based labels use an obstacle-distance approximation different
  from the orchestrator's circle/box surface-distance calculation. Reconcile
  geometry and label provenance before treating the datasets as equivalent.
- A replay matrix needs explicit EOF behavior and recorded-state vehicle
  access; offline replay checks wiring, not counterfactual flight safety.
- `setup.py` lists `setuptools` and `pymavlink` in `install_requires`. A complete pinned
  dependency/environment record and GPU smoke/benchmark path remain work.
- The workspace `.git` was not usable (`git status` failed). No repository
  initialization or Git repair was performed; use the collaboration lock and
  before-copies until version control is established.

## Validation during setup

See the [setup handoff](handoffs/2026-09-06-codex-setup.md) for exact commands and the final
results. Controller, orchestrator, and score-normalization unit tests were
checked without launching a simulator. The live frame-identity test needs a
running simulator and ROS log access; it is outside the offline smoke suite.
Its first attempt failed on ROS log access. An approved integration retry
initialized ROS/MAVLink but stalled waiting for a nonzero simulation clock and
was interrupted. `SimFrameSource._assert_initial_offset` currently waits for
that clock without enforcing its timeout; the advertised no-sim skip is not
reliable. This was not an image-identity assertion failure. Include bounded
startup and cleanup in SETUP-001 before using that check as a smoke test.

Next implementation priority: **GATE-001**, preserving and identifying the
54/102/168 feature schemas before runtime gate integration. Full work order
and suggested division of responsibilities are in `TASKS.md`.
