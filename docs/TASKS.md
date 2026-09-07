# Shared task board

Updated: 2026-09-06. This board records priorities; execute tasks within the
user's current request. Suggested owners below are a useful division, not
claims. Claim the edit lock before assigning an actual owner or writing.

States: `ready`, `in_progress`, `review`, `done`, `blocked`.

| ID | Task | State | Suggested implementer / reviewer | Depends on |
|---|---|---|---|---|
| COLLAB-001 | Shared instructions, status, locking, handoff, and Claude setup review | review | Codex / Claude | — |
| VIZ-REVIEW | Review Claude trajectory plotter and define controller improvements | review | Claude / Codex review completed; follow-up fixes pending | [Review findings](handoffs/2026-09-07-codex-trajectory-review.md) |
| CTRL-REVIEW | Diagnose controller behavior and prioritize versioned fixes | review | Codex analysis complete / implementation pending | [Reproduced cases](handoffs/2026-09-07-codex-controller-review.md) |
| CHEAP-001 | Review/fix CheapStage correctness and show camera sectors/scores | review | Codex / Claude (pending) | User priority, 2026-09-06 |
| SIM-001 | Fix confirmed MAVROS duplicate-ACK startup abort and verify camera restart | review | Codex / Claude (pending) | User crash report; live CHEAP-001 demo restored |
| GATE-001 | Version feature schemas and restore historical dataset loading | ready | Codex / Claude | — |
| GATE-002 | Define gate/artifact contract and implement six policies | ready | Claude / Codex | GATE-001 |
| RUNTIME-001 | Shared cheap/gate/heavy scheduling, decision logging, and replay EOF | ready | Codex / Claude | GATE-002 |
| DEVICE-001 | Explicit depth device selection and standalone benchmark | ready | Claude / Codex | — |
| EVAL-001 | Arm configs, resumable paired-seed runner, aggregation, offline matrix | ready | Codex / Claude | RUNTIME-001, DEVICE-001 |
| DATA-001 | Align labels/geometry and plan current-schema image-logged collection | ready | Claude / Codex | GATE-001; RUNTIME-001 for collection |
| TRAIN-001 | Collect matched data and validate/export the final runtime gate | ready | Claude / Codex | DATA-001, RUNTIME-001, DEVICE-001 |
| SETUP-001 | Reproducible dependencies, stack setup, and no-flight smoke path | ready | Claude / Codex | DEVICE-001 |
| GPU-001 | Measure target GPU and run the closed-loop comparison | blocked | assign when hardware is known | EVAL-001, TRAIN-001, SETUP-001, GPU access |

## COLLAB-001 — current handoff

Actual implementer: Codex. Claude review has not occurred yet.
Scope: workspace/package agent entry files, `docs/`, README navigation,
`tools/collab.py`, `test/test_collab.py`, `.collaboration/.gitignore`.
Handoff: [Codex setup](handoffs/2026-09-06-codex-setup.md).
Acceptance: shared paths/imports resolve, original reference matches its
source, edit claims reject contention, and Claude reviews the initial status
and handoff. Local setup completion does not imply Claude has already read it.

## Acceptance criteria for the next tasks

**CHEAP-001 / SIM-001 handoff:** Codex implemented the CheapStage corrections,
monitoring viewer, and matching-version MAVROS duplicate-ACK backport.
Evidence: [CheapStage review](handoffs/2026-09-06-codex-cheap-review.md) and
[confirmed MAVROS crash/fix](handoffs/2026-09-06-codex-mavros-crash.md).
48 distinct offline tests passed for CHEAP-001. The real patched MAVROS
binary passed the 25-request duplicate-ACK probe and a live cold start with
camera processing and connected/disarmed telemetry. Claude review pending.

The live demo is being left running for the user under launch PID/PGID
944722 (tool session 18763); MAVROS PID 944758, viewer PID 944865, as recorded
at handoff. Recheck process identity before signaling these IDs. Logs are in
`eval_results/cheap_review_20260906_0gYo8b/mavros_fix_1/` at workspace root.
The simulator and MAVLink ports 14550/14551 are occupied while this demo is
open. Releasing the edit lock permits code review; it does not make these
runtime resources available for a competing recorder/viewer/experiment.

**GATE-001:** name and preserve historical 54/5-sector, pre-contour
102/11-sector, and current 168/11-sector schemas (also distinguish current
84/5-sector vectors). Restore loading the historical logs with their original
ordered features; reject unsupported/mixed schemas clearly. Record schema in
datasets/model artifacts; fit imputation/scaling only on training folds.
Add regression coverage for historical loading and incompatible model inputs.
Do not silently fabricate absent contour data or relabel old results.

**GATE-002:** always-heavy, never-heavy/flow-only, periodic, seeded random,
heuristic, and learned policies behind one interface; deterministic/resettable
state. Define input names/order, preprocessing, threshold, model provenance,
and compatibility validation before accepting an artifact. Keep depth and
ground truth out of gate inputs. Tests cover rate/state behavior and schema
rejection. Establish a current operating point from validation data.
An initial artifact/threshold may be provisional for harness development;
the final experiment requires TRAIN-001's validated artifact.

**RUNTIME-001:** cheap runs for each processed frame; one gate decision routes
optional synchronous depth into the same controller. Log decision, chosen
belief, per-stage/total latency, heavy invocation count, decision count, schema,
and seed. Separate replay EOF from temporary missing live frames. Stub tests
exercise all policies through common wiring, including termination and replay
state. Any collection mode uses the same feature and logging definitions.

**DEVICE-001:** select CPU/CUDA explicitly, handle tensor transfer correctly,
fail clearly on unavailable requested devices, and record actual device/model
revision. Benchmark warmup and measured latency separately. GPU-specific
checks can remain pending until hardware exists; CPU portability is testable.

**EVAL-001:** only policy/config varies between arms; common speed, world,
controller, and paired seeds. Resume completed runs without overwriting data.
Track process ownership and stop only processes launched by that runner;
do not copy the broad process-kill patterns in historical batch scripts.
Derive comparison tables from run rows, record failures, and compare realized
heavy calls per processed decision plus measured compute/latency. Dry-run all
six arms offline without claiming replay measures collision avoidance.

**DATA-001:** use the same obstacle geometry/label semantics as evaluation;
version labels and verify synchronization. Define safe-run coverage and
whole-run/zone splits before collection. Keep the recording-based 102-vs-168
ablation separate from the historical 54-feature result. Actual simulation
collection is a distinct task from planning it.

**TRAIN-001:** execute the agreed image-logged data collection, including clear
and successful runs as well as failures. Train using the versioned current
schema; compare feature sets only on matched samples. Repeat whole-run and
held-out-zone validation, leakage checks, and operating-point selection on
validation data. Export the model, preprocessing, feature order, schema,
threshold, training provenance, and measured deferral rate. Keep final test
runs held out and recalibrate comparison arms to the realized compute budget.
Do not mark this complete based on a collection plan or a provisional model.

**SETUP-001 / GPU-001:** record observed dependency versions, model revision,
ROS/Gazebo/ArduPilot/MAVROS setup, machine/device, and reproducible commands.
Make simulator startup/clock waits bounded and clean up on startup failure;
verify no-simulator behavior separately from live image-identity integration.
Then benchmark and run the paired matrix with measured budget matching.
Report collision/completion, deferral, latency distributions, switches, and
lateral deviation with enough run metadata to reproduce the findings.

## CTRL-INTEGRATE — review

Owner: Codex. Review Claude CTRL-COMPARE, preserve baseline, add opt-in
controller mechanism fixes, correct logging and runner, verify and commit.
Backup: `.collaboration/backups/29e6e4807d09ae47e05728edb86e7a26/`.

Integration evidence: [Codex review and corrections](handoffs/2026-09-07-codex-controller-integration.md).
100 tests passed; baseline equivalence checked over 10,000 steps. Claude comparison
reviewed; Codex opt-in changes await peer review and matched flight trials.

## CTRL-RECOVERY — review (paused by user)
Owner Codex; baseline c0e074f, initially clean. Complete controller recovery,
uncertainty response, independent command expiry and matched simulation.

User stopped work; changes uncommitted. [Resume handoff](handoffs/2026-09-07-codex-recovery-paused.md).
116 tests passed; matched flights completed and cleaned up. v3 timed out
after stopping with insufficient flow; final review/commit pending.

## CTRL-FINAL — review
Codex resumed preserved CTRL-RECOVERY edits on c0e074f; inspect dropout
reversals, add opening steering and bounded perception failure, verify/commit.

Final evidence: [controller handoff](handoffs/2026-09-07-codex-controller-final.md).
Controller/runtime changes and live validation complete; 122 offline tests pass.
v4 flight timed out, no detected collision, landed/disarmed, no residual stack.
Mission recovery/depth fallback remain open; peer review pending.
