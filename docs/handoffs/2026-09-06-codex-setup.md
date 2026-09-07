# COLLAB-001 — initial Codex setup handoff

Date: 2026-09-06. Author: Codex. State: ready for Claude review.

## User request and scope

Read the supplied build reference and set up Codex and Claude Code to work
together on `src/obst_avoidance`. This handoff records local collaboration
setup and an evidence-based project audit. No research implementation,
training run, simulator launch, flight, or model download was performed.

The attached document was read as project reference material, not as a list
of commands to execute. Its original contents are archived unchanged.

## Files and baseline

Added workspace `AGENTS.md` and `CLAUDE.md`; added package `AGENTS.md`,
`CLAUDE.md`, `.collaboration/.gitignore`, `tools/collab.py`,
`test/test_collab.py`, `docs/COLLABORATION.md`, `docs/PROJECT_STATUS.md`,
`docs/TASKS.md`, the archived reference, and this handoff.
The only existing project file edited was `README.md`, to add navigation
above its preserved historical content.

`git status --short` reported that the workspace is not a Git repository.
No Git metadata was replaced or initialized. The pre-edit README backup is
`/tmp/obst-collaboration-setup-7qvtq5tj/README.md`; a workspace-local copy also
lives under `.collaboration/backups/initial-setup/README.md`.
The reference SHA256 is recorded in `PROJECT_STATUS.md`.

## Findings for Claude to reconcile

- Your README Steps DM–DR are newer than the reference: contours are already
  detection/features-only, and the 102-vs-168 ablation already ran.
- The historical AUC 0.900 belongs to 54-feature/5-sector data. It is not a
  result for the current 168-feature runtime vector.
- Historical loading now hits `KeyError: 'tau_area_sector_0'`: 54-key log
  records reach an extractor expecting today's 84-feature 5-sector layout.
- Runtime gating/six-arm scheduling remain unbuilt, and replay EOF handling,
  label-geometry consistency, device selection, and artifact versioning need
  attention before the full evaluation.
- Current metric depth still normalizes sector-score span within each frame;
  the reference's "no normalisation code" statement is too broad.

These are setup findings, not fixes to the research code. Suggested next
implementation is GATE-001, after reconciling any newer Claude-side context.

## Validation and its limits

The audit initially ran:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -q -p no:cacheprovider \
  test/test_sector_controller.py test/test_orchestrator.py \
  test/test_log_frame_source_identity.py test/test_score_normalization.py
```

The controller/orchestrator/score-normalization group produced **36 passes**.
The identity test failed while ROS tried to create logs outside the sandbox.
An approved retry of only `test/test_log_frame_source_identity.py` initialized
ROS/MAVLink but stalled waiting for simulation clock, and was interrupted
after approximately 45 seconds. Shutdown reported a failure/warning. No image
comparison was reached. This test is live integration and can configure
MAVLink streams; exclude it from offline checks. The unbounded startup-clock
wait is tracked under SETUP-001.

The new helper was checked with:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test/test_collab.py
```

**14 passed**, covering exclusive claims, parallel contention, incorrect/old
tokens, corrupt state, dangling symlinks, and independence from the current
directory. Thus **50 offline tests passed** across these checks; the live
integration check remains incomplete. No training scores were recomputed.

Additional read-only evidence: historical discovery found 26 accepted logs;
the first accepted record had 54 feature keys and failed current extraction
at `tau_area_sector_0`. The cached recording dataset
`/home/saurabh/ardu_ws/eval_results/gate_recordings_168.npz` has shape
`(15282, 168)` and positive fraction `0.6892422458`.

The setup also checks relative documentation links, both Claude imports,
byte identity of the archived reference, preservation of README history,
and actual claim/status/release in the project directory. A separate Codex
subagent reviewed the status documents against the audit. That review is
not a Claude review.

## Next action

Claude: read `AGENTS.md`, inspect this handoff/status/task board, and report
any factual corrections from your prior work with source/artifact evidence.
Claim COLLAB-001 before writing a uniquely named review handoff, then update
the board and release. The setup can be marked `done` after that review.

This workflow exchanges files; no automatic Claude process or chat relay was
started. Existing Claude sessions need the one-time instruction in
`docs/COLLABORATION.md` to read and adopt it.
