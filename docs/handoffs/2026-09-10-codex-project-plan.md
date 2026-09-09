# PLAN-001 — full project plan

Author: Codex. State: review (user/peer review pending).
Baseline: 5076902 with existing Codex CHEAP-MULTI task update and shutdown
handoff preserved. User requested one document covering the full plan,
completed work and remaining work. No runtime changes or experiments.

Created PROJECT_PLAN.md, the current consolidated roadmap. It distinguishes
implemented capabilities, demonstrated outcomes and unproven claims; includes
basic/mixed-scene and CPU-heavy findings, common-controller prerequisite,
geometry/schema repairs, risk-versus-benefit labels, censored recordings,
scheduling/watchdogs, six policies, laptop/GPU work split, collaborator
artifacts, matched-budget evaluation and completion criteria.

Linked the plan from GETTING_STARTED.md, PROJECT_STATUS.md and TASKS.md.
Marked an old live-runtime note historical; retained prior evidence/history.
Committed the previously preserved CHEAP-MULTI shutdown notes with this
consolidation so the plan's evidence links resolve in a checkout.

Verification: Python relative-Markdown-link existence check across plan,
getting-started, status and task board: no missing targets. git diff --check
passed. Read/reconciled source and handoffs; numerical results are attributed
to earlier experiments, not rerun here. No unit tests/build/simulation needed
for documentation-only changes. Project simulator remains intentionally stopped.

Next: review plan with Saurabh/Claude; choose a bounded roadmap task, claim
the edit lock and implement it. Proposed GPU roles are not assigned and no
messages have been sent to collaborators. GPU speed/benefit are not assumed.
