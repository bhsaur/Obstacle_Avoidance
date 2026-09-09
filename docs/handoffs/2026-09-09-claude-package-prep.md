# Handoff: GitHub package prep (Claude, PACKAGE-PREP)

Goal: make obst_avoidance publishable. No push performed (no remote; user
decides). Lock claimed/released; only additive/cleanup changes.

Changes:
- NEW GETTING_STARTED.md: user-facing entry doc (what's implemented,
  limitations, prerequisites, build, run recipes, repo layout, what's
  git-ignored). README.md remains the historical research log.
- Removed on-disk junk: 13 __pycache__ dirs, .pytest_cache, *.pyc
  (all already git-ignored; disk cleanup only).
- Verified .gitignore already excludes the heavy/data dirs: logs/ (78M
  SITL .BIN), build/ install/ log/ terrain/ recordings/ eval_results/,
  tlogs. Confirmed 0 tracked files under each.

State for pushing:
- Package-local git repo, branch `main`, ~21M tracked (mostly needed
  models/textures/*.png -- Gazebo world textures / CheapStage flow texture).
- Uncommitted at handoff time: GETTING_STARTED.md (new), this handoff (new),
  obst_avoidance/autonomous_demo.py (M -- Claude's --perception flag +
  world_file/world_sha256 installed-path fix from HEAVY-CTRL-TEST),
  docs/TASKS.md (M -- pre-existing Codex edit, not Claude's).
- No remote configured; recordings/eval_results live at the WORKSPACE ROOT
  (outside this repo) and are not pushed.
