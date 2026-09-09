# Handoff: promote getting-started to README front page

Date: 2026-09-10
Agent: claude
Task: PUBLISH (docs-only)

## What changed

GitHub only auto-renders `README.md` as the repository homepage, so the
getting-started guide could not stay under a separate filename and still be
the front page. Resolved by:

- `git mv README.md docs/RESEARCH_LOG.md` (the 3860-line chronological log)
- `git mv GETTING_STARTED.md README.md` (now the front page)
- Reference fixes:
  - `README.md` blockquote now points at `docs/RESEARCH_LOG.md`
  - `obst_avoidance/perception/features.py` comment → `docs/RESEARCH_LOG.md`
  - `PROJECT_PLAN.md` two links `GETTING_STARTED.md` → `README.md`

Also re-authored the previously unpushed commit (Codex's PROJECT_PLAN.md +
GETTING_STARTED.md update) from `Codex <codex@localhost>` to
`saurabh <saurabhbhardwaj911@gmail.com>` per the user's attribution
preference, then pushed.

## Pushed commits (origin/main)

- `abf195b` Document project evidence and staged gate development plan (re-authored)
- `88364c4` Make getting-started content the README front page

## Not done (needs the user)

- The GitHub repo **description** field (one-line blurb) cannot reference a
  file and `gh` is not installed here; account password/token intentionally
  not used. User sets it in repo Settings / the "About" gear.

## Validation

Docs-only + one comment line in features.py (no behavior change). No rebuild
needed (comment only). Offline test suite not re-run (no code-path change).
