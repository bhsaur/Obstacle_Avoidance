# Working with Codex and Claude Code

Both assistants use the same files in `/home/saurabh/ardu_ws/src/obst_avoidance`.
The shared instructions live in [AGENTS.md](../AGENTS.md);
[CLAUDE.md](../CLAUDE.md) imports them. Start with the
[current status](PROJECT_STATUS.md) and [task board](TASKS.md).
The workspace root also has entry files for sessions opened in `ardu_ws`.
There is no automatic chat relay or background agent loop in this setup.
Handoffs, task state, and source files are the shared memory; private chat
history is not synchronized.

## Start using it

For an existing Claude Code session, send:

> Read `/home/saurabh/ardu_ws/src/obst_avoidance/AGENTS.md` and the shared
> status, task board, and collaboration guide it links. Review the latest
> Codex setup handoff. Reconcile anything you know from the earlier project
> work with the code and recorded evidence. Use the edit lock before changes
> and leave a handoff for Codex. Start with COLLAB-001; leave flight code alone
> during this setup review.

For later work, give either assistant a concrete task ID and outcome. A useful
default is Claude implementing from its existing project context and Codex
reviewing/testing, with roles exchanged whenever useful. Nothing requires
re-explaining the entire project on each handoff.

Start fresh sessions in the package directory when possible. In an existing
session, explicitly ask it to reread `AGENTS.md`; file changes do not update
the assistant's already loaded context by themselves.

## Claim, work, hand off

From the package directory:

```bash
python3 tools/collab.py status
python3 tools/collab.py claim --agent codex --task GATE-001
```

Claude substitutes `--agent claude`. Save the printed claim token. The helper
uses an exclusive lock file, so two cooperating assistants cannot both claim
the writer role. The lock covers the package, shared notes, workspace entry
files, and experiments using this project's simulator or outputs. Read-only
inspection may run in parallel. It is an advisory protocol, not a filesystem
permission boundary; tools used outside the protocol can still edit files.

While owning the lock:

1. Set the chosen task to `in_progress`, record the actual owner and scope.
2. Check for existing edits. This workspace currently has no usable Git
   history; save before-copies of files you will change under
   `.collaboration/backups/<claim-token>/` with their original relative paths.
   Record new files separately. If Git becomes available, record baseline
   revision and dirty files instead, preserving existing user edits.
3. Implement, inspect the change, and run relevant checks.
4. Write a new handoff under `docs/handoffs/`, update task/status evidence,
   and mark the task `review` when peer review remains.
5. Release, replacing `YOUR_TOKEN` with the token printed by your claim:

```bash
python3 tools/collab.py release --token YOUR_TOKEN
```

The reviewer reads the completed scope and validation, claims the lock before
writing review findings or corrections, and records a separate review handoff.
Only mark `done` when the acceptance criteria are met. If the writer needs to
pause, first record resumable state and release its claim.

An abandoned or malformed lock is never cleared automatically. Identify its
owner and establish that work has stopped before recovering it. A contender
must not copy the owner's token from disk and release the claim. Do not use
`--state-dir` to create a second lock for real project work; that option is
only for isolated helper tests.

## Handoff contents

Use a unique filename such as `YYYY-MM-DD-HHMM-codex-GATE-001.md` or
`YYYY-MM-DD-HHMM-claude-GATE-001-review.md`. Include:

- Task, author, state, scope, and baseline revision or backup path.
- What changed and why; files changed or added.
- Exact verification commands and results; anything not run and why.
- Evidence/artifact paths and scientific claims affected.
- Open concerns and the next concrete action for the other assistant.

Do not overwrite previous handoffs. Keep the task board as the index and the
current-status document concise. The README retains the detailed historical
findings; additions must distinguish new measurements from old results.

## Discovery and verification

Codex loads `AGENTS.md` according to its project/current-directory discovery
rules. This is why both workspace and package entry points are present.
[Official Codex instructions documentation](https://learn.chatgpt.com/docs/agent-configuration/agents-md)
describes that discovery behavior.

Claude Code supports `@AGENTS.md` imports in `CLAUDE.md`, giving both tools
one instruction source. In a fresh Claude session, `/context` can be used to
check loaded memory files. See [Claude Code memory documentation](https://code.claude.com/docs/en/memory#agentsmd).

The setup review can verify files and locking locally. A successful actual
Claude handoff is only confirmed when Claude reads and acknowledges/reviews
it; Codex subagents are not Claude Code sessions.
