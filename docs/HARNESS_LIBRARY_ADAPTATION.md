# Harness separation and continual learning — 2026-11-05

This revision keeps the Codex API relay. Both models remain `gpt-6-astra`
(main high, auxiliary vision medium). It does not migrate login accounts or start hardware.

## Generic harness

- Motion admission during the finish reserve uses the backend's `mutating` contract,
  independently of robot and model provider. Read-only and hold tools remain available.
- Dashboard telemetry is selected by `services.backends.TELEMETRY`. Device reads and
  geometry conversions live in `services/<robot>/telemetry.py`; the dashboard never
  constructs a motion backend or posts to a Franka control endpoint.
- Arming lives in `services/arming.py`; the old Franka module is an import compatibility
  shim. Disarm reports a request, not a physically verified stop. Each backend's
  existing control loop observes the arming file.
- `scripts/piper_explore.py --task-config FILE` reads a task manifest. Blue-cup prompts,
  version-specific rehearsal instructions, geometry notes and conditional experience
  are under `examples/tasks/blue-cup/`. The launcher retains Piper owner lifecycle code.
- Image transport and site calibration paths use registered robot facts rather than
  robot-name branches in the harness.

## Native skill library

Sources and exact commits are in `UPSTREAM.json`: RAX `real/rainbow` merge mechanics,
RAX main's robot-neutral continual-learning skill, and the unmodified, pinned
`skills-lib/scripts/check_skills.py` format checker.

The Piper shared library is
`/home/ubuntu/code_junqi/rax-piper-franka-20261104/runtime/skill-library/piper`.
`Configuration.skill_library_root` persists the shared base, so per-run artifact roots
cannot accidentally create isolated libraries. Every new workspace clones this repository
on `session/<name>-<root-hash>`; equal directory basenames do not collide.

1. Read `skill-library/INDEX.md`; fetch/merge origin/main and retrieve relevant skills.
2. Check current preconditions and live tool schemas before reuse.
3. Contribute executable `main(ctx, ...)`, seven-section SKILL.md, package modules,
   perception checkpoints, images and real run evidence on the session branch.
4. Commit, then call `skill_library_merge(message=...)`. The host checks committed data,
   compiles code without executing it, validates index/status/dependencies/evidence,
   rejects links and submodules, and merges under a lock. Conflicts leave main intact.
5. A new session obtains the checked result. The agent cannot push directly to the
   shared library: the real Codex sandbox mounts origin read-only, with its clone writable.

The library is initially empty. Existing task memory remains intact; no historical manual
success or assisted rehearsal was relabelled as an unattended verified skill. The checker
validates submitted evidence structure; it is not an independent physical certification.
No GitHub publication was performed.

## Verification

- 5090: 349 unit tests passed in 25.81 s; 60 existing yourdfpy deprecation warnings.
- Mac: 336 passed, 13 asset/platform-dependent skips, 27.36 s.
- Actual 5090 bubblewrap probe: workspace writes and origin fetch succeed; shared-library
  writes and direct pushes fail. No model or hardware service starts in that probe.
- Frozen v0.1 (60 manifest entries) and v0.3 (109 entries): every hash matches, no writable
  files. Working v0.3 Python files remain writable.
- After the operator announced hardware availability, a read-only GET of the existing
  owner endpoint returned connection refused. No owner was started and no motion ran.

Report: `commissioning/harness-library-offline-20261105/SUMMARY.json` on the server;
local copy: `verification/harness-library-offline-20261105/SUMMARY.json`.
