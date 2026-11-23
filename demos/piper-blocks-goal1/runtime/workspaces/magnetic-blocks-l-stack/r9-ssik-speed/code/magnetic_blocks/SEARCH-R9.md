# c-Four-1.0-search-r9

Independent candidate from orientation-r8. No robot motion during development or validation. Only search scheduling, IK transport/cache and candidate ordering changed. Full main remains ordinary code, with no mandatory Agent/Gemini calls.

## Changes
- Near-complete layouts get up to three priority slices, then FIFO exploration resumes. Alternatives/generator cursors remain available; global180s and fast30s budgets unchanged.
- Cardinal grasp headings tried before nearby +/-3deg variants; exact duplicate grasp families share a cursor. Similar but distinct poses remain fallbacks, not declared unreachable.
- Current and diverse fallback seeds sent in one existing native IK batch RPC; original solution validation, fast/full budgets and typed refusals retained.
- Successful seed positions indexed once with vectorized nearest lookup instead of repeated full cache scans.
- Exactly identical target pose can reuse a previously verified IK solution on a nearby joint branch. The0.2rad branch preference is NOT a tracking/arrival/collision tolerance. Actual connecting paths still undergo every original geometry/native check. If a reused target's edge is refused, retry the original seed search with reuse disabled. Exact keys, no rounding or failure promotion; caches live only within this Planner.

## References and scope
MoveIt Task Constructor provides staged partial solutions and cost-based organization: https://github.com/moveit/moveit_task_constructor and https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html
OMPL LazyPRM delays expensive validation until candidate solution paths: https://github.com/ompl/ompl/blob/main/src/ompl/geometric/planners/prm/src/LazyPRM.cpp
These are design references, not imported planning stacks. Existing complete-path native certification already follows a lazy structure and remains required. Keep KDL, existing RAX function main and Piper backend.

## Validation
Actual20261117-095954 initial scene is replayed through production Frontend, Session/fsync, Unix RPC and bubblewrap, with real FK/IK/native collision tools. Every replay forbids owner access and motion tools. Current-scene base/new plus3ms-per-IK-RPC delay comparison and prior-scene regression are saved in WEB-VALIDATION.json. Performance depends on scheduling and machine load; single-scene timings are not universal latency claims. All returned actions and feedback envelopes require original native certification; batch previews retain checked standalone fallback.74 tests+4 subtests pass on host and in production isolation.17 additional legacy search checks pass using the four-color fixture; two legacy harness failures also reproduce unchanged on r8 (stationary-red mocked solver and finish-cache mocked transport), so they are not counted as qualified four-block tests. Original three-color fixtures are incompatible with four-color scene validation. The first experimental batch/index-only change did not improve latency and is recorded in AFTER.log, not used as the performance claim.

## Unchanged
Mechanical joint/workspace bounds, collision/table/payload margins,0.01rad arrival/binding,0.1rad tracking,0.5deg task-orientation allowance,80% motion speed and25mm/s jaws. No frame timestamps or physical success claims manufactured. Original/frozen candidates preserved. Offline success is not a physical task result.
