# v0.2.6 measured retreat anchors

Failure web-main-20261112-132419-e1b2e0: orange-lower-1 was refused before motion.
The nominal retreat Z299.700796mm was0.162741mm below actual arrived
Z299.863537mm. Native descent IK and path passed; a zero-margin comparison
against the obsolete plan stopped the chain. No model/network failure occurred.

Changes:
- Orange first descent creates its recovery target from actual arrived FK.
- The second descent inherits the first descent's checked high retreat, rebinding
  XY to its current start; it no longer reads the old nominal placement plan.
- A shared5mm plan-to-measured retreat binding allowance handles small endpoint
  differences. Within it, anchors are recomputed, never blindly accepted lower.
  Orange preserves a deliberately higher clearance; green reverses to the actual
  measured start. Native IK/path and scene geometry are checked afterward.
- Effective retreat position and adjustment are recorded in the result fact.
  Loaded recovery and green post-release retreat consume this effective position,
  with backward-compatible reading of old reports. They do not consume stale
  input targets when the checked anchor differs.

This allowance is not a reduction of collision clearance, not a new arrival
joint tolerance, and not permission for lateral contact motion. Object/support
geometry, contact checks, native limits and unknown-control handling remain.
Large anchor changes still require re-planning. No automatic retries of actions.

Validation:168 offline tests passed. Original failure reproduced with original
checker; repaired input generation and full LowerChecks passed the saved scene
using native MoveIt IK, FCL and workspace checks.1/3/4.9mm shortfalls pass after
rebinding,6mm refuses. No hardware motion or model calls. Original v0.2.5 and
frozen versions unchanged. A physical full-task test remains necessary.
