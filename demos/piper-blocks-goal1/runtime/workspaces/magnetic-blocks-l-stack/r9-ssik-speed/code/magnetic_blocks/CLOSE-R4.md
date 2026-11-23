# c-Four1.0 close r4

The 2026-11-16 16:29 run stopped at orange-close-4 BEFORE dispatch. The r3 closing-alignment branch called same_state without importing it. This was a code regression, not an IK, image or network refusal. Recorded jaw72.5mm, no close receipt.

Import the binding function and bind orientation to arm state, retaining actual measured jaw width. Execution.guard already binds current jaws to the previous receipt; nominal73mm is not a second opening threshold. Invalid/non-finite references and joint drift >0.01rad still fail. No collision, joint or gripper-control tolerance changed.

Validation: exact historical RGB/observation replay fails with NameError on r3 and passes on r4, with no model or motion tools allowed. 40 tests and14 subtests pass, including actual closing-reference branches for all four colors, bad bindings and optional wrist checks. The r3 release/retreat replay passes again in the production sandbox. Zero hardware dispatches. No new whole-task preflight timing claimed; planner unchanged from r3. Live test pending.
