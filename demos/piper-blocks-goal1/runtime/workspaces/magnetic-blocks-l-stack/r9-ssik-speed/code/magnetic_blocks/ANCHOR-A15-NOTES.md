# r9-ssik-speed-r2-transition-a15

Independent child of 803576106b4ad7ce97bbde19161452ef97ae8b89. Original r2, previous transition candidate and frozen sources remain unchanged.

- Operator selected joint acceleration cap 1.5 rad/s^2. Faster-profile launcher, native acceptance and waypoint compiler agree. SDK 80%, jaw25mm/s, joint arrival/binding0.01rad, moving tracking0.1rad unchanged.
- Execution guard reads current Owner measured CAN feedback, not the lagging bridge cache. Native staging independently checks fresh Owner feedback as well as formal policy identity/observation.
- Before dispatch, reconstruct and check a changed-anchor trajectory at most twice. A native structured RejectedBeforeDispatch may retry only for the exact initial-connector/anchor reasons, explicit executed=false, dispatch_started=false, no_action_executed=true, control_uncertain=false and episode_ended=false. Check unchanged owner/previous receipt before each repreview. All rebuilt wire geometry/dynamics checks remain. No jaw retry, no collision/timeout/network/ambiguous replay.
- Single-arm and multi-waypoint paths share the same bounded dispatch helper; record retry facts and final used curve timing. Original green-to-yellow transition and final-yellow optional retreat remain.
- User cancelled the additional retry-then-disconnect regression test; it is not part of this candidate's test suite. Runtime retains conservative handling of unknown execution results.

Offline: failed orange-lower record reproduced at the old cap. Fresh-start reconstruction also passes at the old cap, proving the fix is not only a raised threshold. Real isolated task/native geometry passes with actual payload reconstructed from recorded close feedback, plus0.003rad drift. Full recorded scene preflight and historical green-yellow transition pass. No live motion/SDK dispatch; no physical qualification claimed.
