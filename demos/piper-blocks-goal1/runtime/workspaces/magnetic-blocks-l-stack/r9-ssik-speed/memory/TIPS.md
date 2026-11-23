# R3 条件化经验 — 2026-11-11

只适用于本轮约 60 mm 磁性积木、当前指垫和左臂。一次受监督成功；最终支撑和退开由现场用户确认。所有长度为毫米，控制点是 left_link6，开度为 SDK 指令值，不是实测两软垫净距。

| Task phase | Preconditions | What worked | Visible effect | Evidence |
|---|---|---|---|---|
| 底块抓取 | 橙块直立、桌面支撑；前后面夹持，侧面留空；已对准 | 本轮法兰高度 205；闭合指令 50，反馈约 50.9–51.2；试抬 10 | 顶面深度由约 78 升到 86.8–87.1，腕部相对位置稳定 | `runtime/frames/75abd4859eef4025a241fd8ac2ad0ae0/` |
| 排除失败组合 | 同块同场景 | 法兰 220/闭合56 和法兰215/闭合53 均未抓起；不要继续沿用 | 试抬后顶面高度仍约78–80 | `runtime/frames/92d371d4e45047478d916717fdf5937a/`、`5513f0b0eb9840a396f41df26c136f97/` |
| 底层并排 | 已证实夹持；邻块在侧方 | 橙块从抓取高度抬75，横向 +Y55（30+25）；回到205释放，张至60再69，退至290 | 红橙侧面相接、前沿大体齐；红块发生位置变化 | `runtime/frames/422fddd5a1814fada9d5aab526bab68c/` |
| 磁吸后的定位 | 相邻积木受到接近/放置影响 | 重新测红色顶面中心；本轮约[304.4,-48.9,80.4] | 更新了绿色放置锚点 | `output/exploration-notes.md`、`evidence/tool-ledger.json` |
| 绿色抓取 | 绿色在较侧方，原朝向IK困难 | 空载分步转45°再90°，沿新朝向接近；本轮法兰205，闭合50，反馈约51；先试抬10 | 绿色相对腕部稳定，随后搬运成功 | `output/manual-action-log.json` M42–M55 |
| 上层放置 | 重新定位红块；已抓住绿块 | 以实测底块约60高为关系：法兰从抓取205对应放置265；逐步下降、张至60/69、退至300 | 松爪后结构稳定；现场确认绿色在红色上、红橙并排且两指离开 | `runtime/frames/63d843b4c9bf4db7b3c16b1899f95a37/`、`evidence/operator-final-confirmation.json` |
| IK恢复 | 尚未接触/未下发失败动作；有当前实测关节 | 比较朝向和种子，在空载位分步旋转；每段重新走现有路径检查 | 直接求解失败的绿色接近经分步种子可解 | `evidence/tool-ledger.json` |
| 退开限制 | 最后法兰[304,-49,300] | 300已到达；330动作前被拒，315/320只读候选也失败；保持已知状态 | 无额外运动、无不确定控制 | M65/M66；`evidence/STOP-RESULT.json` |

不能把205/265作为另一张桌面的绝对目标。需重新测物体、支撑面并检查夹持关系。不能把50 mm单独当成抓稳证据。初始张开73，后续约69；每次张闭不超过工具当前10 mm步长。底部抓前后面、横向拼接的用户策略已得到本场景实测支持；磁性作用的具体力学原因尚未独立测量。腕部无手眼标定，不能把腕部光学坐标直接作为基座目标。

## R4 function evidence (2026-11-11; supervised rehearsal only)
- First empty startup opening: near-zero measured arm,0.3mm jaw, all3 blocks visibly table-level and remote; ordinary RGB/direct depth plus table plane and wrist pad checks.0.069m target via<=9mm feedback-based steps;8 receipts, final68.6mm,25.964s. Evidence output/rehearsal/r4-open-01.json; runtime/frames/25186a33097b435ab7cd6ed25b235616/head.jpg and left.jpg. No grasp claim.
- Two empty_joint_waypoint functions passed after41-sample installed-finger FK/table/top-height checks and nativeCAD/workspace preflight. Reports output/rehearsal/r4-transition-01.json and r4-transition-02.json; each1 joint move. All blocks remain untouched. Known-open path only; not contact/payload qualification.
- Exact downward yaw0 failed read-only at newly measured orange. Yaw -3,-6,-10degree downward candidates solved from live measured seed in output/rehearsal/r4-ik-05.json. Choose scene-relative targets and actual seeds; do not replay joint targets or oldabsolute205/265mm.
- Ordinary detector refinements: near-white-logo sparse depth requires nearby REAL valid samples plus explicitly inferred center ray/top-plane intersection. Green table-edge can mimic compact block; use current measured tabletop-relative height. Partial red side can contaminate top-depth windows; select only visible colored windows with actual depth consistent with previous verified topheight. Missing evidence returns needs_review with zero dispatch.

## R4 repair candidate — offline only
Task judgments use broad alignment/support and obvious slip/displacement, not hidden millimetre clearance or fully separated color components. Expected occlusion alone is not failure. Original same-approach XYZ is reused with source/calibration/control continuity; movement, magnetic contact, owner restart or contradictory observations invalidate it. Wrist FFS remains camera-local without hand-eye calibration. Read-only endpoint recovery preserves the original action receipt. R4 did not trial-lift; new trial logic is unqualified.

## R7 startup evidence
- Fresh separated approximately60mm blocks, near-zero empty gripper: open_empty_gripper to69mm passed (68.7mm feedback),26.812s,8 receipts. Fresh scene and stationary sweep checks; frame e011711df0a34e87b6566337364413f8. No grasp claim.
- First empty transition passed17.751s with sampled native finger envelope above table and blocks. Target chosen from fresh orange-relative IK, intermediate joint fraction to stay within60deg; no historic joints reused. output/rehearsal/r7-transition-01.json.

## R7 orange base placement (one supervised function chain)
- Closure50mm at measured top+128mm, feedback50.9mm;10mm trial actual9.27mm and wrist correlation.977. Raising85mm/transfer passed.
- Nominal60mm center spacing with near-down yaw-3deg produced0.6mm finger/red-envelope overlap before final descent. Rose via prechecked partial-placement retreat, widened to65mm at height, then lowered to same-run table/grip relation. Final pair visibly adjacent.
- At final lowering, red entered the fixed wrist retention template (correlation.037), although orange remained upright at support. Use broad support check at that boundary, not unchanged flying-block template. Read-only support recovery passed; no motion replay.
- Qualitative support/release/retreat passed without onsite assistance. Release68.6mm, rise90mm; frame07d0a1bd4a6a44f28a5c927d84ff64d0. Three auxiliary calls, source reports r7-support-verify/r7-release-01/r7-retreat-01. Not unattended fulltask success.

## R7 software closeout — offline refinements, not new physical outcomes

| Task phase | Conditions | Refined rule | Evidence/qualification |
|---|---|---|---|
| Sparse head depth | Exact saved RGB/depth; color component identifiable but default windows empty | Choose real supported interior windows. If center depth is absent, a flat, spread measured top patch may supply height for an explicitly inferred center ray; never label it direct center depth | Failed frame ed11dd3c4e224bda89363ae91324d679; 15 new offline regressions. Repaired native query/pregrasp not physically run |
| Retained green | Same owner/calibration, original untouched green; fresh base measurements | Retain original report hash/frame/XYZ, reject altered provenance and explicit displacement. Current wrist color is identity evidence only | `green_checks.py`, `validate_stationary_sources`; full unseen action history is not proven |
| Completed placement | Receipts confirm final lowering; old wrist patch now includes red | Broad support assessment and read-only recovery; never replay completed lowering or overwrite its failure | Local learned skill `skills/magnetic-block-placement-boundary/`; R7 observed base-pair effect only |

Historical R3 whole-task success and R4/R6 limitations above remain unchanged.
At the R7 closeout, software was candidate_offline_checked and green stack functions
were unimplemented. That historical closeout had no unattended full-task test or publication.

## R8 live qualification so far
- Fresh scene: near-zero empty opening to69mm passed, feedback68.6mm. First empty transition bug (`False and previous_report is None`) corrected to request initial depth. Three checked transitions passed; direct downward top+235mm lacked IK, top+215mm solved from measured intermediate seed.
- Orange grasp at measured top+128mm,50mm closure ->50.9mm; trial10mm ->9.39mm with stable wrist body/pads.85mm raise and current-scene transfer passed.
- Repaired base planner selected65mm with4.19mm minimum nominal finger/red separation; two lowering segments passed including new qualitative support boundary (one Gemini call). This is R8 live evidence, still a supervised function chain. Reports `output/rehearsal/r8-*.json`.

## R8 green boundary — unresolved, do not promote to success
- Repaired green_pregrasp passed at exact same-run green XY, top+195mm, yaw near-90°. The top+215mm candidate was rejected by pure IK without motion.
- Starting a new green approach requires resetting the active empty history after orange release/retreat. Preserve the old chain only as provenance. A stale-chain attempt refused before motion; explicit new-green root passed.
- Green top+128mm insertion,50mm closure (51mm feedback),9.25mm trial and staged loaded transport passed observed retention checks. Current-scene base remeasurement found red~6mm from initial estimate, so original red coordinates were not used for the stack target.
- First green stack lowering reached numeric target and one model assessment said supported; next release assessment said significant forward offset. Release made zero motions. Same images/geometry are NOT sufficient evidence to override that discrepancy. Operator question ff43deacefc7 had no recorded answer at the physical stop. This is historical evidence, not authority to resume hardware.
- Sparse head samples near an occluded colored silhouette can return background. Two/four valid samples and widely inconsistent green-height estimates were discarded. Wrist FFS is optical only; a planar-patch diagnostic is not support proof or a base target.

## R8 software closeout — no new physical results

See `skills/magnetic-block-stack-support-boundary/` for the refined refusal/provenance lesson. Stack functions now exist but positive green release and retreat remain physically untested. Support-source and retained-history hash checks were repaired offline; 79 guarded tests pass, including 16 new closeout regressions. No live Context, robot/Gemini calls, new observations or skill-library publication occurred. Historical R3 success does not qualify the R8 functions.
