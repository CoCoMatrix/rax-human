# 本轮问题与结论

1. 主模型原上游SSE失败：独立直连复现；本地两层代理确有丢弃会话头缺陷，已修复，不能证明它是上游失败唯一原因。直连/XQ对照选择主模型直连；旧17890代理TLS EOF不再使用。
2. 夹持深度：220/56、215/53组合试抬未带起；205/50成功。指令开度不能单独判断实物夹持。
3. 磁吸影响：红块在底层拼接期间挪动，必须重新测量；不能搬运绿色时沿用最初红块坐标。
4. IK：朝向与种子影响显著；空载分步旋转找到可行接近。最终330/320/315mm退开不可解，300mm已到达且现场确认离开。下一演练须提前验证整条退出路线。
5. 时间：多个模型回合用于小步开爪/搬运、26+次IK比较、上下文压缩。整理为有界函数后可减少往返，但尚未测得提速。
6. 人工交接：1秒请求已过期，但原生late reply成功接收；最后现场事实被Agent消费。记录人工确认，不伪装全自主成功。
7. 收尾预算：120次主模型调用后本地429导致agent_exit_code=1，已完成物理任务和技能库合并。冻结保留原始退出结果，不篡改passed字段。
8. 环境：CAD路径检查没有完整固定环境/负载模型；新函数保留当前路径事实检查。头部标定暂定、腕部无手眼，未在本次升级标定或碰撞能力。

演练启动离线预检发现原evidence.json列出帧目录，原生导入器要求具体文件；可写副本已改为head.jpg并补可导入完整ledger，冻结版未改。

R1流程离线修复：普通函数即时返回；去掉批准文件循环；分开闭合/试抬、下降/释放/退开；JSON输入与回调隔离；真实工具schema测试；自动恢复摘要。23项离线检查通过。见references/rehearsal-r1/REPAIR_REPORT.md；实际后续演练未进行。

## R7 closeout scope (2026-11-11)
Earlier numbered items are historical R3/R1 outcomes, including their assistance and
library-merge statements; they do not describe this closeout. R7 base pair completed,
green pregrasp failed twice before motion, and green stacking remains unimplemented.
Sparse-depth/center-hole and provenance candidates now have offline regressions.
No hardware validation, operator assistance or library merge occurred in software
closeout. See output/closeout/SUMMARY.json for current limits and source identities.
