# 磁性积木控制器 v0.1 — 探索后逐函数候选

策略来源：R3成功手动探索；最终一次现场确认。代码在探索结束后生成，离线测试通过，**未真机逐函数演练，未合并为自主main(ctx)**。不能把函数代码冻结与真机自动化验收混为一谈。

接口：`locate_block(ctx, observation, interior_pixels)`、`open_empty_gripper(ctx, assess=...)`、`approach(ctx, waypoints=..., quaternion=..., assess=...)`、`grasp_and_trial_lift(ctx, quaternion=..., assess=...)`、`transport(ctx, waypoints=..., quaternion=..., assess=...)`、`place_release_retreat(ctx, descent=..., retreat=..., quaternion=..., assess=...)`。

按RAX autonomous-control，在可写副本中加载模块，传入同一个`api.runtime.Context`，一次只演练一个函数，查看`{ok, measured, why, receipts, facts, elapsed_s}`。模块没有启动即运行或自动创建Context的行为。

`assess`是显式传入的场景判断函数，接收phase/before/after/inputs，返回`{ok,frame_id,why}`。逐函数演练由主Agent根据这组实际图像/深度给出判断；以后可替换为普通代码规则，必要时辅助视觉。不得复制测试中恒真的合成fixture来接真机。此接口尚未自动化语义定位/支撑判断，不能宣称无需主Agent整段运行。普通反馈能判断的到位/持稳由原生工具处理，没有在每个小动作固定调用Gemini。

新摆位必须选取当前图像中的目标像素，用同一帧的有效深度建立基座目标；磁吸后重新测红块。路径waypoints/quaternion来自这次定位与IK，不包含历史绝对坐标默认值。始终保留本机工具检查、10mm开合步长与异常hold。IK拒绝不盲重试；试抬未夹住不自动张爪；未知回执不继续下一动作。

参数：成功闭合指令50mm，实际反馈约51mm；块任务试抬10mm；通常张开69mm，初次73mm。本轮205mm抓取/265mm放置是参考证据，不是通用目标。末端退至300mm成功，330mm失败；本版不自动执行该失败退开。

离线验证：`python3 -m unittest discover -s code/magnetic_blocks -p 'test_*.py' -v`。覆盖目标50mm确实下发、每段开合、失败试抬不松爪、未知回执停止、旧帧拒绝、支撑不明不释放、边界拒绝及正常释放退开。测试不调用硬件或模型。

下一阶段：先在此可写副本完成函数演练和感知规则，然后才组合main(ctx)并新摆位测试。冻结版本不修改。
