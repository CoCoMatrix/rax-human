# Piper 反馈和时钟排查（2026-11-05）

三层同步证据保存在5090的 `commissioning/feedback-clock-20261105-r3-readonly` 与
`feedback-clock-20261105-r5-hold-fixed-time`。原始SocketCAN只接收，诊断入口在现有owner
进程记录公开SDK返回值与owner样本，HTTP读取发布结果；全部带单调时钟与墙上时钟。
没有启用第二个机器人writer，没有修改厂商SDK。

## 已修复的适配层时间竞态

原owner调用 `snapshot(now_s=time.time())`，先取时间再读取两个异步SDK缓存；读取期间
到达的新帧可能比先取的时间新，被误报future。现在live owner调用 `snapshot()`，
在读取完成后取当前时间。显式now_s保留给离线回放。未来帧、100ms陈旧限制、关节及
夹爪到位容差、状态一致性阈值全部保留。

旧代码的正常新帧测试明确失败；修改后4个针对性回归通过。RAX全套450通过，原有
yourdfpy弃用警告60条；最终相关46项通过。补丁在
`verification/feedback-clock-20261105/owner-time.patch`，原文件备份位于同目录owner-before。

## 未修复的原始关节反馈

持位对照92.542s中，J5的CAN0x2A7非零包18449个（−0.335°），零值包14个。
SDK和owner各读到8次零值，HTTP读到6次。归零已经存在于原始CAN，不能归咎于RAX
JSON转换或视觉等待。多种持位状态均出现。本轮J1稳定，不等于此前J1问题已解决。
首个SDK零值附近，电机0x255位置字段为raw−6/−5，没有替代成关节反馈。
此前J4/3mm失败仍未验收，不用过滤零值、上次值或目标值绕过。
未找到Piper X S-V1.9-0的对应官方修复，不能从其他型号反馈讨论直接推导固件升级方案。

## 系统校时

原systemd-timesyncd经公网NTP产生大抖动。仅更换时间源曾短时改善，但150s延长观察后
再次出现约0.9s jitter与3.77s网络时延，故该措施未当作最终修复。
随后安装Ubuntu包chrony4.2，使用多个源、拒绝高延迟样本、maxslewrate500ppm，最终配置
**不含makestep**，包括首次同步也不自动跳时。RAX原有时钟跳变/时间戳保护未关闭。
配置变更时机器人owner已退出。新服务的UTC同步收敛须以chronyc tracking为准；配置
生效不等于已获得高精度UTC同步，不应声称网络时间精度问题已完全消除。

## 最新用户指令与运行边界

用户看到未解决项后明确改为“先回到prepare，进行探索”，并要求在当前对话直接启动。
XPolicy原生Prepare已执行一次：15:50:45请求，15:50:48日志prepare_complete；最终
连续8样本双臂关节零、左爪0.4mm、右爪零。首次读取的prepared标签早于实测到位，
未因此继续派发，也未重复Prepare。回执见xpolicy-prepare-before-fresh-20261105-r6。

fresh探索配置不传rehearsal-candidate或continue-from，不加载旧杯子控制器。历史经验
只作有条件参考。所有真实反馈异常/运动拒绝仍停止，不能改容差或回放失败动作。
r6-fresh入口确认code为空、fixed_task_controller_loaded=false，但首个主模型API断开，
没有任务运动；自动Stop8样本静止并关闭。随后真实API只读复核成功约5秒，准备重新启动。

冻结v0.1（60文件）、v0.3（109文件）SHA256复核全部不变。
