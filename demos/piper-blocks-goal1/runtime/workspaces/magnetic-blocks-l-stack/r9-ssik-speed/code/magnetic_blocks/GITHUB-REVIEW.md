# C 版的 GitHub 对照与取舍（2026-11-13）

目标是扩大完整任务的可达摆位、缩短预检，并保留已经验收的执行接口。下列仓库用于核对方法，并非整体替换现有 RAX/Piper 系统。

| 来源 | 核对到的方法 | 本版处理 |
|---|---|---|
| [MoveIt Task Constructor MultiPlanner](https://github.com/moveit/moveit_task_constructor/blob/ros2/core/src/solvers/multi_planner.cpp) | 在剩余时间预算内依次尝试规划器，得到成功结果即返回 | 保留 KDL 5 ms / 100 ms 分级；C 版为布局/顺序分配有限搜索时间，保留已验证 IK/FK/路径结果，不让第一个困难布局占满预算 |
| [MTC ComputeIK](https://github.com/moveit/moveit2_tutorials/blob/main/doc/concepts/moveit_task_constructor/wrappers.rst) | 位姿生成与 IK 分开，限制候选数量，可设解之间的最小距离，默认保留碰撞验证 | 本版扩展夹持朝向，先做源位置、目标承载高度、放置/张爪与后续起抓筛选，再构造完整连接。没有声称已实现 MTC 的多解图搜索 |
| [MoveIt Cached IK](https://github.com/moveit/moveit2/blob/main/moveit_kinematics/cached_ik_kinematics_plugin/include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin-inl.hpp) | 以近邻已知解作为初值再次求解，失败可回到原 seed | 继续保留现有精确请求缓存与前一路点 seed。近邻 pose 的解不能当作当前目标解直接执行。本次未额外引入近邻 seed 库；实测当前主要成本是候选路径/FCL，不是单次 IK |
| [pick_ik 使用说明](https://github.com/PickNikRobotics/pick_ik/blob/main/doc/USAGE.md) | local/global 搜索及目标函数；局部连续运动可以给位移成本，远目标不宜给过大惩罚 | 作为后续替代求解器对照项，本次保留 KDL，未安装并冒充已经比较 pick_ik 的性能 |
| [Pilz motion sequence](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#sequence-of-multiple-segments) | 把相邻运动组成序列，可在非零 blend radius 下通过中间点；整个序列先规划 | C 版继续使用现有原生多路点接口和经过检查的混合曲线；大批次超过 600 点时先验证较小的批次，保留批内速度衔接，不再默认退成 5–6 个单段。没有复制 Pilz 示例里的半径或速度到本机 |

本版新增的笛卡尔包络评估仅为候选排序启发式，完全不替代完整关节路径和 FCL 检查。无障碍的直接路径仍先尝试。发现便宜估算与真实路径冲突时，以真实路径检查为准。

IK 的数值初值不等于机器人起点，也不等于已验证的连续路径。预检计算时间、批次准备/下发开销、实际运动时间必须分开记录。离线可验证候选和曲线；真实动作间的等待仍需新版本真机回执确认，不能据离线预览宣称完全无间隔。
