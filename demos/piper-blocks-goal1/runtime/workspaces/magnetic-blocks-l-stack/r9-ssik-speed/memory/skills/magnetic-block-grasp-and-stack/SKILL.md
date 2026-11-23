---
name: learned-magnetic-block-grasp-and-stack
description: Approximately 60 mm magnetic cubes; supervised front/back base joining and upper-block stacking, with trial retention and re-localization.
---

# 本轮抓取与搭建策略

- Applies when: 同类约60mm直立磁性块、当前软垫，当前新图像和深度可确定待夹两面与支撑关系。
- Avoid when: 尺寸/指垫/桌面变更、碰撞或IK未通过、目标被吸动但未重定位、夹持尚未证实。
- Executed action: R3使用Piper原生工具；成功闭合目标0.050m，反馈约0.051m；原生开合上限按每次实测值分步。
- Pose recipe: 本轮抓取link6 z=.205m，桌面约.018m，顶面约.078–.081m；这些是本轮证据，不是通用标定。绿色放置比其抓取法兰高度增加一个实测底块高度，R3为.265m。
- Actions: 新观测定位→空载可达接近→充分侧面包覆→50mm闭合→10mm试抬验证→抬高到负载越过邻块→底层横向接合→松爪→重测被磁吸挪动的底座→绿色抓取/搬运/支撑/释放→退开核验。
- Expected effect: 物体随夹爪抬起，底块侧面对齐，绿色位于红色上方；用当前观测比较，夹爪到位不等于物体到位。
- Recovery: 本轮两次浅夹未抓起，重新张爪并在桌面支撑条件下加深包覆后成功。IK失败仅比较未执行候选、调整种子/空载朝向；不放宽检查，不重放不确定动作。
- Confirmation: observed for local effects; whole task supervised + onsite-confirmed. NOT camera-only task-confirmed, NOT autonomous, NOT cross-layout verified.
- Code qualification update (R7 closeout): `../../../code/magnetic_blocks/controller.py` now has supervised orange base-pair function evidence. R7 green pregrasp never dispatched, and green stacking is unimplemented. The original R3 supervised result above is unchanged; see `../magnetic-block-placement-boundary/` for R7 evidence. Latest software repairs have offline checks only.
