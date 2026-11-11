"""Trusted one-click startup. No SDK, passwords, or unreviewed motion."""
import json
import shutil
import subprocess
import time
import uuid
from pathlib import Path

from .common import ROOT, DATA, GUARD, read, write

BROKER = 'junqi-rax-clock-broker.service'
QUEUE = DATA / 'private/clock-broker'
UPPER_TASK = '''把蓝色杯子直立放到木质杯架的上层架面，确认整个杯底受支撑后松爪退开。不得改放下层。原中转站gpt-6-astra high，原生自动压缩100000。
这是可编辑的RAX探索，不是冻结测试。code/cup_to_coaster只是候选起点；固定俯抓朝向、抓取侧、19/4/40配方和阶段划分属于可改的经验模板，不是硬件限位。保持真实硬件限制、唯一owner/MCP、有效原始深度、执行结果与新鲜视觉检查。可按当前场景开发不同抓取策略，不能把少量旧模板IK失败当成全局不可达。
目标配置是code/cup_to_coaster/task-target.json；兼容键coaster指上层架面。初始SAM3只识别杯子与上层架面一次，之后看新头/腕原图。已有19mm开爪、4mm闭合、40mm试提经验可复用；上层约比桌面高132mm仅为历史参考，需新测量，40mm不是搬运净空。
首次启动按START_HERE.md工作。先用shelf_preflight.py从当前原始点云测量桌面/上层，并批量验证候选抓取至上层的关键高度。旧杯沿高位俯抓历史上无法到达上层；仅改变水平朝向无效，需根据当前几何探索抓取高度/姿态/位置，不得降低物理安全门槛。Prepare位空爪打开可选择clear_space_sequence，一次规划由代码执行现有小步并逐步核对反馈，接触和搬运仍逐步看图。压缩后直接使用自动注入的当前回执与下一步，RESUME.md仅作缺失时的恢复入口。把批量只读几何/IK结果写output/preflight-summary.json并只输出摘要，避免重复打印快照或逐点模型往返。不要无理由重复读取技能和整份源码。上层落点、朝向与路线可探索；杯子底部遮挡时结合前后头腕配对与实测抬升判断，不仅因遮挡询问人工。
完整成功必须验证上层支撑、释放和退开并生成真实round-result.json。成功后结束代理交给冻结验收。混合源码版本、未知执行、缺失最终图像或任务未完成均不能冒充成功冻结。'''


def remaining_guard_seconds():
    result = subprocess.run(['systemctl', 'show', GUARD, '-p', 'ActiveState',
                             '-p', 'ActiveEnterTimestampMonotonic'],
                            capture_output=True, text=True, timeout=4)
    fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    if fields.get('ActiveState') != 'active':
        return 0.
    entered = int(fields.get('ActiveEnterTimestampMonotonic', '0')) / 1e6
    # The guard Python loop ends at3500s, earlier than systemd's3600s cap.
    return max(0., 3500 - (time.monotonic() - entered)) if entered else 0.


def require_guard_budget(minutes):
    remaining = remaining_guard_seconds()
    if remaining < minutes * 60 + 120:
        raise ValueError('CLOCK_BUDGET: 时钟保护剩余时间不足；请使用一键探索续开保护')
    return remaining


def renew_guard(core, epoch):
    ident = uuid.uuid4().hex
    QUEUE.mkdir(parents=True, exist_ok=True, mode=0o700)
    write(QUEUE / (ident + '.request.json'), {'id': ident, 'action': 'renew', 'created': time.time()})
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        core._check_epoch(epoch)
        reply = read(QUEUE / (ident + '.response.json'))
        if reply:
            if reply.get('id') != ident or not reply.get('ok'):
                raise ValueError('CLOCK_RENEW: ' + str(reply.get('error', 'invalid response')))
            return require_guard_budget(45)
        time.sleep(.1)
    raise ValueError('CLOCK_AUTH: 一键时钟授权不可用；请按页面终端命令重新认证')


def check_visibility(core):
    """Only rejects unusable darkness; brightness is never scene approval."""
    import cv2
    import numpy as np
    evidence = {}
    for camera in ('head', 'left'):
        image = cv2.imdecode(np.frombuffer(core.image(camera), np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('VISIBILITY: 图像解码失败')
        p90 = float(np.percentile(image, 90))
        evidence[camera] = {'p90': p90, 'mean': float(image.mean())}
        if p90 < 30 or image.mean() < 12:
            raise ValueError('VISIBILITY: 现场过暗；请恢复照明后再启动，不会下发Prepare')
    return evidence


def validate_start(body):
    if body.get('scene_clear') is not True or body.get('empty_jaw_confirmed') is not True:
        raise ValueError('请确认空夹爪、人员退出、回程清空、急停可达')
    if body.get('lighting_ready') is not True:
        raise ValueError('请确认桌面与杯架照明已恢复')
    if type(body.get('minutes', 45)) is not int or not 1 <= body.get('minutes', 45) <= 45:
        raise ValueError('任务时限必须为1–45分钟')


def run(core, epoch, body):
    validate_start(body)
    if core.agent and core.agent.poll() is None:
        raise ValueError('已有探索在运行；不能重复启动')
    if shutil.disk_usage(ROOT).free < 6 * 1024**3:
        raise ValueError('DISK_RESERVE: 新探索至少需要6GiB空闲空间；先离线整理，不能边运行边压缩')
    core._check_epoch(epoch)
    core._disconnect(epoch)
    renew_guard(core, epoch)
    core._check_epoch(epoch)
    core._connect(epoch)
    core._fresh()
    visibility = check_visibility(core)
    write(core.run/'START-VISIBILITY.json', visibility)
    # Qualified operator reset does Prepare and bounded empty-jaw closure via MCP.
    from .operator_reset import run as reset
    reset(core, epoch, {**body, 'operator_text':'一键启动：现场确认后的Prepare与空爪闭合'}, close_empty=True)
    core._check_epoch(epoch)
    # Preserve reset evidence in its own session. Existing start creates a fresh
    # owner/session; never inherit operator-reset TASK.md or stale observations.
    task = str(body.get('task') or UPPER_TASK).strip()
    return core._start_task(epoch, {**body, 'task': task, 'minutes':body.get('minutes',45),
                                   'auto_freeze':True})


def maybe_freeze(core):
    if not core.task or not core.task.get('auto_freeze') or core.task.get('auto_freeze_checked'):
        return
    result=read(core.live/'session/agent-workspace/output/round-result.json',{})
    core.task['auto_freeze_checked']=True
    if core.agent.returncode!=0 or result.get('task_success') is not True:
        core.task['auto_freeze_status']='未完整成功，不冻结'
        write(core.run/'TASK-RECORD.json',core.task)
        return
    core.task['auto_freeze_status']='正在核验完整执行证据'
    write(core.run/'TASK-RECORD.json',core.task)
    def finish(epoch):
        try:
            name='蓝色杯子 → 桌面杯垫' if core.task.get('task_target')=='table_coaster' else '蓝色杯子 → 木质杯架上层'
            result=core._freeze(epoch,{'directory':'cup_to_coaster','name':name})
            core.task['auto_freeze_status']='完整证据验收通过，已冻结；待跨摆位验证'
            core.task['frozen_program']=result['id']
            return result
        except Exception as exc:
            core.task['auto_freeze_status']='冻结未通过：'+str(exc)[:300]
            raise
        finally:
            write(core.run/'TASK-RECORD.json',core.task)
    core._start_job('auto-freeze',finish)
