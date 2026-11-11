"""Trusted operator orchestration. All actuation goes through the formal owner."""
import ast
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

from .common import ROOT, DATA, PYTHON, AGENT_PYTHON, QUALIFICATION, GUARD, HARDWARE_UNIT, BASELINE, read, write, source_manifest
from .vision import capture_bundle, CAMERAS
from services import arming
from services.controller import operator
from services.controller.config import Configuration
from services.dashboard.sessions import tail_lines
from services.piper.owner import OwnerReader
from services.piper.calibration_web import Owner
from services.piper.depth import DepthReader
from services.piper.backend import CALIBRATION_SHA
from services.piper.motion_bridge import MotionClient
from services.piper.qualification import load, sha

sys.path.insert(0, str(ROOT / 'scripts'))
from piper_ready import check_clock_guard
from piper_motion_acceptance import wait_prepare

SUPPORT = Path(__file__).parent / 'agent_support'
from .workflow_policy import TASK_POLICY, policy_for, contract as workflow_contract



class WorkbenchError(RuntimeError):
    def __init__(self, code, message, fix=''):
        super().__init__(message)
        self.code, self.fix = code, fix


def system_state(unit, user=False):
    cmd = ['systemctl'] + (['--user'] if user else []) + ['is-active', unit]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=4).stdout.strip()


def agent_messages(session):
    logs = sorted(Path(session).glob('codex-runs/*/agent.jsonl'), key=lambda p: p.stat().st_mtime_ns)
    result = []
    for file in logs[-2:]:
        for line in tail_lines(file, 300):
            try:
                event = json.loads(line)
            except ValueError:
                continue
            item = event.get('item') or {}
            if event.get('type') == 'item.completed' and item.get('type') == 'agent_message':
                result.append({'kind': 'assistant', 'text': str(item.get('text', ''))[:14000]})
            elif event.get('type') in ('error', 'turn.failed'):
                result.append({'kind': 'error', 'text': str(event.get('message') or event.get('error'))[:1200]})
            elif event.get('type') == 'item.completed' and item.get('type') == 'mcp_tool_call':
                result.append({'kind': 'tool', 'text': str(item.get('tool') or item.get('name') or '工具完成')})
    return result[-60:]


class Workbench:
    def __init__(self, data=DATA, *, start_monitor=True):
        self.data = Path(data); self.data.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.operations = threading.Lock()
        self.result_lock = threading.Lock()
        self.epoch = uuid.uuid4().hex
        self.seen = set()
        self.run = None
        self.agent = None
        self.phase = 'disconnected'
        self._clock_broker_active = False
        self.error = None
        self.job = None
        self.robot = None
        self.vision = None
        self.segmentation = None
        self.image_cache = {}
        self.events = read(self.data / 'events.json', [])[-60:]
        self.history = read(self.data / 'trials.json', [])
        self.batch = read(self.data / 'batch.json')
        if self.batch and self.batch.get('state') in ('running', 'awaiting_reset', 'awaiting_result'):
            self.batch['state'] = 'interrupted'
            self.batch['reason'] = '工作台重新启动；需要操作员重新选择，未自动续跑'
            write(self.data / 'batch.json', self.batch)
        self.task = None
        self.selected = 'baseline-5eaccef'
        self.stop_event = threading.Event()
        self._last_sensor_ok = 0.0
        self._health_lock = threading.Lock()
        self.health = {}
        self.robot_error = ''
        self.vision_error = ''
        self._last_guard_check = 0.0
        self._guard_ok = False
        self._last_depth_ok = 0
        self._last_bundle_saved = 0
        self._manifest_check_at = 0
        self.registry = read(self.data / 'programs.json', {})
        if self.selected not in self.registry:
            self.registry[self.selected] = {'id': self.selected, 'name': '杯子 → 杯垫 · 5eaccef',
                'version': '5eaccef561bb2e89a6768a8e6b3f601fb488fdef', 'path': str(BASELINE),
                'manifest': source_manifest(BASELINE), 'mode': 'reviewed_steps', 'batch_ready': False,
                'status': '受监督基线', 'reason': '逐步 action 接口；尚无完整任务入口和固定版连续成功', 'verified_successes': 0}
            write(self.data / 'programs.json', self.registry)
        self._qualification = None
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        if start_monitor: self.monitor.start()

    def event(self, kind, message, **fields):
        row = {'at': time.time(), 'kind': kind, 'message': message, **fields}
        with self.lock:
            self.events.append(row); self.events = self.events[-100:]
            write(self.data / 'events.json', self.events)
            with (self.data / 'audit.jsonl').open('a') as f: f.write(json.dumps(row, ensure_ascii=False) + '\n')
        return row

    def _check_epoch(self, epoch):
        if epoch != self.epoch or self.stop_event.is_set():
            raise WorkbenchError('CANCELLED', '操作已被 Hold 或连接切换取消', '检查实时状态后重新操作')

    @property
    def live(self):
        if self.run is None: raise WorkbenchError('DISCONNECTED', '尚未连接机械臂', '点击「连接 RAX」')
        return self.run / 'live'

    def owner(self):
        return Owner((self.live / 'private/owner-token').read_text().strip())

    def _fresh(self, *, vision=True):
        check_clock_guard(GUARD)
        s = OwnerReader().state()
        health = s['robot_status'].get('component_health', {})
        if not health.get('left_arm', {}).get('ok') or not health.get('left_gripper', {}).get('ok'):
            raise WorkbenchError('HARDWARE_HEALTH', '左臂或左夹爪反馈异常', '先 Hold，再检查机械臂连接')
        if not s['robot_status'].get('hardware_write_enabled'):
            raise WorkbenchError('READ_ONLY', '当前 owner 为只读模式', '通过工作台重新连接')
        DepthReader(self.live / 'depth', CALIBRATION_SHA).read()
        if vision:
            v = self.vision
            if not v or (time.time_ns() - v['oldest_source_ns']) / 1e9 > 2.0:
                raise WorkbenchError('VISION_STALE', '三视角观察尚未就绪或已过期', '等待新鲜图像；持续异常时重连相机与机械臂')
        return s

    def _start_job(self, name, function):
        if not self.operations.acquire(blocking=False):
            raise WorkbenchError('BUSY', '上一项操作尚未结束', '等待完成；Hold 始终可用')
        epoch = self.epoch
        ident = uuid.uuid4().hex
        self.job = {'id': ident, 'name': name, 'status': 'running', 'started': time.time()}
        def run():
            try:
                value = function(epoch)
                self._check_epoch(epoch)
                self.job.update(status='done', result=value, finished=time.time())
            except Exception as exc:
                self.job.update(status='cancelled' if epoch != self.epoch else 'failed', finished=time.time())
                if name in ('prepare', 'start', 'next-round', 'resume-start', 'reset-open', 'reset-close', 'quick-start') and self.run:
                    try: arming.write(self.live / 'session', False, reason='workbench action failed')
                    except Exception: pass
                    try: self.owner().command('stop')
                    except Exception: pass
                    try: self._stop_agent()
                    except Exception: pass
                    try: self._interrupt_trial('任务入口失败：' + str(exc))
                    except Exception: pass
                if epoch == self.epoch:
                    self.error = {'code': getattr(exc, 'code', 'OPERATION_FAILED'), 'message': str(exc),
                                  'fix': getattr(exc, 'fix', '查看事件记录；修复后重试此步骤'), 'at': time.time()}
                    if name in ('connect','reconnect','disconnect','prepare','start','next-round','resume-connect','resume-start') and self.phase not in ('held', 'awaiting_result'):
                        self.phase = 'error'
                    self.event('error', str(exc), code=self.error['code'])
            finally:
                self.operations.release()
        threading.Thread(target=run, daemon=True).start()
        return {'accepted': True, 'job_id': ident}

    def command(self, action, body):
        if action == 'hold': return self.hold('操作员点击 Hold')
        rid = body.get('request_id')
        if not isinstance(rid, str) or not re.fullmatch(r'[a-zA-Z0-9-]{16,80}', rid):
            raise WorkbenchError('REQUEST_ID', '请求标识无效', '刷新页面')
        with self.lock:
            if body.get('epoch') != self.epoch: raise WorkbenchError('STALE_PAGE', '页面连接已过期', '刷新后重新点击')
            if rid in self.seen: raise WorkbenchError('DUPLICATE', '此请求已处理，不会重复执行')
            self.seen.add(rid)
            if len(self.seen) > 10000: self.seen = {rid}
        if action == 'quick-start':
            from .quick_start import run, validate_start
            validate_start(body)
            return self._start_job(action, lambda e: run(self,e,body))
        if action == 'connect': return self._start_job(action, self._connect)
        if action == 'reconnect': return self._start_job(action, lambda e: self._reconnect(e))
        if action == 'disconnect': return self._start_job(action, self._disconnect)
        if action == 'resume-connect':
            self._confirmed_resume(body)
            return self._start_job(action, lambda e: self._reconnect(e, prepare_mode='hold-current'))
        if action == 'resume-start':
            self._confirmed_resume(body)
            return self._start_job(action, lambda e: self._resume_start(e, body))
        if action == 'reset-close':
            self._confirmed(body)
            from .operator_reset import run
            return self._start_job(action, lambda e: run(self,e,body,close_empty=True))
        if action == 'reset-open':
            self._confirmed_resume(body)
            from .operator_reset import run
            return self._start_job(action, lambda e: run(self,e,body))
        if action == 'prepare':
            self._confirmed(body)
            return self._start_job(action, self._prepare)
        if action == 'start':
            self._confirmed(body)
            return self._start_job(action, lambda e: self._start_task(e, body))
        if action == 'reply':
            session=self.live/'session'
            answer=operator.reply(session,str(body.get('id')),str(body.get('text','')),done=body.get('done') is True)
            request=read(session/'operator/requests'/(str(body.get('id'))+'.json'),{})
            delivery=answer.get('delivery') or operator.publish_feedback(session,self.run.name,str(body.get('id')),answer)
            self.event('operator_reply', '现场答复已保存，等待代理检查点接收',
                       request_id=str(body.get('id')),late_reply=answer.get('late_reply',False),
                       delivery_id=delivery['delivery_id'])
            return answer
        if action == 'freeze': return self._start_job(action, lambda e: self._freeze(e, body))
        if action == 'segment': return self._start_job(action, lambda e: self._segment(e, body))
        if action == 'batch': return self._batch_start(body)
        if action == 'next-round':
            self._confirmed(body)
            return self._start_job(action, lambda e: self._next_round(e, body))
        if action == 'result':
            with self.result_lock: return self._record_result(body)
        raise WorkbenchError('UNKNOWN_ACTION', '未知操作')

    @staticmethod
    def _confirmed(body):
        if body.get('scene_clear') is not True:
            raise WorkbenchError('SCENE_CHECK', '请确认夹爪未悬持物体、人员退出、回程清空且急停可达', '在操作区勾选本次现场确认')

    @staticmethod
    def _confirmed_resume(body):
        if body.get('scene_clear') is not True or body.get('hold_current_confirmed') is not True:
            raise WorkbenchError('RESUME_SCENE', '恢复需确认当前夹持稳定、人员退出、路径清空且急停可达')

    def _resume_start(self, epoch, body):
        self._check_epoch(epoch)
        connected = read(self.run / 'CONNECTED.json', {}) if self.run else {}
        if connected.get('prepare_mode') != 'hold-current' or (self.run / 'TASK-RECORD.json').exists():
            raise WorkbenchError('RESUME_MODE', '必须先建立新的保持当前位置连接；不能复用已结束任务或归零模式')
        return self._start_task(epoch, body)

    def _connect(self, epoch, prepare_mode='home'):
        self._check_epoch(epoch)
        if prepare_mode not in ('home','hold-current'): raise WorkbenchError('PREPARE_MODE','准备模式无效')
        try: check_clock_guard(GUARD)
        except Exception:
            raise WorkbenchError('CLOCK_AUTH', '临时时钟保护尚未开启，需要操作者终端认证', '复制页面的 SSH 命令，在终端输入 sudo 密码；保护有效期 60 分钟') from None
        if system_state(HARDWARE_UNIT, user=True) in ('active', 'activating', 'deactivating'):
            if self.run and (self.run / 'CONNECTED.json').exists():
                self._fresh(vision=False); return {'already_connected': True}
            raise WorkbenchError('OWNER_BUSY', '已有硬件运行器占用设备', '先 Hold 并断开已有运行器，不能启动第二个 owner')
        self._qualification = load(QUALIFICATION, sha(QUALIFICATION))
        self._interrupt_trial('连接切换前未确认的任务，记为未完成')
        self.task = None
        self.phase = 'connecting'; self.error = None; self.robot = None; self.vision = None; self.image_cache = {}; self.segmentation = None
        self.run = self.data / 'runs' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6])
        self.run.mkdir(parents=True)
        write(self.data / 'current-run.json', {'run': str(self.run), 'started': time.time()})
        env = ['--setenv=RAX_ARTIFACT_ROOT=' + str(ROOT / 'commissioning'), '--setenv=PYTHONPATH=' + str(ROOT)]
        cmd = ['systemd-run', '--user', '--collect', '--unit=' + HARDWARE_UNIT,
               '--property=KillMode=mixed', '--property=TimeoutStopSec=90', '--working-directory=' + str(ROOT),
               *env, str(PYTHON), '-m', 'services.piper.workbench.runtime', '--run', str(self.run), '--prepare-mode', prepare_mode]
        with self.lock:
            self._check_epoch(epoch)
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if p.returncode: raise WorkbenchError('LAUNCH', p.stderr.strip(), '检查用户 systemd 和已有运行器')
        self.event('connect', '启动唯一硬件 owner、三路相机与 FFS', run=str(self.run))
        deadline = time.monotonic() + 170
        while not (self.run / 'CONNECTED.json').exists():
            self._check_epoch(epoch)
            error = read(self.run / 'ERROR.json')
            if error: raise WorkbenchError('CONNECTION_FAILED', error.get('error', str(error)), '检查连接日志，修复后重连')
            if time.monotonic() > deadline: raise WorkbenchError('CONNECT_TIMEOUT', '相机/机械臂连接超时', '查看 motion-launch.log')
            time.sleep(.3)
        deadline = time.monotonic() + 12
        while not self.vision:
            self._check_epoch(epoch)
            if time.monotonic() > deadline: raise WorkbenchError('CAMERA_START', '硬件已连接，三视角观察仍未就绪', '查看视觉错误后重连')
            time.sleep(.2)
        self._fresh(); self.phase = 'connected'; self._last_sensor_ok = time.monotonic()
        self.event('ready', '三路相机、FFS 与实测反馈通过；尚未下发 Prepare 或任务动作')
        return {'connected': True, 'run': str(self.run)}

    def _stop_agent(self):
        with self.lock:
            process = self.agent
        if process and process.poll() is None:
            # Only this workbench's launcher process group is signalled.
            os.killpg(process.pid, signal.SIGTERM)
        return process

    def hold(self, reason):
        with self.lock:
            self.epoch = uuid.uuid4().hex
            self.phase = 'held'
            if self.batch and self.batch.get('state') in ('running', 'awaiting_reset', 'awaiting_result'):
                self.batch.update(state='interrupted', reason=reason); write(self.data / 'batch.json', self.batch)
        errors = []
        if self.run and (self.live / 'session').exists():
            try: arming.write(self.live / 'session', False, reason=reason)
            except Exception as exc: errors.append('disarm: ' + str(exc))
            try: self.owner().command('stop')
            except Exception as exc: errors.append('owner stop: ' + str(exc))
        try: self._stop_agent()
        except Exception as exc: errors.append('agent stop: ' + str(exc))
        try: self._interrupt_trial(reason)
        except Exception as exc: errors.append('trial record: ' + str(exc))
        self.event('hold', reason, errors=errors)
        return {'stop_requested': True, 'errors': errors, 'measured_rest': False,
                'note': 'Hold 已发出；以实时稳定反馈为准。恢复须显式重新连接。'}

    def _disconnect(self, epoch):
        self._check_epoch(epoch)
        with self.lock:
            self.phase = "disconnecting"
        process = self._stop_agent()
        if self.run:
            try: arming.write(self.live / 'session', False, reason='workbench disconnect')
            except OSError: pass
            try: self.owner().command('stop')
            except Exception: pass
        subprocess.run(['systemctl', '--user', 'stop', HARDWARE_UNIT], capture_output=True, timeout=95)
        if process and process.poll() is None:
            try: process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=10)
        self._interrupt_trial('操作员断开或重新连接；本轮未确认完成')
        self.agent = None
        self._check_epoch(epoch)
        self.phase = 'disconnected'; self.robot = None; self.vision = None; self.image_cache = {}
        self.event('disconnect', '硬件运行器已停止')
        return {'disconnected': True}

    def _reconnect(self, epoch, prepare_mode='home'):
        self._disconnect(epoch)
        return self._connect(epoch, prepare_mode=prepare_mode)

    def _prepare(self, epoch):
        self._check_epoch(epoch)
        if self.agent and self.agent.poll() is None: raise WorkbenchError('AGENT_RUNNING', '任务执行中，请先 Hold', 'Hold 后重新连接，再执行 Prepare')
        before = self._fresh()
        prepare_mode = read(self.run / 'CONNECTED.json', {}).get('prepare_mode', 'home')
        if prepare_mode not in ('home','hold-current'): raise WorkbenchError('PREPARE_MODE','未知准备模式')
        bridge = MotionClient(self.live / 'bridge', self.live / 'session').status()
        if bridge['phase'] == 'FAULT': raise WorkbenchError('LATCHED_FAULT', '当前连接有停止/故障锁存', '显式重连后再 Prepare')
        self.phase = 'preparing'
        with self.lock:
            self._check_epoch(epoch)
            self.owner().command('prepare')
        self.event('prepare', '保持当前双臂与夹爪位置；不归零、不闭爪' if prepare_mode=='hold-current' else '左臂按xpolicy速度归零、夹爪保持当前开度；右侧保持实测位置', prepare_mode=prepare_mode)
        reader = OwnerReader()
        # Reuse the qualified completion condition, with cancellation checked on every read.
        core = self
        class CancellableReader:
            def state(self):
                core._check_epoch(epoch)
                check_clock_guard(GUARD)
                return reader.state()
        prepared = wait_prepare(CancellableReader())
        self._check_epoch(epoch)
        q = prepared['robot_state']['values']; old = before['robot_state']['values']
        valid = (max(abs(a-b) for a,b in zip(q,old)) <= .003 and abs(q[6]-old[6]) <= .001 and abs(q[13]-old[13]) <= .001) if prepare_mode=='hold-current' else (max(abs(x) for x in q[:6]) <= .003 and abs(q[6]-old[6]) <= .001 and max(abs(a-b) for a,b in zip(q[7:], old[7:])) <= .003)
        if not valid:
            self.owner().command('stop')
            raise WorkbenchError('PREPARE_MEASURED', 'Prepare 实测位置未通过', '检查实测反馈，停止后诊断')
        result = {'passed': True, 'prepare_mode': prepare_mode, 'jaw_policy': 'preserve_measured', 'measured14': q, 'right_max_change': max(abs(a-b) for a,b in zip(q[7:], old[7:])), 'at': time.time()}
        write(self.run / 'PREPARED.json', result)
        self.phase = 'prepared'; self.event('prepared', '当前位置保持实测通过' if prepare_mode=='hold-current' else 'Prepare 实测通过', measured14=q)
        return result

    def _configure(self, task, program=None, minutes=20, task_target=None):
        from harness.codex_cli.launcher import prepare_workspace
        qualifier = self._qualification or load(QUALIFICATION, sha(QUALIFICATION))
        mode = 'explore' if program is None else 'frozen_test'
        text = task + '\n\n' + policy_for(task_target)
        if program:
            if source_manifest(SUPPORT) != program.get('support_manifest'):
                raise WorkbenchError('SUPPORT_CHANGED', '该冻结版的视觉规则与当前支持库不同', '保留旧版本；将新规则连同程序重新冻结')
            if program.get('workflow_contract') != workflow_contract():
                raise WorkbenchError('WORKFLOW_CHANGED', '冻结工作流提示词或模型设置不一致', '使用匹配的工作流版本，或重新验收并冻结')
            if program.get('qualification_sha256') != sha(QUALIFICATION):
                raise WorkbenchError('QUALIFICATION_CHANGED', '该冻结版的运动与标定资格记录已变化', '核对变更后重新冻结并累计测试')
            text += ('\nFIXED-VERSION TEST: run the frozen program code/frozen_program. Do not edit its files, '
                     'change motion parameters, improvise replacement motions or hand-finish a failed run. '
                     'Keep only its declared agent perception checkpoints. Any corrective code/action is a failed fixed trial. '
                     f"Expected controller SHA256: {program['manifest']['sha256']}. Run {program.get('entrypoint','controller:main')}(ctx), "
                     'and report the actual complete outcome with saved frames.\n')
        config = Configuration(root=self.live / 'session', task='workbench_task', task_description=text,
            robot='piper', agent_python=str(AGENT_PYTHON), agent_provider='codex-api',
            agent_model=program.get('model') if program else None,
            agent_effort='high', agent_auto_compact_token_limit=100000, subagents=0, session_seconds=minutes*60,
            backend_options={'motion_bridge': str(self.live / 'bridge'), 'depth_root': str(self.live / 'depth'),
                'record_stream': True, 'camera_candidate': str(ROOT / 'commissioning/handeye-solve-20261028/camera-all13-candidate.json'),
                'camera_candidate_sha256': qualifier['camera_candidate_sha256'], 'provisional_calibration': True,
                'qualification': str(QUALIFICATION), 'qualification_sha256': sha(QUALIFICATION)})
        write(self.run / 'operator-input.json', config.to_dict())
        workspace = prepare_workspace(config)
        write(workspace/'.codex/rax-workflow.json', {'mode':mode, 'workflow_contract':workflow_contract(), 'run':self.run.name})
        shutil.copytree(SUPPORT, workspace / 'code/rax_support', dirs_exist_ok=True)
        self._copy_experience(workspace)
        from .startup_context import install
        install(workspace)
        if program:
            if source_manifest(program['path']) != program['manifest']: raise WorkbenchError('FROZEN_CHANGED', '冻结源文件校验失败')
            shutil.copytree(program['path'], workspace / 'code/frozen_program', ignore=shutil.ignore_patterns('.git', '__pycache__', '.pytest_cache'))
            if source_manifest(workspace / 'code/frozen_program') != program['manifest']: raise WorkbenchError('COPY_CHANGED', '程序复制校验失败')
        else:
            shutil.copytree(BASELINE, workspace / 'code/baseline_5eaccef', ignore=shutil.ignore_patterns('.git', '__pycache__'))
            maintained = self.data / 'experience/blue-cup-wall/repaired_controller'
            if maintained.is_dir():
                shutil.copytree(maintained, workspace / 'code/cup_to_coaster', ignore=shutil.ignore_patterns('.git','__pycache__'), dirs_exist_ok=True)
        if program is None and task_target == 'table_coaster':
            write(workspace/'code/cup_to_coaster/task-target.json', {
                'task':'blue-cup-to-table-coaster','object_prompt':'blue cup',
                'destination_prompt':'round coaster on the table','destination_label':'桌面圆形杯垫',
                'compatibility_key':'coaster','requires_full_base_support':True,'recipe_is_editable':True})
        if program is None and task_target == 'table_coaster':
            shutil.copy2(Path(__file__).with_name('controller_coaster_readme.md'),workspace/'code/cup_to_coaster/README.md')
        self.task = {'text': task, 'mode': mode, 'minutes': minutes, 'started': time.time(),
                     'program': None if program is None else program['id'], 'result_recorded': False, 'task_target':task_target}
        self.selected = 'baseline-5eaccef' if program is None else program['id']
        write(self.run / 'TASK-RECORD.json', self.task)
        return config

    def _copy_experience(self, workspace):
        # Operator-curated, evidence-attributed task history, never live state.
        source = self.data / 'experience'
        if source.is_dir():
            manifest = source_manifest(source)  # Reject links and oversized files.
            destination = workspace / 'references/workbench-experience'
            shutil.copytree(source, destination, dirs_exist_ok=True)
            if source_manifest(destination) != manifest:
                raise WorkbenchError('EXPERIENCE_CHANGED', '历史经验复制校验失败')
            from .experience_memory import seed_memory, restore_history
            seed_memory(workspace, source / 'blue-cup-wall/task-memory')
            restore_history(workspace, source / 'blue-cup-wall/last-task-memory')

    def _start_task(self, epoch, body):
        task_target=body.get('task_target')
        if task_target not in (None,'upper_shelf','table_coaster'):raise WorkbenchError('TASK_TARGET','Unknown task target')
        task = str(body.get('task', '')).strip()
        if not 1 <= len(task) <= 4000: raise WorkbenchError('TASK', '请输入 1–4000 字的任务')
        if self.agent and self.agent.poll() is None: raise WorkbenchError('AGENT_RUNNING', '已有任务正在运行')
        minutes = body.get('minutes', 20)
        if type(minutes) is not int or not 1 <= minutes <= 45: raise WorkbenchError('DURATION', '单轮任务时限为 1–45 分钟')
        program = self.registry.get(body.get('program')) if body.get('program') else None
        if body.get('program') and (not program or not program['batch_ready']): raise WorkbenchError('NOT_READY', '此程序尚无可测试的完整入口')
        if self.run is None or (self.live / 'session/TASK-RECORD.json').exists() or (self.run / 'TASK-RECORD.json').exists():
            self._reconnect(epoch)
        from .quick_start import require_guard_budget, check_visibility
        require_guard_budget(minutes)
        self._fresh()
        check_visibility(self)
        if self.phase != 'prepared': self._prepare(epoch)
        self._check_epoch(epoch)
        config = self._configure(task, program, minutes, task_target=task_target)
        self.task['auto_freeze'] = body.get('auto_freeze') is True and program is None
        write(self.run/'TASK-RECORD.json',self.task)
        self._fresh()
        with self.lock:
            self._check_epoch(epoch)
            self.owner().command('start')
        deadline = time.monotonic() + 12
        while MotionClient(self.live / 'bridge', config.root).status().get('latest_policy_observation') is None:
            self._check_epoch(epoch)
            if time.monotonic() > deadline: raise WorkbenchError('POLICY_FEEDBACK', '正式 owner 未产生策略反馈')
            time.sleep(.1)
        self._fresh()
        env = {**os.environ, 'RAX_ARTIFACT_ROOT': str(ROOT / 'commissioning'), 'PYTHONPATH': str(ROOT),
               'RAX_PYTHON': str(PYTHON), 'RAX_SKILL_LIBRARY': str(ROOT / 'commissioning/task60-20261029/skill-library')}
        cmd = [str(PYTHON), '-m', 'harness.codex_cli.session', '--config', str(self.run / 'operator-input.json'), '--phase', 'explore', '--run']
        try:
            with self.lock:
                self._check_epoch(epoch)
                arming.write(config.root, True, minutes=minutes+2, reason='operator started this task from authenticated workbench')
                with (self.run / 'rax-session.log').open('x') as log:
                    self.agent = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=log, start_new_session=True)
                self.phase = 'running'; self.error = None
                if self.batch and program:
                    self.batch.update(state='running', current_run=str(self.run)); write(self.data / 'batch.json', self.batch)
        except BaseException:
            arming.write(config.root, False, reason='task startup failed'); self.owner().command('stop'); raise
        self.event('task', '真实可写任务已启动；保留代理视觉检查', task=task, mode=self.task['mode'], run=str(self.run))
        return {'started': True, 'run': str(self.run), 'mode': self.task['mode']}

    def _freeze(self, epoch, body):
        self._check_epoch(epoch)
        if self.agent and self.agent.poll() is None: raise WorkbenchError('AGENT_RUNNING', '任务运行中不能冻结正在改写的程序')
        name = str(body.get('directory', '')).strip()
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', name): raise WorkbenchError('PROGRAM_PATH', '请输入 code/ 下的一个程序目录名')
        source = self.live / 'session/agent-workspace/code' / name
        if source.is_symlink() or source.resolve().parent != (self.live / 'session/agent-workspace/code').resolve(): raise WorkbenchError('PROGRAM_PATH', '程序路径越界')
        manifest = source_manifest(source)
        contract = read(source / 'workbench-program.json', {})
        # Require an explicit full-round contract, not an arbitrary label on a step function.
        if not contract.get('entrypoint') or contract.get('full_round') is not True or contract.get('perception_mode') != 'agent_checkpoints':
            raise WorkbenchError('PROGRAM_CONTRACT', '缺少完整任务接口声明 workbench-program.json', '按工作台文档声明 controller:main、full_round:true、perception_mode:agent_checkpoints')
        from .freeze_contract import entry_source, completion_evidence
        try:
            entry_source(source,contract['entrypoint'])
            completion_evidence(self.live/'session/agent-workspace',manifest['sha256'])
        except (ValueError,OSError,KeyError) as exc:
            raise WorkbenchError('PROGRAM_EVIDENCE',str(exc)) from exc
        if not (source / 'README.md').is_file(): raise WorkbenchError('PROGRAM_DOC', '缺少程序 README 与检查点说明')
        support_manifest = source_manifest(SUPPORT)
        if source_manifest(self.live/'session/agent-workspace/code/rax_support') != support_manifest:
            raise WorkbenchError('SUPPORT_CHANGED', '本轮运行支持库与冻结支持库不一致')
        from .freeze_contract import verify_receipts
        verify_receipts(self.live/'session/agent-workspace', completion_evidence(self.live/'session/agent-workspace',manifest['sha256']), manifest['sha256'])
        model = read(self.live/'session/CODEX_PREPARED.json',{}).get('model')
        if not model: raise WorkbenchError('MODEL_VERSION', '未找到本轮实际模型配置', '完成 RAX 连接预检后再冻结')
        qualified_sha = sha(QUALIFICATION)
        workflow = workflow_contract()
        execution_sha = hashlib.sha256((manifest['sha256'] + support_manifest['sha256'] + qualified_sha + model + json.dumps(workflow,sort_keys=True)).encode()).hexdigest()
        ident = 'frozen-' + execution_sha[:12]
        if ident not in self.registry:
            destination = self.data / 'programs' / ident
            shutil.copytree(source, destination, ignore=shutil.ignore_patterns('.git', '__pycache__', '.pytest_cache'))
            if source_manifest(destination) != manifest: raise WorkbenchError('FREEZE_COPY', '冻结副本校验失败')
            self.registry[ident] = {'id': ident, 'name': str(body.get('name') or name)[:100], 'version': execution_sha[:12],
                'support_manifest': support_manifest, 'execution_sha256': execution_sha, 'entrypoint': contract['entrypoint'],
                'model': model, 'qualification_sha256': qualified_sha, 'workflow_contract': workflow,
                'path': str(destination), 'manifest': manifest, 'mode': 'agent_checkpoints', 'batch_ready': True,
                'status': '已冻结·待真机验证', 'verified_successes': 0, 'created': time.time()}
            write(self.data / 'programs.json', self.registry)
        self.selected = ident; self.event('freeze', '程序已冻结，可进入跨摆位测试；尚未宣称可靠', program=ident)
        return self.registry[ident]

    def _segment(self, epoch, body):
        self._fresh()
        self._check_epoch(epoch)
        prompts = str(body.get('prompts') or 'cup,round coaster')
        if len(prompts) > 500: raise WorkbenchError('SEGMENT_PROMPT', '分割提示过长')
        source = self.live / 'session/stream/multiview' / self.vision['id']
        target = self.live / 'session/stream/sam3' / uuid.uuid4().hex
        target.parent.mkdir(parents=True, exist_ok=True)
        cache = Path('/home/ubuntu/code_junqi/.cache')
        env = {**os.environ, 'XDG_CACHE_HOME': str(cache), 'TORCH_HOME': str(cache/'torch'),
               'HF_HOME': str(cache/'huggingface'), 'TRITON_CACHE_DIR': str(cache/'triton'),
               'CUDA_CACHE_PATH': str(cache/'cuda'), 'PYTHONDONTWRITEBYTECODE': '1'}
        command = ['/home/ubuntu/code_junqi/perception/sam3/.venv/bin/python', str(Path(__file__).parent/'sam3_snapshot.py'),
                   '--source', str(source), '--output', str(target), '--prompts', prompts]
        with (self.run / ('sam3-' + target.name + '.log')).open('x') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=log, start_new_session=True)
        deadline = time.monotonic() + 90
        try:
            while process.poll() is None:
                self._check_epoch(epoch)
                if time.monotonic() > deadline: raise WorkbenchError('SAM3_TIMEOUT', 'SAM3 处理超时', '查看 SAM3 日志；它不替代实时视觉检查')
                time.sleep(.2)
            if process.returncode: raise WorkbenchError('SAM3_FAILED', 'SAM3 处理失败', '查看本轮 sam3 日志')
            result = read(target / 'result.json')
            if not result: raise WorkbenchError('SAM3_RESULT', 'SAM3 未生成有效结果')
            result.update(bundle_id=source.name, source_created_ns=read(source/'bundle.json')['created_ns'], directory=str(target))
            write(target / 'result.json', result)
            write(target.parent / 'latest.json', {'id':target.name, 'bundle_id':source.name})
            self.segmentation = result
            self.event('vision', 'SAM3 双视角分割完成；结果是快照辅助，不证明接触或空间间隔', counts={c:len(v['objects']) for c,v in result['views'].items()})
            return {'completed': True, 'views': {c:len(v['objects']) for c,v in result['views'].items()}, 'duration_s':result['total_s']}
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)

    def _batch_start(self, body):
        program = self.registry.get(body.get('program'))
        if not program or not program.get('batch_ready'): raise WorkbenchError('NOT_READY', '请选择有完整入口的冻结程序；5eaccef 是逐步基线')
        count = body.get('rounds', 3)
        if type(count) is not int or not 1 <= count <= 20: raise WorkbenchError('ROUNDS', '批量轮数必须是 1–20')
        if self.agent and self.agent.poll() is None: raise WorkbenchError('AGENT_RUNNING', '请先结束当前任务')
        if self.batch and self.batch['state'] not in ('complete', 'interrupted'):
            raise WorkbenchError('BATCH_ACTIVE', '已有未完成批量计划', '完成该计划，或先 Hold 终止它')
        if self.batch: write(self.data / 'batches' / (self.batch['id'] + '.json'), self.batch)
        self.batch = {'id': uuid.uuid4().hex, 'program': program['id'], 'version_sha256': program['manifest']['sha256'],
                      'task': str(body.get('task') or '将黄色杯子直立放在黄色杯垫上，释放并退开'),
                      'target_rounds': count, 'results': [], 'state': 'awaiting_reset', 'created': time.time()}
        write(self.data / 'batch.json', self.batch)
        self.event('batch', '批量计划已创建，等待第一轮摆位确认', rounds=count)
        return self.batch

    def _next_round(self, epoch, body):
        if not self.batch or self.batch['state'] != 'awaiting_reset': raise WorkbenchError('BATCH_STATE', '当前没有等待摆位的批量计划')
        layout = str(body.get('layout', '')).strip()
        if not layout: raise WorkbenchError('LAYOUT', '请填写本轮摆位，例如「左侧近处、杯柄朝外」')
        self.batch['current_layout'] = layout
        write(self.data / 'batch.json', self.batch)
        return self._start_task(epoch, {'task': self.batch['task'], 'program': self.batch['program'], 'minutes': body.get('minutes', 20)})

    def _record_result(self, body):
        if self.phase != 'awaiting_result' or not self.task or self.task.get('result_recorded'):
            raise WorkbenchError('RESULT_STATE', '本轮尚未结束，或结果已经记录')
        success = body.get('success') is True
        if success and self.agent and self.agent.poll() != 0:
            raise WorkbenchError('RESULT_EXIT', 'RAX 未正常结束，不能计作成功')
        reported = read(self.live / 'session/agent-workspace/output/round-result.json', {})
        if success and (reported.get('task_success') is not True or reported.get('retreat_verified') is not True or not reported.get('checkpoint_frames')):
            raise WorkbenchError('RESULT_EVIDENCE', '缺少完整成功、退开及检查点图像报告，不能计作成功', '先查看 RAX 回答；可记录为失败或未完成')
        if success:
            workspace = self.live / 'session/agent-workspace'
            for name in reported['checkpoint_frames']:
                if not isinstance(name, str): raise WorkbenchError('RESULT_FRAME', '图像证据路径格式错误')
                path = Path(name)
                path = (path if path.is_absolute() else workspace / path).resolve()
                if not (path.is_relative_to(workspace.resolve()) or path.is_relative_to((self.live / 'session/stream').resolve())) or not path.is_file() or path.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
                    raise WorkbenchError('RESULT_FRAME', '图像证据不存在或不属于本轮会话')
        row = {'run': str(self.run), 'at': time.time(), 'success': success, 'operator_confirmed': True,
               'note': str(body.get('note', ''))[:2000], 'task': self.task['text'], 'mode': self.task['mode'],
               'program': self.task['program'], 'report': reported, 'autonomous_success': False}
        if self.task['program']:
            program = self.registry[self.task['program']]
            intact = source_manifest(self.live / 'session/agent-workspace/code/frozen_program') == program['manifest']
            intact = intact and source_manifest(self.live / 'session/agent-workspace/code/rax_support') == program['support_manifest']
            if not intact: row.update(success=False, source_changed=True)
            if success and reported.get('controller_sha256') != program['manifest']['sha256']:
                raise WorkbenchError('RESULT_VERSION', '成功报告的程序摘要与冻结版本不一致')
            if success:
                from .freeze_contract import completion_evidence, verify_receipts
                try:
                    proof=completion_evidence(self.live/'session/agent-workspace',program['manifest']['sha256'])
                    verify_receipts(self.live/'session/agent-workspace',proof,program['manifest']['sha256'])
                except (ValueError,OSError,KeyError) as exc:
                    raise WorkbenchError('RESULT_EVIDENCE',str(exc)) from exc
            row['controller_sha256'] = program['manifest']['sha256']
        self.history.append(row); write(self.data / 'trials.json', self.history)
        self.task['result_recorded'] = True; write(self.run / 'TASK-RECORD.json', self.task)
        if self.batch and self.batch.get('current_run') == str(self.run):
            row['layout'] = self.batch.get('current_layout')
            self.batch['results'].append(row)
            self.batch['state'] = 'complete' if len(self.batch['results']) >= self.batch['target_rounds'] else 'awaiting_reset'
            write(self.data / 'batch.json', self.batch)
            write(self.data / 'batches' / (self.batch['id'] + '.json'), self.batch)
        self._refresh_program_results(self.task['program'])
        write(self.data / 'trials.json', self.history)
        self.event('result', '本轮结果已记录', success=row['success'], run=str(self.run))
        return row

    def _refresh_program_results(self, ident):
        if not ident: return
        program = self.registry[ident]
        trials = [r for r in self.history if r.get('program') == ident]
        consecutive = []
        for row in reversed(trials):
            if not row['success']: break
            consecutive.append(row)
        layouts = {row.get('layout') for row in consecutive if row.get('layout')}
        program.update(verified_successes=sum(row['success'] for row in trials),
                       consecutive_successes=len(consecutive), varied_layout_count=len(layouts))
        program['status'] = ('通过初步验收·代理检查模式' if len(consecutive)>=3 and len(layouts)>=3
                             else '已冻结·待真机验证')
        write(self.data / 'programs.json', self.registry)

    def _save_task_memory(self):
        if not self.run or not self.task or self.task.get("mode") == "operator_reset": return
        from .experience_memory import persist_memory
        try:
            persist_memory(self.live/'session/agent-workspace', self.data/'experience/blue-cup-wall/last-task-memory', self.run.name)
        except (OSError, ValueError) as exc:
            self.event('memory_save_error', str(exc))

    def _interrupt_trial(self, reason):
        """Keep aborted trials in the denominator and break a success streak."""
        self._save_task_memory()
        with self.result_lock:
            if not self.run or not self.task or self.task.get('result_recorded'): return
            row = {'run':str(self.run), 'at':time.time(), 'success':False,
                   'operator_confirmed':False, 'interrupted':True, 'note':reason,
                   'task':self.task['text'], 'mode':self.task['mode'],
                   'program':self.task['program'], 'autonomous_success':False}
            if self.task['program']:
                row['controller_sha256'] = self.registry[self.task['program']]['manifest']['sha256']
            self.history.append(row)
            self.task['result_recorded'] = True
            write(self.run / 'TASK-RECORD.json', self.task)
            if self.batch and self.batch.get('current_run') == str(self.run):
                row['layout'] = self.batch.get('current_layout')
                self.batch['results'].append(row)
                self.batch.update(state='interrupted', reason=reason)
                write(self.data/'batch.json',self.batch)
                write(self.data/'batches'/(self.batch['id']+'.json'),self.batch)
            write(self.data/'trials.json',self.history)
            self._refresh_program_results(self.task['program'])

    def _monitor(self):
        workers = [threading.Thread(target=target, daemon=True, name=name)
                   for name, target in (('workbench-feedback', self._feedback_loop),
                                        ('workbench-vision', self._vision_loop),
                                        ('workbench-clock', self._guard_loop))]
        self.sensor_workers = workers
        for worker in workers: worker.start()
        while not self.stop_event.is_set():
            try:
                self._monitor_loop()
            except Exception as exc:
                if self.phase in ('running','preparing'): self.hold('监测异常，已停止：' + str(exc))
                self.error = {'code':'MONITOR_ERROR','message':str(exc),'fix':'检查运行日志；不要忽略监测中断','at':time.time()}
                self.stop_event.wait(1)

    def _connected_run(self):
        run = self.run
        return run if run and (run / 'CONNECTED.json').exists() and self.phase != 'disconnected' else None

    def _sampling_begin(self, kind):
        with self._health_lock:
            self.health[kind] = {**self.health.get(kind, {}),
                                 'started_at': time.time(), 'in_flight': True}

    def _sampling_result(self, kind, started, error='', **extra):
        with self._health_lock:
            old = self.health.get(kind, {})
            duration = time.monotonic() - started
            self.health[kind] = {'duration_s': duration,
                'max_duration_s': max(duration, old.get('max_duration_s', 0)),
                'started_at': old.get('started_at'), 'in_flight': False,
                'completed_at': time.time(), 'error': error,
                'samples': old.get('samples', 0) + (not error),
                'failures': old.get('failures', 0) + bool(error), **extra}

    def _feedback_loop(self):
        # No cameras, depth, encoding, disk traversal or guard subprocess on this path.
        while not self.stop_event.is_set():
            run = self._connected_run()
            if run:
                started = time.monotonic()
                self._sampling_begin('feedback')
                try:
                    state = OwnerReader().state()
                    meta = state['robot_state'].get('feedback_metadata', {})
                    source = meta.get('source_timestamp_ns')
                    source_age = max(0., (time.time_ns() - source) / 1e9) if source else 0.
                    if run == self.run and self.phase != 'disconnected':
                        self.robot = state
                        self._last_sensor_ok = time.monotonic() - source_age
                        self.robot_error = ''
                        self._sampling_result('feedback', started, source_age_s=source_age,
                                              maximum_age_ms=meta.get('maximum_age_ms'))
                except Exception as exc:
                    if run == self.run:
                        self.robot_error = str(exc)[:400]
                        self._sampling_result('feedback', started, self.robot_error)
            self.stop_event.wait(.2)

    def _vision_loop(self):
        while not self.stop_event.is_set():
            run = self._connected_run()
            if run:
                started = time.monotonic()
                self._sampling_begin('vision')
                try:
                    live = run / 'live'
                    bundle, folder = capture_bundle(OwnerReader, DepthReader(live / 'depth', CALIBRATION_SHA),
                                                    live / 'session/stream/multiview')
                    cache = {c: ((folder / f'{c}.jpg').read_bytes(), bundle['oldest_source_ns']) for c in CAMERAS}
                    age = (time.time_ns() - bundle['oldest_source_ns']) / 1e9
                    if not 0 <= age <= 1.0:
                        raise ValueError(f'三视角发布前已过期：{age:.3f}s')
                    if run == self.run and self.phase != 'disconnected':
                        self.vision = bundle
                        self.image_cache = cache
                        self._last_depth_ok = time.monotonic() - age
                        self.vision_error = ''
                        # Packs are retained as evidence; do not rescan/sort every old pack.
                        self._last_bundle_saved += 1
                        self._sampling_result('vision', started, source_age_s=age,
                                              stages=bundle.get('timing_s', {}))
                except Exception as exc:
                    if run == self.run:
                        self.vision_error = str(exc)[:400]
                        self._sampling_result('vision', started, self.vision_error)
            self.stop_event.wait(.7)

    def _guard_loop(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            self._sampling_begin('clock_guard')
            try:
                check_clock_guard(GUARD)
                self._last_guard_check = time.monotonic()
                self._guard_ok = True
                self._sampling_result('clock_guard', started)
            except Exception as exc:
                self._guard_ok = False
                self._last_guard_check = time.monotonic()
                self._sampling_result('clock_guard', started, str(exc)[:400])
            try: self._clock_broker_active = system_state('junqi-rax-clock-broker.service') == 'active'
            except Exception: self._clock_broker_active = False
            self.stop_event.wait(2)

    def _health_fault(self):
        now = time.monotonic()
        if not self._guard_ok: return '时钟保护结束；任务已停止，请重新认证'
        if now - self._last_guard_check > 8: return '时钟保护检查超时；任务已停止'
        if now - self._last_sensor_ok > 2: return '机械臂反馈丢失超过 2 秒'
        if self.phase == 'running' and now - self._last_depth_ok > 4:
            return '三视角或 FFS 持续失效超过 4 秒'
        return None

    def _monitor_loop(self):
        while not self.stop_event.is_set():
            if self._connected_run():
                if self.phase in ('running', 'preparing'):
                    fault = self._health_fault()
                    if fault:
                        # Capture the cause BEFORE stop/API latency changes the evidence.
                        with self._health_lock:
                            evidence = dict(self.health)
                        detected = time.time()
                        ages = {'feedback_s': time.monotonic() - self._last_sensor_ok,
                                'vision_s': time.monotonic() - self._last_depth_ok}
                        self.hold(fault)
                        self.error = {'code': 'LIVE_HEALTH', 'message': fault, 'fix': '排查后显式重连',
                                      'at': detected, 'health': evidence, 'ages': ages}
                        self.event('health_fault', fault, health=evidence, ages=ages, detected_at=detected)
                if self.agent and self.agent.poll() is not None and self.phase == 'running':
                    arming.write(self.live / 'session', False, reason='agent session ended')
                    try: self.owner().command('stop')
                    except Exception: pass
                    self._save_task_memory()
                    self.phase = 'awaiting_result'
                    if self.batch and self.batch.get('state') == 'running':
                        self.batch['state'] = 'awaiting_result'; write(self.data / 'batch.json', self.batch)
                    from .quick_start import maybe_freeze
                    try: maybe_freeze(self)
                    except Exception as exc: self.event('freeze_refused', str(exc))
                    self.event('agent_finished', 'RAX 本轮结束，已停止下发；请核对结果', returncode=self.agent.returncode)
                    if self.agent.returncode:
                        self.error = {'code': 'RAX_EXIT', 'message': f'RAX 退出码 {self.agent.returncode}', 'fix': '查看 RAX 回答和运行日志；不能计为成功', 'at': time.time()}
                if self.phase == 'running' and self.task and self.task.get('program') and time.monotonic() > self._manifest_check_at:
                    self._manifest_check_at = time.monotonic() + 2
                    try:
                        program = self.registry[self.task['program']]
                        if (source_manifest(self.live / 'session/agent-workspace/code/frozen_program') != program['manifest'] or
                            source_manifest(self.live / 'session/agent-workspace/code/rax_support') != program['support_manifest']):
                            self.hold('固定版本程序发生改变，本轮作废')
                            self.error = {'code': 'FROZEN_CHANGED', 'message': '冻结程序被修改，已停止', 'fix': '修复后冻结新版本并重新累计', 'at': time.time()}
                    except Exception as exc:
                        self.hold('固定版本完整性检查失败：' + str(exc))
            self.stop_event.wait(.1)

    def state(self):
        with self.lock:
            session = self.live / 'session' if self.run else None
            programs = [{k:v for k,v in row.items() if k not in ('path','manifest','support_manifest')} | {'sha256': row['manifest']['sha256']} for row in self.registry.values()]
            vision = self.vision
            age = None if not vision else (time.time_ns() - vision['oldest_source_ns']) / 1e9
            robot = self.robot
            robot_age = time.monotonic() - self._last_sensor_ok if robot else None
            with self._health_lock:
                health = dict(self.health)
            return {'epoch': self.epoch, 'phase': self.phase, 'job': self.job, 'error': self.error,
                'prepare_mode': read(self.run/'CONNECTED.json',{}).get('prepare_mode','home') if self.run else 'home',
                'run': None if self.run is None else self.run.name, 'task': self.task, 'selected_program': self.selected,
                'programs': programs, 'batch': self.batch, 'history': self.history[-20:], 'events': self.events[-40:],
                'clock_broker_active': self._clock_broker_active,
                'clock_broker': read(DATA/'private/clock-broker/authorization.json',{}),
                'clock_guard': self._guard_ok, 'robot_fresh': robot_age is not None and robot_age < 2,
                'robot_age_s': robot_age, 'robot': robot, 'vision': vision, 'vision_age_s': age,
                'vision_ready': age is not None and 0 <= age < 2,
                'sensor_error': self.robot_error or self.vision_error,
                'robot_error': self.robot_error, 'vision_error': self.vision_error, 'health': health,
                'segmentation': self._segmentation_record(),
                'arming': arming.read(session) if session else {'armed': False},
                'requests': operator.pending(session) if session else [],
                'progress': read(session/'agent-workspace/output/progress.json', {}) if session else {},
                'model_effort': read(session/'CODEX_PREPARED.json',{}).get('effort') if session else None,
                'model': read(session/'CODEX_PREPARED.json',{}).get('model') if session else None,
                'checkpoint_evidence': [read(session/'agent-workspace/output/round-result.json', {})] if session else [],
                'messages': agent_messages(session) if session else [],
                'logs': tail_lines(self.run / 'rax-session.log', 16) if self.run else [],
                'hardware_logs': tail_lines(self.run / 'motion-launch.log', 10) if self.run else [],
                'guard_command': 'ssh -t ubuntu@192.168.200.102 \'sudo bash ' + str(ROOT / 'scripts/piper_workbench_authorize_nextday.sh') + '\'',
                'capabilities': {'real_write_available': self._guard_ok and robot_age is not None and robot_age < 2 and bool(robot['robot_status'].get('hardware_write_enabled')),
                                 'metric_wrist_fusion': False, 'perception': 'bounded-time head + wrist evidence',
                                 'planner': 'qualified PiPER flange IK + sole owner', 'model_provider': 'native Codex / existing provider'},
                'plan': {'exploration': '先做 3–5 轮有记录的完整探索，包含对齐、失败恢复与退开；关键阶段尚未稳定就继续探索',
                         'freeze': '冻结代码、参数和视觉检查规则；每轮记录版本摘要',
                         'qualification': '至少 3 个不同摆位连续完整成功；改版后重新累计',
                         'validation': '再做 10 轮预先列出的摆位验证，记录所有成功、失败和介入；目标 ≥9/10，安全故障为 0'}}

    def image(self, camera):
        if camera not in CAMERAS: raise ValueError('未知相机')
        item = self.image_cache.get(camera)
        if not item or (time.time_ns() - item[1]) > 2_000_000_000:
            raise WorkbenchError('CAMERA_STALE', '相机画面过期；不会用旧图冒充直播')
        return item[0]

    def _segmentation_record(self):
        result = self.segmentation
        if self.run:
            queue = self.live/'session/agent-workspace/output/perception-queue'
            pointer = read(queue/'latest.json', {})
            ident = pointer.get('id', '')
            if re.fullmatch('[a-f0-9]{32}', ident):
                candidate = read(queue/'responses'/(ident+'.json'), {})
                if candidate.get('ok') and candidate.get('id') == ident:
                    folder = Path(candidate.get('directory','')).resolve()
                    if folder.parent == (queue/'artifacts').resolve() and folder.name == ident:
                        if not result or candidate.get('finished_ns',0) > result.get('finished_ns',0):
                            result = {**candidate, 'bundle_id':ident, 'total_s':candidate['inference_s'],
                                      'source_created_ns':candidate.get('source_received_ns'),
                                      'origin':'agent_same_frame_broker'}
        return result

    def segmentation_image(self, camera, expected_id=None):
        result = self._segmentation_record()
        if camera not in ('head','left') or not result:
            raise WorkbenchError('NO_SEGMENTATION', '本轮尚无 SAM3 快照')
        if expected_id and result.get('bundle_id') != expected_id:
            raise WorkbenchError('SEGMENTATION_CHANGED', '分割检查点已更新；请刷新')
        folder = Path(result['directory']).resolve()
        roots = [self.live/'session/stream/sam3',self.live/'session/agent-workspace/output/perception-queue/artifacts']
        if not any(folder.is_relative_to(p.resolve()) for p in roots):
            raise WorkbenchError('SEGMENTATION_PATH', '快照不属于当前会话')
        return (folder/(camera+'-segmentation.jpg')).read_bytes()

    def close(self):
        self.hold('工作台退出')
        self.stop_event.set()
        subprocess.run(['systemctl', '--user', 'stop', HARDWARE_UNIT], capture_output=True, timeout=95)
