"""Piper operator orchestration for one native main(ctx) at a time.

Reuses existing launchers, session-bound XPolicy commands and dashboard HTTP auth.
No controller, perception or collision logic lives here. Importing starts no devices.
"""
from __future__ import annotations
import argparse
import csv
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
import uuid
from http.server import ThreadingHTTPServer
from services.dashboard.server import Handler, COOKIE

ROOT=Path(__file__).resolve().parents[2]
STATIC=Path(__file__).parent/'static/blocks'
DEFAULT_CANDIDATE='workspaces/magnetic-blocks-l-stack/r9-ssik-speed'
PYTHON='/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python'
# goal1: goal photo -> fixed goal description. Host-side only; same key file and proxy as RAX.
GEMINI_KEY_FILE='/home/ubuntu/code_junqi/gemini-key.py'
GEMINI_PROXY='http://127.0.0.1:7892'
GOAL_DAILY_CALLS=300
GOAL_VOTES=3  # readings per photo, compared with each other
GOAL_IMAGE_NAME=r'goal-\d{8}-\d{6}-[0-9a-f]{8}\.jpg'
PLANNER_SUPPORTS_CUSTOM_GOAL=False  # Phase 3/4 will generalize the planner; until then only the default goal may run.

def read(path,default=None):
    try: return json.loads(Path(path).read_text())
    except (OSError,ValueError): return default

def tail(path,limit=12000):
    try:
        with Path(path).open('rb') as f:
            f.seek(0,2); f.seek(max(0,f.tell()-limit)); return f.read().decode(errors='replace')
    except OSError: return ''

def save(path,data):
    tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)); tmp.replace(path)

def normalize_report(raw):
    if not raw:return {}
    r=dict(raw)
    path=Path(r.get('active_step_report') or '/nonexistent')
    if path.resolve().is_relative_to(ROOT/'commissioning'):
        detail=read(path,{}).get('result',{}).get('review',{}).get('assessment') or {}
        reason=detail.get('why')
        if reason and reason not in (r.get('why') or ''):
            r['why']=(r.get('why') or 'Scene check refused')+': '+reason

    r['main_agent_calls']=r.get('primary_model_calls',0)
    r['metrics_complete']=r.get('counters_complete',False)
    r['steps']=[dict(s,stage=s.get('label',s.get('stage')),stage_elapsed_s=s.get('wall_s')) for s in r.get('steps',[])]
    return r

def controller_report(out):
    done=read(out/'REHEARSAL-DONE.json')
    if done:return normalize_report(done)
    paths=list((out/'rax-session/agent-workspace/output/composition/runs').glob('*/REPORT.json'))
    return normalize_report(read(max(paths,key=lambda p:p.stat().st_mtime),{})) if paths else {}

def initial_layout(out):
    paths=list((out/'rax-session/agent-workspace/output/composition/runs').glob('*/001-initial-state.json'))
    if not paths:return None
    r=read(max(paths,key=lambda p:p.stat().st_mtime),{})
    facts=r.get('result',{}).get('facts',[])
    scene=next((f['scene'] for f in facts if f.get('scene')),None)
    if not scene:return None
    return {'source':'initial_rgbd','frame_id':scene.get('frame_id'),
            'blocks':{k:v.get('top_center',{}).get('surface_position_m') for k,v in scene.get('blocks',{}).items()},
            'historical_measurement_only':True}

def layout_label(layout):
    if not layout:return '初始定位尚无结果'
    names={'red':'红','orange':'橙','green':'绿','yellow':'黄'}
    return '；'.join(names.get(k,k)+': '+str([round(t*1000,1) for t in v])+' mm' for k,v in layout['blocks'].items() if v)

def at_prepare(state,profile):
    q=state['robot_state']['values']; pose=profile['initial_pose']
    target=pose['left_arm']+[pose['left_gripper']]+pose['right_arm']+[pose['right_gripper']]
    return state['robot_status'].get('lifecycle_state')=='prepared' and all(
        abs(a-b)<=(.0005 if i in (6,13) else .01) for i,(a,b) in enumerate(zip(q,target)))

class Cancelled(Exception): pass

class Manager:
    def __init__(self,root=ROOT,python=PYTHON):
        self.root=Path(root); self.python=python; self.candidate=self.root/DEFAULT_CANDIDATE
        self.data=self.root/'runtime/blocks-scale-tests'; self.data.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock(); self.cancel=threading.Event(); self.thread=None
        self.process=None; self.runner=None; self.out=None; self.mode='disconnected'
        self.phase='未连接'; self.error=None; self.events=[]
        self.started=None; self.active_record=None; self.prepared=False
        self.finished_detail_cache={}
        self.goal_dir=self.data/'goal'; self.goal_lock=threading.Lock(); self.goal_pending=None
        self.guard_file=self.data/'requires-rest.json'
        self.recovery_required=self.guard_file.exists()
        if self.recovery_required:self.phase='需要恢复连接'
        self.event('控制台已启动；硬件未连接，未发送动作。')

    def event(self,message):
        with self.lock:
            self.events.append({'time':time.time(),'message':str(message)})
            self.events=self.events[-150:]

    def launch(self,script,out,*args):
        out.parent.mkdir(parents=True,exist_ok=True)
        log=self.data/(out.name+'.log')
        if self.mode in ('run','prepare'):
            save(self.guard_file,{'session':out.name,'time':time.time()})
        with log.open('x') as f:
            proc=subprocess.Popen([self.python,str(script),'--output',str(out),*args,'--run'],cwd=self.root,
                env={**os.environ,'PYTHONPATH':str(self.root),'PYTHONDONTWRITEBYTECODE':'1'},stdout=f,stderr=f,start_new_session=True)
        self.process=proc; self.out=out
        return proc

    def fresh_output(self,kind):
        return self.root/'commissioning'/('web-'+kind+'-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])

    def wait(self,predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.cancel.is_set(): raise Cancelled('操作已停止')
            value=predicate()
            if value: return value
            if self.process and self.process.poll() is not None:
                result=read(self.out/'RESULT.json',{})
                reason=result.get('error') or tail(self.data/(self.out.name+'.log'),1800)
                raise RuntimeError('硬件会话已退出；'+str(reason))
            self.cancel.wait(.15)
        raise TimeoutError(label)

    def operator(self):
        sys.path.insert(0,str(self.root/'scripts'))
        from piper_minimal_live import Operator
        return Operator(self.out)

    def owner(self):
        if not self.process or self.process.poll() is not None or not self.out: raise RuntimeError('没有本控制台连接的硬件')
        # Verify session identity before serving telemetry, including read-only preview.
        return self.operator().state()

    def verify_prepared(self):
        from services.piper.owner import OwnerReader
        self.operator().state()
        if not at_prepare(OwnerReader().state(),read(self.out/'PLAN.json',{})['native_prepare']):
            raise RuntimeError('当前实测状态已离开 Prepare；请核对后重新回 Prepare')

    def close_owner(self):
        """Stop/disarm before terminating a write session; never kill another owner."""
        if self.process and self.process.poll() is None:
            if self.mode in ('run','prepare'):
                sys.path.insert(0,str(self.root/'scripts'))
                from piper_minimal_live import stop_session
                # During startup no live owner yet: launcher finally performs Stop.
                if (self.out/'private/owner-token').exists():
                    try:
                        self.operator().state()
                    except Exception:
                        self.event('Owner 尚未可读；退出启动器并检查其 Stop 结果。')
                    else:
                        stop_session(self.out)
                        self.event('XPolicy Stop 与静止反馈已确认。')
            self.process.send_signal(signal.SIGTERM)
            self.process.wait(timeout=55)
        if self.runner and self.runner.poll() is None:
            self.runner.terminate(); self.runner.wait(timeout=15)
        # The native supervisor is responsible for terminating its children.
        # A failure leaves the device guard blocking any subsequent new owner.
        if self.mode in ('run','prepare') and self.out:
            result=read(self.out/'RESULT.json',{})
            receipt=read(self.out/'STOP-RESULT.json',{}) or {}
            if result.get('stop_error') or not ((receipt.get('rest') or result.get('rest') or {}).get('measured_rest')):
                self.recovery_required=True
                raise RuntimeError('停止未确认：'+str(result.get('stop_error') or '缺少静止回执'))
            self.guard_file.unlink(missing_ok=True)
        self.process=None; self.runner=None; self.mode='disconnected'

    def validate_run_source(self):
        """Check the native source/evidence contract before releasing Prepare."""
        sys.path.insert(0,str(self.root/'scripts'))
        from piper_explore import validate_rehearsal_source,load_task,validate_task_sources,check_startup_capacity
        validate_task_sources(load_task(self.root/'examples/tasks/magnetic-blocks-l-stack/session.json'),self.candidate/'memory')
        from piper_handoff import copy_memory_evidence
        validate_rehearsal_source(self.candidate,self.root)
        check_startup_capacity(self.root,self.candidate)
        copy_memory_evidence(self.candidate,self.candidate,memory_dir=self.candidate/'memory')
        meta=read(self.candidate/'WORKSPACE.json',{}) | read(self.candidate/'output/composition/VERIFICATION.json',{})
        if meta.get('aplus_test_snapshot_ready') is not True:raise RuntimeError('A+ 固定测试版尚未完成核验，禁止执行')
        code=self.candidate/'code/magnetic_blocks'
        commit=subprocess.check_output(['git','-C',str(code),'rev-parse','HEAD'],text=True).strip()
        if subprocess.check_output(['git','-C',str(code),'status','--porcelain','--untracked-files=no'],text=True).strip():
            raise RuntimeError('工作副本有未提交代码；先完成验证和版本记录')
        if commit!=meta.get('commit'):raise RuntimeError('工作副本版本元数据不匹配')
        validation=read(meta['validation_report'],{})
        current={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in code.glob('*.py')}
        if current!=validation.get('code_hashes'):raise RuntimeError('固定 A+ 代码与离线核验不符，禁止执行')
        for rel,h in meta.get('required_host_sha256',{}).items():
            p=(self.root/rel).resolve()
            if not p.is_relative_to(self.root.resolve()) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:raise RuntimeError('已验证的执行接口版本不匹配：'+rel)

    def recover(self):
        # Do not detach a live writer. The independent device occupancy check also
        # detects orphaned processes after a dashboard restart.
        if any(p and p.poll() is None for p in (self.process,self.runner)) and self.mode!='preview':
            raise RuntimeError('原会话仍占用设备，请先停止本轮，再恢复')
        self.recovery_required=True
        if not self.guard_file.exists():save(self.guard_file,{'session':self.out.name if self.out else None,'time':time.time()})
        self.prepared=False
        sys.path.insert(0,str(self.root/'scripts'))
        from piper_observe_only import no_existing_devices
        from piper_minimal_live import measured_rest
        from services.piper.owner import OwnerReader
        if self.mode=='preview':self.close_owner()
        no_existing_devices()
        self.mode='preview'
        out=self.fresh_output('preview')
        self.launch(self.root/'scripts/piper_preview.py',out,'--seconds','3600')
        try:
            def available():
                try:
                    s=self.owner()
                    return s.get('robot_state',{}).get('available') and all((s.get('cameras',{}).get(k) or {}).get('available') for k in ('left','right','head'))
                except (OSError,ValueError,RuntimeError):return False
            self.wait(available,65,'等待新只读会话超时')
            rest=measured_rest(OwnerReader(),self.operator())
            if self.cancel.is_set():raise Cancelled('恢复已取消')
            save(out/'RECOVERY-REST.json',{'previous':read(self.guard_file),'rest':rest})
            self.guard_file.unlink()
            self.recovery_required=False
            self.phase='已恢复 · 只读预览'
            self.event('已核对当前双臂静止和三路相机。持物状态须现场确认。')
        except BaseException:
            self.close_owner()
            raise

    # ---- goal1: goal description. Reads/writes files under data/goal only; never starts devices. ----
    def goal_active(self):
        from services.dashboard import goal_spec
        saved=read(self.goal_dir/'active.json')
        if saved:
            checked=goal_spec.check(saved.get('spec'))
            if checked['ok'] and checked['digest']==saved.get('digest'):
                return {**checked,'source':saved.get('source'),'confirmed_at':saved.get('confirmed_at'),'image':saved.get('image')}
        checked=goal_spec.check(goal_spec.DEFAULT_SPEC)
        return {**checked,'source':'default','confirmed_at':None,'image':None}

    def goal_runnable(self,active=None):
        active=active or self.goal_active()
        return bool(active['is_default'] or PLANNER_SUPPORTS_CUSTOM_GOAL)

    def goal_state(self,body=None):
        active=self.goal_active()
        return {'active':active,'pending':self.goal_pending,'planner_supports_custom':PLANNER_SUPPORTS_CUSTOM_GOAL,
                'runnable':self.goal_runnable(active),'model':'gemini-3.8-flash'}

    def _goal_log(self,row):
        self.goal_dir.mkdir(parents=True,exist_ok=True)
        with (self.goal_dir/'analyses.jsonl').open('a') as f:f.write(json.dumps({'time':time.time(),**row},ensure_ascii=False)+'\n')

    def goal_analyze(self,body):
        from services.dashboard import goal_spec,goal_gemini
        jpeg,meta=goal_gemini.load_image(body.get('image'))
        if not self.goal_lock.acquire(blocking=False):raise RuntimeError('上一张目标图还在解析，请稍候')
        try:
            day=time.strftime('%Y-%m-%d');calls=0
            try:
                for line in (self.goal_dir/'analyses.jsonl').read_text().splitlines():
                    row=json.loads(line)
                    if time.strftime('%Y-%m-%d',time.localtime(row.get('time',0)))==day:calls+=int(row.get('called') or 0)
            except (OSError,ValueError):pass
            if calls>=GOAL_DAILY_CALLS:raise RuntimeError(f'今天已解析 {calls} 次，达到每日上限 {GOAL_DAILY_CALLS} 次')
            self.goal_dir.mkdir(parents=True,exist_ok=True)
            name='goal-'+time.strftime('%Y%m%d-%H%M%S')+'-'+hashlib.sha256(jpeg).hexdigest()[:8]+'.jpg'
            (self.goal_dir/name).write_bytes(jpeg)
            result=goal_gemini.analyze_votes(jpeg,key_file=GEMINI_KEY_FILE,proxy_url=GEMINI_PROXY,votes=GOAL_VOTES)
            answers=[r['answer'] for r in result['results'] if 'answer' in r];failed=[r['error'] for r in result['results'] if 'error' in r]
            if not answers:
                self._goal_log({'image':name,'called':GOAL_VOTES,'ok':False,'errors':failed})
                raise RuntimeError('Gemini 解析失败：'+failed[0][:240]+'。可以重试，或直接在下方手动填写目标。')
            candidates,agreement=goal_spec.tally(answers);notes=[]
            best=candidates[0] if candidates else {'check':goal_spec.check(answers[0]),'answer':answers[0],'votes':0}
            answer=best['answer'];checked=best['check']
            if agreement in ('majority','split'):
                notes.append('同一张照片解析了 %d 次，结果不一致（%s）。请对照照片选一个方案，或者正对积木的一个面重拍。'%(len(result['results']),'、'.join('%d 次：%s'%(c['votes'],c['check']['summary']) for c in candidates)))
            if failed:notes.append('%d 次解析没有返回结果（%s）'%(len(failed),failed[0][:60]))
            if answer.get('viewpoint')=='corner_on':notes.append('这张照片是对着积木的棱角斜拍的，前后左右的位置关系容易读错；正对一个面拍更可靠')
            if checked['ok']:
                for line in goal_spec.contact_mismatch(checked['spec'],answer.get('contacts')):notes.append('前后矛盾，请重点核对：'+line)
            if answer.get('image_usable') is False:notes.append('Gemini 认为这张照片不能可靠读出四块积木的摆放')
            confidence=answer.get('confidence')
            if isinstance(confidence,(int,float)) and confidence<.7:notes.append(f'Gemini 的把握度只有 {confidence:.2f}，请仔细核对')
            self._goal_log({'image':name,'called':GOAL_VOTES,'ok':checked['ok'],'agreement':agreement,'answers':answers,'errors':failed,'digest':checked.get('digest'),
                            'tokens':result['tokens'],'cost_upper_bound_usd':result['cost_upper_bound_usd'],'duration_s':result['duration_s']})
            self.goal_pending={'image':name,'image_size':meta,'answer':answer,'check':checked,'notes':notes,'time':time.time(),
                               'agreement':agreement,'readings':len(result['results']),'candidates':[{'check':c['check'],'votes':c['votes']} for c in candidates],
                               'duration_s':result['duration_s'],'tokens':result['tokens'],'cost_upper_bound_usd':result['cost_upper_bound_usd']}
            self.event('目标图已解析（Gemini %d 次，%.1f 秒，%s）：%s'%(len(result['results']),result['duration_s'],{'unanimous':'结果一致','majority':'多数一致','split':'结果不一致','none':'无有效结果'}[agreement],checked.get('summary') or '格式校验未通过'))
            return self.goal_state()
        finally:self.goal_lock.release()

    def goal_validate(self,body):
        from services.dashboard import goal_spec
        raw=goal_spec.from_cells(body.get('cells'),body.get('spacing','tight')) if 'cells' in body else body.get('spec')
        return goal_spec.check(raw)

    def goal_confirm(self,body):
        import re
        if self.mode=='run' and self.thread and self.thread.is_alive():raise RuntimeError('任务运行中不能更换目标')
        checked=self.goal_validate(body)
        if not checked['ok']:raise ValueError('；'.join(checked['errors']))
        image=body.get('image');source=body.get('source') if body.get('source') in ('gemini','manual') else 'manual'
        if not (isinstance(image,str) and re.fullmatch(GOAL_IMAGE_NAME,image) and (self.goal_dir/image).is_file()):image=None
        self.goal_dir.mkdir(parents=True,exist_ok=True)
        save(self.goal_dir/'active.json',{'spec':checked['spec'],'digest':checked['digest'],'source':source,'image':image,'confirmed_at':time.time()})
        self.goal_pending=None
        self.event('已确认搭建目标：'+checked['summary']+('' if self.goal_runnable() else '（本版本规划器暂不支持，不能运行）'))
        return self.goal_state()

    def goal_reset(self,body=None):
        if self.mode=='run' and self.thread and self.thread.is_alive():raise RuntimeError('任务运行中不能更换目标')
        (self.goal_dir/'active.json').unlink(missing_ok=True);self.goal_pending=None
        self.event('搭建目标已恢复为默认结构。')
        return self.goal_state()

    def goal_image(self,name):
        import re
        if not re.fullmatch(GOAL_IMAGE_NAME,name):raise KeyError('goal image')
        try:return (self.goal_dir/name).read_bytes()
        except OSError:raise KeyError('goal image') from None

    def submit(self,action,body):
        if action not in ('connect','prepare','run','stop','recover'): raise ValueError('未知操作')
        with self.lock:
            if action=='stop':
                self.cancel.set(); self.event('收到停止请求；调用原生 Stop，不自动回零或松爪。')
                if self.thread and self.thread.is_alive(): return {'accepted':True}
            elif self.thread and self.thread.is_alive(): raise RuntimeError('已有操作在执行，禁止重复启动')
            if action in ('connect','prepare','run') and self.recovery_required:
                raise ValueError('请先恢复连接并核对当前静止反馈')
            if action=='run':
                if body.get('scene_ready') is not True: raise ValueError('请确认已重新摆位、夹爪空载并可现场监督')
                if not self.prepared: raise ValueError('先使用 XPolicy 回 Prepare，再重新摆位')
                if not self.goal_runnable():
                    raise ValueError('已确认的搭建目标不是默认结构。本版本的规划器还不支持自定义目标，没有发送任何动作。请先“恢复默认目标”再运行。')
                try:self.validate_run_source()
                except Exception as exc:
                    self.error='执行前文件检查失败，保留当前连接与 Prepare：'+str(exc)
                    self.event(self.error)
                    raise ValueError(self.error) from exc
            if action!='stop': self.cancel.clear()
            self.error=None
            self.thread=threading.Thread(target=self._job,args=(action,body),daemon=True)
            self.thread.start()
        return {'accepted':True}

    def _job(self,action,body):
        if action=='recover':
            self.phase='恢复中 · 核对设备与静止反馈'
            try:self.recover()
            except Exception as exc:
                self.error=str(exc);self.phase='恢复未完成';self.event(self.error)
            return
        try:
            self.phase={'connect':'连接三路相机与只读反馈','prepare':'切换 XPolicy Prepare','run':'启动本轮硬件与深度服务','stop':'正在停止'}[action]
            if action=='run': self.verify_prepared()
            self.close_owner()
            if action=='stop': self.phase='已停止'; return
            self.prepared=False
            if self.cancel.is_set(): raise Cancelled('已取消')
            if action=='connect':
                self.mode='preview'; out=self.fresh_output('preview')
                self.launch(self.root/'scripts/piper_preview.py',out,'--seconds','3600')
                def ready():
                    try:
                        s=self.owner(); return s if s.get('robot_state',{}).get('available') and all(s.get('cameras',{}).get(c,{}).get('available') for c in ('head','left','right')) else None
                    except Exception: return None
                self.wait(ready,65,'三路相机连接超时'); self.phase='只读预览'; self.event('三路相机与状态已连接；未使能电机。')
            elif action=='prepare':
                self.mode='prepare'; out=self.fresh_output('prepare')
                self.launch(self.root/'scripts/piper_prepare.py',out,'--seconds','3600')
                self.wait(lambda:read(out/'READY.json'),65,'Prepare 服务未就绪')
                if self.cancel.is_set(): raise Cancelled()
                start=time.monotonic(); self.operator().command('prepare'); self.event('已发送 XPolicy 原生 Prepare。')
                samples=[]; last=None
                def prepared():
                    nonlocal last
                    from services.piper.owner import OwnerReader
                    try: s=OwnerReader().state()
                    except (ValueError,OSError): return False
                    stamp=s['robot_state']['feedback_metadata']['source_timestamp_ns']
                    good=at_prepare(s,read(out/'PLAN.json',{})['native_prepare'])
                    # Native lifecycle owns jaw targets/calibration; do not issue an extra close command.
                    if not good: samples.clear(); return False
                    if stamp!=last: samples.append(stamp); last=stamp
                    return s if len(samples)>=4 and samples[-1]-samples[0]>=300_000_000 else False
                s=self.wait(prepared,90,'Prepare 到位超时')
                save(out/'WEB-PREPARE-RESULT.json',{'ok':True,'elapsed_s':time.monotonic()-start,'measured14':s['robot_state']['values']})
                self.prepared=True; self.phase='Prepare 已到位'; self.event('已回零并使用原生闭爪目标；可重新摆位。')
            else: self.run_once(body)
        except BaseException as exc:
            self.error=f'{type(exc).__name__}: {exc}'; self.event(self.error)
            try: self.close_owner()
            except Exception as stop_exc: self.recovery_required=True; self.error+='；停止结果不确定：'+str(stop_exc); self.event(str(stop_exc))
            self.prepared=False; self.phase='已中断' if isinstance(exc,Cancelled) else '需要处理'
            if self.active_record:
                self.active_record.update(outcome='interrupted' if isinstance(exc,Cancelled) else 'failed',error=self.error)
                self.finish_record()

    def run_once(self,body):
        meta=read(self.candidate/'WORKSPACE.json',{}) | read(self.candidate/'output/composition/VERIFICATION.json',{})
        if meta.get('aplus_test_snapshot_ready') is not True:raise RuntimeError('A+ 固定测试版尚未完成核验，禁止执行')
        commit=subprocess.check_output(['git','-C',str(self.candidate/'code/magnetic_blocks'),'rev-parse','HEAD'],text=True).strip()
        if subprocess.check_output(['git','-C',str(self.candidate/'code/magnetic_blocks'),'status','--porcelain','--untracked-files=no'],text=True).strip():
            raise RuntimeError('工作副本有未提交代码；先完成验证和版本记录')
        if commit!=meta.get('commit'): raise RuntimeError('工作副本版本元数据不匹配')
        out=self.fresh_output('main'); self.mode='run'; self.started=time.time()
        goal=self.goal_active()
        self.active_record={'id':out.name,'layout':'初始视觉定位中','version':meta.get('candidate_revision'),'commit':commit,
            'started':self.started,'output':str(out),'outcome':'running','human_assisted':False,
            'goal_digest':goal['digest'],'goal_summary':goal['summary'],'goal_source':goal['source'],'goal_image':goal['image']}
        save(self.data/(out.name+'.json'),self.active_record)
        self.launch(self.root/'scripts/piper_explore.py',out,'--task-config',str(self.root/'examples/tasks/magnetic-blocks-l-stack/session.json'),'--rehearsal-candidate',str(self.candidate),'--direct-model','--http-proxy','http://127.0.0.1:7892','--seconds','1800')
        ready=out/'REHEARSAL-READY.json'; self.wait(lambda:read(ready),300,'硬件/深度启动超时')
        if self.cancel.is_set(): raise Cancelled()
        self.phase='运行 '+meta['candidate_revision']+' main(ctx)'; self.event('启动一次 '+meta['candidate_revision']+' main(ctx)，完成全任务预检后执行，不启动主 Agent。')
        with (self.data/(out.name+'-runner.log')).open('x') as log:
            self.runner=subprocess.Popen([self.python,str(self.root/'services/dashboard/blocks_runner.py'),'--ready',str(ready),'--run'],cwd=self.root,stdout=log,stderr=log,start_new_session=True)
        self.wait(lambda:read(out/'REHEARSAL-DONE.json'),1800,'整段运行超时')
        report=controller_report(out); self.phase='停止并收尾'; self.runner.wait(timeout=15)
        self.process.wait(timeout=90)
        stop=read(out/'STOP-RESULT.json',{})
        service_failure=read(out/'RESULT.json',{}).get('error')
        if service_failure and 'service_connection' in (report.get('why') or ''):
            report['why']+='；设备退出原因：'+service_failure
        clean=stop.get('disarmed') is True and (stop.get('rest') or {}).get('measured_rest') is True and not stop.get('errors')
        self.active_record.update(outcome='success' if report.get('ok') is True and clean else 'failed',
            elapsed_s=report.get('elapsed_s'),completion_mode=report.get('completion_mode'),terminal_release_hold=report.get('terminal_release_hold'),stages=report.get('steps',[]),main_agent_calls=report.get('main_agent_calls'),
            auxiliary_visual_calls=report.get('auxiliary_visual_calls'),wrist_depth_calls=report.get('wrist_depth_calls'),
            failed_step=report.get('failed_step'),why=report.get('why'),failure_category=report.get('failure_category'),stop_confirmed=clean,metrics_complete=report.get('metrics_complete',False))
        self.finish_record()
        if not clean:
            self.recovery_required=True
            raise RuntimeError('停止结果不确定：'+str(stop.get('errors') or '缺少静止回执'))
        self.guard_file.unlink(missing_ok=True)
        self.process=None; self.runner=None; self.mode='disconnected'
        self.phase=('黄色已释放、未退开，已停止持位' if report.get('completion_mode')=='released_in_place' else '任务成功，已停止') if report.get('ok') and clean else '运行失败，检查报告'
        self.event(self.phase+'；当前位置不是 Prepare。')

    def finish_record(self):
        self.active_record['finished']=time.time()
        self.active_record['total_wall_s']=self.active_record['finished']-self.active_record['started']
        # Partial / cancelled runs retain completed-stage timings, never silently become successes.
        report=controller_report(Path(self.active_record['output']))
        layout=initial_layout(Path(self.active_record['output']))
        self.active_record.update(layout=layout_label(layout),initial_localization=layout)
        for key,source in [('elapsed_s','elapsed_s'),('stages','steps'),('main_agent_calls','main_agent_calls'),
                           ('auxiliary_visual_calls','auxiliary_visual_calls'),('failed_step','failed_step'),('failure_category','failure_category'),('metrics_complete','metrics_complete')]:
            if key not in self.active_record and source in report: self.active_record[key]=report[source]
        save(self.data/(self.active_record['id']+'.json'),self.active_record)
        self.active_record=None

    def records(self):
        rows=[read(p,{}) for p in self.data.glob('web-main-*.json')]
        for row in rows:
            if row.get('outcome')=='running' and row.get('output'):
                layout=initial_layout(Path(row['output']))
                if layout: row.update(layout=layout_label(layout),initial_localization=layout)
        for row in rows:
            if row.get('outcome')=='failed' and row.get('output'):
                folder=Path(row['output'])
                done=folder/'REHEARSAL-DONE.json'
                if done.exists():
                    stamp=done.stat().st_mtime_ns
                    cached=self.finished_detail_cache.get(folder)
                    if cached is None or cached[0]!=stamp:
                        detail=controller_report(folder)
                        cached=(stamp,{k:detail.get(k) for k in ('why','failure_category')})
                        self.finished_detail_cache[folder]=cached
                    detail=cached[1]
                else:detail=controller_report(folder)
                if detail.get('why'): row['why']=detail['why']
                if detail.get('failure_category'):row['failure_category']=detail['failure_category']
        return sorted(rows,key=lambda r:r.get('started',0),reverse=True)[:200]

    def snapshot(self):
        busy=bool(self.thread and self.thread.is_alive()); connected=bool(self.process and self.process.poll() is None)
        if not connected: self.prepared=False
        report=controller_report(self.out) if self.out else {}
        telemetry={}; telemetry_error=None
        if connected:
            try:
                s=self.owner(); telemetry={k:s.get(k) for k in ('robot_state','robot_status','robot_status_age_seconds','cameras')}
            except Exception as exc: telemetry_error=str(exc)
        meta=read(self.candidate/'WORKSPACE.json',{}) | read(self.candidate/'output/composition/VERIFICATION.json',{})
        if meta.get('aplus_test_snapshot_ready') is not True:raise RuntimeError('A+ 固定测试版尚未完成核验，禁止执行')
        phase,mode=self.phase,self.mode
        if not busy and not connected and mode=='prepare' and self.out and not self.error:
            result=read(self.out/'RESULT.json',{})
            if result.get('expired') and (result.get('rest') or {}).get('measured_rest') and not result.get('stop_error'):
                phase,mode='Prepare 会话已到期并已停止，可重新回到 Prepare','disconnected'
        goal=self.goal_active()
        return {'goal':{'summary':goal['summary'],'digest':goal['digest'],'source':goal['source'],'is_default':goal['is_default'],'runnable':self.goal_runnable(goal)},
            'recovery_required':self.recovery_required,'phase':phase,'busy':busy,'connected':connected,'prepared':self.prepared,'mode':mode,
            'error':self.error,'telemetry_error':telemetry_error,'telemetry':telemetry,'report':report,
            'events':self.events.copy(),'records':self.records(),'output':str(self.out) if self.out else None,
            'version':meta.get('candidate_revision'),'commit':meta.get('commit'),'ik_solver':meta.get('ik_solver','kdl'),
            'gripper_speed_mm_s':25,
            'sdk_speed_percent':80 if meta.get('motion_speed_profile')=='faster' else 10,
            'motion_tracking_tolerance_rad':meta.get('motion_tracking_tolerance_rad'),
            'joint_arrival_tolerance_rad':meta.get('joint_arrival_tolerance_rad'),
            'offline_replay_scenes':meta.get('offline_replay_scenes'),'offline_layouts':meta.get('offline_passed_layouts'),'offline_layout_total':meta.get('offline_layout_count'),'write_controls_available':True,'entrypoint':'assembly_controller.main',
            'console':tail(self.out/'rax-session/agent-workspace/output/merged-console.log') if self.out else '',
            'startup_log':(tail(self.data/(self.out.name+'.log'),4000)+'\n'+tail(self.out/'graph.log',3000)) if self.out else ''}

    def camera(self,name):
        if name not in ('head','left','right'): raise ValueError('Unknown camera')
        s=self.owner()
        from services.piper.owner import OwnerReader,check_camera_metadata
        check_camera_metadata(name,s['cameras'][name])
        return OwnerReader().get('/api/camera/'+name+'.jpg')

class ScaleHandler(Handler):
    def send_header(self,name,value):
        if name.lower()=='content-security-policy': value=value.replace("img-src 'self'", "img-src 'self' blob:")
        super().send_header(name,value)
    def log_message(self,*args): pass  # Never log a token-bearing URL.
    def _goal_body(self):
        # Same JSON-only rule as _body, with room for one downscaled photo.
        if self.headers.get('Content-Type','').split(';')[0].strip()!='application/json':raise ValueError('POST bodies must be application/json')
        length=int(self.headers.get('Content-Length') or 0)
        if length>5*1024*1024:raise ValueError('图片太大，请缩小后再上传')
        value=json.loads(self.rfile.read(length) or b'{}')
        if not isinstance(value,dict):raise ValueError('expected a JSON object')
        return value
    def do_GET(self):
        path,query=self._route()
        if path=='/' and secrets.compare_digest((query.get('token') or [''])[0],self.token):
            self._send(303,b'','text/plain',[('Location','/'),('Set-Cookie',f'{COOKIE}={self.token}; HttpOnly; SameSite=Strict; Path=/')]); return
        if not self._authorized(): self._error(401,'使用 Mac 上的启动链接打开控制台'); return
        def handle():
            if path in ('/','/scale.js'):
                self._send(200,(STATIC/('scale.html' if path=='/' else 'scale.js')).read_bytes(),'text/html; charset=utf-8' if path=='/' else 'application/javascript; charset=utf-8')
            elif path=='/api/health': self._json(200,{'busy':bool(self.manager.thread and self.manager.thread.is_alive()),'phase':self.manager.phase})
            elif path=='/api/status': self._json(200,self.manager.snapshot())
            elif path=='/api/goal': self._json(200,self.manager.goal_state())
            elif path.startswith('/api/goal/image/'): self._send(200,self.manager.goal_image(path.rsplit('/',1)[-1]),'image/jpeg')
            elif path.startswith('/api/camera/') and path.endswith('.jpg'): self._send(200,self.manager.camera(path.split('/')[-1][:-4]),'image/jpeg')
            elif path=='/api/runs.csv':
                f=io.StringIO(); fields=['id','layout','initial_localization_json','version','commit','outcome','completion_mode','elapsed_s','total_wall_s','stage_timings_json','main_agent_calls','auxiliary_visual_calls','failed_step','failure_category','why','stop_confirmed','metrics_complete']
                writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore'); writer.writeheader()
                for r in self.manager.records():
                    row={k:r.get(k) for k in fields}
                    row['initial_localization_json']=json.dumps(r.get('initial_localization'),ensure_ascii=False)
                    row['stage_timings_json']=json.dumps([{'stage':s.get('stage'),'elapsed_s':s.get('stage_elapsed_s'),'ok':s.get('ok')} for s in r.get('stages',[])],ensure_ascii=False)
                    # Spreadsheet formula escaping for operator-provided layout descriptions.
                    for k,v in row.items():
                        if isinstance(v,str) and v.startswith(('=','+','-','@','\t','\r')): row[k]="'"+v
                    writer.writerow(row)
                self._send(200,f.getvalue().encode('utf-8-sig'),'text/csv; charset=utf-8',[('Content-Disposition','attachment; filename="rax-runs.csv"')])
            else: self._error(404,'not found')
        self._guarded(handle)
    def do_POST(self):
        if not self._authorized(): self._error(401,'operator token required'); return
        path,_=self._route()
        if path in ('/api/goal/analyze','/api/goal/validate','/api/goal/confirm','/api/goal/reset'):
            self._guarded(lambda:self._json(200,getattr(self.manager,'goal_'+path.rsplit('/',1)[-1])(self._goal_body()))); return
        if not path.startswith('/api/action/'): self._error(404,'not found'); return
        self._guarded(lambda:self._json(202,self.manager.submit(path.rsplit('/',1)[-1],self._body())))

def make_server(manager,port=8786,token=None):
    token=token or secrets.token_urlsafe(32)
    cls=type('BoundScaleHandler',(ScaleHandler,),{'manager':manager,'token':token})
    return ThreadingHTTPServer(('127.0.0.1',port),cls),token

class ExistingPrepareProcess:
    def __init__(self,pid,out):
        self.pid=pid; self.out=out
        self.start=(Path('/proc')/str(pid)/'stat').read_text().split()[21]
    def poll(self):
        try:
            base=Path('/proc')/str(self.pid)
            if (base/'stat').read_text().split()[21]!=self.start:return 0
            args=(base/'cmdline').read_bytes().split(b'\0')
            scripts={b'scripts/piper_prepare.py',str(ROOT/'scripts/piper_prepare.py').encode()}
            return None if any(arg in scripts for arg in args) and str(self.out).encode() in args else 0
        except (OSError,IndexError):return 0
    def send_signal(self,sig):
        if self.poll() is None:os.kill(self.pid,sig)
    def wait(self,timeout):
        until=time.monotonic()+timeout
        while self.poll() is None:
            if time.monotonic()>until:raise TimeoutError('Original Prepare supervisor did not exit')
            time.sleep(.1)
        return 0

def adopt_prepare(manager,out):
    out=out.resolve()
    if not out.is_relative_to(ROOT/'commissioning'):raise ValueError('Invalid Prepare session')
    r=read(out/'READY.json',{});proc=ExistingPrepareProcess(r['pid'],out)
    if proc.poll() is not None:raise ValueError('Prepare session exited')
    manager.out=out;manager.process=proc;manager.mode='prepare'
    manager.verify_prepared()  # authenticates original owner; read-only
    manager.prepared=True;manager.phase='Prepare 已到位'
    manager.event('已绑定本次 XPolicy Prepare 会话，未发送动作。')

def main():
    p=argparse.ArgumentParser(); p.add_argument('--port',type=int,default=8786); p.add_argument('--adopt-prepare',type=Path); a=p.parse_args()
    manager=Manager(); lock=(manager.data/'server.lock').open('w'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if a.adopt_prepare: adopt_prepare(manager,a.adopt_prepare)
    server,token=make_server(manager,a.port,token=(manager.data/'access-token').read_text().strip() if (manager.data/'access-token').exists() else None)
    (manager.data/'access-token').write_text(token); (manager.data/'access-token').chmod(0o600)
    print(f'RAX scale-test console listening on loopback:{a.port}; no hardware started',flush=True)
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        manager.cancel.set()
        if manager.thread and manager.thread.is_alive(): manager.thread.join(timeout=100)
        if manager.thread and manager.thread.is_alive(): raise RuntimeError('操作线程尚未结束，请核对原生 Stop')
        manager.close_owner(); server.server_close()
if __name__=='__main__': main()
