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
STATIC=Path(__file__).parent/'static'
DEFAULT_CANDIDATE='workspaces/blue-cup/20261109-v0.4-merged-candidate'
from scripts.site_paths import path as site_path, value as site_value
PYTHON=str(site_path('host_python',sys.executable))

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

def controller_report(out):
    done=read(out/'REHEARSAL-DONE.json')
    if done: return done
    paths=list((out/'rax-session/agent-workspace/output/rehearsal').glob('merged-*/report.json'))
    return read(max(paths,key=lambda p:p.stat().st_mtime),{}) if paths else {}

def initial_layout(out):
    """Read the controller's already-measured target; never re-localize or drive it."""
    path=out/'rax-session/agent-workspace/output/rehearsal/approach-target.json'
    target=read(path,{})
    geometry=target.get('geometry',{})
    if not geometry: return None
    return {'source':'controller_initial_rgbd','source_frame':target.get('source_frame'),
        'frame':target.get('frame'),'measured_at_ns':target.get('created_ns'),
        'cup_rim_center_estimate_m':geometry.get('cup_center_estimate'),
        'near_wall_m':geometry.get('near_wall'),'coaster_center_m':geometry.get('coaster_center'),
        'table_point_m':geometry.get('table'),'artifact':str(path),
        'historical_measurement_only':True}

def layout_label(layout):
    if not layout: return '初始定位尚无结果'
    def xyz(v): return '('+', '.join(f'{x*1000:.1f}' for x in v)+') mm' if v else '未取得'
    return '杯口中心估计 '+xyz(layout.get('cup_rim_center_estimate_m'))+'；杯垫 '+xyz(layout.get('coaster_center_m'))+' · '+str(layout.get('frame'))

def at_prepare(state,profile):
    q=state['robot_state']['values']; pose=profile['initial_pose']
    target=pose['left_arm']+[pose['left_gripper']]+pose['right_arm']+[pose['right_gripper']]
    return state['robot_status'].get('lifecycle_state')=='prepared' and all(
        abs(a-b)<=(.0005 if i in (6,13) else .01) for i,(a,b) in enumerate(zip(q,target)))

class Cancelled(Exception): pass

class Manager:
    def __init__(self,root=ROOT,python=PYTHON):
        self.root=Path(root); self.python=python; self.candidate=self.root/DEFAULT_CANDIDATE
        self.data=self.root/'runtime/scale-tests'; self.data.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock(); self.cancel=threading.Event(); self.thread=None
        self.process=None; self.runner=None; self.out=None; self.mode='disconnected'
        self.phase='未连接'; self.error=None; self.events=[]
        self.started=None; self.active_record=None; self.prepared=False
        self.event('控制台已启动；硬件未连接，未发送动作。')

    def event(self,message):
        with self.lock:
            self.events.append({'time':time.time(),'message':str(message)})
            self.events=self.events[-150:]

    def launch(self,script,out,*args):
        out.parent.mkdir(parents=True,exist_ok=True)
        log=self.data/(out.name+'.log')
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
            if self.process and self.process.poll() is not None: raise RuntimeError('硬件会话已退出；'+tail(self.data/(self.out.name+'.log'),1800))
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
            if result.get('stop_error'): raise RuntimeError('停止未确认：'+result['stop_error'])
        self.process=None; self.runner=None; self.mode='disconnected'

    def submit(self,action,body):
        if action not in ('connect','prepare','run','stop'): raise ValueError('未知操作')
        with self.lock:
            if action=='stop':
                self.cancel.set(); self.event('收到停止请求；调用原生 Stop，不自动回零或松爪。')
                if self.thread and self.thread.is_alive(): return {'accepted':True}
            elif self.thread and self.thread.is_alive(): raise RuntimeError('已有操作在执行，禁止重复启动')
            if action=='run':
                if body.get('scene_ready') is not True: raise ValueError('请确认已重新摆位、夹爪空载并可现场监督')
                if not self.prepared: raise ValueError('先使用 XPolicy 回 Prepare，再重新摆位')
            if action!='stop': self.cancel.clear()
            self.error=None
            self.thread=threading.Thread(target=self._job,args=(action,body),daemon=True)
            self.thread.start()
        return {'accepted':True}

    def _job(self,action,body):
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
            except Exception as stop_exc: self.error+='；停止结果不确定：'+str(stop_exc); self.event(str(stop_exc))
            self.prepared=False; self.phase='已中断' if isinstance(exc,Cancelled) else '需要处理'
            if self.active_record:
                self.active_record.update(outcome='interrupted' if isinstance(exc,Cancelled) else 'failed',error=self.error)
                self.finish_record()

    def run_once(self,body):
        meta=read(self.candidate/'WORKSPACE.json',{})
        commit=subprocess.check_output(['git','-C',str(self.candidate/'code/blue_cup'),'rev-parse','HEAD'],text=True).strip()
        if subprocess.check_output(['git','-C',str(self.candidate/'code/blue_cup'),'status','--porcelain','--untracked-files=no'],text=True).strip():
            raise RuntimeError('工作副本有未提交代码；先完成验证和版本记录')
        if commit!=meta.get('code_commit'): raise RuntimeError('工作副本版本元数据不匹配')
        out=self.fresh_output('main'); self.mode='run'; self.started=time.time()
        self.active_record={'id':out.name,'layout':'初始视觉定位中','version':meta.get('candidate_version'),'commit':commit,
            'started':self.started,'output':str(out),'outcome':'running','human_assisted':False}
        save(self.data/(out.name+'.json'),self.active_record)
        self.launch(self.root/'scripts/piper_explore.py',out,'--rehearsal-candidate',str(self.candidate),'--direct-model','--http-proxy',site_value('vision_proxy',''),'--seconds','1800')
        ready=out/'REHEARSAL-READY.json'; self.wait(lambda:read(ready),300,'硬件/深度启动超时')
        if self.cancel.is_set(): raise Cancelled()
        self.phase='运行 main(ctx)'; self.event('启动一次 main(ctx)，不启动主 Agent。')
        with (self.data/(out.name+'-runner.log')).open('x') as log:
            self.runner=subprocess.Popen([self.python,str(self.candidate/'run_no_agent.py'),'--ready',str(ready),'--python-runtime',str(site_path('agent_runtime',self.root/'.venv-agent')),'--run'],cwd=self.root,stdout=log,stderr=log,start_new_session=True)
        self.wait(lambda:read(out/'REHEARSAL-DONE.json'),1800,'整段运行超时')
        report=controller_report(out); self.phase='停止并收尾'; self.runner.wait(timeout=15)
        self.process.wait(timeout=90)
        stop=read(out/'STOP-RESULT.json',{})
        clean=stop.get('disarmed') is True and stop.get('rest',{}).get('measured_rest') is True and not stop.get('errors')
        self.active_record.update(outcome='success' if report.get('ok') is True and clean else 'failed',
            elapsed_s=report.get('elapsed_s'),stages=report.get('steps',[]),main_agent_calls=report.get('main_agent_calls'),
            auxiliary_visual_calls=report.get('auxiliary_visual_calls'),wrist_depth_calls=report.get('wrist_depth_calls'),
            failed_step=report.get('failed_step'),why=report.get('why'),stop_confirmed=clean,metrics_complete=report.get('metrics_complete',False))
        self.finish_record(); self.process=None; self.runner=None; self.mode='disconnected'
        self.phase='任务成功，已停止' if report.get('ok') and clean else '运行失败，检查报告'
        self.event(self.phase+'；当前位置不是 Prepare。')

    def finish_record(self):
        self.active_record['finished']=time.time()
        self.active_record['total_wall_s']=self.active_record['finished']-self.active_record['started']
        # Partial / cancelled runs retain completed-stage timings, never silently become successes.
        report=controller_report(Path(self.active_record['output']))
        layout=initial_layout(Path(self.active_record['output']))
        self.active_record.update(layout=layout_label(layout),initial_localization=layout)
        for key,source in [('elapsed_s','elapsed_s'),('stages','steps'),('main_agent_calls','main_agent_calls'),
                           ('auxiliary_visual_calls','auxiliary_visual_calls'),('failed_step','failed_step'),('metrics_complete','metrics_complete')]:
            if key not in self.active_record and source in report: self.active_record[key]=report[source]
        save(self.data/(self.active_record['id']+'.json'),self.active_record)
        self.active_record=None

    def records(self):
        rows=[read(p,{}) for p in self.data.glob('web-main-*.json')]
        for row in rows:
            if row.get('outcome')=='running' and row.get('output'):
                layout=initial_layout(Path(row['output']))
                if layout: row.update(layout=layout_label(layout),initial_localization=layout)
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
        meta=read(self.candidate/'WORKSPACE.json',{})
        return {'phase':self.phase,'busy':busy,'connected':connected,'prepared':self.prepared,'mode':self.mode,
            'error':self.error,'telemetry_error':telemetry_error,'telemetry':telemetry,'report':report,
            'events':self.events.copy(),'records':self.records(),'output':str(self.out) if self.out else None,
            'version':meta.get('candidate_version'),'commit':meta.get('code_commit'),
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
    def do_GET(self):
        path,query=self._route()
        if path=='/' and secrets.compare_digest((query.get('token') or [''])[0],self.token):
            self._send(303,b'','text/plain',[('Location','/'),('Set-Cookie',f'{COOKIE}={self.token}; HttpOnly; SameSite=Strict; Path=/')]); return
        if not self._authorized(): self._error(401,'使用 Mac 上的启动链接打开控制台'); return
        def handle():
            if path in ('/','/scale.js'):
                self._send(200,(STATIC/('scale.html' if path=='/' else 'scale.js')).read_bytes(),'text/html; charset=utf-8' if path=='/' else 'application/javascript; charset=utf-8')
            elif path=='/api/status': self._json(200,self.manager.snapshot())
            elif path.startswith('/api/camera/') and path.endswith('.jpg'): self._send(200,self.manager.camera(path.split('/')[-1][:-4]),'image/jpeg')
            elif path=='/api/runs.csv':
                f=io.StringIO(); fields=['id','layout','initial_localization_json','version','commit','outcome','elapsed_s','total_wall_s','stage_timings_json','main_agent_calls','auxiliary_visual_calls','failed_step','stop_confirmed','metrics_complete']
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
        if not path.startswith('/api/action/'): self._error(404,'not found'); return
        self._guarded(lambda:self._json(202,self.manager.submit(path.rsplit('/',1)[-1],self._body())))

def make_server(manager,port=8781,token=None):
    token=token or secrets.token_urlsafe(32)
    cls=type('BoundScaleHandler',(ScaleHandler,),{'manager':manager,'token':token})
    return ThreadingHTTPServer(('127.0.0.1',port),cls),token

def main():
    p=argparse.ArgumentParser(); p.add_argument('--port',type=int,default=8781); a=p.parse_args()
    manager=Manager(); lock=(manager.data/'server.lock').open('w'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    server,token=make_server(manager,a.port)
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
