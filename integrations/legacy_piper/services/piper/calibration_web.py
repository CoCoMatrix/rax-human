"""Local operator UI. HTTP to formal owner only; no SDK/device/enable endpoint."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler

from .calibration_control import COMMANDS, MOVE_COMMANDS, ROTATE_COMMANDS, ROTATION_DEG

ROOT = Path(__file__).resolve().parents[2]


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('Owner redirect forbidden')


class Owner:
    def __init__(self, token):
        self.token = token
        self.session = hashlib.sha256(token.encode()).hexdigest()
        self.http = build_opener(ProxyHandler({}), NoRedirect())

    def call(self, path, body=None):
        headers = {'X-XPolicy-Operator-Token': self.token}
        data = None if body is None else json.dumps(body).encode()
        if data is not None:
            headers['Content-Type'] = 'application/json'
        with self.http.open(Request('http://127.0.0.1:8091'+path, data=data, headers=headers),timeout=2) as response:
            if response.headers.get('X-XPolicy-Operator-Session') != self.session:
                raise ValueError('Owner session changed; reload, never replay')
            data = response.read(4_000_001)
            if len(data)>4_000_000:
                raise ValueError('Owner response oversized')
            return data

    def state(self):
        return json.loads(self.call('/api/state'))

    def command(self, command):
        return json.loads(self.call('/api/command',dict(command=command,source='keyboard_ui')))


class Bridge:
    def __init__(self, owner, output, depth, intrinsics, board):
        self.owner, self.output, self.depth = owner, output, depth
        self.intrinsics, self.board = intrinsics, board
        self.lock = threading.RLock()
        self.ordinary = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.seen = set()
        self.capture_job = {'state':'idle'}
        self.pending = None
        self.events = []
        self.output.mkdir(parents=True, exist_ok=True)

    def record(self, event, **fields):
        row = dict(time_ns=time.time_ns(), event=event, **fields)
        with self.lock:
            self.events.append(row)
            self.events = self.events[-20:]
            with (self.output/'audit.jsonl').open('a') as stream:
                stream.write(json.dumps(row,ensure_ascii=False)+'\n')

    def samples(self):
        return [json.loads(p.read_text()) for p in sorted(self.output.glob('samples/*/sample.json'))
                if (p.parent/'ACCEPTED.json').exists()]

    def state(self):
        error=''; state={}
        try:
            state=self.owner.state()
            r=state.get('robot_status',{}); c=r.get('calibration_control',{})
            age=state.get('robot_status_age_seconds')
            if type(age) not in (int,float) or not 0<=age<.6:
                raise ValueError('Owner 状态过期')
            if c.get('schema')!='rax.piper.calibration_control.v1':
                raise ValueError('连接的不是标定 owner，拒绝控制')
            with self.lock:
                if self.pending:
                    receipt=c.get('last_attempt') or c.get('last_command') or {}
                    if receipt.get('timestamp_ns',0)>=self.pending['time_ns']:
                        self.record('owner_rejected' if c.get('error') else 'owner_observed', control=c)
                        self.pending=None
                    elif time.time_ns()-self.pending['time_ns']>5_000_000_000:
                        self.record('unconfirmed_timeout',command=self.pending['command'])
                        self.pending=None
            if r.get('feedback_error'):
                error='硬件反馈不可用：'+str(r['feedback_error'])[:200]
        except Exception as exc:
            error=str(exc)[:250]
        with self.lock:
            rows=self.samples()
            return dict(connected=not error,error=error,owner=state,session=self.owner.session,
                capture=self.capture_job,pending=self.pending,events=list(self.events),
                counts={k:sum(v['split']==k for v in rows) for k in ('fit','heldout')},
                board_verified=self.board.exists(),hardware_validated=False)

    def submit(self, p):
        command=p.get('command')
        if command not in COMMANDS|{'capture','verify_board'}:
            raise ValueError('Unknown command')
        # Hold and disable never wait behind capture or another network request.
        urgent=command in {'stop','cal_disable_left','cal_restore_feedback'}
        acquired=False
        if not urgent:
            acquired=self.ordinary.acquire(blocking=False)
            if not acquired:
                raise ValueError('正在处理请求；不会排队')
        try:
            with self.lock:
                rid=p.get('request_id')
                if not isinstance(rid,str) or not 16<=len(rid)<=100 or rid in self.seen:
                    raise ValueError('Invalid or reused request ID')
                if p.get('session')!=self.owner.session:
                    raise ValueError('页面会话已变化；刷新后人工操作')
                if len(self.seen)>=10000:
                    raise ValueError('Session request limit reached')
                self.seen.add(rid)
                if command=='verify_board':
                    if self.samples() or self.capture_job['state']=='running':
                        raise ValueError('采集开始后不能更改标定板')
                    if p.get('horizontal_mm')!=100 or p.get('vertical_mm')!=100 or p.get('rigid_fixed') is not True:
                        raise ValueError('需确认横、纵各四格实测 100 mm，板已平整刚性固定')
                    original=ROOT/'commissioning/workflow-audit-20261027/print-board-plain/board.json'
                    config=json.loads(original.read_text()); config['print_scale_verified']=True
                    self.board.write_text(json.dumps(config,sort_keys=True,indent=2))
                    self.record('board_measured',horizontal_mm=100,vertical_mm=100,rigid_fixed=True)
                    return {'ok':True}
            state=self.state()
            urgent_owner=(urgent and state.get('owner',{}).get('robot_status',{}).get(
                'calibration_control',{}).get('schema')=='rax.piper.calibration_control.v1')
            recover_hold=(command=='cal_enable_hold' and state.get('owner',{}).get('robot_status',{}).get(
                'calibration_control',{}).get('enable_hold_supported') is True
                and type(state.get('owner',{}).get('robot_status_age_seconds')) in (int,float)
                and 0<=state['owner']['robot_status_age_seconds']<.6)
            if not state['connected'] and not urgent_owner and not recover_hold:
                raise ValueError(state['error'])
            s=state['owner']; r=s['robot_status']; c=r['calibration_control']
            if command!='capture' and s.get('write_enabled') is not True:
                raise ValueError('只读模式，控制按钮未授权')
            if not urgent:
                if state['pending'] or state['capture']['state']=='running':
                    raise ValueError('等待当前请求结束')
                if p.get('generation')!=r.get('generation'):
                    raise ValueError('状态已变化；检查后重新点击')
                manual_capture=(command=='capture' and c.get('phase')=='DISABLED'
                    and c.get('workflow')!='drag_hold_capture'
                    and c.get('disable_latched') is True and c.get('left_enable_bits')==[False]*6
                    and not c.get('error') and not c.get('enable_error'))
                if manual_capture and p.get('arm_supported') is not True:
                    raise ValueError('人工采集时需确认左臂仍被可靠承托')
                if command!='cal_enable_hold' and not manual_capture and r.get('hardware_write_enabled') is True and (c.get('phase')!='HOLD' or c.get('disable_latched') or c.get('left_enable_bits')!=[True]*6 or c.get('error')):
                    raise ValueError('先 Hold，等待停稳和新鲜反馈')
            if command=='cal_enable_hold':
                if c.get('enable_hold_supported') is not True or c.get('enable_hold_stage'):
                    raise ValueError('当前 owner 不支持此操作或正在恢复使能')
                if p.get('arm_supported') is not True or p.get('workspace_checked') is not True:
                    raise ValueError('恢复使能前请继续承托，并确认现场空间和急停可触达')
            if command=='cal_drag_left':
                if c.get('drag_supported') is not True or c.get('hold_verified') is not True or c.get('drag_error'):
                    raise ValueError('当前 owner 未确认原位 Hold，不能进入拖动')
                if p.get('arm_supported') is not True or p.get('workspace_checked') is not True:
                    raise ValueError('先承托左臂，并确认现场空间和急停可触达')
            if (command=='stop' and c.get('workflow')=='drag_hold_capture'
                    and c.get('controller_mode')!=1 and p.get('arm_supported') is not True):
                raise ValueError('退出拖动前请继续承托左臂，直到显示已锁定')
            if command=='cal_disable_left' and p.get('arm_supported') is not True:
                raise ValueError('先用可靠支撑承托左臂，并确认')
            if command=='cal_restore_feedback':
                if c.get('feedback_recovery_supported') is not True:
                    raise ValueError('当前 owner 不支持失能恢复反馈')
                if p.get('arm_supported') is not True:
                    raise ValueError('恢复反馈前必须可靠承托左臂')
            if command.startswith(('cal_jog_','cal_move_','cal_wrist_')) and p.get('workspace_checked') is not True:
                raise ValueError('需确认现场空间和急停可触达')
            if command.startswith(('cal_move_','cal_wrist_')) and c.get('cartesian_jog_supported') is not True:
                raise ValueError('Owner 版本不支持方向点动；需使用新版标定会话')
            if command in MOVE_COMMANDS:
                direction,encoded_step=MOVE_COMMANDS[command]
                if p.get('orientation_mode','fixed')!='fixed':
                    raise ValueError('平移必须保持末端朝向')
                step=p.get('step_mm',encoded_step)
                if type(step) is not int or step not in (2,5,10):
                    raise ValueError('平移步长只能选择 2、5、10 mm')
                if command != f'cal_move_{direction}' and step != encoded_step:
                    raise ValueError('步长与指令不一致')
                if step not in c.get('translation_steps_mm',[2]):
                    raise ValueError('Owner 版本不支持所选步长；请刷新新版标定会话')
                command=f'cal_move_{direction}' if step==2 else f'cal_move_{direction}_{step}mm'
            if command in ROTATE_COMMANDS:
                joint,sign,encoded_step=ROTATE_COMMANDS[command]
                step=p.get('step_deg',encoded_step)
                if type(step) not in (int,float) or step not in ROTATION_DEG:
                    raise ValueError('旋转步长只能选择 0.5、2、5、10 度')
                base=command.rsplit('_',1)[0] if command.endswith('deg') else command
                if command.endswith('deg') and step != encoded_step:
                    raise ValueError('旋转步长与指令不一致')
                if step not in c.get('rotation_steps_deg',[.5]):
                    raise ValueError('Owner 版本不支持所选旋转步长；请刷新新版标定会话')
                command=base if step==.5 else f'{base}_{step:g}deg'
            if command=='capture':
                if c.get('workflow')=='drag_hold_capture' and c.get('capture_ready') is not True:
                    raise ValueError('先 Hold 锁定当前位置并等待停稳；拖动中不能采集')
                if not self.board.exists():
                    raise ValueError('先完成标定板尺寸和安装确认')
                split=p.get('split')
                if split not in {'fit','heldout'}:
                    raise ValueError('选择拟合或验证样本')
                with self.lock:
                    self.capture_job={'state':'running','split':split,'started_ns':time.time_ns()}
                self.pool.submit(self.capture, split)
                return {'ok':True,'execution_confirmed':False}
            with self.lock:
                self.pending={'command':command,'time_ns':time.time_ns()}
                if command=='cal_drag_left':
                    self.capture_job={'state':'idle'}
            self.record('intent',command=command,request_id=rid)
            reply=self.owner.command(command)
            if reply.get('ok') is not True:
                raise ValueError('Owner did not acknowledge; do not retry automatically')
            self.record('queued',command=command,physical_execution_confirmed=False)
            return {'ok':True,'execution_confirmed':False}
        except Exception as exc:
            if command=='capture':
                with self.lock:
                    if self.capture_job['state']!='running':
                        self.capture_job=dict(state='failed',error=str(exc)[:350])
            raise
        finally:
            if acquired:self.ordinary.release()

    def capture(self, split):
        try:
            from .handeye import capture
            from .owner import OwnerReader
            import numpy as np
            out=self.output/'samples'/str(time.time_ns())
            rows=self.samples()
            reader=OwnerReader()
            # Preserve strict CAN schema/age checks AND pin every observation
            # to this UI's authenticated owner session, including during capture.
            reader.get=lambda path,limit=4_000_000:self.owner.call(path)
            result=capture(self.depth,self.intrinsics,self.board,out,split,
                allow_calibration_hold=True,allow_manual_disabled=True,owner_reader=reader)
            if result['reprojection_max_px']>2:
                raise ValueError('标定板重投影误差超过 2 px；检查清晰度、平整度和内参')
            G=np.array(result['T_base_flange'])
            for row in rows:
                previous=np.array(row['T_base_flange'])
                if (np.linalg.norm(G[:3,3]-previous[:3,3])<.01 and
                    np.linalg.norm(G[:3,:3]-previous[:3,:3])<.05):
                    raise ValueError('与已有样本过近；改变位置和倾角再采')
            (out/'ACCEPTED.json').write_text(json.dumps({'accepted_for_solve':True,'physical_acceptance':False}))
            with self.lock:
                self.capture_job=dict(state='done',split=split,path=str(out),corners=result['corners'],
                    reprojection_max_px=result['reprojection_max_px'])
            self.record('sample_saved',path=str(out),split=split)
        except Exception as exc:
            with self.lock:self.capture_job=dict(state='failed',error=str(exc)[:350])
            self.record('capture_rejected',error=str(exc)[:350])


def handler(bridge, csrf):
    page=Path(__file__).with_name('calibration_web.html').read_text()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,body,mime='application/json'):
            self.send_response(code)
            for key,value in {'Content-Type':mime,'Content-Length':str(len(body)),
                'Cache-Control':'no-store','X-Frame-Options':'DENY',
                'X-Content-Type-Options':'nosniff','Content-Security-Policy':
                "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self'; frame-ancestors 'none'; base-uri 'none'"}.items():
                self.send_header(key,value)
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def js(self,code,obj):self.send(code,json.dumps(obj,ensure_ascii=False).encode())
        def do_GET(self):
            if self.headers.get('Host') not in {f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'}:
                self.js(403,{'error':'Loopback Host required'});return
            try:
                path=self.path.partition('?')[0]
                if path=='/':self.send(200,page.replace('__CSRF__',csrf).encode(),'text/html; charset=utf-8')
                elif path=='/api/state':self.js(200,bridge.state())
                elif path=='/api/camera/head.jpg':self.send(200,bridge.owner.call(path),'image/jpeg')
                else:self.js(404,{'error':'Not found'})
            except Exception as exc:self.js(503,{'error':str(exc)[:250]})
        def do_POST(self):
            if (self.headers.get('Host') not in {f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'} or
                    self.headers.get('Origin')!='http://'+self.headers.get('Host','') or
                    self.headers.get('X-Calibration-Token')!=csrf):
                self.js(403,{'error':'Origin/session mismatch'});return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if self.path!='/api/command' or not 1<=size<=2048:
                    raise ValueError('Invalid request')
                self.js(200,bridge.submit(json.loads(self.rfile.read(size))))
            except Exception as exc:self.js(409,{'error':str(exc)[:350]})
    return Handler


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('token-file','output','depth-root','intrinsics'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--port',type=int,default=18879)
    a=p.parse_args()
    if not a.output.resolve().is_relative_to(ROOT/'commissioning'):
        p.error('Output must be under commissioning')
    token=a.token_file.read_text().strip()
    if not 24<=len(token)<=256:raise ValueError('Invalid owner token')
    bridge=Bridge(Owner(token),a.output,a.depth_root,a.intrinsics,a.output/'board.json')
    server=ThreadingHTTPServer(('127.0.0.1',a.port),handler(bridge,secrets.token_urlsafe(32)))
    print(f'CALIBRATION_WEB http://127.0.0.1:{a.port}',flush=True)
    try:server.serve_forever()
    finally:server.server_close();bridge.pool.shutdown(wait=True)


if __name__=='__main__':main()
