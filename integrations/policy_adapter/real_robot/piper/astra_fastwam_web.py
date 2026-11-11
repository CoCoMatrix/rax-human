"""Read-only preparation console; Stop forwards only the formal owner's HOLD."""
import argparse,json,os,re,secrets,signal,socket,subprocess,threading,time,urllib.request,urllib.error
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

GAP=Path('/home/ubuntu/code_junqi/GPT-as-Policy')
ROOT=Path('/home/ubuntu/code_junqi/migration/astra-fastwam-20260920')
OWNER='http://127.0.0.1:8091'
def read(path,default=None):
    try:return json.loads(Path(path).read_text())
    except (OSError,json.JSONDecodeError):return {} if default is None else default
def get(path):
    with urllib.request.urlopen(OWNER+path,timeout=2) as r:return r.read(),r.headers.get('Content-Type','application/octet-stream')
def newest():
    active=read(ROOT/'active-run.json')
    if active.get('agent_dir'):
        p=Path(active['agent_dir'])
        if p.is_relative_to(ROOT):return p
    paths=[p for p in ROOT.glob('agent-shadow-*') if p.is_dir()]
    return max(paths,key=lambda p:p.stat().st_mtime) if paths else None
def events(path):
    if not path:return []
    out=[]
    for line in (path/'events.jsonl').read_text().splitlines()[-150:] if (path/'events.jsonl').exists() else []:
        try:r=json.loads(line)
        except json.JSONDecodeError:continue
        if any(s in str(r.get('event','')).lower() for s in ('reasoning','analysis','raw_rpc')):continue
        out.append(r)
    return out

class Console:
    def __init__(self):self.proc=None;self.lock=threading.Lock();self.last_stop=None
    def state(self):
        try:owner=json.loads(get('/api/state')[0]);error=''
        except Exception as e:owner={};error=type(e).__name__
        try:
            with socket.create_connection(('127.0.0.1',19023),timeout=.3):online=True
        except OSError:online=False
        run=newest()
        active=read(ROOT/'active-run.json');live=active.get('authorized') is True and not active.get('cancelled')
        return {'dispatch_locked':not live,'mode':'live' if live else 'prepare_only','owner':owner,'owner_error':error,
            'model_online':online,'model_readiness':read(ROOT/'model/readiness.json'),
            'run':str(run) if run else None,'agent_status':read(run/'status.json') if run else {},
            'result':read(run/'result.json') if run else {},'failure':read(run/'failure.json') if run else {},
            'events':events(run),'last_stop':self.last_stop,
            'source_note':('本轮使用正式 owner 的当前原始 RGB 与关节反馈；Astra 审阅期间保持 HOLD。' if live else '三路画面是当前实时预览；准备验证使用 9 月 18 日原始 1280×720 RGB 回放。'),
            'task':'把桌上的棕色瓶装水拿起来，放到杯垫上。'}
    def stop(self):
        # This is the only owner POST allowed by this console. It never exposes
        # prepare/start/home, task changes, SDK calls, or a write-unlock API.
        page=get('/')[0].decode();match=re.search(r'const token = ("[^"\n]+");',page)
        if not match:raise ValueError('Owner stop token unavailable')
        req=urllib.request.Request(OWNER+'/api/command',data=json.dumps({'command':'stop','source':'keyboard_ui'}).encode(),headers={'Content-Type':'application/json','X-XPolicy-Operator-Token':json.loads(match[1])},method='POST')
        with urllib.request.urlopen(req,timeout=3) as response:receipt=json.load(response)
        if self.proc and self.proc.poll() is None:self.proc.terminate()
        active=read(ROOT/'active-run.json')
        if active.get('authorized'):
            active['cancelled']=True;active['phase']='operator_stopped'
            (ROOT/'active-run.json').write_text(json.dumps(active,ensure_ascii=False,indent=2))
            for key in ('agent_pid','relay_pid'):
                pid=active.get(key)
                if type(pid) is int:
                    try:
                        cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
                        if 'real_robot.piper.astra_fastwam' in cmd and str(active.get('run_id','')) in cmd:os.kill(pid,signal.SIGTERM)
                    except (OSError,ProcessLookupError):pass
        self.last_stop={'time_ns':time.time_ns(),'owner_response':receipt,'meaning':'HOLD at current position; no disable'}
        return self.last_stop

def make_handler(console,token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,status,data,kind):
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def json(self,value,status=200):self.send(status,json.dumps(value,ensure_ascii=False).encode(),'application/json; charset=utf-8')
        def do_GET(self):
            path=self.path.partition('?')[0]
            try:
                if path=='/':self.send(200,(GAP/'real_robot/piper/web_fastwam.html').read_text().replace('__TOKEN__',json.dumps(token)).encode(),'text/html; charset=utf-8')
                elif path=='/api/state':self.json(console.state())
                elif path in ('/api/camera/head.jpg','/api/camera/left.jpg','/api/camera/right.jpg'):
                    data,kind=get(path);self.send(200,data,kind)
                else:self.json({'error':'not found'},404)
            except Exception as exc:self.json({'error':type(exc).__name__},503)
        def do_POST(self):
            if not secrets.compare_digest(self.headers.get('X-Junqi-Astra-Token',''),token):self.json({'error':'invalid request token'},403);return
            try:
                if self.path=='/api/stop':self.json(console.stop())
                else:self.json({'error':'Motion is locked. Waiting for the user command.'},403)
            except Exception as exc:self.json({'error':str(exc)},503)
    return Handler

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8095);a=p.parse_args()
    ROOT.mkdir(parents=True,exist_ok=True)
    server=ThreadingHTTPServer(('127.0.0.1',a.port),make_handler(Console(),secrets.token_urlsafe(32)))
    print(f'Astra + FastWAM preparation console http://127.0.0.1:{a.port}; motion locked',flush=True)
    server.serve_forever()
if __name__=='__main__':main()
