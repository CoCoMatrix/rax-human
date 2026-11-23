"""Independent read-only numerical services; never connects to a hardware owner."""
from pathlib import Path
import json, os, subprocess, sys, time, uuid, threading, socket, signal

PURE = frozenset({'piper_fk','piper_fk_batch','piper_solve_ik','piper_solve_ik_fast',
                  'piper_solve_ik_candidates','piper_check_path'})

class PlanningPool:
    def __init__(self, backend):
        self.backend=backend; self.active=None

    def open(self, count):
        if type(count) is not int or not 1<=count<=4: raise ValueError('Planning workers must be1..4')
        if self.active is not None: raise ValueError('Close the previous planning pool first')
        root=Path(self.backend.config.root); ident=uuid.uuid4().hex
        rel=Path('runtime/planning-lanes')/ident
        directory=root/'agent-workspace'/rel;directory.mkdir(parents=True)
        opts={k:v for k,v in self.backend.config.backend_options.items() if k in
              ('ik_solver','moveit_manifest','workspace_min','workspace_max','collision_legacy_root')}
        if opts.get('ik_solver')!='ssik_rank1':raise ValueError('Parallel candidate requires unchanged SSIK solver')
        processes=[];self.active=(ident,directory,processes)
        try:
            for i in range(count):
                work=directory/str(i);work.mkdir()
                cfg={'root':str(root/'planning-workers'/ident/str(i)), 'socket_dir':str(work),
                     'options':opts,'artifact_root':os.environ.get('RAX_ARTIFACT_ROOT'),
                     'parent_pid':os.getpid(),'lifetime_s':210}
                (work/'input.json').write_text(json.dumps(cfg))
                with (work/'service.log').open('w') as log:
                    proc=subprocess.Popen([sys.executable,'-m','services.piper.planning_pool',str(work/'input.json')],
                        cwd=Path(__file__).resolve().parents[2],stdout=log,stderr=subprocess.STDOUT,
                        env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'},start_new_session=True)
                processes.append(proc)
            deadline=time.monotonic()+20
            while not all((directory/str(i)/'READY.json').exists() for i in range(count)):
                if any(p.poll() is not None for p in processes):raise RuntimeError('Numerical worker failed to start; see service.log')
                if time.monotonic()>deadline:raise TimeoutError('Numerical workers startup timeout')
                time.sleep(.03)
            lanes=[{'id':i,'socket':str(rel/str(i)/'math.sock'),
                    **json.loads((directory/str(i)/'READY.json').read_text())} for i in range(count)]
            return {'pool_id':ident,'lanes':lanes,'read_only':True,'independent_processes':True}
        except BaseException:
            self.close(ident);raise

    def close(self, ident=None):
        if self.active is None:return {'closed':True,'workers':[]}
        own,directory,processes=self.active
        if ident is not None and ident!=own:raise ValueError('Planning pool identity mismatch')
        for i in range(len(processes)):(directory/str(i)/'STOP').touch()
        deadline=time.monotonic()+4
        while any(p.poll() is None for p in processes) and time.monotonic()<deadline:time.sleep(.03)
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
        for p in processes:
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        rows=[]
        for i,p in enumerate(processes):
            f=directory/str(i)/'STATS.json'
            rows.append({'id':i,'pid':p.pid,'exit_code':p.returncode,**(json.loads(f.read_text()) if f.exists() else {'interrupted':True})})
        self.active=None
        return {'closed':True,'pool_id':own,'workers':rows}

def worker(path):
    from collections import Counter,defaultdict
    from services.controller.config import Configuration
    from services.controller.session import Session
    from services.controller.frontend import Frontend
    from services.piper.backend import PiperBackend
    from services.piper.installed_collision import load_checker,geometry_identity
    cfg=json.loads(Path(path).read_text());directory=Path(cfg['socket_dir'])
    if cfg['artifact_root']:os.environ['RAX_ARTIFACT_ROOT']=cfg['artifact_root']
    class NoOwner:
        def state(self):raise AssertionError('Planning worker cannot access hardware')
    config=Configuration(root=Path(cfg['root']),task='parallel_math',robot='piper',backend_options=cfg['options'])
    backend=PiperBackend(config,owner=NoOwner(),cameras=object())
    backend.solver.start();backend.collision=load_checker(cfg['options']['collision_legacy_root'])
    original=backend.call;tools=backend.tools;counts=Counter();durations=defaultdict(float)
    backend.tools=lambda:[t for t in tools() if t['name'] in PURE]
    def call(name,args):
        if name not in PURE:raise ValueError('Pure planning tool required: '+name)
        if 'solve_ik' in name and name!='piper_solve_ik_candidates' and 'seed' not in args:raise ValueError('Explicit seed required')
        started=time.monotonic();counts[name]+=1
        try:return original(name,args)
        finally:durations[name]+=time.monotonic()-started
    backend.call=call
    from services.controller.session import AUDIT_LIMIT_BYTES
    class MathSession(Session):
        # The private NoOwner worker exposes only PURE math. Actuator/owner sessions keep durable per-event writes.
        def __init__(self,config):
            super().__init__(config);self.pending=0;self.flushed=time.monotonic()
            self.log=os.fdopen(os.open(config.root/'mcp.jsonl',os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'wb')
        def sync_math(self):
            self.log.flush();os.fsync(self.log.fileno());self._save();self.pending=0;self.flushed=time.monotonic()
        def audit(self,event):
            raw=(json.dumps({'time':time.time(),**event},allow_nan=False,default=str)+'\n').encode()
            with self._lock:
                if self.state['audit_bytes']+len(raw)>AUDIT_LIMIT_BYTES:raise RuntimeError('Session audit budget exhausted')
                self.log.write(raw);self.state['audit_bytes']+=len(raw);self.pending+=1
                if self.pending>=32 or time.monotonic()-self.flushed>=.20:self.sync_math()
        def close(self):
            try:
                with self._lock:self.sync_math()
            finally:self.log.close();super().close()
    session=MathSession(config);frontend=Frontend(session,backend,started_at=time.time())
    os.chdir(directory);listener=socket.socket(socket.AF_UNIX);listener.bind('math.sock');listener.listen(8)
    stop=threading.Event();started=time.monotonic()
    def watchdog():
        while not stop.wait(.1):
            if (directory/'STOP').exists() or os.getppid()!=cfg['parent_pid'] or time.monotonic()-started>cfg['lifetime_s']:
                stop.set();return
    thread=threading.Thread(target=watchdog,daemon=True);thread.start()
    (directory/'READY.json').write_text(json.dumps({'pid':os.getpid(),'solver':backend.solver.name,
        'geometry_identity':geometry_identity(),'motion_enabled':False,'allowed_tools':sorted(PURE)}))
    try:frontend.serve_socket(listener,stop)
    finally:
        stop.set();listener.close();Path('math.sock').unlink(missing_ok=True)
        backend.close();session.close()
        (directory/'STATS.json').write_text(json.dumps({'calls':dict(counts),'tool_s':dict(durations),'hardware_dispatches':0}))

if __name__=='__main__':worker(sys.argv[1])
