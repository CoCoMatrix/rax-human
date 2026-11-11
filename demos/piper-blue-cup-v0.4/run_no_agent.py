"""Run one controller against an existing frontend-only session; default prints a plan.

No owner/camera/arming/start/home calls. Host utility, never imported by controller.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys


def command(ready_file, runtime):
    ready=json.loads(ready_file.read_text());workspace=Path(ready['workspace']).resolve()
    sock=Path(ready['socket']).resolve();runtime=runtime.resolve()
    if ready.get('main_planning_model_started') is not False:
        raise ValueError('Require frontend-only session; do not run alongside an Agent')
    if not sock.is_socket():raise ValueError('Session socket unavailable; never resume old READY')
    if not (runtime/'bin/python').is_file():raise ValueError('Isolated agent Python missing')
    if not (workspace/'code/blue_cup/controller.py').is_file():raise ValueError('Merged controller missing')
    bwrap=shutil.which('bwrap')
    if not bwrap or sys.platform!='linux':raise ValueError('Linux bubblewrap required')
    cmd=[bwrap,'--die-with-parent','--new-session','--unshare-all','--clearenv']
    for name in ('/usr','/bin','/sbin','/lib','/lib64'):
        p=Path(name)
        if p.is_symlink():cmd+=['--symlink',os.readlink(p),name]
        elif p.exists():cmd+=['--ro-bind',name,name]
    cmd+=['--proc','/proc','--dev','/dev','--tmpfs','/tmp','--dir','/run','--dir','/home/agent',
          '--bind',str(workspace),str(workspace),'--ro-bind',str(sock),'/run/robot.sock',
          '--ro-bind',str(runtime),'/agent-runtime',
          '--setenv','HOME','/home/agent','--setenv','PATH','/agent-runtime/bin:/usr/bin:/bin',
          '--setenv','PYTHONPATH',str(workspace),'--setenv','PYTHONDONTWRITEBYTECODE','1',
          '--setenv','RAX_MCP_SOCKET','/run/robot.sock','--setenv','LANG','C.UTF-8',
          '--chdir',str(workspace),'/agent-runtime/bin/python','code/blue_cup/controller.py','--run']
    return cmd,workspace


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ready',type=Path,required=True)
    p.add_argument('--python-runtime',type=Path,default=Path('/home/ubuntu/code_junqi/rax-piper-agent-isolated'))
    p.add_argument('--run',action='store_true');a=p.parse_args();cmd,workspace=command(a.ready,a.python_runtime)
    if not a.run:print(json.dumps({'will_execute':False,'command':cmd}));return
    lock=workspace/'output/merged-running.lock';lock.parent.mkdir(exist_ok=True)
    fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600);os.write(fd,str(os.getpid()).encode());os.close(fd)
    # A stale lock requires inspection, never auto-clearing after an uncertain exit.
    def interrupted(*_):raise KeyboardInterrupt('Host runner interrupted; supervisor must Stop')
    signal.signal(signal.SIGTERM,interrupted)
    result={'ok':False,'why':'Controller interrupted before a complete report'}
    try:
        with (workspace/'output/merged-console.log').open('x') as log:
            proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
        reports=list((workspace/'output/rehearsal').glob('merged-*/report.json'))
        result=json.loads(max(reports,key=lambda f:f.stat().st_mtime).read_text()) if reports else {'ok':False,'why':'No controller report'}
        result['process_exit_code']=proc.returncode
        if proc.returncode:result['ok']=False
        result['execution_mode']='compiled_controller_no_agent'
    except BaseException as exc:
        result={'ok':False,'why':f'{type(exc).__name__}: {exc}', 'control_uncertain':True}
        raise
    finally:
        done=a.ready.parent/'REHEARSAL-DONE.json';tmp=done.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2));tmp.replace(done)
        if not result.get('control_uncertain'):lock.unlink(missing_ok=True)


if __name__=='__main__':main()
