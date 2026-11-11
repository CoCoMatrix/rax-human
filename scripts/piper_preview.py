#!/usr/bin/env python3
"""Bounded three-camera/feedback owner. No motor writes, FFS, policy or Agent."""
import argparse
import json
import os
import secrets
import signal
import subprocess
from pathlib import Path
import yaml
from piper_observe_only import ROOT, OWNER, GRAPH, DORA, make_graph, no_existing_devices

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=3600)
    p.add_argument('--run',action='store_true')
    a=p.parse_args(); out=a.output.resolve()
    if not out.is_relative_to(ROOT/'commissioning') or out.exists(): p.error('New commissioning directory required')
    if not 30<=a.seconds<=3600: p.error('seconds must be 30..3600')
    if not a.run: print(json.dumps({'hardware_write_enabled':False,'three_camera_preview':True})); return
    no_existing_devices(); out.mkdir(mode=0o700,parents=True)
    for name in ('private','tmp','cache'): (out/name).mkdir(mode=0o700)
    token=secrets.token_urlsafe(32)
    (out/'private/owner-token').write_text(token); (out/'private/owner-token').chmod(0o600)
    (out/'tasks.json').write_text(json.dumps({'canonical_prompts':['Read-only scale-test preview']}))
    graph=make_graph(yaml.safe_load(GRAPH.read_text()),out,token)
    for node in graph['nodes']:
        node['env'].update(TASK='Read-only scale-test preview',REALSENSE_STEREO_EXPORT_DIR='',REALSENSE_STEREO_ADAPTER_ROOT='',REALSENSE_STEREO_CALIBRATION='')
    (out/'graph.yml').write_text(yaml.safe_dump(graph)); (out/'graph.yml').chmod(0o600)
    env={k:v for k,v in os.environ.items() if not k.startswith(('PIPER_','RAX_','REALSENSE_','ENABLE_WRITE','POLICY_','ACTION_','XPOLICY_'))}
    env.update(PYTHONPATH=str(OWNER),PYTHONDONTWRITEBYTECODE='1')
    with (out/'graph.log').open('w') as log:
        child=subprocess.Popen([str(DORA),'run','--stop-after',f'{a.seconds}s',str(out/'graph.yml')],cwd=out,env=env,stdout=log,stderr=log,start_new_session=True)
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    try: child.wait()
    except KeyboardInterrupt: pass
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            child.wait(timeout=30)
        (out/'RESULT.json').write_text(json.dumps({'graph_exit_code':child.returncode,'hardware_write_enabled':False}))
    if child.returncode: raise SystemExit(child.returncode)
if __name__=='__main__': main()
