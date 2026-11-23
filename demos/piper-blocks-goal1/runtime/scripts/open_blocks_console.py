#!/usr/bin/env python3
"""Open the blocks console. Never sends Prepare/Run or takes over hardware."""
from pathlib import Path
import os,socket,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
PY='/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/.venv/bin/python'
def main():
 with socket.socket() as s:running=s.connect_ex(('127.0.0.1',8797))==0
 data=ROOT/'runtime/blocks-scale-tests';data.mkdir(exist_ok=True)
 if not running:
  with (data/'server.log').open('a') as f:
   p=subprocess.Popen([PY,'-m','services.dashboard.blocks_scale','--port','8797'],cwd=ROOT,env={**os.environ,'PYTHONPATH':str(ROOT),'PYTHONDONTWRITEBYTECODE':'1'},stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)
  until=time.monotonic()+10
  while time.monotonic()<until:
   if p.poll() is not None:raise RuntimeError('Console startup failed; inspect server.log')
   with socket.socket() as s:
    if s.connect_ex(('127.0.0.1',8797))==0:break
   time.sleep(.1)
  else:raise TimeoutError('Console not ready')
 token=(data/'access-token').read_text().strip()
 with (data/'browser.log').open('a') as f:
  subprocess.Popen(['/usr/bin/google-chrome','--user-data-dir='+str(ROOT/'runtime/blocks-web-browser'),'--no-first-run','--new-window','http://127.0.0.1:8797/?token='+token],stdout=f,stderr=f,start_new_session=True)
if __name__=='__main__':main()
