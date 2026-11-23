"""Measure read-only tools through the normal Frontend; never creates a motion bridge."""
import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from services.controller.config import Configuration
from services.controller.session import Session
from services.controller.frontend import Frontend
from services.piper.backend import PiperBackend

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=False);os.environ['RAX_ARTIFACT_ROOT']=str(a.output)
 config=Configuration(root=a.output/'session',task='tool-characterization',robot='piper',agent_provider='codex-relay',backend_options={'motion_enabled':False,'depth_root':str(a.source/'depth'),'collision_legacy_root':'/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025','stream_enabled':True})
 session=Session(config);backend=PiperBackend(config);f=Frontend(session,backend);rows=[];observations=[]
 def call(name,**args):
  t=time.monotonic();reply=f.call(name,args);v=json.loads(reply['content'][0]['text']);row={'tool':name,'args':args,'elapsed_s':time.monotonic()-t,'value':v,'images':len(reply['content'])-1};rows.append(row)
  (a.output/'calls.json').write_text(json.dumps(rows,indent=2));return v
 try:
  start=backend.owner.state();assert start['robot_status']['hardware_write_count']==0 and start['robot_status']['hardware_write_enabled'] is False
  q=start['robot_state']['values'];(a.output/'initial-state.json').write_text(json.dumps(start,indent=2))
  for _ in range(10):call('piper_status');call('status')
  for depth in [False,True,True,True]:
   obs=call('piper_observe',cameras=['head','left','right'],require_depth=depth)
   if 'error' not in obs:observations.append(obs)
  (a.output/'observations.json').write_text(json.dumps(observations,indent=2))
  for side,angles in [('left',q[:6]),('right',q[7:13])]:
   for _ in range(5):pose=call('piper_fk',joints=angles,arm=side)
   if side=='left':left_pose=pose
  for _ in range(6):call('piper_solve_ik',position=left_pose['position'],quaternion_xyzw=left_pose['quaternion_xyzw'],seed=q[:6])
  call('piper_solve_ik',position=[9,9,9],quaternion_xyzw=left_pose['quaternion_xyzw'],seed=q[:6])
  for _ in range(3):call('piper_check_path',start14=q,target14=q)
  if observations:
   obs=observations[-1];call('piper_pixel_to_position',frame_id=obs['frame_id'],camera='head',pixels=[[640,360]],radius=2)
   call('piper_pixel_to_position',frame_id=obs['frame_id'],camera='left',pixels=[[640,360]],radius=2)
  # Validate refusals only; no action tool is ever dispatched.
  for name,args in [('piper_move_joints',{'joints':q[:6]}),('piper_move_pose',{'position':left_pose['position']}),('piper_gripper',{'width_m':q[6]})]:
   try:backend.validate(name,args)
   except PermissionError as e:rows.append({'tool':name,'validation_only':True,'rejected':str(e)})
   else:raise AssertionError('NO_SEND motion validation unexpectedly allowed')
  call('piper_hold');time.sleep(12)
  now=time.time_ns()//1000000
  for camera in ['head','left','right']:call('piper_stream',camera=camera,since_ms=now-10000,until_ms=now,count=5)
  end=backend.owner.state();assert end['robot_status']['hardware_write_count']==0
  report={'hardware_writes':0,'physical_motion_test':False,'state_drift_max':float(np.max(np.abs(np.array(q)-end['robot_state']['values']))),'recording':backend.recorder.status(),'tools':{}}
  for name in sorted({r['tool'] for r in rows}):
   samples=[r for r in rows if r['tool']==name and 'elapsed_s' in r];times=[r['elapsed_s'] for r in samples]
   report['tools'][name]={'calls':len(samples),'errors':sum('error' in r['value'] for r in samples),'median_s':float(np.median(times)) if times else None,'p95_s':float(np.percentile(times,95)) if times else None,'max_s':max(times) if times else None}
  (a.output/'SUMMARY.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
 finally:backend.close();session.close()
if __name__=='__main__':main()
