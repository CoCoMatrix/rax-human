import json,shutil,hashlib
from pathlib import Path
root=Path('/home/ubuntu/code_junqi/rax-piper-franka-20261104');dest=root/'candidates/blue-cup/20261104-v0.2-candidate/validation/live-rehearsal';dest.mkdir(parents=True,exist_ok=True);rows=[]
for number in (1,2,3):
 source=root/'commissioning'/f'blue-cup-v02-rehearsal-20261104-r{number}';out=dest/f'r{number}';out.mkdir(exist_ok=True);w=source/'rax-session/agent-workspace'
 for name in ('RESULT.json','STOP-RESULT.json','REHEARSAL-DONE.json','OPERATOR-CONFIRMATION.json','CANDIDATE-VERSION.json'):
  if (source/name).is_file():shutil.copy2(source/name,out/name)
 for dirname in ('rehearsal','maneuvers'):
  if (w/'output'/dirname).is_dir():shutil.copytree(w/'output'/dirname,out/dirname,dirs_exist_ok=True)
 ids=set()
 def frames(value):
  if isinstance(value,dict):
   for k,v in value.items():
    if k in ('frame_id','before_frame','after_frame') and isinstance(v,str) and len(v)==32:ids.add(v)
    frames(v)
  elif isinstance(value,list):
   for v in value:frames(v)
 for path in sorted((out/'rehearsal').glob('*.json')):
  d=json.load(open(path));frames(d)
  if not isinstance(d,dict) or 'name' not in d:continue
  v=d.get('result',{});timing=v.get('timing_s',{})
  rows.append({'session':f'r{number}','step':d['name'],'function':d.get('function'),'started_at':d.get('started_at'),'elapsed_s':d.get('elapsed_s'),'ok':d.get('ok'),'executed':v.get('executed',False),'measured_arrival':v.get('receipt',{}).get('measured_arrival',False),'motion_and_observation_s':timing.get('motion_and_observation',0),'vision_s':timing.get('vision_before',0)+timing.get('vision_after',0),'issue':d.get('error') or v.get('reason') or (v.get('result_check') or v.get('precondition') or v.get('check') or {}).get('reason'),'operator_confirmation_used':d.get('operator_confirmation_used',False),'report':str(path.relative_to(dest))})
 for ident in ids:
  f=w/'runtime/frames'/ident
  if f.is_dir():
   (out/'frames'/ident).mkdir(parents=True,exist_ok=True)
   for n in ('head.png','left.png','right.png','meta.json'):
    if (f/n).is_file():shutil.copy2(f/n,out/'frames'/ident/n)
 for n in ('mcp.jsonl','frontend.log'):
  if (source/'rax-session'/n).is_file():shutil.copy2(source/'rax-session'/n,out/n)
prepare=root/'commissioning/xpolicy-prepare-v02-handoff-20261104';pd=dest/'prepare';pd.mkdir(exist_ok=True)
for name in ('PREPARE-CONFIRMED.json','CAD-HOME-CHECK.json','PLAN.json','READY.json'):
 shutil.copy2(prepare/name,pd/name)
pr=json.load(open(prepare/'PREPARE-CONFIRMED.json'));ident=pr['observation']['frame_id'];shutil.copytree(prepare/'frames'/ident,pd/'frames'/ident,dirs_exist_ok=True)
summary={'outcome':'partial rehearsal stopped at user request; no grasp/lift/placement yet','physical_actions_with_arrival':sum(x['executed'] is True and x['measured_arrival'] for x in rows),'no_unattended_success':True,'operator_confirmation_used':True,'step_call_elapsed_sum_s':sum(x['elapsed_s'] or 0 for x in rows),'motion_and_observation_sum_s':sum(x['motion_and_observation_s'] for x in rows),'recorded_step_window_s':max(x['started_at']+x['elapsed_s'] for x in rows if x['started_at'])-min(x['started_at'] for x in rows if x['started_at']),'prepare_elapsed_s':pr['elapsed_s'],'rows':rows}
(dest/'TIMINGS.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False));print(json.dumps({k:v for k,v in summary.items() if k!='rows'},ensure_ascii=False));print('step_reports',len(rows))
