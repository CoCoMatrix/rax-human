import json,math
from pathlib import Path
from types import SimpleNamespace
import pytest
from test_piper_action_observation import rig,value
import controller as c
from rehearse_step import run_one

@pytest.fixture(autouse=True)
def artifact_scope(tmp_path,monkeypatch):
 monkeypatch.setenv('RAX_ARTIFACT_ROOT',str(tmp_path))

ROOT=Path('/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-C-speed02-20261113/runtime/commissioning/web-main-20261113-164307-a7fe0f')
RUN=next((ROOT/'rax-session/agent-workspace/output/composition/runs').iterdir())
FAILED=next(r for p in RUN.glob('*red-hover*.tools.jsonl') for r in map(json.loads,p.read_text().splitlines()) if r['tool']=='piper_move_joints')
REJECTION=json.loads(FAILED['error'].split('MCP tool failed: ',1)[1])['error']


def test_exact_recorded_red_hover_enters_bridge_once(tmp_path):
 frontend,bridge,events=rig(tmp_path)
 try:
  args=FAILED['arguments'];assert math.degrees(max(map(abs,args['joints'])))==pytest.approx(95.271332573)
  r=value(frontend.call('piper_move_joints',args))
  assert r.get('executed') and len(bridge.moves)==1 and bridge.moves[0][:6]==args['joints']
 finally:frontend.close()


@pytest.mark.parametrize('name',('piper_move_joints','piper_move_pose'))
@pytest.mark.parametrize('degrees,accepted',[(60,True),(95.271332573,True),(120,True),(120.001,False)])
def test_single_joint_and_ik_result_share_operator_scope(tmp_path,name,degrees,accepted):
 frontend,bridge,events=rig(tmp_path);q=[math.radians(degrees),0,0,0,0,0]
 frontend.backend.solver.solve=lambda *a:{'q':q}
 args={'joints':q} if name=='piper_move_joints' else {'position':[.057,0,.213]}
 try:
  r=value(frontend.call(name,args));assert bool(bridge.moves)==accepted
  if accepted:assert r['executed']
  else:assert r['error']['no_action_executed'] is True and not r['error']['control_uncertain']
 finally:frontend.close()


def context(error):
 calls=[]
 def value(tool,**kw):
  calls.append(tool)
  if tool=='status':return {'wall_budget':{'remaining_seconds':500,'finish_reserve_seconds':60,'phase':'active'},'control_uncertain':False,'robot_status':{'bridge':{'phase':'DONE'}}}
  if tool=='piper_hold':return {'stop_requested':True}
  raise error
 return SimpleNamespace(value=value),calls


def test_recorded_native_rejection_stays_certain_no_hold_and_correct_counters(tmp_path):
 exc=RuntimeError('MCP tool failed: '+json.dumps({'error':REJECTION}))
 ctx,calls=context(exc)
 def fn(ctx):return c._run(ctx,'red-hover',lambda report:c._move(ctx,'piper_move_joints',{'joints':FAILED['arguments']['joints']},report,observe_after=False))
 record=run_one(ctx,'red-hover',{},tmp_path/'run/report.json',operation=fn)
 r=record['result']
 assert not r['ok'] and r['pre_dispatch_rejected'] and not r['control_uncertain']
 assert r['failure_category']=='pre_dispatch_rejection' and not r['executed']
 assert 'piper_hold' not in calls and record['ambiguous_dispatch_count']==0
 assert record['motion_dispatch_count']==0 and record['motion_rpc_attempt_count']==1 and record['pre_dispatch_rejection_count']==1


@pytest.mark.parametrize('field,replacement',[('control_uncertain',True),('control_uncertain','false'),('executed',True),('dispatch_started',None),('no_action_executed',False),('type','RuntimeError')])
def test_ambiguous_error_never_treated_as_no_dispatch(tmp_path,field,replacement):
 e={**REJECTION,field:replacement};exc=RuntimeError('MCP tool failed: '+json.dumps({'error':e}));ctx,calls=context(exc)
 r=c._run(ctx,'test',lambda report:c._move(ctx,'piper_move_joints',{'joints':[0.]*6},report,observe_after=False))
 assert r['control_uncertain'] and 'piper_hold' in calls


@pytest.mark.parametrize('error',[TimeoutError('timeout'),RuntimeError('RejectedBeforeDispatch but no receipt'),RuntimeError('MCP tool failed: invalid-json')])
def test_transport_failure_retains_stop_and_uncertainty(error):
 ctx,calls=context(error)
 r=c._run(ctx,'test',lambda report:c._move(ctx,'piper_move_joints',{'joints':[0.]*6},report,observe_after=False))
 assert r['control_uncertain'] and 'piper_hold' in calls
