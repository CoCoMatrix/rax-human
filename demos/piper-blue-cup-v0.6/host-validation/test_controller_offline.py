"""Offline new-scene IK/path checks with synthetic motion receipts. No hardware."""
import sys,json,copy,time
from pathlib import Path
from types import SimpleNamespace
import pytest
ROOT=Path(__file__).parent/'runtime'
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'workspaces/blue-cup/trajectory-v06/code/blue_cup'))
import trajectory_task as task
import scheduled,approach_target,execution,controller,perception
from services.piper.backend import PiperBackend
from services.piper.moveit import IKNoSolution

@pytest.fixture
def rig(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path)
    saved=json.loads((Path(__file__).parent/'recorded-target.json').read_text());obs=copy.deepcopy(saved['source_observation'])
    (tmp_path/'output/rehearsal').mkdir(parents=True)
    class FakeContext:
        def __init__(self):
            self.q=obs['measured14'].copy();self.last=None;self.calls=[];self.scene=0
            opts={'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36],'collision_legacy_root':'/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025'}
            self.backend=PiperBackend(SimpleNamespace(root=tmp_path/'solver',backend_options=opts),owner=SimpleNamespace(),cameras=SimpleNamespace())
        def robot(self):return dict(motion_ready=True,motion_enabled=True,arming={'armed':True},bridge={'phase':'DONE','instance':'test'},owner_status={'generation':1},measured_state={'values':self.q.copy()},last_motion=self.last,workspace_min=[.055,-.4,.17],workspace_max=[.5,.045,.36])
        def value(self,name,**args):
            self.calls.append((name,copy.deepcopy(args)))
            if name=='piper_status':return self.robot()
            if name=='status':return dict(robot_status=self.robot(),wall_budget={'remaining_seconds':999})
            if name in ('piper_fk','piper_check_path','piper_solve_ik'):
                try:return self.backend.call(name,args)[0]
                except IKNoSolution:raise RuntimeError('MOVEIT_NO_IK_SOLUTION')
            if name in ('piper_move_joints','piper_gripper','piper_close_gripper','piper_move_waypoints'):
                if name=='piper_move_joints':self.q[:6]=args['joints']
                elif name in ('piper_gripper','piper_close_gripper'):self.q[6]=args['width_m']
                else:self.q[:6]=args['waypoints'][-1]
                self.last=dict(executed=True,measured_arrival=True,measured_settled=True,owner_generation=1,final14=self.q.copy(),target14=self.q.copy(),owner_request_id=str(len(self.calls)))
                return self.last
            raise AssertionError('Unexpected tool '+name)
    ctx=FakeContext()
    def localize(ctx,inputs):
        ctx.scene+=1
        return dict(ok=True,observation=obs,**copy.deepcopy(saved['geometry']),contours=saved['source_contours'],before_frame=obs['frame_id'])
    def create(loc,robot,phase):
        # Test fixture only: historical measurement is explicitly synthetic scene input.
        t=copy.deepcopy(saved);t.update(valid=True,contact_started=False,phase=phase,owner_generation=1,bridge_instance='test',expected14=ctx.q.copy(),last_motion=None,created_ns=time.time_ns(),geometry={k:loc[k] for k in ('near_wall','cup_center_estimate','coaster_center','table')})
        approach_target.save(t);return t
    monkeypatch.setattr(task,'localize_supported',localize);monkeypatch.setattr(task,'observation_recent',lambda *a,**k:True)
    monkeypatch.setattr(task,'create',create);monkeypatch.setattr(task,'assess',lambda *a,**k:{'ok':True})
    monkeypatch.setattr(task,'context_for',lambda *a,**k:{})
    yield ctx,saved,localize
    ctx.backend.close()

@pytest.mark.parametrize('invalid_preplan',[False,True])
@pytest.mark.parametrize('shift',[0,.005,-.005])
def test_new_layout_plans_two_real_ik_paths(rig,monkeypatch,shift,invalid_preplan):
    ctx,saved,loc=rig
    def changed(c,i):
        result=loc(c,i)
        for k in ('near_wall','cup_center_estimate','coaster_center'):result[k][0]+=shift
        return result
    monkeypatch.setattr(task,'localize_supported',changed)
    prep=task.prepare_and_enter(ctx);assert prep['ok'],prep.get('reason')
    assert ctx.scene==1
    calls=[args for name,args in ctx.calls if name=='piper_move_waypoints'];assert len(calls)==1 and len(calls[0]['waypoints'])>=3
    assert prep['localization']['near_wall'][0]==saved['geometry']['near_wall'][0]+shift
    closed=scheduled.grasp(ctx);assert closed['ok'],closed.get('reason')
    assert len([1 for name,args in ctx.calls if name=='piper_close_gripper'])==1
    entry_q=ctx.q.copy();fk=ctx.value('piper_fk',joints=entry_q[:6],arm='left')
    # Synthetic trial receipt uses actual MoveIt IK for 40mm upward goal.
    liftq=ctx.value('piper_solve_ik',position=[*fk['position'][:2],fk['position'][2]+.04],quaternion_xyzw=fk['quaternion_xyzw'],seed=entry_q[:6])['q']
    ctx.q[6]=.003;receipt=ctx.value('piper_move_joints',joints=liftq)
    path=Path('lift.json');path.write_text(json.dumps(dict(ok=True,function='trial_lift',receipt=receipt)))
    target=scheduled.localize_coaster(ctx,dict(preparation_report=prep['report_path'],entry_report=prep['report_path'],lift_report=str(path)));assert target['ok'],target.get('reason')
    if invalid_preplan:
        p=approach_target.PATH;d=json.loads(p.read_text());d['transfer_preplan']['lift_joints'][0]+=.02;p.write_text(json.dumps(d))
    ik_before=len([1 for name,_ in ctx.calls if name=='piper_solve_ik'])
    result=task.transfer_and_lower(ctx,dict(target_report=target['report_path'],previous_report=target['report_path']));assert result['ok'],result.get('reason')
    assert result['transfer_plan_reused'] is (not invalid_preplan)
    ik_after=len([1 for name,_ in ctx.calls if name=='piper_solve_ik'])
    assert (ik_after==ik_before) is (not invalid_preplan)
    assert len([1 for name,args in ctx.calls if name=='piper_move_waypoints'])==2
    assert result['support_state']=='predicted_height_reached' and result['support_visually_verified'] is False
    released=scheduled.release(ctx,dict(width_m=.012,target_report=target['report_path'],previous_report=result['report_path']));assert released['ok'],released.get('reason')


def test_ik_failure_never_dispatches_arm_chunk(rig,monkeypatch):
    ctx,_,_=rig
    def reject(*a,**k):raise ValueError('IK fixture rejection')
    monkeypatch.setattr(task,'solve_leg',reject)
    result=task.prepare_and_enter(ctx);assert not result['ok']
    assert not any(name=='piper_move_waypoints' for name,args in ctx.calls)


def test_main_stops_at_failed_trial_without_transfer(rig,monkeypatch):
    ctx,_,_=rig;calls=[]
    def success(name):
        def f(*a,**k):calls.append(name);return dict(ok=True,report_path='unused')
        return f
    monkeypatch.setattr(controller,'prepare_and_enter',success('approach'))
    monkeypatch.setattr(controller,'grasp',success('grasp'))
    monkeypatch.setattr(controller,'trial_lift',lambda *a,**k:dict(ok=False,why='uncertain pickup'))
    monkeypatch.setattr(controller,'transfer_and_lower',lambda *a,**k:pytest.fail('Transferred after failed lift'))
    result=controller.main(ctx);assert not result['ok'] and result['failed_step']=='trial_lift' and result['held_object_possible']
