"""Replay the R16 delayed J4 receipt without sending robot actions."""
import copy,time
import pytest
from test_piper_visual_facts import perception,rehearsal_rig
from test_cup_approach import rehearsal


def rig_case(delta=.011):
    q=[0.,.0856607596878817,-.1635897107894285,.07620107514207242,0.,0.,.0156]+[0.]*6+[.001]
    now=q.copy();now[3]=q[3]+delta
    obs={'frame_id':'receipt','measured14':q,'state_binding':{'owner_generation':1,'owner_unchanged':True,'rest_consistent':True},
         'cameras':{k:{'source_timestamp_ns':time.time_ns(),'metric_depth':k=='head'} for k in ('head','left','right')}}
    fresh=copy.deepcopy(obs);fresh.update(frame_id='rebound',measured14=now,observation_settling={'window_s':1.05,'samples':21,'max_range':0.,'max_range_limit':.0003})
    robot={'motion_ready':True,'motion_enabled':True,'arming':{'armed':True},'owner_status':{'generation':1},
           'bridge':{'phase':'DONE','instance':'same'},'measured_state':{'values':now.copy()},
           'last_motion':{'executed':True,'measured_arrival':True,'measured_settled':True,'owner_generation':1}}
    class Rig:
        def __init__(self):self.calls=[];self.error=None;self.changed=None
        def value(self,name,**kw):
            self.calls.append(name)
            if name=='status':return copy.deepcopy({'control_uncertain':False,'robot_status':robot})
            assert name=='piper_observe', 'Recovery cannot dispatch or call models'
            assert kw=={'cameras':['head','left','right'],'require_depth':True}
            if self.error:raise ValueError(self.error)
            if self.changed:self.changed()
            return copy.deepcopy(fresh)
    return Rig(),obs,fresh,robot


def test_feedback_beyond_binding_reacquires_without_replaying(perception):
    rig,old,fresh,robot=rig_case()
    new,event=perception.rebind_observation(rig,old)
    assert new['measured14'][3]==pytest.approx(.07620107514207242+.011)
    assert old['measured14'][3]==.07620107514207242 # old evidence not rewritten
    assert event['feedback_delta_max']==pytest.approx(.011)
    assert event['reason']=='feedback_changed' and not event['motion_replayed']
    assert not event['scene_facts_reused']
    assert rig.calls==['status','piper_observe','status']


def test_bound_fresh_receipt_does_not_duplicate_capture(perception):
    rig,old,fresh,robot=rig_case();robot['measured_state']['values']=old['measured14'].copy()
    new,event=perception.rebind_observation(rig,old)
    assert new is old and event is None and rig.calls==['status']


@pytest.mark.parametrize('failure',['fault','disarmed','uncertain_receipt','owner','unstable','target_error','clock','drift_again','restarted','no_rest','stale_frame','motion_replaced'])
def test_failed_rebind_stops_without_retry_or_motion(perception,failure):
    rig,old,fresh,robot=rig_case()
    if failure=='fault':robot['bridge']['phase']='FAULT'
    if failure=='disarmed':robot['arming']['armed']=False
    if failure=='uncertain_receipt':robot['last_motion']['measured_arrival']=False
    if failure=='owner':robot['owner_status']['generation']=2
    if failure in ('unstable','target_error','clock'):rig.error='Backend '+failure+' guard refused'
    if failure=='drift_again':fresh['measured14'][3]+=.011
    if failure=='restarted':rig.changed=lambda:robot['bridge'].update(instance='new')
    if failure=='no_rest':fresh.pop('observation_settling')
    if failure=='stale_frame':fresh['cameras']['head']['source_timestamp_ns']-=2_000_000_000
    if failure=='motion_replaced':rig.changed=lambda:robot['last_motion'].update(owner_request_id='other')
    with pytest.raises(ValueError):perception.rebind_observation(rig,old)
    assert rig.calls.count('piper_observe')<=1


def test_localization_uses_rebound_pixels_and_records_recovery(perception,monkeypatch,tmp_path):
    import numpy as np,cv2
    rig,old,fresh,robot=rig_case()
    im=tmp_path/'head.jpg';cv2.imwrite(str(im),np.zeros((10,10,3),np.uint8))
    fresh['cameras']['head']['inspection_image']=str(im)
    monkeypatch.setattr(perception,'_contours',lambda image:np.array([[1,1,1],[8,8,1]]))
    monkeypatch.setattr(perception,'_track_scene',lambda *args:({'cup':[1,1,1],'coaster':[8,8,1]},[5,5]))
    frames=[]
    def measure(ctx,obs,*args):
        frames.append(obs['frame_id']);return {'ok':True,'near_wall':[0,0,0],'coaster_center':[1,1,1]}
    monkeypatch.setattr(perception,'_measure_scene',measure)
    r=perception.localize_supported(rig,{'observation':old,'previous':{'before_frame':'earlier','near_wall':[0,0,0],'coaster_center':[1,1,1]}})
    assert r['ok'],r
    assert frames==['rebound'] and r['visual_calls']==0
    assert r['observation_rebinding']['reason']=='feedback_changed'


def test_snapshot_uses_actual_schema_once_and_preserves_full_evidence(rehearsal_rig,tmp_path):
    import json
    m,rig=rehearsal_rig
    r=m.snapshot(rig,{})
    assert r['read_only'] and rig.moves==0
    assert [n for n,k in rig.calls]==['status','piper_observe']
    assert r['measured14']==rig.obs['measured14']
    assert r['owner_generation']==1 and r['bridge_phase']=='DONE'
    assert r['image_paths']['head']==rig.obs['cameras']['head']['inspection_image']
    from pathlib import Path
    saved=json.loads(Path(r['report_path']).read_text())
    assert saved['observation']==rig.obs and saved['status']['robot_status']
    assert r['historical_report_is_current_scene_evidence'] is False


def test_child_summary_keeps_failures_without_repeating_large_data(rehearsal):
    r={'steps':[{'function':'segment','ok':False,'executed':None,'reason':'unknown completion',
        'report_path':'saved.json','next_action':'observe_only','wall_budget':{'huge':'x'*10000},
        'args':{'huge':'x'*10000},'image_paths':{'huge':'x'*10000}}]}
    result=rehearsal.compact(r)
    assert len(str(result))<500
    assert result['steps'][0]['executed'] is None and result['steps'][0]['next_action']=='observe_only'


def test_recorded_j4_change_is_now_within_observation_binding(perception):
    rig,old,fresh,robot=rig_case(.00178023583703422)
    new,event=perception.rebind_observation(rig,old)
    assert new is old and event is None and rig.calls==['status']


@pytest.mark.parametrize('index,delta,ok',[(3,.01,True),(3,.01001,False),(10,.01,True),(6,.0005,True),(6,.000501,False),(13,.001,False)])
def test_observation_binding_keeps_joint_and_jaw_units_separate(perception,rehearsal,index,delta,ok):
    a=[0.]*14;b=a.copy();b[index]=delta
    assert perception.observation_pose_matches(a,b) is ok
    assert rehearsal._same_pose(a,b) is ok
