"""Offline contracts: new pixels, bounded subgoals, no implicit clearance/retry."""
import copy
import time
import numpy as np
import pytest
from test_piper_visual_facts import perception, rehearsal_rig
from test_cup_approach import rehearsal, scene, fk


def test_tracking_remeasures_current_contours_and_rejects_ambiguity(perception):
    image=np.zeros((240,400,3),np.uint8)
    import cv2
    cv2.circle(image,(90,100),40,(180,70,30),3)
    cv2.circle(image,(280,100),45,(30,170,210),3)
    old={'cup':[90,100,40],'coaster':[280,100,45]}
    current=np.roll(image,4,axis=1)
    circles=np.array([[94,100,40],[284,100,45]],float)
    result=perception.track_contours(image,current,old,circles)
    assert result['cup']==[94,100,40] and result['coaster']==[284,100,45]
    with pytest.raises(ValueError):
        perception.track_contours(image,current,old,np.vstack([circles,[99,100,40]]))
    with pytest.raises(ValueError):
        perception.track_contours(image,np.zeros_like(image),old,circles)


def test_observation_reuse_is_bound_to_source_age_and_same_state(rehearsal_rig):
    m,rig=rehearsal_rig
    obs=copy.deepcopy(rig.obs)
    obs['state_binding']['owner_generation']=1
    for c in obs['cameras'].values():c['source_timestamp_ns']=time.time_ns()
    e={'name':'numeric','tool':'piper_move_pose','args':{'delta':[0,0,.003]},
       'pre_facts':{},'post_facts':{},'observation':obs}
    r=m.move_once(rig,e)
    assert r['ok'],r
    assert not any(n=='piper_observe' for n,k in rig.calls)
    rig.calls.clear();obs['measured14'][0]=.1
    r=m.move_once(rig,e)
    assert not r['executed'] and rig.moves==1
    rig.calls.clear();obs['measured14'][0]=0
    for c in obs['cameras'].values():c['source_timestamp_ns']=time.time_ns()-60_000_000_000
    r=m.move_once(rig,e)
    assert not r['executed'] and rig.moves==1


def test_rgb_only_checks_do_not_force_depth(rehearsal_rig):
    m,rig=rehearsal_rig
    m.stage(rig,{'delta':[0,0,.003]})
    calls=[k for n,k in rig.calls if n=='piper_observe']
    assert calls and calls[0]['require_depth'] is False


@pytest.mark.parametrize('failure',[None,'uncertain','no_progress','scene_change','post_unknown','budget'])
def test_approach_completes_subgoal_or_stops_without_replaying(rehearsal,monkeypatch,failure):
    state={'p':np.array([.23,.5 if failure=='budget' else -.04,.29]),'n':0}
    actions=[];locations=[];verifications=[]
    from test_cup_retained_target import observed, robot_status
    q=scene()['observation']['measured14']
    def obs(depth=False):return observed(q,depth=depth,ident='frame-'+str(state['n']))
    def localize(ctx,inputs):
        locations.append(inputs)
        s=scene();s['observation']=obs(True)
        s['before_frame']=s['observation']['frame_id']
        s['visual_calls']=int(len(locations)==1);return s
    class Rig:
        def value(self,name,**kw):
            if name=='piper_fk':return fk(state['p'])
            if name=='status':return {}
            if name=='piper_status':return robot_status(q,state['n'])
            if name=='piper_observe':
                assert kw['require_depth'] is False
                return obs()
            raise AssertionError(name)
    def move(ctx,e):
        actions.append(e);state['n']+=1
        if failure=='scene_change':return {'ok':False,'executed':False,'reason':'Visible cup displacement'}
        if failure=='uncertain':
            return {'ok':False,'executed':None,'reason':'uncertain','visual_calls':1,'perception_s':.1,'motion_s':1,'planning_s':.01}
        if failure!='no_progress':state['p']=np.array(e['args']['position'])
        return {'ok':True,'executed':True,'motion_arrived':True,'visual_calls':1,'perception_s':.1,'motion_s':1,'planning_s':.01,
                'after_frame':obs()['frame_id'],'receipt':{'final14':q,'executed':True,'measured_arrival':True,'observation':obs()}}
    def visual(ctx,obs,question,**kw):
        verifications.append(kw)
        return {'ok':failure!='post_unknown','perception_s':.1,'frame_id':obs['frame_id']}
    monkeypatch.setattr(rehearsal,'localize_supported',localize)
    monkeypatch.setattr(rehearsal,'move_once',move)
    monkeypatch.setattr(rehearsal,'visual',visual)
    result=rehearsal.approach(Rig(),{'phase':'hover'})
    if failure:
        assert not result['ok'] and not result['task_completed'],result
        assert len(actions)==(6 if failure=='budget' else 3 if failure=='post_unknown' else 1)
    else:
        assert result['ok'] and result['task_completed'],result
        assert len(actions)==3
        assert len(verifications)==1
        assert all(a['post_facts']=={} for a in actions)
        assert all(a['pre_facts'] for a in actions)  # incomplete environment stays checked
        assert all('observation' in a for a in actions)
        assert len(locations)==1
        assert all(a['scene_reference']['frame_id']=='frame-0' for a in actions)
        assert state['p']==pytest.approx([.23,-.16,.29])


def test_compact_report_does_not_repeat_full_observations(rehearsal):
    r={'ok':True,'pre':{'large':'x'*1000},'receipt':{'observation':{'images':'large'}},
       'steps':[{'ok':True,'pre':{'large':'y'*1000},'report_path':'saved.json'}],
       'localization':{'observation':{'images':'large'}},'report_path':'parent.json','visual_calls':3}
    compact=rehearsal.compact(r)
    assert 'pre' not in compact and 'receipt' not in compact and 'localization' not in compact
    assert compact['steps']==[{'ok':True,'report_path':'saved.json'}]
    assert compact['report_path']=='parent.json'


@pytest.mark.parametrize("model_pause",[False,True])
def test_localization_tracks_new_frame_without_model_and_falls_back(perception,tmp_path,model_pause):
    import cv2
    class SavedScene:
        def __init__(self):self.n=0;self.model_calls=0;self.pixel_frames=[];self.hide=False
        def value(self,name,**kw):
            if name=='piper_observe':
                self.n+=1;ident='frame'+str(self.n);d=tmp_path/ident;d.mkdir()
                shift=3*(self.n-1);cup=(160+shift,150);coaster=(400+shift,160)
                im=np.full((400,640,3),90,np.uint8)
                if not self.hide:cv2.circle(im,cup,45,(240,180,120),4)
                cv2.circle(im,coaster,50,(30,170,220),4)
                cv2.imwrite(str(d/'head.jpg'),im)
                yy,xx=np.indices(im.shape[:2]);z=np.full(xx.shape,.02)
                z[np.abs(np.hypot(xx-cup[0],yy-cup[1])-45)<8]=.1
                K=np.array([[400,0,320],[0,400,200],[0,0,1.]])
                xyz=np.stack([(xx-320)/400*z,(yy-200)/400*z,z],axis=-1)
                np.savez(d/'head-depth.npz',xyz_rgb_m=xyz,valid=np.ones(z.shape,bool))
                self.arrays=xyz;self.current=ident
                self.obs={'frame_id':ident,'measured14':[0.]*14,'wall_budget':{'remaining_seconds':900},
                    'cameras':{'head':{'inspection_image':str(d/'head.jpg'),'shape_hw':[400,640],
                        'source_timestamp_ns':time.time_ns(),'K':K.tolist(),'T_base_camera':np.eye(4).tolist(),'metric_depth':True}}}
                return self.obs
            if name=='piper_pixel_to_position':
                self.pixel_frames.append(kw['frame_id']);assert kw['frame_id']==self.current
                return {'points':[{'position':self.arrays[round(v),round(u)].tolist()} for u,v in kw['pixels']],
                        'wall_budget':{'remaining_seconds':900}}
            if name=='gemini_generate':
                import json
                self.model_calls+=1
                if model_pause:self.obs['cameras']['head']['source_timestamp_ns']-=5_000_000_000
                return {'finish_reason':'stop','message':{'content':json.dumps({'ok':True,
                    'cup_center_norm':[(160+3*(self.n-1))/640*1000,375],
                    'coaster_center_norm':[(400+3*(self.n-1))/640*1000,400],'table_norm':[80,800]})}}
            raise AssertionError(name)
    rig=SavedScene();first=perception.localize_supported(rig,{})
    assert first['ok'],first
    if model_pause:assert first['before_frame']=='frame2'
    second=perception.localize_supported(rig,{'previous':first})
    assert second['ok'],second
    assert rig.model_calls==1 and second['visual_calls']==0
    assert second['near_wall_pixel'][0]>first['near_wall_pixel'][0]
    assert second['before_frame']==('frame3' if model_pause else 'frame2') and rig.pixel_frames[-1]==second['before_frame']
    rig.hide=True
    failed=perception.localize_supported(rig,{'previous':second})
    assert not failed['ok'] and rig.model_calls==2
    assert failed['visual_calls']==1 and failed['tracking_failure']


def test_rgb_observation_keeps_installed_geometry_identity_without_depth(tmp_path):
    from test_piper_adapter import backend
    from services.piper.finger_geometry import geometry_identity
    b=backend(tmp_path)
    obs,_=b._capture_observation({'require_depth':False})
    assert obs['finger_geometry']['geometry_identity']==geometry_identity()
    assert obs['finger_geometry']['physical_clearance_authority'] is False
    assert not obs['finger_geometry'].get('fingers')


def test_shared_observation_from_previous_owner_never_dispatches(rehearsal_rig):
    m,rig=rehearsal_rig;obs=copy.deepcopy(rig.obs)
    obs['state_binding']['owner_generation']=1
    for c in obs['cameras'].values():c['source_timestamp_ns']=time.time_ns()
    obs['state_binding']['owner_generation']=0
    result=m.move_once(rig,{'name':'numeric','tool':'piper_move_pose','args':{'delta':[0,0,.003]},
        'pre_facts':{},'post_facts':{},'observation':obs})
    assert not result['executed'] and rig.moves==0


@pytest.mark.parametrize('age_ns',[31_000_000_000,-1_000_000_000])
def test_tracking_pause_or_future_timestamp_requires_reacquisition(perception,age_ns):
    import cv2
    old={'ok':True,'before_frame':'old','observation':{'frame_id':'old','cameras':{'head':{
        'source_timestamp_ns':time.time_ns()-age_ns,'metric_depth':True,'K':[],
        'T_base_camera':[],'inspection_image':'must-not-be-read.jpg'}}}}
    current={'frame_id':'new','cameras':{'head':{'K':[],'T_base_camera':[]}}}
    with pytest.raises(ValueError,match='pause'):
        perception._track_scene(old,current,np.zeros((100,100,3),np.uint8),np.empty((0,3)))


def test_model_timeout_counts_wait_and_never_dispatches(rehearsal_rig,monkeypatch):
    m,rig=rehearsal_rig;t={'now':100.}
    import types
    m.time=types.SimpleNamespace(monotonic=lambda:t['now'],time_ns=time.time_ns)
    def fail(*a,**kw):t['now']+=9;raise TimeoutError('offline injected model timeout')
    monkeypatch.setattr(m,'visual',fail)
    r=m.stage(rig,{'delta':[0,0,.003]})
    assert not r['executed'] and rig.moves==0
    assert r['visual_calls']==1 and r['perception_s']==9


@pytest.mark.parametrize('failure',[None,'stale','pose','owner','long_pause'])
def test_short_boundary_refresh_preserves_new_frame_and_state_guards(rehearsal_rig,failure):
    m,rig=rehearsal_rig
    rig.obs['state_binding']['owner_generation']=1
    for c in rig.obs['cameras'].values():c['source_timestamp_ns']=time.time_ns()
    old=copy.deepcopy(rig.obs);old['frame_id']='old'
    for c in old['cameras'].values():c['source_timestamp_ns']-=2_000_000_000
    if failure=='stale':
        for c in rig.obs['cameras'].values():c['source_timestamp_ns']-=2_000_000_000
    elif failure=='pose':rig.obs['measured14']=rig.q.copy();rig.obs['measured14'][0]=.1
    elif failure=='owner':rig.obs['state_binding']['owner_generation']=2
    elif failure=='long_pause':
        for c in old['cameras'].values():c['source_timestamp_ns']-=30_000_000_000
    r=m.move_once(rig,{'name':'numeric','tool':'piper_move_pose','args':{'delta':[0,0,.003]},
                       'pre_facts':{},'post_facts':{},'observation':old})
    if failure:
        assert not r['executed'] and rig.moves==0,r
    else:
        assert r['ok'] and rig.moves==1,r
        assert r['boundary_reacquired'] and not r['observation_reused']
        assert r['before_frame']=='fresh'
        assert [kw for n,kw in rig.calls if n=='piper_observe']==[{'cameras':['head','left','right'],'require_depth':False}]
