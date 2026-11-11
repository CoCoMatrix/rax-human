import asyncio,json,time
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
from real_robot.piper.continuous_review import ContinuousReviewModel
from real_robot.piper.reviewed_prefix import SERIALS
from real_robot.piper.contracts import PiperObservation,ValidatedCandidate
from real_robot.piper.teacher_client import STUDENT_TASK,instructions_for_context,BOTTLE_PLACE_TASK

def obs(state=None):
    now=time.time_ns()
    return PiperObservation(np.zeros(14,np.float32) if state is None else state,
        {k:np.zeros((4,4,3),np.uint8) for k in SERIALS},STUDENT_TASK,now,now,'test',{'robot_id':'test'},0.,
        {'owner_generation':1,'policy_session_id':'session','camera_frames':{k:{'camera_serial':v,'source_timestamp_ns':now} for k,v in SERIALS.items()}})
class Backend:
    description=SimpleNamespace(validate_candidate_limits=lambda x:None)
    def __init__(self):self.contract=self;self.calls=0
    def observe(self,o):return o
    def validate_candidate(self,a,**kw):return ValidatedCandidate(a,30,kw['prefix_steps'],{'robot_id':'test'},{})
    async def propose(self,o):self.calls+=1;return {'actions':np.zeros((32,14),np.float32)}
    def diagnostics(self,o,a):return {'current_flange':{},'student_flange_trajectory':[{}]*32},a
class Teacher:
    def __init__(self):self.calls=0
    def review(self,**kw):
        self.calls+=1
        return {'decision':{'binding':kw['binding'],'mode':'student' if self.calls==1 else 'stop','steps':15,'reason':'test'}}
class Reviewer:
    def review(self,o,p,d,**kw):return SimpleNamespace(candidate=SimpleNamespace(actions=p.actions[:15]) if d['mode']=='student' else None)
def make(tmp):return ContinuousReviewModel(Backend(),Reviewer(),Teacher(),tmp,tmp/'journal',max_reviews=2)
def test_fresh_review_ack_next_observation_loop(tmp_path):
    m=make(tmp_path)
    async def run():
        m.next_review_time=0
        assert (await m.infer(obs()))['actions'].shape==(0,14)
        await m.pending
        assert (await m.infer(obs()))['actions'].shape==(15,14)
        assert m.executed_steps==0
        release=m.active
        rows=[{'event':'sdk_dispatch_completed','review_release_id':release['release_id'],
            'teacher_request_id':release['teacher_request_id'],'model_chunk_step_index':i,
            'hardware_dispatch_confirmed':True,'applied14':[0.]*14} for i in range(15)]
        m.journal.write_text('\n'.join(json.dumps(r) for r in rows))
        assert (await m.infer(obs()))['actions'].shape==(0,14)
        assert m.executed_steps==15
        m.next_review_time=0;await m.infer(obs());await m.pending
        assert m.phase=='stopped' and m.backend.calls==2
    asyncio.run(run())
def test_state_change_cannot_release_review(tmp_path):
    m=make(tmp_path)
    async def run():
        m.next_review_time=0;await m.infer(obs());await m.pending
        changed=np.zeros(14);changed[0]=.002
        assert (await m.infer(obs(changed)))['actions'].shape==(0,14)
        assert m.phase=='error' and m.active is None
    asyncio.run(run())
def test_original_segment_lengths_in_continuous_prompt():
    text=instructions_for_context({'user_goal':BOTTLE_PLACE_TASK,'continuous_execution':True})
    assert '1..15 steps' in text and 'edit/eef select 1..5 steps' in text
    assert 'MUST choose steps=1' not in text

def test_rgb_only_task_segment_has_no_scene_geometry(tmp_path):
    m=make(tmp_path);m.max_reviews=1;captured={}
    original=m.teacher.review
    def review(**kw):
        captured.update(kw['context']);return original(**kw)
    m.teacher.review=review
    async def run():
        m.next_review_time=0;await m.infer(obs());await m.pending
    asyncio.run(run())
    assert captured['perception_mode']=='rgb_only'
    assert captured['scene_geometry'] is None
    assert captured['known_nominal_geometry_disagreement'] is None
    assert captured['depth_status']=='disabled_by_operator_no_depth_input'
    text=instructions_for_context(captured)
    assert 'Additional depth is disabled' in text
    assert 'This is not a transport canary' in text
    assert captured['user_goal']==BOTTLE_PLACE_TASK

def test_correction_recomputes_only_after_each_fresh_ack(tmp_path):
    m=make(tmp_path);seeds=[]
    m.teacher.review=lambda **kw:{'decision':{'binding':kw['binding'],'mode':'eef','steps':3,'reason':'fixture'}}
    m.reviewer.review=lambda *a,**kw:SimpleNamespace(candidate=SimpleNamespace(actions=np.full((3,14),.0001)))
    def correct(o,p,d,n):
        seeds.append(o.state14.copy());assert n==1
        return (o.state14+.0001)[None,:]
    m.reviewer._correct=correct
    async def run():
        m.next_review_time=0;await m.infer(obs());await m.pending
        first=await m.infer(obs());assert first['actions'].shape==(1,14)
        assert seeds==[]
        for i in range(3):
            r=m.active
            with m.journal.open('a') as f:f.write(json.dumps({'event':'sdk_dispatch_completed',
                'review_release_id':r['release_id'],'teacher_request_id':r['teacher_request_id'],
                'model_chunk_step_index':0,'hardware_dispatch_confirmed':True,'applied14':r['targets14'][0]})+'\n')
            result=await m.infer(obs(np.full(14,(i+1)*.0001)))
            assert result['actions'].shape==((1,14) if i<2 else (0,14))
        assert m.executed_steps==3 and len(seeds)==2
        np.testing.assert_allclose(seeds[0],.0001);np.testing.assert_allclose(seeds[1],.0002)
    asyncio.run(run())
