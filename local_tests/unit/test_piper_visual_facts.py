import importlib.util
import json
from pathlib import Path
import pytest
import sys

SOURCE=Path(__file__).resolve().parents[2]/'workspaces/blue-cup/20261104-v0.3-r12-working/code/blue_cup/perception.py'

@pytest.fixture
def perception(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.syspath_prepend(str(SOURCE.parent))
    sys.modules.pop('approach_target',None)
    spec=importlib.util.spec_from_file_location('working_perception',SOURCE.resolve())
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

class Context:
    def __init__(self,result):self.result=result;self.calls=[]
    def value(self,name,**kwargs):
        assert name=='gemini_generate';self.calls.append(kwargs)
        return {'message':{'content':json.dumps(self.result)},'finish_reason':'stop'}

@pytest.mark.parametrize('reply',[
    {'finish_reason':'length','message':{'content':'{"ok":true}'}},
    {'finish_reason':'stop','message':{'content':'','refusal':None}},
    {'finish_reason':'stop','message':{'content':'{"ok":true}','refusal':'refused'}},
])
def test_incomplete_gemini_output_never_approves_action(perception,tmp_path,reply):
    class Incomplete:
        def value(self,*args,**kwargs):return reply
    with pytest.raises(ValueError,match='Incomplete or refused'):
        perception.visual(Incomplete(),observation(tmp_path),'Inspect')

def test_prior_thumbnail_preserves_current_native_pixel_coordinates(perception,tmp_path):
    import base64,io
    from PIL import Image
    from services.controller.google_gemini import GoogleGeminiRouter
    from services.controller.billing import reserve_cost,SESSION_LIMIT
    p=tmp_path/'native.jpg';Image.new('RGB',(1280,720),'white').save(p)
    obs={'frame_id':'native','cameras':{c:{'inspection_image':str(p),'shape_hw':[720,1280]} for c in ('head','left')}}
    ctx=Context({'ok':True})
    perception.visual(ctx,obs,'Compare',prior=obs)
    blocks=ctx.calls[0]['messages'][0]['content'][1:]
    assert [Image.open(io.BytesIO(base64.b64decode(x['data']))).size for x in blocks]==[(640,360)]*2+[(1280,720)]*2
    assert obs['cameras']['head']['shape_hw']==[720,1280]
    payload,reservation=GoogleGeminiRouter('unused').prepare(ctx.calls[0])
    assert reserve_cost(reservation,payload['max_tokens'])<SESSION_LIMIT

def observation(tmp_path):
    p=tmp_path/'view.jpg';p.write_bytes(b'fake-inlined-jpeg')
    return {'frame_id':'fresh','cameras':{c:{'inspection_image':str(p),'shape_hw':[480,640]} for c in ('head','left')}}

def test_trial_lift_comparison_has_equal_image_scale_and_accepts_indirect_cues(perception,tmp_path):
    import base64,io
    from PIL import Image
    p=tmp_path/'native.jpg';Image.new('RGB',(1280,720),'white').save(p)
    obs={'frame_id':'native','cameras':{c:{'inspection_image':str(p),'shape_hw':[720,1280]} for c in ('head','left')}}
    ctx=Context({'ok':True,'unknowns':['cup bottom hidden'], 'visible_hazards':[],
        'facts':{'cup_lifted_and_held':{'state':'supported',
        'evidence':'Cup remains at broadly the same pad location in wrist views while cup/table shadow separation changes in head views',
        'cameras':['before head','current head','current left']}}})
    r=perception.visual(ctx,obs,'Compare pickup',prior=obs,required_facts={'cup_lifted_and_held':'Qualitative pickup'})
    blocks=ctx.calls[0]['messages'][0]['content'][1:]
    assert [Image.open(io.BytesIO(base64.b64decode(x['data']))).size for x in blocks]==[(640,360)]*4
    assert r['ok'] and r['fact_sources']=={'cup_lifted_and_held':'visual'}
    assert obs['cameras']['head']['shape_hw']==[720,1280]

@pytest.mark.parametrize('state',['unknown','contradicted',None])
def test_required_unknown_missing_or_contradicted_cannot_pass(perception,tmp_path,state):
    facts={} if state is None else {'separation':{'state':state,'evidence':'occluded','cameras':['left']}}
    ctx=Context({'ok':True,'facts':facts})
    r=perception.visual(ctx,observation(tmp_path),'Inspect separation',required_facts={'separation':'pad vs rim'})
    assert r['ok'] is False and len(ctx.calls)==1

def test_unrelated_occlusion_does_not_create_requirement(perception,tmp_path):
    ctx=Context({'ok':False,'unknowns':['hidden underside'],
        'facts':{'cup_upright':{'state':'supported','evidence':'Cup visibly upright in head view','cameras':['head']}}})
    r=perception.visual(ctx,observation(tmp_path),'Inspect current cup pose',required_facts={'cup_upright':'cup upright'})
    assert r['ok'] is True and r['frame_id']=='fresh'
    assert '480, 640' in ctx.calls[0]['messages'][0]['content'][0]['text'] and '1280x720' not in ctx.calls[0]['messages'][0]['content'][0]['text']
    assert 'transient contact' in ctx.calls[0]['messages'][0]['content'][0]['text']

def test_nonexistent_camera_or_empty_evidence_not_support(perception,tmp_path):
    for cameras,evidence in [(['imagined'],'gap'),(['head'],'')]:
        ctx=Context({'facts':{'gap':{'state':'supported','evidence':evidence,'cameras':cameras}}})
        assert not perception.visual(ctx,observation(tmp_path),'Inspect',required_facts={'gap':'gap'})['ok']

def test_visible_hazard_blocks_even_when_requested_facts_supported(perception,tmp_path):
    ctx=Context({'facts':{'jaw_empty':{'state':'supported','evidence':'No held object','cameras':['HEAD']}},
                 'visible_hazards':['Cable lies in the opening direction']})
    assert not perception.visual(ctx,observation(tmp_path),'Opening',required_facts={'jaw_empty':'empty'})['ok']

def test_native_camera_labels_bind_only_supplied_current_views(perception,tmp_path):
    obs=observation(tmp_path)
    for refs,expected in [(['current head','current left wrist'],True),(['before head','current head'],True),(['before head'],False),(['current right wrist'],False)]:
        ctx=Context({'facts':{'jaw_empty':{'state':'supported','evidence':'Empty exposed pads','cameras':refs}}})
        r=perception.visual(ctx,obs,'Inspect',prior=obs,required_facts={'jaw_empty':'empty'})
        assert r['ok'] is expected


def test_unknown_critical_fact_reports_only_the_fact_to_resolve(perception,tmp_path):
    ctx=Context({'facts':{'gap':{'state':'unknown','evidence':'Rim hides one pad','cameras':['left']}},
                 'unknowns':['hidden underside'], 'visible_hazards':[]})
    r=perception.visual(ctx,observation(tmp_path),'Inspect current separation',required_facts={'gap':'pad vs rim'})
    assert r['ok'] is False
    assert r.get('unresolved_facts')==['gap']
    assert r.get('contradicted_facts')==[]


@pytest.mark.parametrize('image_state,scene_state,hazards,operator_state,expected',[
    ('unknown','supported',[],'supported',True),
    ('contradicted','supported',[],'supported',False),
    ('unknown','unknown',[],'supported',False),
    ('unknown','contradicted',[],'supported',False),
    ('unknown','supported',['Cable crosses route'],'supported',False),
    ('unknown','supported',[],'contradicted',False),
    ('unknown','supported',[],'unknown',False),
])
def test_scoped_onsite_fact_does_not_require_image_proof_or_override_conflict(
        perception,tmp_path,image_state,scene_state,hazards,operator_state,expected):
    reply='Both pads are separated from the cup.'
    ctx=Context({'facts':{'gap':{'state':image_state,'evidence':'Rim hides one pad','cameras':['left']}},
        'operator_facts':{'gap':{'state':operator_state,'evidence':reply}},
        'scene_consistent':{'state':scene_state,'evidence':'Cup and visible surroundings unchanged','cameras':['before head','current head']},
        'visible_hazards':hazards})
    r=perception.visual(ctx,observation(tmp_path),'Inspect separation',prior=observation(tmp_path),
        required_facts={'gap':'pad vs rim'},onsite={'facts':{'gap':'pad vs rim'},'reply':reply,'request_id':'host-id'})
    assert r['ok'] is expected
    if expected:
        assert r['fact_sources']=={'gap':'operator'}
        assert r['facts']['gap']['state']=='unknown'  # Never relabel hidden as visible.


def test_onsite_cannot_supply_unasked_fact_or_fabricated_quote(perception,tmp_path):
    reply='The cup is upright.'
    ctx=Context({'facts':{'gap':{'state':'unknown','evidence':'hidden','cameras':['left']}},
        'operator_facts':{'gap':{'state':'supported','evidence':'Everything is safe'}},
        'scene_consistent':{'state':'supported','evidence':'unchanged','cameras':['before head','current head']},
        'visible_hazards':[]})
    r=perception.visual(ctx,observation(tmp_path),'Inspect',prior=observation(tmp_path),
        required_facts={'gap':'pad vs rim'},onsite={'facts':{'gap':'pad vs rim'},'reply':reply,'request_id':'host-id'})
    assert r['ok'] is False


def test_numeric_geometry_never_resolves_unknown_contact(perception,tmp_path):
    obs=observation(tmp_path);obs['finger_geometry']={'nominal_min_z_m':.2,'installed_tcp_validated':False}
    ctx=Context({'facts':{'support':{'state':'unknown','evidence':'Base hidden','cameras':['head']}},'visible_hazards':[]})
    assert not perception.visual(ctx,obs,'Check support',required_facts={'support':'Cup base supported'})['ok']


@pytest.fixture
def rehearsal_rig(perception,tmp_path,monkeypatch):
    import copy
    monkeypatch.setitem(sys.modules,'perception',perception)
    spec=importlib.util.spec_from_file_location('rehearsal_evidence_test',SOURCE.with_name('rehearsal.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class Rig:
        def __init__(self):
            self.calls=[];self.moves=0;self.onsite=False;self.post_unknown=False;self.hazards=[]
            self.q=[0.]*14;self.answer=None
            self.robot={'motion_ready':True,'motion_enabled':True,'arming':{'armed':True},
                'owner_status':{'generation':1},'bridge':{'instance':'bridge','phase':'DONE'},
                'measured_state':{'values':self.q},'wall_budget':{'remaining_seconds':900}}
            self.obs=observation(tmp_path);self.obs['cameras']['right']=self.obs['cameras']['left']
            self.obs.update(measured14=self.q,wall_budget={'remaining_seconds':900},
                state_binding={'rest_consistent':True},finger_geometry={'geometry_identity':{'test':'same'}})
        def value(self,name,**kw):
            self.calls.append((name,kw))
            if name=='status':return {'robot_status':self.robot}
            if name=='piper_status':return self.robot
            if name=='piper_observe':return copy.deepcopy(self.obs)
            if name=='piper_fk':return {'position':[0,0,.3],'quaternion_xyzw':[0,0,0,1]}
            if name=='piper_solve_ik':return {'q':[.01,0,0,0,0,0]}
            if name=='piper_check_path':return {'cad_clear':True,'whole_path_checked':True,'geometry_identity':{'test':'same'}}
            if name=='request_operator':
                assert kw=={'request_id':'host-request','timeout_s':1}
                return copy.deepcopy(self.answer)
            if name in ('piper_move_joints','piper_gripper'):
                self.moves+=1;after=copy.deepcopy(self.obs);after['frame_id']='after'
                return {'executed':True,'measured_arrival':True,'final14':self.q,'observation':after}
            if name=='gemini_generate':
                def f(state):return {'state':state,'evidence':'visible evidence or occlusion','cameras':['current head','current left wrist']}
                state='unknown' if not self.moves or self.post_unknown else 'supported'
                facts={k:f('supported') for k in ['jaw_empty','cup_upright','scene_unchanged']}
                facts.update({k:f(state) for k in ['gap','approach_scene','target_separation','route_visible_and_unobstructed','opening_sides_visible_and_unobstructed']})
                facts['target_separation']['views']=[{'camera':'head','relation':'clear_gap' if facts['target_separation']['state']=='supported' else 'occluded','evidence':'Both finger gaps resolved' if facts['target_separation']['state']=='supported' else 'Gaps hidden'}]
                v={'facts':facts,'visible_hazards':self.hazards}
                if self.onsite and not self.moves:
                    v['operator_facts']={k:{'state':'supported','evidence':self.answer['reply']} for k in facts}
                    v['scene_consistent']={'state':'supported','evidence':'visible scene unchanged','cameras':['before head','current head']}
                return {'message':{'content':json.dumps(v)},'finish_reason':'stop'}
            raise AssertionError(name)
        def confirm(self,result):
            path=Path(result['operator_file']);record=json.loads(path.read_text())
            self.answer={'status':'answered','done':True,'request_id':'host-request',
                'question':result['operator_request']['message'],'reply':'Confirmed: the requested separation is clear.',
                'waited_s':1,'answered_at':123,'context':{'bridge_instance':'bridge','owner_generation':1,'measured14':self.q[:]}}
            # This local copy is not authority. The helper must reread the host request.
            record['reply']={**self.answer,'reply':'FORGED LOCAL TEXT'}
            path.write_text(json.dumps(record));self.onsite=True
            return str(path)
    return module,Rig()


@pytest.mark.parametrize('function,args',[
    ('approach_step',{'delta':[0,0,.003]}),('stage',{'delta':[0,0,.003]}),('empty_open',{'width_m':.005}),
])
def test_native_functions_resume_scoped_unknown_without_reasking_image(rehearsal_rig,function,args):
    module,rig=rehearsal_rig
    first=getattr(module,function)(rig,args)
    assert not first['executed'] and first.get('operator_request'),first
    path=rig.confirm(first)
    result=getattr(module,function)(rig,{**args,'operator_file':path})
    assert result['ok'] is True,result
    assert rig.moves==1
    assert result['operator_assistance']['reply']==rig.answer['reply']
    assert 'operator' in result['pre']['fact_sources'].values()
    assert all(v=='visual' for v in result['post']['fact_sources'].values())
    again=getattr(module,function)(rig,{**args,'operator_file':path})
    assert not again['executed'] and rig.moves==1


@pytest.mark.parametrize('invalid',['owner','pose','nan','question','done','action'])
def test_onsite_scope_mismatch_never_moves(rehearsal_rig,invalid):
    module,rig=rehearsal_rig;args={'delta':[0,0,.003]}
    first=module.approach_step(rig,args)
    assert first.get('operator_request'),first
    path=rig.confirm(first)
    if invalid=='owner':rig.answer['context']['bridge_instance']='old'
    elif invalid=='pose':rig.answer['context']['measured14'][0]=.011
    elif invalid=='nan':rig.answer['context']['measured14'][0]=float('nan')
    elif invalid=='question':rig.answer['question']='Unrelated previous confirmation'
    elif invalid=='done':rig.answer['done']=False
    else:args={'delta':[0,0,.004]}
    result=module.approach_step(rig,{**args,'operator_file':path})
    assert result['executed'] is False and rig.moves==0


def test_visible_hazard_is_not_a_question_to_override(rehearsal_rig):
    module,rig=rehearsal_rig;rig.hazards=['Cable crosses the approach']
    result=module.approach_step(rig,{'delta':[0,0,.003]})
    assert not result['executed'] and not result.get('operator_request') and rig.moves==0


def test_before_confirmation_never_resolves_after_contact_or_replays_action(rehearsal_rig):
    module,rig=rehearsal_rig;args={'delta':[0,0,.003]}
    first=module.approach_step(rig,args);assert first.get('operator_request'),first
    path=rig.confirm(first);rig.post_unknown=True
    result=module.approach_step(rig,{**args,'operator_file':path})
    assert result['executed'] is True and result['ok'] is False and rig.moves==1
    assert result.get('next_action')=='observe_only'
    assert not result['post']['fact_sources'].get('target_separation')
    module.approach_step(rig,{**args,'operator_file':path})
    assert rig.moves==1

def test_rehearsal_checks_same_geometry_and_solves_once(perception,tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'perception',perception)
    spec=importlib.util.spec_from_file_location('working_rehearsal',SOURCE.with_name('rehearsal.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    questions=[]
    def visual(ctx,obs,question,**kw):
        questions.append(question);return {'ok':True,'perception_s':.01,'reason':'visible required facts'}
    monkeypatch.setattr(module,'visual',visual)
    q=[0.]*14;identity={'geometry':'test'};budget={'remaining_seconds':900}
    before={'frame_id':'before','measured14':q,'wall_budget':budget,'state_binding':{'rest_consistent':True},
            'finger_geometry':{'geometry_identity':identity},'cameras':{}}
    robot={'motion_ready':True,'motion_enabled':True,'arming':{'armed':True},'owner_status':{'generation':1},
           'bridge':{'instance':'same','phase':'DONE'},'measured_state':{'values':q},'wall_budget':budget}
    calls=[]
    class Fake:
        geometry_clear=True
        def value(self,name,**kw):
            calls.append((name,kw))
            if name=='status':return {'robot_status':robot}
            if name=='piper_status':return robot
            if name=='piper_observe':return before
            if name=='piper_fk':return {'position':[0,0,.3],'quaternion_xyzw':[0,0,0,1]}
            if name=='piper_solve_ik':return {'q':[.1,0,0,0,0,0]}
            if name=='piper_check_path':return {'cad_clear':self.geometry_clear,'whole_path_checked':self.geometry_clear,'geometry_identity':identity}
            if name=='piper_move_joints':
                assert kw['joints']==[.1,0,0,0,0,0]
                return {'executed':True,'measured_arrival':True,'final14':q,'wall_budget':budget,
                        'observation':{'frame_id':'after','cameras':{}}}
            raise AssertionError(name)
    e={'name':'stage','tool':'piper_move_pose','args':{'delta':[0,0,.01]},'pre':'Visible route','post':'Current object layout',
       'pre_facts':{'route':'Visible route'},'post_facts':{'scene':'Current object layout'}}
    r=module.move_once(Fake(),e)
    assert r['ok'] is True and r['wall_budget']==budget
    assert [n for n,k in calls].count('piper_solve_ik')==1
    assert [n for n,k in calls].count('piper_move_joints')==1
    assert len(questions)==2
    assert r['environment_clearance_certified'] is False
    calls.clear();questions.clear();f=Fake();f.geometry_clear=False
    assert module.move_once(f,e)['executed'] is False
    assert not questions and not any(n=='piper_move_joints' for n,k in calls)


def test_read_only_localization_does_not_gate_on_jaw_clearance(perception,tmp_path,monkeypatch):
    questions=[]
    def visual(ctx,obs,question,**kw):
        questions.append(question);return {'ok':False,'perception_s':0}
    monkeypatch.setattr(perception,'visual',visual)
    class Ctx:
        def value(self,name,**kw):
            assert name=='piper_observe';return observation(tmp_path)
    perception.localize_scene(Ctx())
    assert 'jaw empty' not in questions[0] and 'visibly separated' not in questions[0]


def test_old_unscoped_approval_file_does_not_allow_motion(rehearsal_rig,tmp_path):
    module,rig=rehearsal_rig
    path=tmp_path/'old.json';path.write_text(json.dumps({'before':rig.obs,'reply':{'done':True,'status':'answered'}}))
    r=module.approach_step(rig,{'delta':[0,0,.003],'operator_file':str(path)})
    assert r['executed'] is False and rig.moves==0


def test_owner_generation_change_during_visual_call_blocks_dispatch(rehearsal_rig):
    import copy
    module,rig=rehearsal_rig;args={'delta':[0,0,.003]}
    first=module.approach_step(rig,args);path=rig.confirm(first)
    original=rig.value
    def changed(name,**kw):
        value=original(name,**kw)
        if name=='piper_status':
            value=copy.deepcopy(value);value['owner_status']['generation']=2
        return value
    rig.value=changed
    r=module.approach_step(rig,{**args,'operator_file':path})
    assert r['executed'] is False and rig.moves==0


def test_onsite_prompt_has_unambiguous_quote_and_frame_schema(perception,tmp_path):
    ctx=Context({})
    obs=observation(tmp_path)
    perception.visual(ctx,obs,'Inspect',prior=obs,required_facts={'gap':'pad vs rim'},
        onsite={'facts':{'gap':'pad vs rim'},'reply':'Both pads are separated.','request_id':'test'})
    call=ctx.calls[0]
    prompt=call.get('prompt') or call['messages'][0]['content'][0]['text']
    assert 'exact substring of reply, with no prefix or explanation' in prompt
    assert 'scene_consistent.cameras' in prompt
    assert '["before head", "before left"]' in prompt
    assert '["current head", "current left"]' in prompt


@pytest.mark.parametrize('function,args,expected_calls',[
    ('empty_open',{'width_m':.005},1),
    ('stage',{'delta':[0,0,.003]},1),
    ('approach_step',{'delta':[0,0,.003]},2),
])
def test_auxiliary_calls_follow_function_facts_not_each_motion(rehearsal_rig,monkeypatch,function,args,expected_calls):
    module,rig=rehearsal_rig;calls=[]
    def check(ctx,obs,question,**kw):
        calls.append(kw['required_facts'])
        return {'ok':True,'perception_s':.01,'reason':'visible facts','fact_sources':{k:'visual' for k in kw['required_facts']}}
    monkeypatch.setattr(module,'visual',check)
    r=getattr(module,function)(rig,args)
    assert r['ok'] is True and rig.moves==1,r
    assert len(calls)==expected_calls
    assert r['visual_calls']==expected_calls
    assert r['post']['visual_used'] is (function=='approach_step')
    if function!='approach_step':
        assert r['post']['basis']=='control_feedback'
        assert r['post']['verified']==['measured_arrival','right_state_preserved']
        assert r['post']['fact_sources']=={}
        assert r['environment_clearance_certified'] is False
    assert r['after_frame']=='after'  # Observation is retained even without a model call.


def numeric_step():
    return {'name':'robot_only_step','tool':'piper_move_pose','args':{'delta':[0,0,.003]},
            'pre_facts':{},'post_facts':{}}


def test_explicit_robot_only_step_uses_tools_without_auxiliary(rehearsal_rig,monkeypatch):
    module,rig=rehearsal_rig
    def forbidden(*a,**k):raise AssertionError('Robot-only checks must not call auxiliary vision')
    monkeypatch.setattr(module,'visual',forbidden)
    r=module.move_once(rig,numeric_step())
    assert r['ok'] is True and rig.moves==1,r
    assert r['visual_calls']==0 and r['perception_s']==0
    assert r['pre']['visual_used'] is False and r['post']['visual_used'] is False
    assert r['environment_clearance_certified'] is False
    names=[name for name,kw in rig.calls]
    assert names.count('piper_solve_ik')==1 and names.count('piper_check_path')==1
    assert names.index('piper_check_path')<names.index('piper_move_joints')


@pytest.mark.parametrize('change',['missing','none','string','operator_without_fact'])
def test_missing_fact_contract_is_not_a_silent_skip(rehearsal_rig,change):
    module,rig=rehearsal_rig;e=numeric_step()
    if change=='missing':del e['pre_facts']
    elif change=='none':e['post_facts']=None
    elif change=='string':e['pre_facts']=''
    else:e['operator_file']='unscoped.json'
    r=module.move_once(rig,e)
    assert r['executed'] is False and rig.moves==0
    assert 'explicit' in r['reason'].lower() or 'fact' in r['reason'].lower()


@pytest.mark.parametrize('failure',['arrival','nan','short','right_drift'])
def test_feedback_failure_cannot_be_fixed_by_visual_approval(rehearsal_rig,monkeypatch,failure):
    import copy
    module,rig=rehearsal_rig;original=rig.value
    def changed(name,**kw):
        result=original(name,**kw)
        if name=='piper_move_joints':
            result=copy.deepcopy(result)
            if failure=='arrival':result['measured_arrival']=False
            elif failure=='nan':result['final14'][0]=float('nan')
            elif failure=='short':result['final14']=[]
            else:result['final14'][7]=.010001
        return result
    rig.value=changed
    def forbidden(*a,**k):raise AssertionError('Feedback failure must not ask a model to approve motion')
    monkeypatch.setattr(module,'visual',forbidden)
    r=module.move_once(rig,numeric_step())
    assert r['executed'] is True and not r['ok'] and rig.moves==1,r
    assert r['next_action']=='observe_only'
    assert r['visual_calls']==0


def test_only_declared_post_fact_calls_visual_after_arrival(rehearsal_rig,monkeypatch):
    module,rig=rehearsal_rig;e=numeric_step()
    e.update(post='Inspect the current cup',post_facts={'cup_upright':'Cup visibly upright'})
    calls=[]
    def check(ctx,obs,question,**kw):
        assert rig.moves==1 and obs['frame_id']=='after'
        calls.append(kw['required_facts'])
        return {'ok':True,'perception_s':.01,'reason':'upright'}
    monkeypatch.setattr(module,'visual',check)
    r=module.move_once(rig,e)
    assert r['ok'] and r['visual_calls']==1 and calls==[e['post_facts']]


@pytest.mark.parametrize('guard',['collision','geometry_version','owner','budget','uncertain'])
def test_robot_only_step_preserves_pre_dispatch_guards(rehearsal_rig,guard):
    import copy
    module,rig=rehearsal_rig;original=rig.value
    def changed(name,**kw):
        value=copy.deepcopy(original(name,**kw))
        if name=='piper_check_path':
            if guard=='collision':value['cad_clear']=False
            if guard=='geometry_version':value['geometry_identity']={'test':'different'}
        if name=='piper_status':
            if guard=='owner':value['owner_status']['generation']=2
            if guard=='budget':value['wall_budget']['remaining_seconds']=10
        if name=='status' and guard=='uncertain':value['control_uncertain']=True
        return value
    rig.value=changed
    r=module.move_once(rig,numeric_step())
    assert not r['ok'] and not r['executed'] and rig.moves==0
    assert r['visual_calls']==0


def test_missing_after_image_recovers_observation_without_visual_or_replay(rehearsal_rig):
    module,rig=rehearsal_rig;original=rig.value
    def changed(name,**kw):
        value=original(name,**kw)
        if name=='piper_move_joints':value['observation']=None
        if name=='piper_observe' and rig.moves:value['frame_id']='recovered'
        return value
    rig.value=changed
    r=module.move_once(rig,numeric_step())
    assert r['ok'] and rig.moves==1 and r['after_frame']=='recovered'
    assert r['visual_calls']==0
    assert sum(name=='piper_observe' for name,kw in rig.calls)==2


@pytest.mark.parametrize('bad_post',[
    {'post_facts':{'cup':'Cup upright'}},
    {'post_facts':{'cup':''},'post':'Check cup'},
    {'post_facts':{1:'Cup upright'},'post':'Check cup'},
])
def test_invalid_post_contract_rejected_before_motion(rehearsal_rig,bad_post):
    module,rig=rehearsal_rig;e={**numeric_step(),**bad_post}
    r=module.move_once(rig,e)
    assert not r['executed'] and rig.moves==0,r
