"""Synthetic branch/contract tests; real curves are checked in separate replay."""
from types import SimpleNamespace
from unittest.mock import patch
import copy,numpy as np,pytest
import ordinary_checks as ordinary
import qualitative
from scene_checks import PerceptionUnknown,ViewUnavailable
import assembly_plan as ap
from payload_replan import bind_pickup

def scene():
 return {'blocks':{'green':{'top_center':{'surface_position_m':[.3,0.,.06]}}},'table':{'z_ax_by_c':[0.,0.,0.]}}

def fkctx(offset=0):
 T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[.3+offset,0.,.188]
 return SimpleNamespace(allow_auxiliary_vision=False,value=lambda name,**kw:{'T_base_flange':T.tolist(),'position':T[:3,3].tolist()} if name=='piper_fk' else pytest.fail('No model or motion'))

def test_partial_wrist_patch_and_geometry_resolve_alignment_without_model():
 im=np.zeros((720,1280,3),np.uint8);im[320:380,630:690]=[50,200,50];im[550:570,200:220]=[0,128,255]
 obs={'frame_id':'test','measured14':[0.]*14}
 with patch('scene_checks.scene_geometry'),patch('ordinary_checks.visible_near_point',return_value={'visible':False}),patch('ordinary_checks.current_image',return_value=im):
  r=qualitative.alignment_evidence(fkctx(),obs,'green',scene=scene())
 assert r['ok'] and r['wrist']['visible']

def test_real_grasp_geometry_contradiction_still_refuses():
 with patch('scene_checks.scene_geometry'),pytest.raises(PerceptionUnknown,match='geometry contradicts'):
  qualitative.alignment_evidence(fkctx(.03),{'measured14':[0.]*14},'green',scene=scene())

def test_head_positive_does_not_require_optional_wrist_decode():
 s={'blocks':{'orange':{'top_center':{'surface_position_m':[.3,0.,.06]}}},'table':{'z_ax_by_c':[0.,0.,0.]}}
 origin={'color':'orange','grasp_flange':[.3,0.,.188],'flange_to_bottom_m':.188}
 with patch('ordinary_checks.visible_near_point',return_value={'visible':True}),patch('ordinary_checks.wrist_body',side_effect=ViewUnavailable('offline missing wrist')) as wrist,patch('history_scene.reject_visible_displacement',return_value={}):
  r=ordinary.placement(fkctx(),'support',{'frame_id':'test','measured14':[0.]*14},s,origin)
 assert r['ok'];wrist.assert_not_called()

def test_view_unavailable_is_optional_but_bad_binding_is_not():
 assert not ordinary.view_evidence(lambda:(_ for _ in ()).throw(ViewUnavailable('missing')))['visible']
 with pytest.raises(PerceptionUnknown):ordinary.view_evidence(lambda:(_ for _ in ()).throw(PerceptionUnknown('changed owner')))

def release_fixture():
 q=[0.]*14;q[6]=.073;up=q.copy();up[2]=.06
 receipt={'executed':True,'measured_arrival':True,'measured_settled':True,'owner_generation':3,'final14':q}
 ctx=SimpleNamespace(value=lambda name,**kw:{'position':[.3,0.,.2+kw['joints'][2]]})
 return ctx,{'frame_id':'test','measured14':up,'state_binding':{'owner_generation':3}}, {'receipt':receipt,'support':{'ok':True}},dict(receipt,final14=up)

def test_green_release_uses_wrist_structure_when_head_is_occluded():
 ctx,obs,release,retreat=release_fixture()
 with patch('ordinary_checks.wrist_body',return_value={'visible':False}),patch('ordinary_checks.visible_near_point',return_value={'visible':False}),patch('ordinary_checks.wrist_pair',return_value={'ok':True}) as pair,patch('history_scene.reject_visible_displacement',return_value={}):
  assert ordinary.released_endpoint(ctx,obs,{'blocks':{'green':{'top_center':{'surface_position_m':[.3,0,.12]}},'red':{'top_center':{'surface_position_m':[.3,0,.06]}}}},'green',release,retreat)['ok']
  assert pair.call_args.args[1]==('red','green')

@pytest.mark.parametrize('fault',['unresolved','closed','incomplete'])
def test_release_needs_layout_and_complete_open_receipts(fault):
 ctx,obs,release,retreat=release_fixture()
 if fault=='closed':release['receipt']['final14'][6]=.05
 if fault=='incomplete':retreat['measured_arrival']=False
 with patch('ordinary_checks.wrist_body',return_value={'visible':False}),patch('ordinary_checks.visible_near_point',return_value={'visible':False}),patch('ordinary_checks.wrist_pair',return_value={'ok':None if fault=='unresolved' else True}),pytest.raises(PerceptionUnknown):
  ordinary.released_endpoint(ctx,obs,scene(),'green',release,retreat)

def test_pickup_rebinds_actual_column_without_changing_planned_height():
 start=[0.]*14;start[6]=.051;actual=start.copy();actual[0]=.009
 target=start.copy();target[2]=.05;later=target.copy();later[1]=.1
 actions=[dict(kind='arm',stage='green-raise',color='green',pickup=True,start14=start,target14=target),dict(kind='arm',stage='green-transfer',color='green',start14=target,target14=later)]
 calls=[]
 def fk(q):return {'position':[.3+q[0],q[1],.2+q[2]],'quaternion_xyzw':[1.,0.,0.,0.]}
 def route(a,p,q,scene,**kw):
  calls.append((p,kw));b=a.copy();b[:3]=[p[0]-.3,p[1],p[2]-.2];return [{'start14':a,'target14':b}]
 e=SimpleNamespace(planner=SimpleNamespace(fk=fk,route=route),payload={'color':'green'},held='green',scene=scene())
 bound,meta=bind_pickup(e,actions,actual)
 assert calls[0][0]==pytest.approx([.309,0.,.25]);assert calls[0][1]['pickup']
 assert bound[-1]['target14']==later and actions[0]['target14']==target
 assert meta['rebound'] and bound[-1]['start14']==bound[0]['target14']

def test_low_hover_is_searched_after_high_ik_succeeds_but_paths_fail():
 from test_composition import SceneContext
 s=ap.initial_scene(SceneContext().scene);src=ap.xyz(s,'orange');p=ap.Planner.__new__(ap.Planner)
 p.local={ap.OPEN:np.zeros((16,3))};p.ctx=SimpleNamespace();p.progress=lambda **kw:None
 p.solve=lambda pos,q,seed:list(pos)+[0.]*3+seed[6:];p.endpoint=p.solve;p.ik_mode='full'
 p.check=lambda *a,**kw:{}
 def fk(a):
  T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=a[:3];return {'position':list(a[:3]),'T_base_flange':T.tolist()}
 p.fk=fk;attempts=[]
 def route(start,dest,q,scene,**kw):
  if not kw.get('contact') and not kw.get('payload') and not kw.get('withdraw'):
   attempts.append(dest[2]-src[2])
   if dest[2]-src[2]>=.179:raise ap.NoPlan('Synthetic high route collision despite reachable IK')
  b=list(dest)+[0.]*3+start[6:];return [{'start14':start,'target14':b,'check':{}}]
 p.route=route
 with patch.object(ap,'YAW_DEGREES',(180.,)):
  result=p.transfer([0.]*14,s,'orange',[src[0]-.02,src[1],src[2]])
 assert result['actions'] and any(x>=.18 for x in attempts) and any(x<.18 for x in attempts)
