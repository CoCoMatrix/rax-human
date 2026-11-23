import copy
from types import SimpleNamespace
import pytest
import assembly_batch as b
from assembly_plan import NoPlan


def action(stage,*,contact=False,withdraw=False):
 q=[0.]*14;q[6]=.073
 return {'kind':'arm','stage':stage,'color':stage.split('-')[0],
         'payload':None,'start14':q.copy(),'target14':q.copy(),'scene':{'same':'placed green'},'contact':contact,'withdraw':withdraw}


def test_release_and_yellow_entry_remain_boundaries():
 release={'kind':'jaw','stage':'green-release'}
 retreat=action('green-retreat',contact=True)
 hover=action('yellow-hover');entry=action('yellow-entry',contact=True)
 groups=b.groups([release,retreat,copy.deepcopy(retreat),hover,copy.deepcopy(hover),entry,{'kind':'jaw','stage':'yellow-close'}])
 assert [len(g) for g in groups]==[1,4,1,1]
 assert b.green_yellow_transition(groups[1])
 assert not b.compatible(hover,retreat)
 assert not b.compatible(retreat,entry)


def test_changed_scene_or_discontinuous_endpoints_do_not_merge():
 a=action('green-retreat',withdraw=True);v=action('yellow-hover')
 v['scene']={'different':'scene'};assert not b.compatible(a,v)
 v['scene']=a['scene'];v['start14'][0]=.02;assert not b.compatible(a,v)


def test_transition_requires_actual_completed_opening():
 actions=[action('green-retreat'),action('yellow-hover')]
 e=SimpleNamespace(release={'color':'green','receipt':{'owner_request_id':'release'}},previous={'owner_request_id':'release'},held=None,payload=None,completed=['green-release'])
 b.require_released_green(e,actions)
 for attr,value in [('held','green'),('payload',{'held':True}),('release',None),('previous',{'owner_request_id':'other'})]:
  previous=getattr(e,attr);setattr(e,attr,value)
  with pytest.raises(NoPlan):b.require_released_green(e,actions)
  setattr(e,attr,previous)


def test_contact_hover_cannot_extend_empty_handoff():
 assert not b.compatible(action('green-retreat'),action('yellow-hover',contact=True))


@pytest.mark.parametrize('color,payload,gap,contact,withdraw',[
 ('yellow',None,.073,True,False),('green',{'held':True},.073,True,False),
 ('green',None,.05,True,False),('green',None,.073,False,False)])
def test_released_curve_exception_is_narrow(color,payload,gap,contact,withdraw):
 from test_c_four_search import planner
 p=planner();q=[0.]*14;q[6]=gap
 with pytest.raises(NoPlan,match='Released empty curve requires'):
  p._check(q,q,p.scene,active=color,payload=payload,contact=contact,withdraw=withdraw,
           curve14=[q,q],released_empty_curve=True,native_result={'cad_clear':True,'workspace_clear':True})


def test_released_curve_still_requires_native_certificate():
 from test_c_four_search import planner
 p=planner();q=[0.]*14;q[6]=.073
 with pytest.raises(NoPlan,match='native collision'):
  p._check(q,q,p.scene,active='green',contact=True,curve14=[q,q],released_empty_curve=True,native_result={})
