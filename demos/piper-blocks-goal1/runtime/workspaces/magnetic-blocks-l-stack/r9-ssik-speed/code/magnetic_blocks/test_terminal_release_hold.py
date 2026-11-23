import pytest
import assembly_plan as a
from test_c_four_search import planner

def drain(gen):
 while True:
  try:next(gen)
  except StopIteration as done:return done.value

def terminal_planner(*,good_retreat=False,fail_release=False):
 p=planner();p.ik_mode='full';p.progress=lambda **kw:None;p.defer_native=True
 p.prefetch_endpoints=lambda *args:None;p.fk=lambda q:{'position':[.2,-.2,.30]}
 def check(start,end,*args,**kw):
  if fail_release and end[6]!=start[6]:raise a.NoPlan('release obstructed')
  return {'native':{'cad_clear':True,'workspace_clear':True}}
 p.check=check
 def route(start,dest,q,scene,**kw):
  if not good_retreat:raise a.NoPlan('no withdrawal')
  return [{'start14':start,'target14':list(start),'check':check(start,start)}]
 p.route=route;p.connect_edge=lambda *args,**kw:(route(*args,**kw),list(args[0]))
 return p


def terminal_args(p,color='yellow'):
 start=[0.]*14;start[6]=a.CLOSED
 return (start,p.scene,color,a.xyz(p.scene,color).tolist(),{},a.quat(180),'green' if color=='yellow' else 'red',None)


def test_final_yellow_hold_only_after_complete_withdrawal_failure():
 p=terminal_planner();r=drain(p.finish_steps(*terminal_args(p)))
 assert len(r['actions'])==1 and r['actions'][0]['stage']=='yellow-release'
 assert r['terminal_release_hold']['retreat_performed'] is False
 assert r['terminal_release_hold']['withdrawal_refusals']>1
 assert len(r['actions'][0]['release_feedback_preflight']['checks'])==2
 assert r['end14'][6]==a.OPEN
 # The certification path includes final opening-envelope checks as well.
 p.search_active=False;p.certify_moves([{'actions':r['actions']}])
 assert p.lazy_stats['native_certified_actions']==1


@pytest.mark.parametrize('color,mode',[('green','full'),('yellow','fast')])
def test_nonterminal_or_fast_failure_cannot_be_promoted(color,mode):
 p=terminal_planner();p.ik_mode=mode
 with pytest.raises(a.NoPlan):drain(p.finish_steps(*terminal_args(p,color)))


def test_full_retreat_wins_over_terminal_hold():
 p=terminal_planner(good_retreat=True);r=drain(p.finish_steps(*terminal_args(p)))
 assert 'terminal_release_hold' not in r and len(r['actions'])>1
 assert r['actions'][-1]['stage']=='yellow-retreat'


def test_failed_release_is_never_optional():
 p=terminal_planner(fail_release=True)
 with pytest.raises(a.NoPlan,match='release obstructed'):drain(p.finish_steps(*terminal_args(p)))


def test_terminal_hold_does_not_absorb_timeout():
 p=terminal_planner()
 def timeout(*args,**kw):raise a.PlanningTimeout('global budget')
 p.connect_edge=timeout
 with pytest.raises(a.PlanningTimeout):drain(p.finish_steps(*terminal_args(p)))
 assert not p.finish_cache
