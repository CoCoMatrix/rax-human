import copy,time,threading
from concurrent.futures import ThreadPoolExecutor
import pytest
from shared_ik import ExactIKStore,cache_key,stable_result,start_cache,attach_cache

IDENTITY={'pool_id':'one','solver':'ssik_rank1','model':{'urdf_sha256':'model'}}
ARGS={'position':[.3,0.,.2],'quaternion_xyzw':[0.,0.,0.,1.],'seeds':[[0.]*6],'mode':'fast'}
def result(ok=True,reason='SSIK_NO_IK_SOLUTION'):
 row={'ok':ok,'executed':False,'seed':[0.]*6}
 if ok:row['q']=[.1]*6
 else:row.update(reason='MOVEIT_NO_IK_SOLUTION',solver_reason=reason)
 return {'ok':ok,'attempts':[row],'executed':False,'hardware_writes':0,'scene_collision_checked':False,'mode':'fast','urdf_sha256':'model','wall_budget':{'remaining_seconds':12}}

def test_exact_keys_no_rounding_and_pool_mode_seed_separation():
 key=cache_key(IDENTITY,ARGS)
 for field,val in [('position',[.30000000000000004,0.,.2]),('mode','full'),('seeds',[[1e-12]+[0.]*5])]:
  a=copy.deepcopy(ARGS);a[field]=val;assert cache_key(IDENTITY,a)!=key
 other=dict(IDENTITY,pool_id='two');assert cache_key(other,ARGS)!=key

@pytest.mark.parametrize('reason',['SSIK_BUDGET_EXCEEDED','MOVEIT_NO_IK_SOLUTION','unknown',None])
def test_unresolved_failures_never_cached(reason):
 assert stable_result(result(False,reason),ARGS,IDENTITY) is None

def test_completed_negative_and_success_only_pure_math():
 assert stable_result(result(False),ARGS,IDENTITY)['ok'] is False
 good=stable_result(result(),ARGS,IDENTITY);assert 'wall_budget' not in good
 for field,val in [('executed',True),('hardware_writes',1),('urdf_sha256','other'),('scene_collision_checked',True)]:
  bad=result();bad[field]=val;assert stable_result(bad,ARGS,IDENTITY) is None

def test_single_flight_and_copy_isolation():
 store=ExactIKStore();barrier=threading.Barrier(4)
 def work(_):
  barrier.wait();a=store.acquire('same')
  if not a['hit']:time.sleep(.03);store.finish('same',a['token'],{'q':[1]})
  return a
 with ThreadPoolExecutor(4) as pool:rows=list(pool.map(work,range(4)))
 assert sum(not x['hit'] for x in rows)==1
 a=store.acquire('same');a['value']['q'][0]=9
 assert store.acquire('same')['value']=={'q':[1]}

def test_cancel_and_stale_claim_cannot_poison_cache():
 s=ExactIKStore();a=s.acquire('a');s.finish('a',a['token'],None)
 b=s.acquire('a');assert not b['hit'];assert not s.finish('a',a['token'],{'bad':True})
 s.finish('a',b['token'],{'ok':True});assert s.acquire('a')['value']=={'ok':True}

def test_manager_and_context_only_reuse_ik(tmp_path,monkeypatch):
 monkeypatch.chdir(tmp_path)
 from pathlib import Path
 Path('cache').mkdir()
 pool={'pool_id':'one','lanes':[{'solver':'ssik_rank1','geometry_identity':IDENTITY['model']}]}
 manager,cache,packet=start_cache(Path('cache'),pool)
 class Context:
  def __init__(self):self.calls=[]
  def value(self,name,**kw):self.calls.append(name);return result()
 try:
  a=Context();b=Context();attach_cache(a,packet);attach_cache(b,packet)
  a.value('piper_solve_ik_candidates',**ARGS);b.value('piper_solve_ik_candidates',**ARGS)
  assert a.calls==['piper_solve_ik_candidates'] and b.calls==[]
  b.value('piper_check_path');assert b.calls==['piper_check_path']
  assert cache.stats()['hits']==1
 finally:manager.shutdown()
