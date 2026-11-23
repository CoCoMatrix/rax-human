import copy
import pytest
from assembly_parallel import partition_layouts,validate_plan,execution_cache,restore_execution_cache

def test_partition_has_exact_union_no_overlap_even_duplicates():
    layouts=[([i*.01,-.1],s) for i in range(7) for s in (-1,1)]
    layouts += layouts[:3]
    assigned=[partition_layouts(layouts,i,4) for i in range(4)]
    sets=[set(ids) for _,ids in assigned]
    assert sum(len(x) for x,_ in assigned)==len(layouts)
    assert all(not sets[i]&sets[j] for i in range(4) for j in range(i))
    assert set.union(*sets)==set(partition_layouts(layouts,0,1)[1])
    assert [a for a,_ in assigned]==[[x for x in layouts if x in a] for a,_ in assigned]

def test_reject_bad_partition():
    with pytest.raises(ValueError):partition_layouts([],4,4)

def fixture():
    return {'ok':True,'fully_preflighted':True,'executed':False,'start14':[0]*14,
        'source_scene':{'frame':'same'},'moves':[{'actions':[{'check':{'native':{'cad_clear':True,'workspace_clear':True}}}]}]}

def test_winner_requires_native_and_exact_input():
    p=fixture();validate_plan(p,{'frame':'same'},[0]*14)
    with pytest.raises(ValueError):validate_plan(p,{'frame':'other'},[0]*14)
    q=copy.deepcopy(p);q['moves'][0]['actions'][0]['check']['native']['cad_clear']=False
    with pytest.raises(ValueError):validate_plan(q,{'frame':'same'},[0]*14)

def test_feedback_paths_cannot_be_skipped():
    p=fixture();p['moves'][0]['actions'][0]['release_feedback_preflight']={'checks':[{'native':{'cad_clear':False}}]}
    with pytest.raises(ValueError):validate_plan(p,{'frame':'same'},[0]*14)

def test_execution_cache_roundtrip_preserves_tuple_keys_and_checked_paths():
    import json
    from types import SimpleNamespace
    old=SimpleNamespace(native_cache={'edge':{'cad_clear':True}},fk_cache={(0.,1.):{'position':[1,2,3]}},
        ik_cache={('full',(1.,2.)):[0.]*6},path_cache={'certified':{'native':{'cad_clear':True}},'pending':{'native':{'deferred':True}}})
    new=SimpleNamespace()
    restore_execution_cache(new,json.loads(json.dumps(execution_cache(old))))
    assert new.fk_cache==old.fk_cache and new.ik_cache==old.ik_cache
    assert new.native_cache==old.native_cache and set(new.path_cache)=={'certified'}

def test_unknown_cache_fields_rejected():
    from types import SimpleNamespace
    with pytest.raises(ValueError):restore_execution_cache(SimpleNamespace(),{'ctx':[]})
