"""Lazy validation, native rejection and precise error contracts; no devices."""
import copy
from types import SimpleNamespace
import pytest
import controller
import assembly_plan as ap
from test_composition import SceneContext


def planner():
    s=SceneContext()
    return ap.Planner(s,s.scene,s.robot(),[[0.,0.,0.]]*16,[[0.,0.,0.]]*16)


def check(key):
    return {'native':{'deferred':True,'collision_checked':False},
            '_pending_native':{'key':key,'arguments':{'start14':[0.]*14,'target14':[.01]*6+[0.]*8,'obstacles':[]}}}


def test_complete_candidate_requires_every_native_result():
    p=planner();seen=[]
    p.ctx=SimpleNamespace(value=lambda name,**kw:seen.append(name) or {'cad_clear':True,'workspace_clear':True})
    rows=[check('first'),check('second')]
    assert not seen
    p.certify_moves([{'actions':[{'check':r} for r in rows]}])
    assert seen==['piper_check_path']*2
    assert all(r['native']['cad_clear'] and '_pending_native' not in r for r in rows)
    assert p.lazy_stats['native_certified_actions']==2


def test_native_rejection_retained_and_stale_routes_invalidated():
    p=planner();seen=[]
    p.ctx=SimpleNamespace(value=lambda name,**kw:seen.append(name) or {'cad_clear':False,'workspace_clear':True,'contacts':['collision']})
    p.route_cache['stale']={};p.path_cache['stale']={};p.prefix_cache={'stale':{}}
    rows=[check('bad'),check('not_reached')]
    with pytest.raises(ap.NativePathRefusal):p.certify_moves([{'actions':[{'check':r} for r in rows]}])
    assert len(seen)==1 and p.native_cache['bad']['cad_clear'] is False
    assert not p.route_cache and not p.path_cache and not p.prefix_cache
    assert '_pending_native' in rows[0] and '_pending_native' in rows[1]
    with pytest.raises(ap.NativePathRefusal):p.certify_moves([{'actions':[{'check':check('bad')}]}])
    assert len(seen)==1  # Exact rejected edge is not recalculated.


def test_connection_failure_is_not_cached_as_collision_or_accepted():
    p=planner()
    def broken(*a,**kw):raise ConnectionError('The robot service closed the connection')
    p.ctx=SimpleNamespace(value=broken);r=check('edge')
    with pytest.raises(ConnectionError):p.certify_moves([{'actions':[{'check':r}]}])
    assert 'edge' not in p.native_cache and '_pending_native' in r


def test_unchecked_plan_cannot_be_certified():
    p=planner()
    with pytest.raises(RuntimeError,match='lacks native certification'):
        p.certify_moves([{'actions':[{'check':{'native':{'deferred_to_full_path':True}}}]}])


@pytest.mark.parametrize('exception,category',[
    (ConnectionError('The robot service closed the connection'),'service_connection'),
    (TimeoutError('RPC timeout'),'service_timeout'),
    (OSError('file unavailable'),'io_error'),
    (ap.PlanningTimeout('90 second planning budget'),'planning_budget'),
    (ap.IKUnavailable('No full-pose IK'),'ik_no_solution'),
    (ap.NativePathRefusal('Native collision'),'native_path_collision'),
    (ap.NoPlan('Finger/object clearance'),'planning_or_geometry'),
])
def test_error_categories_not_missing_images(exception,category):
    def operation(r):raise exception
    r=controller._run(SimpleNamespace(),'whole_task_preflight',operation)
    assert not r['ok'] and r['failure_category']==category
    assert 'Missing scene evidence' not in r['why'] and not r['needs_review']
    assert not r['dispatch_attempted'] and 'hold' not in r


def test_transport_failure_after_dispatch_is_still_control_uncertain():
    calls=[]
    def operation(r):
        r['dispatch_attempted']=True
        raise ConnectionError('connection lost during motion')
    r=controller._run(SimpleNamespace(value=lambda name:calls.append(name) or {'stop_requested':True}),'move',operation)
    assert r['control_uncertain'] and calls==['piper_hold']
    assert not r['action_completed'] and r['failure_category']=='service_connection'


def test_cancelled_exception_classified_without_inventing_geometry_error():
    class Cancelled(RuntimeError):pass
    def operation(r):raise Cancelled('operator stopped')
    r=controller._run(SimpleNamespace(),'whole_task_preflight',operation)
    assert r['status']=='cancelled' and r['failure_category']=='cancelled'


def test_plan_failure_always_disables_lazy_mode(monkeypatch):
    p=planner()
    def broken(start):
        p.defer_native=True
        raise ConnectionError('service closed')
    monkeypatch.setattr(p,'_plan',broken)
    with pytest.raises(ConnectionError):p.plan([0.]*14)
    assert not p.defer_native and not p.search_active


def test_search_and_execution_accept_same_clearance(monkeypatch):
    # Two fingers end 3.1mm above the cube: above existing 2.5mm minimum,
    # below the old search-only 3.5mm threshold. No native call on rejected paths.
    p=planner();s=copy.deepcopy(p.scene);center=ap.xyz(s,'orange')
    q=[0.]*14;q[6]=ap.OPEN
    import numpy as np
    points=np.array([[dx,dy,dz] for dx in (-.002,.002) for dy in (-.002,.002) for dz in (0.,.002)])
    p.local={ap.OPEN:np.vstack([points,points]),ap.CLOSED:np.vstack([points,points])}
    T=np.eye(4);T[:3,3]=center+[0.,0.,.0031]
    monkeypatch.setattr(p,'fk',lambda q:{'position':T[:3,3].tolist(),'T_base_flange':T.tolist()})
    p.robot['workspace_min']=[-1.,-1.,-1.];p.robot['workspace_max']=[1.,1.,1.]
    monkeypatch.setattr(p,'progress',lambda **kw:None)
    for searching in (False,True):
        p.search_active=searching
        r=p.check(q,q,s,active='orange',geometry_only=True)
        assert r['min_neighbor_m']==pytest.approx(.0031)
