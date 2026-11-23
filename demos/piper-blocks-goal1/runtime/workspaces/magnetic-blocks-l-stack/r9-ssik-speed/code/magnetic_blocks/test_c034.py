"""Source screening may not equate one overhead pose family with reachability."""
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import assembly_plan as ap
from test_composition import SceneContext


def planner():
    scene=ap.initial_scene(SceneContext().scene)
    fingers=np.concatenate([ap.box_corners([x-.007,-.028,.066],[x+.007,.028,.170]) for x in (-.0435,.0435)])
    p=ap.Planner(SimpleNamespace(),scene,{'workspace_min':[.055,-.4,.17],'workspace_max':[.5,.045,.36]},fingers,fingers)
    return p


def test_source_grasp_solution_reaches_route_search_even_without_overhead_ik():
    p=planner();queries=[];p.check=lambda *a,**kw:{}
    def solve(position,q,seed):
        queries.append(position)
        if not any(np.allclose(position,ap.xyz(p.scene,c)+[0,0,.128]) for c in p.scene['blocks']):
            raise ap.IKUnavailable('No same-column overhead')
        return list(seed)
    p.solve=solve
    with patch.object(ap,'layout_candidates',side_effect=RuntimeError('destination search reached')):
        with pytest.raises(RuntimeError,match='destination search reached'):p._plan([0.]*14)
    assert not queries  # No task-wide source-grasp gate before choosing moved objects.


def test_missing_source_grasp_cannot_reject_stationary_layout_before_search():
    p=planner();p.check=lambda *a,**kw:{}
    p.solve=lambda *a,**kw:(_ for _ in ()).throw(ap.IKUnavailable('No grasp'))
    with patch.object(ap,'layout_candidates',side_effect=RuntimeError('layout search reached')):
        with pytest.raises(RuntimeError,match='layout search reached'):p._plan([0.]*14)


@pytest.mark.parametrize('yaw',[0.,180.,-90.,90.])
def test_offset_entries_keep_original_table_and_contact_corridor(yaw):
    p=planner();src=ap.xyz(p.scene,'green')
    rotated=p.local[ap.OPEN]@ap.Rotation.from_quat(ap.quat(yaw)).as_matrix().T
    candidates=p.source_approaches(p.scene,'green',yaw,'adaptive')
    assert any(np.linalg.norm(np.array(c['position_m'])[:2]-src[:2])>.009 for c in candidates)
    for candidate in candidates:
        point=np.array(candidate['position_m'])
        assert np.linalg.norm(point[:2]-src[:2])<.012
        gap=min(v[2]-ap.table(p.scene,v) for v in rotated+point)
        assert gap>=ap.MIN_TABLE+.020+.002-1e-10
        native_table=ap.boxes(p.scene,(),table_allowance=.032)[0]
        table_top=native_table['center_m'][2]+native_table['size_m'][2]/2
        assert min((rotated+point)[:,2])+.008-table_top>=.00325-1e-10


def test_existing_overhead_family_preserved():
    p=planner();src=ap.xyz(p.scene,'orange')
    options=p.source_approaches(p.scene,'orange',180.,(.195,.180,.215))
    assert all(c['strategy']=='overhead' and np.allclose(c['position_m'][:2],src[:2]) for c in options)
    lower=p.source_approaches(p.scene,'orange',180.,'low_overhead')
    assert all(c['strategy']=='low_overhead_entry' and np.allclose(c['position_m'][:2],src[:2]) for c in lower)


def test_malformed_geometry_never_produces_nan_candidate():
    p=planner();p.local[ap.OPEN]=np.zeros((16,3))
    with pytest.raises(ap.NoPlan,match='Invalid open-finger geometry'):
        p.source_approaches(p.scene,'green',180.,'adaptive')
