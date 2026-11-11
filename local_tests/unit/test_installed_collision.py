"""Real FCL regression; runs on the 5090 with its pinned CAD installation."""
import os
from pathlib import Path
import numpy as np
import pytest
from services.piper.finger_geometry import finger_envelopes,geometry_identity
from services.piper.installed_collision import load_checker

ROOT=Path(os.environ.get('RAX_COLLISION_LEGACY_ROOT','/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025'))
pytestmark=pytest.mark.skipif(not (ROOT/'services/piper/collision.py').exists(),reason='Real pinned FCL assets are on 5090')

@pytest.fixture(scope='module')
def checker():return load_checker(ROOT)

@pytest.mark.parametrize('jaw',[0,.0007,.0096,.02,.075])
@pytest.mark.parametrize('pose',[[0.]*6,[0,.4661948965,-.4611159884,.7455697499,0,0]])
def test_projection_and_actual_collision_vertices_agree(checker,jaw,pose):
    q=pose+[jaw]+[0.]*6+[.0021];report=checker.check(q)
    assert report['geometry_identity']==geometry_identity()
    assert report['cad_clear'],report['contacts']
    flange=checker.scene.graph['left_link6'][0]
    for row in finger_envelopes(q):
        node=next(n for n,l in checker.links.items() if l==f"left_gripper_link{row['finger']}")
        actual=checker.local_bounds[node]@checker.scene.graph[node][0].T
        expected=np.c_[row['corners_flange_m'],np.ones(8)]@flange.T
        assert np.allclose(actual,expected,atol=1e-8)

def test_extension_blocks_obstacle_old_fingers_miss(checker):
    q=[0.]*14;q[6]=.02;q[13]=.0021;checker.check(q)
    corners=np.array(finger_envelopes(q)[0]['corners_flange_m'])
    center=corners.mean(axis=0);center[2]=.161
    world=checker.scene.graph['left_link6'][0]@np.r_[center,1]
    obstacle={'name':'tip_extension_probe','center_m':world[:3].tolist(),'size_m':[.004]*3}
    old=checker.__class__.__bases__[0]()
    assert old.check(q,obstacles=[obstacle])['cad_clear']
    result=checker.check(q,obstacles=[obstacle])
    assert not result['cad_clear'] and result['environment_checked']
    assert any('environment:tip_extension_probe' in c['links'] for c in result['contacts'])

def test_path_and_empty_environment_are_explicit(checker):
    q=[0.]*14;q[13]=.0021;target=list(q);target[6]=.01
    result=checker.path([q,target])
    assert result['whole_path_checked'] and result['cad_clear']
    assert result['environment_checked'] is False


def test_exact_right_pair_cache_preserves_changed_scene_and_parameters(checker):
    q=[0.]*14;q[6]=.02;q[13]=.0021
    checker.check(q)
    second=checker.check(q)
    assert second['exact_static_pairs_reused']>0
    q[7]+=1e-12
    assert checker.check(q)['exact_static_pairs_reused']==0
    assert checker.check(q,margin_m=.004)['exact_static_pairs_reused']==0
    # Same right state is reusable, but a new obstacle still receives every check.
    checker.check(q)
    right=next(n for n,l in checker.links.items() if l=='right_link3')
    center=checker.scene.graph[right][0][:3,3].tolist()
    obstacle={'name':'right_scene_changed','center_m':center,'size_m':[.05]*3}
    changed=checker.check(q,obstacles=[obstacle])
    assert changed['exact_static_pairs_reused']>0
    assert not changed['cad_clear'] and any('environment:right_scene_changed' in h['links'] for h in changed['contacts'])
    baseline=checker.__class__.__bases__[0].check(checker,q,obstacles=[obstacle])
    for field in ('cad_clear','contacts','expected_gripper_contacts','checked_pairs'):
        assert changed[field]==baseline[field]


def test_collision_cache_keeps_right_collision_and_recovers_after_error(checker):
    rng=np.random.default_rng(19);found=None
    for _ in range(100):
        q=np.zeros(14);q[6]=.02;q[13]=.002
        q[7:13]=rng.uniform(checker.limits[7:13,0],checker.limits[7:13,1])
        report=checker.check(q)
        if any(all(link.startswith('right_') for link in c['links']) for c in report['contacts']):
            found=q;break
    assert found is not None
    found[0]=.1
    cached=checker.check(found)
    baseline=checker.__class__.__bases__[0].check(checker,found)
    assert cached['exact_static_pairs_reused']>0 and not cached['cad_clear']
    assert cached['contacts']==baseline['contacts']
    pairs=list(checker.pairs)
    with pytest.raises(ValueError):checker.check(found,obstacles=[{'invalid':True}])
    assert checker.pairs==pairs
    assert checker.check(found)['contacts']==baseline['contacts']
