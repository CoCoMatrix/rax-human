from types import SimpleNamespace
import numpy as np
import pytest
from services.piper.geometry import Kinematics, URDF_SHA
from pure_fk import local_kinematics, batch_rows


class Reference:
    _blocks_local_fk = True
    def __init__(self):
        self.native = Kinematics()
        self.calls = []
    def value(self, name, **kw):
        assert name == 'piper_fk_batch'
        self.calls.append(name)
        return {'executed': False, 'frame': 'left_base_link', 'controlled_link': 'left_link6',
                'urdf_sha256': URDF_SHA,
                'T_base_flange': self.native.fk_batch(kw['joints_batch']).tolist()}


def test_local_matches_native_for_1027_samples_and_validates_once():
    c=Reference();kin=local_kinematics(c,[0.]*6)
    bounds=np.array([[lo,hi] for _,_,lo,hi in kin.chains['left']])
    qs=np.random.default_rng(31).uniform(bounds[:,0],bounds[:,1],size=(1027,6)).tolist()
    rows=batch_rows(kin,qs)
    for q,r in zip(qs,rows):
        np.testing.assert_allclose(r['T_base_flange'],c.native.fk(q),rtol=0,atol=1e-12)
    assert local_kinematics(c,qs[-1]) is kin and len(c.calls)==1


@pytest.mark.parametrize('field,value',[
    ('executed',True),('frame','dual_base_link'),('controlled_link','right_link6'),('urdf_sha256','different'),
    ('T_base_flange',[[[0.]*4]*4]*5),
])
def test_mismatched_service_never_enables_local_fk(field,value):
    c=Reference();native=c.value
    c.value=lambda *a,**kw:{**native(*a,**kw),field:value}
    with pytest.raises(ValueError):local_kinematics(c,[0.]*6)
    assert not hasattr(c,'_blocks_validated_fk')


def test_joint_limits_remain_enforced():
    c=Reference();kin=local_kinematics(c,[0.]*6);bad=[0.]*6;bad[0]=100.
    with pytest.raises(ValueError,match='limits'):kin.fk(bad)
    with pytest.raises(ValueError,match='limits'):batch_rows(kin,[bad])


def test_disabled_context_does_not_touch_backend_or_import_assets():
    assert local_kinematics(SimpleNamespace(),[0.]*6) is None


def test_single_and_batch_cache_share_exact_joint_samples():
    from assembly_plan import Planner
    from test_composition import SceneContext
    c=Reference();s=SceneContext()
    p=Planner(c,s.scene,s.robot(),[[0.]*3]*16,[[0.]*3]*16)
    qs=[[0.]*6,[.1,0.,0.,0.,0.,0.]]
    sampled=p.sample_fks(qs)
    single=p.fk(qs[1])
    assert sampled[1]['T_base_flange']==single['T_base_flange']
    assert 'quaternion_xyzw' in single
    assert [r['T_base_flange'] for r in p.sample_fks(qs+[qs[0]])]==[r['T_base_flange'] for r in sampled+[sampled[0]]]
    assert len(c.calls)==1
