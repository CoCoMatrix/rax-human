"""Quarter-turn placement keeps the modeled cube upright and its goal footprint."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from assembly_plan import placement_yaws,quat,box_corners

@pytest.mark.parametrize('grasp_yaw',[180.,0.,-90.,177.,-93.])
def test_alternative_placement_preserves_upright_square_payload(grasp_yaw):
    source=np.array([.395,-.209,.078])
    target=np.array([.278,-.04,.078])
    source_corners=box_corners(source-[.03,.03,.06],source+[.03,.03,0.])
    source_R=Rotation.from_quat(quat(grasp_yaw)).as_matrix()
    local=(source_corners-(source+[0.,0.,.128]))@source_R
    alternatives=placement_yaws(grasp_yaw)
    assert len({round((x-grasp_yaw)%360.,6) for x in alternatives})==4
    for yaw in alternatives:
        target_R=Rotation.from_quat(quat(yaw)).as_matrix()
        corners=local@target_R.T+target+[0.,0.,.128]
        np.testing.assert_allclose(corners.min(0),target-[.03,.03,.06],atol=1e-12)
        np.testing.assert_allclose(corners.max(0),target+[.03,.03,0.],atol=1e-12)
        assert target_R[2,2]<-1+1e-12

@pytest.mark.parametrize('turn',[-90.,90.,180.])
def test_support_estimate_rotates_measured_offcenter_grasp(monkeypatch,turn):
    import ordinary_checks as ordinary
    import history_scene
    from types import SimpleNamespace
    from test_composition import SceneContext
    scene=SceneContext().scene
    source=np.asarray(scene['blocks']['orange']['top_center']['surface_position_m'])
    grasp=source+[.008,-.006,.128]
    initial_R=Rotation.from_quat(quat(-90.)).as_matrix()
    local_top=(source-grasp)@initial_R
    local_corners=(box_corners(source-[.03,.03,.06],source+[.03,.03,0.])-grasp)@initial_R
    target=np.array([.28,-.04,.078]);R=Rotation.from_quat(quat(-90.+turn)).as_matrix()
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=target-R@local_top
    origin={'color':'orange','grasp_flange':grasp.tolist(),'flange_to_bottom_m':.188,'object_top_flange_m':local_top.tolist(),'corners_flange_m':local_corners.tolist()}
    ctx=SimpleNamespace(value=lambda *a,**kw:{'position':T[:3,3].tolist(),'T_base_flange':T.tolist()})
    monkeypatch.setattr(ordinary,'view_evidence',lambda *a,**kw:{'visible':True})
    monkeypatch.setattr(history_scene,'reject_visible_displacement',lambda *a,**kw:{})
    result=ordinary.placement(ctx,'support',{'frame_id':'offline','measured14':[0.]*14},scene,origin)
    assert result['ok'] is True
    np.testing.assert_allclose(result['estimated_object_top_m'],target,atol=1e-12)
    assert abs(result['estimated_support_height_error_m'])<1e-12
