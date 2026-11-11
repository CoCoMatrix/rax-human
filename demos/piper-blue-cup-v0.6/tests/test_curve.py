"""Pure compiler checks; no robot packages or device imports."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
p=Path(__file__).resolve().parents[1]/'host-overlay/services/piper/waypoint_plan.py'
s=importlib.util.spec_from_file_location('curve_v06',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

def compile(indices):
    seen=[]
    def path(q):seen.extend(q);return dict(cad_clear=True,workspace_clear=True)
    a=[0.]*14;points=[[.10,.02,0,0,0,0]+[0.]*8,[.2,.04,0,0,0,0]+[0.]*8,[.3,0,0,0,0,0]+[0.]*8]
    r=m.compile_arm_path(a,points,check_state=lambda q:None,check_path=path,blend_indices=indices)
    return r,seen

def test_speed_continuity_and_wire_revalidation():
    r,seen=compile([0,1]);q=np.array(r['targets14']);assert len(seen)==len(q)+1
    for w in r['waypoints'][:-1]:
        i=w['sample_end']-1
        assert w['velocity_at_waypoint'][0]>0 and q[i+1,0]>q[i,0]>q[i-1,0]
    assert r['waypoints'][-1]['velocity_at_waypoint']==[0.]*6
    assert np.all(q[:,6:]==0)
    assert r['dynamics']['wire_peak_velocity_rad_s']<=m.VELOCITY+1e-5
    assert r['dynamics']['wire_peak_acceleration_rad_s2']<=m.ACCELERATION+1e-4
    assert r['dynamics']['max_joint_line_departure_rad']<=.01

@pytest.mark.parametrize('indices',[[2],[-1],[0,0],[True]])
def test_invalid_contact_endpoint_selection_rejected(indices):
    with pytest.raises(ValueError):compile(indices)
