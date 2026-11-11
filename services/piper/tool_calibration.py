"""Operator-measured LEFT TCP. Pure geometry; existing motion API stays flange."""
import hashlib
import json
import numpy as np
from .geometry import ASSETS, URDF_SHA

CALIBRATION_SHA = 'a33bcac99a8ff7f0a0884e12b5440976d8944db64f210af89ca395a64c14627a'

def load_calibration():
    raw=(ASSETS/'left-tcp140-tip170-20261104.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=CALIBRATION_SHA:
        raise ValueError('Tool measurement identity changed')
    data=json.loads(raw)
    if data['urdf_sha256']!=URDF_SHA or data['parent_frame']!='left_link6':
        raise ValueError('Tool measurement reference mismatch')
    return data

def tool_pose(T_base_flange):
    data=load_calibration();t=np.asarray(T_base_flange,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('Invalid flange transform')
    tcp=t@np.asarray(data['T_flange_tcp']);tip=t@np.array([0,0,data['tip_axial_m'],1])
    return {'calibration_id':data['id'],'sha256':CALIBRATION_SHA,
            'status':data['status'],'T_base_tcp':tcp.tolist(),'position':tcp[:3,3].tolist(),
            'tip_axis_point_base_m':tip[:3].tolist(),'accuracy_m':None,
            'independent_physical_validation':False,'motion_permit':False}
