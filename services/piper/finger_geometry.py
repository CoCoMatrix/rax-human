"""Read-only CAD finger geometry projected using the existing head calibration.

This supplies inspection evidence, never a motion permit or an installed TCP.
LEFT operator measurement now supplies140mm TCP and170mm tip. No wrist hand-eye is invented.
"""
import hashlib
import itertools
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import ASSETS, Kinematics, vector

BOUNDS_SHA='fb77932538519c2c7cb9a2312e6130b4524893990b6ba9dfd260befa05896f59'


def finger_envelopes(measured14):
    """Installed LEFT finger boxes, in link6, shared by inspection and FCL.

    Original transverse bounds, operator measured axial tip. No environment or
    accuracy claim. Transforming these boxes must not change the jaw-gap axis.
    """
    q=vector(measured14,14)
    if not 0<=q[6]<=.075:raise ValueError('Invalid measured full jaw gap')
    raw=(ASSETS/'finger-cad-bounds.json').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=BOUNDS_SHA:raise ValueError('Finger bounds identity changed')
    bounds=json.loads(raw)
    tree=ET.parse(ASSETS/'dual_piper_x.urdf').getroot()
    def origin(name):
        j=tree.find(f"joint[@name='{name}']");o=j.find('origin');t=np.eye(4)
        t[:3,3]=np.fromstring(o.attrib['xyz'],sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.attrib['rpy'],sep=' ')).as_matrix()
        return j,t
    _,gripper=origin('left_gripper_base_joint')
    from .tool_calibration import load_calibration
    calibration=load_calibration()
    rows=[]
    for i in (1,2):
        j,t=origin('left_gripper_joint'+str(i));mimic=j.find('mimic')
        displacement=q[6]*float(mimic.attrib['multiplier'])+float(mimic.attrib.get('offset',0))
        shift=np.eye(4);shift[:3,3]=np.fromstring(j.find('axis').attrib['xyz'],sep=' ')*displacement
        b=bounds['finger_bounds'][str(i)]
        corners=np.array(list(itertools.product(*zip(b['min'],b['max']))))
        flange=(gripper@t@shift@np.c_[corners,np.ones(8)].T).T[:,:3]
        lower,upper=flange.min(axis=0),flange.max(axis=0)
        upper[2]=max(upper[2],calibration['tip_axial_m'])
        envelope=np.array(list(itertools.product(*zip(lower,upper))))
        rows.append({'finger':i,'corners_flange_m':envelope.tolist(),
                     'T_flange_mesh':(gripper@t@shift).tolist()})
    return rows


def geometry_identity():
    from .tool_calibration import CALIBRATION_SHA
    from .geometry import URDF_SHA
    return {'urdf_sha256':URDF_SHA,'cad_bounds_sha256':BOUNDS_SHA,
            'tool_calibration_sha256':CALIBRATION_SHA,
            'representation':'link6_axis_aligned_finger_envelopes_v1'}


def inspection_geometry(measured14, head):
    q=vector(measured14,14)
    if head.get('metric_depth') is not True:raise ValueError('Bound calibrated head required')
    tcam=np.asarray(head['T_base_camera'],dtype=float);k=np.asarray(head['K'],dtype=float)
    if tcam.shape!=(4,4) or k.shape!=(3,3) or not np.isfinite(tcam).all() or not np.isfinite(k).all():raise ValueError('Invalid head calibration')
    base_flange=Kinematics().fk(q[:6].tolist());camera_base=np.linalg.inv(tcam);rows=[]
    from .tool_calibration import load_calibration,tool_pose
    calibration=load_calibration()
    for row in finger_envelopes(measured14):
        envelope=np.asarray(row['corners_flange_m'])
        base=(base_flange@np.c_[envelope,np.ones(8)].T).T[:,:3]
        cam=(camera_base@np.c_[base,np.ones(8)].T).T[:,:3]
        pixels=[]
        for p in cam:
            uv=k@p;pixels.append((uv[:2]/uv[2]).tolist() if p[2]>0 else None)
        rows.append({'finger':row['finger'],'cad_corners_base_m':base.tolist(),'head_pixels_native':pixels,'nominal_min_z_m':float(base[:,2].min())})
    return {'status':'operator_measured_axial_extent_CAD_transverse_envelope','physical_clearance_authority':False,
            'installed_tcp_validated':False,'wrist_handeye_available':False,
            'measured_full_gap_m':float(q[6]),'head_calibration_provisional':True,
            'geometry_sha256':BOUNDS_SHA,'geometry_identity':geometry_identity(),'fingers':rows,
            'effective_contact_tcp':tool_pose(base_flange),
            'tip_axial_m':calibration['tip_axial_m'],
            'limitations':['Axial dimensions and alignment are operator-reported; accuracy unmeasured.',
                           'Prior CAD transverse bounds retained; conservative axial envelope extended to170mm.',
                           'No full environment collision proof; head calibration remains provisional.']}
