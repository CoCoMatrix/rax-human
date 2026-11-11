"""Offline left flange pose IK. Dispatch remains with the existing joint owner."""
import numpy as np
from scipy.spatial.transform import Rotation
from .calibration_cartesian import CartesianJog


class CartesianPlanner:
    def __init__(self):self.kinematics=CartesianJog()

    def warmup(self):
        from .moveit_ik import client
        return client()

    def solve(self, position, quaternion_xyzw, seed):
        quat=np.asarray(quaternion_xyzw,float)
        if quat.shape!=(4,) or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>1e-6:
            raise ValueError("unit quaternion XYZW required")
        return self.warmup().solve(position,Rotation.from_quat(quat).as_matrix(),seed)

    def plan(self, measured14, position, quaternion_xyzw=None, *, max_joint_delta=.02,max_translation_m=.02):
        q14=np.asarray(measured14,dtype=float);position=np.asarray(position,dtype=float)
        if q14.shape!=(14,) or position.shape!=(3,) or not np.isfinite(q14).all() or not np.isfinite(position).all():
            raise ValueError('finite measured14 and left-base XYZ required')
        k=self.kinematics;q=k.vector(q14[:6]);start=k.fk(q);goal=start.copy();goal[:3,3]=position
        if not np.isfinite(max_translation_m) or not 0<max_translation_m<=.1:raise ValueError('invalid Cartesian cap')
        if np.linalg.norm(position-start[:3,3])>max_translation_m+1e-9:raise ValueError('Cartesian step exceeds operator scope')
        if quaternion_xyzw is not None:
            quat=np.asarray(quaternion_xyzw,dtype=float)
            if quat.shape!=(4,) or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.001:
                raise ValueError('unit quaternion XYZW required')
            goal[:3,:3]=Rotation.from_quat(quat).as_matrix()
        if not 0<max_joint_delta<=np.deg2rad(60):raise ValueError('invalid joint step cap')
        from .moveit_ik import client
        solved=client().solve(position,goal[:3,:3],q,max_joint_delta=max_joint_delta)
        target=q14.copy();target[:6]=solved['q']
        pe=solved['position_error_m'];re=solved['orientation_error_rad']
        return dict(target14=target.tolist(),frame='left_base_link',controlled_link='left_link6',
            target_transform=goal.tolist(),ik_position_error_m=pe,ik_angle_error_rad=re,
            calibrated_tcp=False,executed=False,solver=solved["solver"],ik=solved)
