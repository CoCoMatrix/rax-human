"""Measured camera candidates for inspection, never a hardware release gate."""
import hashlib
import json
from pathlib import Path
import numpy as np

from .calibration import transform
from .geometry import URDF_SHA


class CameraCandidate:
    def __init__(self, path, sha256, intrinsics_sha256):
        raw=Path(path).read_bytes()
        if not isinstance(sha256,str) or len(sha256)!=64 or hashlib.sha256(raw).hexdigest()!=sha256:
            raise ValueError('Camera candidate SHA mismatch')
        row=json.loads(raw)
        expected=dict(schema='rax.piper.eye_to_hand_candidate.v1',base_frame='left_base_link',
            camera_frame='global_d435_color_optical_frame',camera_serial='262522073779',
            urdf_sha256=URDF_SHA,intrinsics_sha256=intrinsics_sha256,units='m')
        if any(row.get(k)!=v for k,v in expected.items()):
            raise ValueError('Camera candidate frame, units or identity mismatch')
        # A JSON flag cannot authorize motion. Accepted calibration will need
        # independent physical evidence and a separate reviewed release path.
        if row.get('candidate_only') is not True or row.get('physical_acceptance') is not False or row.get('motion_permit') is not False:
            raise ValueError('Only explicitly unaccepted camera candidates are supported')
        self.T=transform(row['T_base_camera']);self.sha256=sha256;self.row=row

    def status(self):
        return dict(loaded=True,sha256=self.sha256,base_frame=self.row['base_frame'],
            camera_frame=self.row['camera_frame'],T_base_camera=self.T.tolist(),
            numerical_quality_pass=self.row.get('numerical_quality_pass') is True,
            validation=self.row.get('validation',self.row.get('heldout')),
            candidate_only=True,physical_acceptance=False,motion_permit=False)

    def points(self, points):
        a=np.asarray(points,dtype=float)
        if a.ndim!=2 or a.shape[1]!=3 or not np.isfinite(a).all():
            raise ValueError('Finite camera XYZ points in metres required')
        return (a@self.T[:3,:3].T+self.T[:3,3]).tolist()
