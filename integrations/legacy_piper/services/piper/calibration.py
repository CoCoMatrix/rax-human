"""Operator-only calibration mathematics. Solving never authorizes physical motion."""
import hashlib
import json
import numpy as np
from .geometry import URDF_SHA, ORDER


def transform(value):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9,rtol=0): raise ValueError('finite homogeneous SE3 required')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6,rtol=0) or not np.isclose(np.linalg.det(R),1,atol=1e-6,rtol=0): raise ValueError('proper orthonormal rotation required')
    return T


def fit_points(camera_points_m, base_points_m, heldout_camera_m, heldout_base_m):
    """Rigid alignment of independently measured 3D correspondences in METRES.
    A plane is sufficient; collinear points are not. Holdouts are mandatory.
    Does not use object poses from simulation or mark measurements as accepted.
    """
    a,b=np.asarray(camera_points_m,dtype=float),np.asarray(base_points_m,dtype=float)
    h,k=np.asarray(heldout_camera_m,dtype=float),np.asarray(heldout_base_m,dtype=float)
    for x,y,minimum in ((a,b,4),(h,k,3)):
        if x.ndim!=2 or x.shape[1]!=3 or x.shape!=y.shape or len(x)<minimum or not np.isfinite(x).all() or not np.isfinite(y).all(): raise ValueError('matched finite 3D fit/holdout points required')
        if np.linalg.svd(x-x.mean(0),compute_uv=False)[1]<.005 or np.linalg.svd(y-y.mean(0),compute_uv=False)[1]<.005: raise ValueError('degenerate or insufficient spatial spread')
        if len(np.unique(x,axis=0))!=len(x) or len(np.unique(y,axis=0))!=len(y): raise ValueError('duplicate calibration landmarks')
    if any(np.linalg.norm(p-q)<1e-8 for train,test in ((a,h),(b,k)) for p in train for q in test): raise ValueError('holdout reuses fitting points')
    U,S,Vt=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)))
    D=np.eye(3); D[2,2]=np.linalg.det(Vt.T@U.T)
    R=Vt.T@D@U.T; t=b.mean(0)-R@a.mean(0)
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=t
    errors=np.linalg.norm(h@R.T+t-k,axis=1)
    fit_errors=np.linalg.norm(a@R.T+t-b,axis=1)
    return dict(T_base_camera=transform(T).tolist(),heldout_rmse_m=float(np.sqrt(np.mean(errors**2))),heldout_max_m=float(errors.max()),
                fit_rmse_m=float(np.sqrt(np.mean(fit_errors**2))),fit_max_m=float(fit_errors.max()),
                fit_errors_m=fit_errors.tolist(),heldout_errors_m=errors.tolist(),
                candidate_only=True,physical_acceptance=False,fit_points=len(a),heldout_points=len(h))


def assess_points(data):
    """Solve an operator measurement record; numerical quality is NOT release.

    Limits are supplied from the actual grasp/placement tolerance budget, not
    invented by a solver. Landmark labels and original capture references make
    swapped correspondences and circular CAD-based 'measurements' reviewable.
    """
    required={'schema','units','camera_frame','base_frame','camera_serial','urdf_sha256',
              'measurement_method','measured_by','fit','heldout','limits_m'}
    if not isinstance(data,dict) or set(data)!=required:
        raise ValueError('calibration record fields do not match measurement schema')
    if (data['schema']!='rax.piper.point_measurements.v1' or data['units']!='m'
            or data['camera_frame']!='global_d435_color_optical_frame'
            or data['base_frame'] not in ('left_base_link','right_base_link','dual_base_link')
            or data['camera_serial']!='262522073779' or data['urdf_sha256']!=URDF_SHA):
        raise ValueError('calibration frame, units or device/model identity mismatch')
    for name in ('measurement_method','measured_by'):
        if not isinstance(data[name],str) or not data[name].strip():
            raise ValueError('independent measurement provenance required: '+name)
    limits=data['limits_m']
    if not isinstance(limits,dict) or set(limits)!={'fit_max','heldout_max'}:
        raise ValueError('fit_max and heldout_max tolerance budgets required')
    for value in limits.values():
        if type(value) not in (int,float) or not np.isfinite(value) or value<=0:
            raise ValueError('positive finite tolerance budget in metres required')
    groups=[];ids=set()
    for name in ('fit','heldout'):
        rows=data[name]
        if not isinstance(rows,list):raise ValueError('landmark rows required')
        camera=[];base=[]
        for row in rows:
            if not isinstance(row,dict) or set(row)!={'id','camera_m','base_m','capture_reference'}:
                raise ValueError('landmark fields invalid')
            for key in ('id','capture_reference'):
                if not isinstance(row[key],str) or not row[key].strip():raise ValueError('landmark identity and capture reference required')
            if row['id'] in ids:raise ValueError('landmark reused between fit and holdout')
            ids.add(row['id']);camera.append(row['camera_m']);base.append(row['base_m'])
        groups.extend((camera,base))
    result=fit_points(*groups)
    result.update(schema='rax.piper.point_candidate.v1',units='m',
                  camera_frame=data['camera_frame'],base_frame=data['base_frame'],
                  camera_serial=data['camera_serial'],urdf_sha256=URDF_SHA,
                  limits_m=limits,numerical_quality_pass=(result['fit_max_m']<=limits['fit_max']
                      and result['heldout_max_m']<=limits['heldout_max']),
                  motion_permit=False,
                  remaining=['independent TCP/orientation and physical reach checks',
                             'scene/payload clearance and controller/stop characterization',
                             'reviewed sole-owner task transport'])
    return result


def fit_pivot(flange_transforms):
    """TCP position from >=6 recorded flange poses at one fixed pivot.
    This validates only numerical identifiability; no movement is initiated.
    TCP orientation cannot be inferred from pivot samples.
    """
    poses=[transform(T) for T in flange_transforms]
    if len(poses)<6: raise ValueError('at least six varied pivot poses required')
    A=np.vstack([np.c_[T[:3,:3],-np.eye(3)] for T in poses]); b=np.concatenate([-T[:3,3] for T in poses])
    if np.linalg.matrix_rank(A)!=6 or np.linalg.cond(A)>100: raise ValueError('degenerate pivot orientations')
    x,_,_,_=np.linalg.lstsq(A,b,rcond=None)
    residual=(A@x-b).reshape(-1,3)
    return dict(tcp_in_flange_m=x[:3].tolist(),pivot_in_base_m=x[3:].tolist(),rmse_m=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
                orientation_measured=False,candidate_only=True,physical_acceptance=False)


def nominal_manifest(geometry):
    geometry.model.update_cfg(dict(zip(ORDER,np.zeros(14))))
    frames={name:transform(geometry.model.get_transform(name)).tolist() for name in ('left_base_link','right_base_link','global_d435_color_optical_frame')}
    left,right=map(np.asarray,(frames['left_base_link'],frames['right_base_link']))
    frames['T_left_base_right_base']=(np.linalg.inv(left)@right).tolist()
    return dict(schema='rax.piper.nominal_calibration.v1',urdf_sha256=URDF_SHA,units='m/rad',base_frame='dual_base_link',
                frames=frames,tcp=geometry.tcp,source='v12 with operator-measured 32cm mast revision',
                camera_extrinsics_measured=False,dual_base_measured=False,tcp_measured=False,physical_acceptance=False)

if __name__ == '__main__':
    import argparse
    from pathlib import Path
    from types import SimpleNamespace
    from .geometry import PiperNoSendBackend
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['nominal','points','pivot','assess'],required=True)
    parser.add_argument('--input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.mode=='nominal':
        result=nominal_manifest(PiperNoSendBackend(SimpleNamespace(backend_options={})))
    else:
        if args.input is None:parser.error('--input required')
        raw=args.input.read_bytes();data=json.loads(raw)
        if args.mode=='points':result=fit_points(**data)
        elif args.mode=='assess':result=assess_points(data)
        else:result=fit_pivot(**data)
        result['input_sha256']=hashlib.sha256(raw).hexdigest()
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'saved':str(args.output),'physical_acceptance':False}))
    if result.get('numerical_quality_pass') is False:raise SystemExit(2)
