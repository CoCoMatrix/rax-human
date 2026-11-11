"""Operator-only ChArUco eye-to-hand measurements. Never moves or arms a robot.

T_a_b maps coordinates in b into a. The fixed head camera observes a board
rigidly attached to the LEFT flange. Solve G_i B = X C_i using OpenCV, where
G=T_base_flange, B=T_flange_board, X=T_base_camera, C=T_camera_board.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import transform
from .geometry import URDF_SHA


def board_from_config(config):
    keys={'type','squares_x','squares_y','square_length_m','marker_length_m',
          'dictionary','legacy_pattern','print_scale_verified'}
    if set(config)!=keys or config['type']!='charuco':raise ValueError('ChArUco board schema required')
    if config['print_scale_verified'] is not True:raise ValueError('Measure the printed board scale first')
    if config['legacy_pattern'] is not False or config['dictionary']!='DICT_4X4_100':
        raise ValueError('Only the supplied non-legacy DICT_4X4_100 board is supported')
    if any(type(config[k]) is not int or not 3<=config[k]<=15 for k in ('squares_x','squares_y')):
        raise ValueError('Invalid board grid')
    square,marker=config['square_length_m'],config['marker_length_m']
    if any(type(v) not in (int,float) or not np.isfinite(v) for v in (square,marker)) or not 0<marker<square<.1:
        raise ValueError('Board dimensions must be measured metres')
    return cv2.aruco.CharucoBoard((config['squares_x'],config['squares_y']),square,marker,
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100))


def detect(rgb, calibration, board_config):
    board=board_from_config(board_config)
    color=calibration['profiles']['color']
    if rgb.shape!=(color['height'],color['width'],3):
        raise ValueError('Native RGB size required; never use resized UI preview with native intrinsics')
    K=np.asarray(color['K'],dtype=float);dist=np.asarray(color['distortion_coeffs'],dtype=float)
    if not np.isfinite(K).all() or not np.isfinite(dist).all():raise ValueError('Invalid intrinsics')
    # RealSense inverse Brown is not OpenCV forward Brown. Zero coefficients
    # (this pinned head camera) coincide; otherwise require an explicit conversion.
    if color['distortion_model'] not in ('distortion.brown_conrady','distortion.none') and np.any(dist):
        raise ValueError('Unsupported nonzero distortion model; do not silently substitute OpenCV Brown')
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    corners,ids,_,_=cv2.aruco.CharucoDetector(board).detectBoard(gray)
    if ids is None or len(ids)<12:raise ValueError('At least 12 visible ChArUco corners required')
    if board.checkCharucoCornersCollinear(ids):raise ValueError('Collinear board corners')
    obj,img=board.matchImagePoints(corners,ids)
    ok,rvec,tvec=cv2.solvePnP(obj,img,K,dist,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:raise ValueError('Board pose solve failed')
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rvec)[0];T[:3,3]=tvec.ravel();transform(T)
    if np.any((obj.reshape(-1,3)@T[:3,:3].T+T[:3,3])[:,2]<=0):raise ValueError('Board behind camera')
    projected,_=cv2.projectPoints(obj,rvec,tvec,K,dist)
    error=np.linalg.norm(projected.reshape(-1,2)-img.reshape(-1,2),axis=1)
    return dict(T_camera_board=T.tolist(),corners=len(ids),corner_ids=ids.ravel().tolist(),
                reprojection_rmse_px=float(np.sqrt(np.mean(error**2))),
                reprojection_max_px=float(error.max()))


def solve(samples, *, max_position_m, max_angle_deg, max_reprojection_px):
    for value in (max_position_m,max_angle_deg,max_reprojection_px):
        if type(value) not in (int,float) or not np.isfinite(value) or value<=0:
            raise ValueError('Positive finite independent tolerance budgets required')
    if not samples:raise ValueError('No measured poses')
    groups={'fit':[],'heldout':[]};ids=set();identities=set();poses=[]
    for row in samples:
        if row.get('schema')!='rax.piper.handeye_sample.v1' or row.get('split') not in groups:
            raise ValueError('Measured sample schema/split required')
        if row.get('urdf_sha256')!=URDF_SHA or row.get('base_frame')!='left_base_link':
            raise ValueError('Robot model/base identity mismatch')
        if row.get('camera_serial')!='262522073779' or row.get('stationary_capture') is not True:
            raise ValueError('Pinned head camera and stationary capture required')
        sid=row['sample_id']
        if not isinstance(sid,str) or not sid or sid in ids:raise ValueError('Reused sample ID')
        ids.add(sid)
        identities.add((row['board_sha256'],row['intrinsics_sha256']))
        G=transform(row['T_base_flange']);C=transform(row['T_camera_board'])
        if any(np.allclose(G,previous,atol=1e-6,rtol=0) for previous in poses):
            raise ValueError('Repeated robot pose; collect independent heldout poses')
        poses.append(G)
        error=row['reprojection_max_px']
        if type(error) not in (int,float) or not np.isfinite(error) or not 0<=error<=max_reprojection_px:
            raise ValueError('Board reprojection error exceeds supplied budget')
        groups[row['split']].append((G,C))
    if len(identities)!=1:raise ValueError('Board or intrinsics changed between samples')
    fit,heldout=groups['fit'],groups['heldout']
    if len(fit)<8 or len(heldout)<3:raise ValueError('At least 8 fit and 3 independent heldout poses required')
    rotations=np.array([Rotation.from_matrix(fit[0][0][:3,:3].T@G[:3,:3]).as_rotvec() for G,_ in fit[1:]])
    sv=np.linalg.svd(rotations,compute_uv=False)
    if sv[1]<.1 or sv[1]/sv[0]<.1:raise ValueError('Insufficient nonparallel rotation excitation')
    inverses=[np.linalg.inv(G) for G,_ in fit]
    # Eye-to-hand: INVERT base<-flange before using the standard hand-eye API.
    R,t=cv2.calibrateHandEye([G[:3,:3] for G in inverses],[G[:3,3] for G in inverses],
        [C[:3,:3] for _,C in fit],[C[:3,3] for _,C in fit],method=cv2.CALIB_HAND_EYE_PARK)
    X=np.eye(4);X[:3,:3]=R;X[:3,3]=t.ravel();transform(X)
    boards=[np.linalg.inv(G)@X@C for G,C in fit]
    B=np.eye(4);B[:3,:3]=Rotation.from_matrix(np.array([v[:3,:3] for v in boards])).mean().as_matrix()
    B[:3,3]=np.mean([v[:3,3] for v in boards],axis=0)
    def errors(group):
        measured=[np.linalg.inv(G)@X@C for G,C in group]
        pos=[float(np.linalg.norm(v[:3,3]-B[:3,3])) for v in measured]
        angle=[float(np.degrees(Rotation.from_matrix(B[:3,:3].T@v[:3,:3]).magnitude())) for v in measured]
        return dict(position_errors_m=pos,angle_errors_deg=angle,max_position_m=max(pos),max_angle_deg=max(angle))
    a,b=errors(fit),errors(heldout)
    return dict(schema='rax.piper.eye_to_hand_candidate.v1',T_base_camera=X.tolist(),
        T_flange_board=B.tolist(),base_frame='left_base_link',camera_frame='global_d435_color_optical_frame',
        camera_serial='262522073779',urdf_sha256=URDF_SHA,units='m',fit=a,heldout=b,
        board_sha256=samples[0]['board_sha256'],intrinsics_sha256=samples[0]['intrinsics_sha256'],
        method='OpenCV PARK; eye-to-hand with inverted base/flange poses',
        numerical_quality_pass=all(v['max_position_m']<=max_position_m and v['max_angle_deg']<=max_angle_deg for v in (a,b)),
        limits=dict(max_position_m=max_position_m,max_angle_deg=max_angle_deg,max_reprojection_px=max_reprojection_px),
        candidate_only=True,physical_acceptance=False,motion_permit=False,
        limitation='Depends on measured robot feedback and nominal FK; TCP, depth scale and independent physical reach still require verification')


def capture_status_mode(status, *, allow_calibration_hold=False, allow_manual_disabled=False):
    if status.get('hardware_write_enabled') is False:return 'no_write'
    c=status.get('calibration_control',{})
    if c.get('workflow')=='drag_hold_capture' and (
            c.get('capture_ready') is not True or c.get('hold_verified') is not True
            or c.get('controller_mode')!=1 or c.get('drag_error')):
        raise ValueError('拖动采集需要已确认的原位 Hold；切换中或异常时不能采集')
    common=(status.get('hardware_write_enabled') is True
            and c.get('schema')=='rax.piper.calibration_control.v1'
            and not c.get('error') and not c.get('enable_error')
            and status.get('lifecycle_state')=='idle' and not status.get('boundary_event')
            and status.get('takeover') is False and status.get('recording') is False)
    if common and allow_calibration_hold and c.get('phase')=='HOLD' and c.get('disable_latched') is False and c.get('left_enable_bits')==[True]*6:
        return 'calibration_hold'
    if common and allow_manual_disabled and c.get('phase')=='DISABLED' and c.get('disable_latched') is True and c.get('left_enable_bits')==[False]*6:
        return 'manual_disabled'
    raise ValueError('采集需要已确认的 Hold 或六关节均已失能；反馈异常或状态未确认时不能采集')



def capture_motion(states):
    """Left flange/board stability only; do not mix radians and jaw metres."""
    q=np.asarray([s['robot_state']['values'] for s in states],dtype=float)
    ranges=np.ptp(q[:,:6],axis=0);jaw=float(np.ptp(q[:,6]))
    joint=int(np.argmax(ranges))
    details=dict(left_joint_range_deg=np.degrees(ranges).tolist(),
        left_gripper_range_mm=jaw*1000, joint_limit_deg=float(np.degrees(.001)),
        gripper_limit_mm=1.)
    if ranges[joint]>.001:
        return details,f'左臂 J{joint+1} 在最近 2 秒变化 {np.degrees(ranges[joint]):.3f}°（上限 {np.degrees(.001):.3f}°）；请支撑前臂和肘部，持续托稳后重采'
    if jaw>.001:
        return details,f'左夹爪开口变化 {jaw*1000:.2f} mm（上限 1 mm）；请保持夹爪和标定板固定'
    return details,''


def stable_observation(owner, depth, states, *, allow_calibration_hold=False,
                       allow_manual_disabled=False, clock=time.monotonic, sleep=time.sleep):
    """Wait up to 8 s for the original 2 s stability budget; never command motors.

    Control changes fail immediately. Only motion or a genuinely expired depth
    result may wait for a new window/frame. All raw observations remain audited.
    """
    deadline=clock()+8;initial=None;reason='尚未取得连续 2 秒的新鲜关节反馈'
    def observe():
        nonlocal initial
        state=owner.state();states.append(state)
        mode=capture_status_mode(state['robot_status'],allow_calibration_hold=allow_calibration_hold,
                                allow_manual_disabled=allow_manual_disabled)
        identity=(state['robot_status']['generation'],mode)
        if initial is None:initial=identity
        if identity!=initial:raise ValueError('采集期间控制代次或使能状态发生变化，已取消；确认状态后重新点击采集')
        return state
    def window():
        stamps=[s['robot_state']['feedback_metadata']['source_timestamp_ns'] for s in states]
        if any(b<a for a,b in zip(stamps,stamps[1:])):
            raise ValueError('关节反馈时间戳倒退，样本未保存')
        cutoff=stamps[-1]-2_000_000_000
        candidates=[i for i,t in enumerate(stamps) if t<=cutoff]
        return states[candidates[-1]:] if candidates else []
    while clock()<deadline:
        observe();selected=window()
        if selected:
            _,reason=capture_motion(selected)
            if not reason:
                try:arrays,report=depth.read()
                except ValueError as exc:
                    if not str(exc).startswith('Depth is stale/future'):raise
                    reason='等待新鲜相机帧超时；本次未保存，请重新采集'
                else:
                    observe();selected=window()
                    _,reason=capture_motion(selected)
                    stamps=[s['robot_state']['feedback_metadata']['source_timestamp_ns'] for s in selected]
                    if not reason and min(stamps)<=report['host_received_unix_ns']<=max(stamps):
                        return arrays,report,selected,initial[1]
                    if not reason:reason='图像时间戳未被静止关节反馈包围；本次未保存'
        sleep(.05)
    raise ValueError('等待托稳超时：'+reason)


def capture(depth_root, calibration_path, board_path, out, split, *, allow_calibration_hold=False, allow_manual_disabled=False, owner_reader=None):
    from .owner import OwnerReader
    from .depth import DepthReader
    from .backend import CALIBRATION_SHA
    from .geometry import PiperNoSendBackend,ORDER
    from .sensor.stereo_packet import calibration_identity
    calibration=json.loads(calibration_path.read_text());board=json.loads(board_path.read_text())
    if calibration_identity(calibration)!=CALIBRATION_SHA:raise ValueError('Intrinsics identity changed')
    board_from_config(board)
    if out.exists():raise ValueError('Use a new sample output directory')
    owner=owner_reader if owner_reader is not None else OwnerReader()
    states=[]
    try:
        arrays,report,selected,capture_mode=stable_observation(owner,DepthReader(depth_root,CALIBRATION_SHA),states,
            allow_calibration_hold=allow_calibration_hold,allow_manual_disabled=allow_manual_disabled)
    except Exception as exc:
        out.mkdir(parents=True)
        (out/'feedback.json').write_text(json.dumps(states,indent=2))
        detail=capture_motion(states)[0] if states else {}
        (out/'REJECTED.json').write_text(json.dumps(dict(error=str(exc),**detail),indent=2,ensure_ascii=False))
        raise
    q=np.array([s['robot_state']['values'] for s in selected])
    stamps=[s['robot_state']['feedback_metadata']['source_timestamp_ns'] for s in selected]
    # Use the closest actual feedback to the native RGB receipt, not the last
    # pose after depth processing; the entire enclosing window is stationary.
    matched=int(np.argmin(np.abs(np.asarray(stamps)-report['host_received_unix_ns'])))
    pose=detect(arrays['rgb'],calibration,board)
    geometry=PiperNoSendBackend(SimpleNamespace(backend_options={}))
    geometry.model.update_cfg(dict(zip(ORDER,q[matched])))
    G=geometry.model.get_transform('left_link6',frame_from='left_base_link')
    result=dict(schema='rax.piper.handeye_sample.v1',split=split,
        sample_id=report['sample_id'],T_base_flange=transform(G).tolist(),**pose,
        base_frame='left_base_link',camera_serial='262522073779',urdf_sha256=URDF_SHA,
        board_sha256=hashlib.sha256(board_path.read_bytes()).hexdigest(),
        intrinsics_sha256=CALIBRATION_SHA,stationary_capture=True,
        physical_acceptance=False,capture_mode=capture_mode,
        control_generation=selected[matched]['robot_status']['generation'],
        calibration_control=selected[matched]['robot_status'].get('calibration_control',{}),
        stationary_joint_range_rad=np.ptp(q[:,:6],axis=0).tolist(),
        feedback_bracket_ns=[min(stamps),max(stamps)],
        matched_feedback_ns=stamps[matched],
        capture_quality=capture_motion(selected)[0],
        rgb_received_ns=report['host_received_unix_ns'],source_report=report)
    out.mkdir(parents=True)
    if not cv2.imwrite(str(out/'head.png'),cv2.cvtColor(arrays['rgb'],cv2.COLOR_RGB2BGR)):
        raise ValueError('原始图像写入失败；本次不计为成功样本')
    (out/'feedback.json').write_text(json.dumps(states,indent=2))
    (out/'sample.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    c=sub.add_parser('capture');c.add_argument('--depth-root',type=Path,required=True)
    c.add_argument('--intrinsics',type=Path,required=True);c.add_argument('--board',type=Path,required=True)
    c.add_argument('--output',type=Path,required=True);c.add_argument('--split',choices=['fit','heldout'],required=True)
    s=sub.add_parser('solve');s.add_argument('--samples',type=Path,nargs='+',required=True)
    s.add_argument('--output',type=Path,required=True)
    for name in ('max-position-m','max-angle-deg','max-reprojection-px'):s.add_argument('--'+name,type=float,required=True)
    a=p.parse_args()
    if a.command=='capture':
        result=capture(a.depth_root,a.intrinsics,a.board,a.output,a.split)
    else:
        result=solve([json.loads(path.read_text()) for path in a.samples],max_position_m=a.max_position_m,
            max_angle_deg=a.max_angle_deg,max_reprojection_px=a.max_reprojection_px)
        result['input_sha256']={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in a.samples}
        with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    if result.get('numerical_quality_pass') is False:raise SystemExit(2)


if __name__=='__main__':main()
