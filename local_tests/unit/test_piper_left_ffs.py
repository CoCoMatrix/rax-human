import json,time
import numpy as np
import pytest
from services.piper.cameras import Cameras,LEFT_CALIBRATION_SHA,_FramePending
from test_piper_adapter import Owner,depth_packet
import importlib.util
from pathlib import Path


def worker_module():
    path=Path(__file__).resolve().parents[2]/'scripts/left_ffs_worker.py'
    spec=importlib.util.spec_from_file_location('tested_left_ffs_worker',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('offset,accepted',[(-369753265,False),(-1,False),(0,True),(1000000,True)])
def test_request_skips_old_packet_without_rewriting_its_timestamp(offset,accepted):
    worker=worker_module();request={'id':'same-request','created_ns':1794280927795760825,'after_ns':1794280927795760825}
    packet={'host_received_unix_ns':request['created_ns']+offset};original=dict(packet)
    assert worker.packet_satisfies_request(packet,request) is accepted
    assert packet==original


def test_worker_respects_later_explicit_rest_boundary():
    worker=worker_module()
    assert not worker.packet_satisfies_request({'host_received_unix_ns':110},{'created_ns':100,'after_ns':120})
    with pytest.raises(ValueError):worker.packet_satisfies_request({}, {'created_ns':100})


def left_packet(root,request='request'):
    report=depth_packet(root)
    report.update(serial='262422071560',calibration_payload_sha256=LEFT_CALIBRATION_SHA,
        frame='left_d435_color_optical_frame',base_transform_applied=False,request_id=request)
    (root/'sample/report.json').write_text(json.dumps(report))
    (root/'latest.json').write_text(json.dumps(dict(report,result_dir='sample')))
    return report


def test_wrist_depth_is_bound_to_own_calibration_and_request(tmp_path):
    root=tmp_path/'depth';report=left_packet(root)
    cam=Cameras(Owner(),tmp_path/'frames',left_depth_root=root)
    arrays,got=cam.depth(camera='left',request_id='request')
    assert got['serial']=='262422071560' and arrays['valid'].any()
    with pytest.raises(_FramePending):cam.depth(camera='left',request_id='other')
    report['request_id']='wrong';(root/'sample/report.json').write_text(json.dumps(report))
    with pytest.raises(ValueError,match='identity'):cam.depth(camera='left',request_id='request')


@pytest.mark.parametrize('change',[{'serial':'262522073779'},{'base_transform_applied':True},{'frame':'left_base_link'},{'calibration_payload_sha256':'head'}])
def test_wrist_rejects_other_camera_or_unavailable_transform(tmp_path,change):
    root=tmp_path/'depth';report=left_packet(root);report.update(change)
    (root/'sample/report.json').write_text(json.dumps(report))
    cam=Cameras(Owner(),tmp_path/'frames',left_depth_root=root)
    with pytest.raises(ValueError):cam.depth(camera='left',request_id='request')


def test_explicit_wrist_depth_no_head_inference_and_no_robot_target(tmp_path,monkeypatch):
    root=tmp_path/'depth';root.mkdir();calls=[]
    owner=Owner();original=owner.camera
    def rgb(name):calls.append(name);return original(name)
    owner.camera=rgb
    cam=Cameras(owner,tmp_path/'frames',left_depth_root=root)
    def depth(**kw):
        assert kw['camera']=='left'
        request=json.loads((root/'request.json').read_text());assert kw['request_id']==request['id']
        assert request['after_ns']>=request['created_ns']
        calls.append('left-depth');im=np.zeros((720,1280,3),np.uint8)
        xyz=np.zeros(im.shape,np.float32);xyz[...,2]=.15
        return dict(rgb=im,xyz_rgb_m=xyz,valid=np.ones(im.shape[:2],bool)),dict(host_received_unix_ns=time.time_ns())
    monkeypatch.setattr(cam,'depth',depth)
    obs,_=cam.capture(['head','left','right'],False,depth_cameras=['left'])
    assert calls==['left-depth','head','right']
    assert 'T_base_camera' not in obs['cameras']['left']
    assert obs['cameras']['head']['K'] and not obs['cameras']['head']['metric_depth']
    k=np.asarray(cam.left_intrinsics)
    point=cam.pixel(obs['frame_id'],'left',[[float(k[0,2]),float(k[1,2])]],output_frame='camera_optical')
    assert point['points'][0]['position']==pytest.approx([0,0,.15])
    assert point['frame']=='left_d435_color_optical_frame' and not point['robot_target_usable']
    assert point['calibration_sha256']==LEFT_CALIBRATION_SHA and not point['motion_permit']
    with pytest.raises(ValueError,match='hand-eye'):cam.pixel(obs['frame_id'],'left',[[10,10]])


def test_left_pending_error_identifies_wrist_not_head(tmp_path):
    root=tmp_path/'left';report=left_packet(root)
    cam=Cameras(Owner(),tmp_path/'frames',left_depth_root=root)
    with pytest.raises(_FramePending,match='left source timestamp precedes the observation boundary'):
        cam.depth(camera='left',request_id='request',after_ns=report['host_received_unix_ns']+1)


def test_backend_exposes_optical_only_wrist_capability_and_arguments(tmp_path):
    from test_piper_adapter import backend
    b=backend(tmp_path)
    assert not b.status()['capabilities']['wrist_metric_depth']
    b.cameras.left_depth_root=str(tmp_path/'left')
    caps=b.status()['capabilities']
    assert caps['wrist_metric_depth'] and not caps['wrist_hand_eye']
    b.validate('piper_observe',{'cameras':['left'],'require_depth':False,'depth_cameras':['left']})
    b.validate('piper_pixel_to_position',{'frame_id':'a'*32,'camera':'left','pixels':[[100,100]],'output_frame':'camera_optical'})


def test_backend_accepts_launcher_left_depth_option(tmp_path):
    from types import SimpleNamespace
    from services.piper.backend import PiperBackend
    config=SimpleNamespace(root=tmp_path,backend_options={'motion_enabled':False,
        'depth_root':str(tmp_path/'head'),'left_depth_root':str(tmp_path/'left')},observation_profile='rgbd')
    b=PiperBackend(config,owner=Owner())
    assert b.cameras.left_depth_root==str(tmp_path/'left')
    assert b.status()['capabilities']['wrist_metric_depth']
    b.close()
