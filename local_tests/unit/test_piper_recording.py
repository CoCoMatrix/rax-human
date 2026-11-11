import json
import time
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from services.piper.recording import Recorder,minute
from services.franka.stream import frame_times,robot_at
from test_piper_adapter import Owner,backend

class Depth:
    def depth(self):
        rgb=np.zeros((12,16,3),np.uint8);xyz=np.ones((12,16,3),np.float32)*.432
        valid=np.ones((12,16),bool);valid[0,0]=False
        return {'rgb':rgb,'xyz_rgb_m':xyz,'valid':valid},{'host_received_unix_ns':time.time_ns()}

def test_upstream_layout_depth_pair_robot_and_pruning(tmp_path):
    r=Recorder(Owner(),Depth(),tmp_path,window_s=120)
    old=tmp_path/'left'/'20200101T0000';old.mkdir(parents=True);(old/'1.jpg').write_bytes(b'old')
    for name in ('head','left','right','robot','depth'):r.tick(name)
    assert not old.exists()
    index=json.loads((tmp_path/'index.json').read_text())
    assert set(index['cameras'])=={'head','left','right'} and index['inspection_only']
    assert index['measured_hz']['left']==0  # One frame cannot establish a rate.
    frames=frame_times(tmp_path,'head',0,time.time_ns()//1_000_000)
    assert 1<=len(frames)<=2  # Same-ms RGBD intentionally supersedes bracketed RGB.
    depth=next(tmp_path.glob('head/*/*_depth.png'))
    image=cv2.imread(str(depth),cv2.IMREAD_UNCHANGED)
    assert image.dtype==np.uint16 and image[0,0]==0 and image[1,1]==432
    assert depth.with_name(depth.name.replace('_depth.png','.jpg')).exists()
    assert robot_at(tmp_path,index['newest_ms']['robot'])['measured14'][6]==.019
    assert not list(tmp_path.rglob('.*'))

def test_same_millisecond_http_frame_cannot_break_exact_depth_pair(tmp_path):
    r=Recorder(Owner(),Depth(),tmp_path);t=1234567890
    rgb=np.full((12,16,3),120,np.uint8);depth=np.full((12,16),400,np.uint16)
    assert r._image('head',t,rgb,{'binding':'exact'},depth)
    assert r._image('head',t,np.zeros_like(rgb),{'binding':'bracket'}) is False
    folder=tmp_path/'head'/minute(t)
    assert cv2.imread(str(folder/f'{t}.jpg')).mean()>100
    assert json.loads((folder/f'{t}.json').read_text())['binding']=='exact'
    assert (folder/f'{t}_depth.png').exists()

def test_duplicate_source_not_counted_and_actual_rate(tmp_path,monkeypatch):
    owner=Owner();original=owner.camera
    rgb,info=original('left');owner.camera=lambda c:(rgb,info)
    r=Recorder(owner,Depth(),tmp_path)
    r.tick('left');r.tick('left')
    assert len(r.samples['left'])==1
    owner.camera=original;time.sleep(.02);r.tick('left')
    assert r.status()['measured_hz']['left']>0

def test_threads_stop_and_source_failure_reported(tmp_path):
    class Broken:
        def depth(self):raise ValueError('offline missing depth')
    r=Recorder(Owner(),Broken(),tmp_path)
    r.start();r.stop.wait(.12);r.close()
    assert not any(t.is_alive() for t in r.threads)
    assert r.status()['errors']['depth']['type']=='ValueError'
    assert r.status()['running'] is False

def test_stream_is_read_only_and_camera_names_are_validated(tmp_path):
    b=backend(tmp_path);r=Recorder(b.owner,Depth(),b.stream_root)
    r.tick('left');t=r.latest['left']
    result,images=b.call('piper_stream',{'camera':'left','since_ms':t-10,'until_ms':t+10})
    assert result['times_ms']==[t] and len(images)==1 and result['executed'] is False
    with pytest.raises(ValueError):b.call('piper_stream',{'camera':'../frames','since_ms':0,'until_ms':1})
    b.close()
