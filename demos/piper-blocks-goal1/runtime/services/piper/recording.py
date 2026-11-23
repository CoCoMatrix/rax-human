"""Read-only rolling recording in the upstream RAX stream layout.

Consumes the existing owner/FFS readers; never acquires a device or commands it.
HTTP RGB is timestamp-bracketed, not atomically synchronized. Saved depth has its
own exactly matching RGB. This recording is for review, not metric control.
"""
from collections import deque
import json
import math
from pathlib import Path
import shutil
import threading
import time
import uuid
import cv2
import numpy as np
from .owner import SERIALS


def minute(t_ms):
    return time.strftime('%Y%m%dT%H%M',time.gmtime(t_ms/1000))


def atomic(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name('.'+path.name+'.'+uuid.uuid4().hex)
    try:
        temporary.write_bytes(data);temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class Recorder:
    def __init__(self,owner,cameras,root,*,rgb_hz=5,robot_hz=10,depth_hz=1,window_s=600):
        for name,value,maximum in [('rgb_hz',rgb_hz,30),('robot_hz',robot_hz,30),('depth_hz',depth_hz,1),('window_s',window_s,600)]:
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=maximum:raise ValueError('Invalid '+name)
        self.owner=owner;self.cameras=cameras;self.root=Path(root);self.window_s=window_s
        self.rates={**{c:rgb_hz for c in SERIALS},'robot':robot_hz,'depth':depth_hz}
        self.stop=threading.Event();self.lock=threading.RLock();self.threads=[]
        self.samples={k:deque(maxlen=300) for k in self.rates};self.errors={};self.latest={};self.last_source={}

    def status(self):
        with self.lock:
            now=time.time_ns()//1_000_000
            rates={}
            for k,rows in self.samples.items():
                recent=[t for t in rows if now-t<10_000]
                rates[k]=(len(recent)-1)*1000/(recent[-1]-recent[0]) if len(recent)>1 and recent[-1]>recent[0] else 0.
            return {'schema':'rax.piper.stream.v1','cameras':list(SERIALS),'window_s':self.window_s,
                    'requested_hz':self.rates,'measured_hz':rates,'newest_ms':dict(self.latest),
                    'errors':dict(self.errors),'updated_ms':now,'running':bool(self.threads) and not self.stop.is_set(),
                    'inspection_only':True,'rgb_binding':'HTTP metadata bracket; depth pairs bind exact RGB',
                    'depth_units':'uint16 millimetres; 0 invalid'}

    def prune(self,now_ms):
        oldest=minute(now_ms-self.window_s*1000)
        for name in [*SERIALS,'robot']:
            folder=self.root/name
            if not folder.exists():continue
            for entry in folder.iterdir():
                key=entry.stem if name=='robot' else entry.name
                if len(key)==13 and key[8]=='T' and key<oldest:
                    if entry.is_dir():shutil.rmtree(entry)
                    else:entry.unlink()

    def _image(self,camera,t_ms,rgb,metadata,depth=None):
        folder=self.root/camera/minute(t_ms)
        ok,jpg=cv2.imencode('.jpg',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,85])
        if not ok:raise ValueError('Recording JPEG encoding failed')
        png=None
        if depth is not None:
            ok,png=cv2.imencode('.png',depth)
            if not ok:raise ValueError('Recording depth encoding failed')
        with self.lock:
            depth_path=folder/f'{t_ms}_depth.png'
            # RGB polling and exact RGBD can share a millisecond. Preserve the
            # exact pair; a later bracketed HTTP image must not overwrite it.
            if depth is None and depth_path.exists():return False
            atomic(folder/f'{t_ms}.json',json.dumps(metadata).encode())
            atomic(folder/f'{t_ms}.jpg',jpg.tobytes())
            if png is not None:atomic(depth_path,png.tobytes())
        return True

    def tick(self,source):
        """One read/record iteration, also used for offline fake/replay tests."""
        self.root.mkdir(parents=True,exist_ok=True)
        now=time.time_ns()//1_000_000
        with self.lock:self.prune(now)
        if shutil.disk_usage(self.root).free<1024**3:raise OSError('Recording paused: less than 1 GiB free')
        if source=='robot':
            state=self.owner.state();stamp=state['robot_state'].get('feedback_metadata',{}).get('source_timestamp_ns')
            if stamp is not None and stamp==self.last_source.get(source):return
            row={'t_ms':now,'source_timestamp_ns':stamp,'measured14':state['robot_state']['values'],
                 'robot_state':state['robot_state'],'owner_status':state['robot_status']}
            path=self.root/'robot'/f'{minute(now)}.jsonl';path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a') as f:f.write(json.dumps(row)+'\n')
        elif source=='depth':
            arrays,report=self.cameras.depth();stamp=report['host_received_unix_ns']
            if stamp==self.last_source.get(source):return
            z=arrays['xyz_rgb_m'][...,2];valid=arrays['valid']&np.isfinite(z)&(z>=.001)&(z<=65.535)
            depth=np.zeros(z.shape,dtype=np.uint16);depth[valid]=np.rint(z[valid]*1000).astype(np.uint16)
            self._image('head',stamp//1_000_000,arrays['rgb'],{'binding':'same immutable RGB/depth packet','source':report,'recorded_ms':now},depth)
        else:
            rgb,info=self.owner.camera(source)
            stamp=(info.get('before',{}).get('timestamp_ns'),info.get('after',{}).get('timestamp_ns'))
            if stamp!=(None,None) and stamp==self.last_source.get(source):return
            if not self._image(source,now,rgb,info):return
        with self.lock:
            self.last_source[source]=stamp;self.samples[source].append(now);self.latest[source]=now;self.errors.pop(source,None)
            atomic(self.root/'index.json',json.dumps(self.status()).encode())

    def _loop(self,source):
        while not self.stop.is_set():
            start=time.monotonic()
            try:self.tick(source)
            except Exception as exc:
                with self.lock:
                    self.errors[source]={'type':type(exc).__name__,'at_ms':time.time_ns()//1_000_000}
                    try:atomic(self.root/'index.json',json.dumps(self.status()).encode())
                    except OSError:pass
            self.stop.wait(max(0,1/self.rates[source]-(time.monotonic()-start)))

    def start(self):
        if self.threads:raise RuntimeError('Recorder already started')
        self.root.mkdir(parents=True,exist_ok=True)
        for source in self.rates:
            thread=threading.Thread(target=self._loop,args=(source,),name='piper-record-'+source,daemon=True)
            self.threads.append(thread);thread.start()

    def close(self):
        self.stop.set()
        for thread in self.threads:thread.join(timeout=8)
        if any(t.is_alive() for t in self.threads):raise RuntimeError('Recorder reader did not stop')
        if self.root.exists():atomic(self.root/'index.json',json.dumps(self.status()).encode())
