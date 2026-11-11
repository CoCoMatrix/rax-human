"""Read existing camera owner, snapshot bound head RGB/FFS; no camera SDK ownership."""
import hashlib
import io
import json
from pathlib import Path
import time
import uuid
import cv2
import numpy as np
from .geometry import ASSETS

CALIBRATION_SHA = '28db0ac28c85f52dc7497ebeac82f46b361b0787d56ad07c791f4b7bfba4f091'
CANDIDATE_SHA = 'c4a393c074890ff8f0d28532e8916dd7c6bdedc7ee8f4e91b7a6f6c07155fae6'
LEFT_CALIBRATION_SHA = 'c82d6b82c46a76081035271cf5a87283cbaec36c700a455b05939a92a1825c94'
MODEL_SHA = '98b5a9acf39fbfa795025de8cea95ce123daa40f6b6234d719167751024cf692'


class _FramePending(Exception):
    """A valid source is still from before the requested action boundary."""


def within(root, relative):
    rel = Path(relative)
    if rel.is_absolute() or '..' in rel.parts: raise ValueError('Invalid relative sample path')
    path = (Path(root) / rel).resolve(strict=True)
    if not path.is_relative_to(Path(root).resolve()): raise ValueError('Sample path escapes root')
    return path


class Cameras:
    def __init__(self, owner, frames, depth_root=None, left_depth_root=None):
        self.owner = owner; self.frames = Path(frames); self.depth_root = depth_root
        self.left_depth_root=left_depth_root
        raw = (ASSETS / 'camera-all13-candidate.json').read_bytes()
        if hashlib.sha256(raw).hexdigest() != CANDIDATE_SHA: raise ValueError('Camera calibration changed')
        self.calibration = json.loads(raw); self.T = np.asarray(self.calibration['T_base_camera'])
        intrinsics = json.loads((ASSETS / 'head-stereo-calibration.json').read_text())
        payload = {k:v for k,v in intrinsics.items() if k != 'calibration_payload_sha256'}
        digest = hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',', ':'),ensure_ascii=False).encode()).hexdigest()
        if digest != CALIBRATION_SHA or intrinsics.get('calibration_payload_sha256') != digest:
            raise ValueError('Head intrinsics calibration identity changed')
        profile = intrinsics['profiles']['color']
        self.intrinsics = profile['K']
        self.head_shape = (profile['height'],profile['width'])
        left=json.loads((ASSETS/'left-stereo-calibration.json').read_text())
        payload={k:v for k,v in left.items() if k!='calibration_payload_sha256'}
        digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',', ':'),ensure_ascii=False).encode()).hexdigest()
        if digest!=LEFT_CALIBRATION_SHA or left['calibration_payload_sha256']!=digest:raise ValueError('Left calibration identity changed')
        lp=left['profiles']['color'];self.left_intrinsics=lp['K'];self.left_shape=(lp['height'],lp['width'])

    def depth(self, *, after_ns=None, camera="head", request_id=None):
        if camera not in ('head','left'):raise ValueError('Unsupported depth camera')
        source=self.depth_root if camera=='head' else self.left_depth_root
        if not source:raise ValueError('No live depth root configured for '+camera)
        root=Path(source)
        try:pointer=json.loads((root/'latest.json').read_text())
        except FileNotFoundError:
            if camera=='left':raise _FramePending('Left FFS result pending')
            raise
        if camera=='left' and pointer.get('request_id')!=request_id:raise _FramePending('Left FFS request pending')
        folder = within(root, pointer['result_dir'])
        report = json.loads((folder / 'report.json').read_text())
        expected = {'schema':'rax.piper.ffs.v1','mode':'live','source':'fast_foundation_stereo',
                    'inference_kind':'real_model','serial':'262522073779' if camera=='head' else '262422071560',
                    'calibration_payload_sha256':CALIBRATION_SHA if camera=='head' else LEFT_CALIBRATION_SHA,'checkpoint_sha256':MODEL_SHA}
        if any(report.get(k) != v for k,v in expected.items()): raise ValueError('Head depth provenance mismatch')
        if camera=='left' and (report.get('frame')!='left_d435_color_optical_frame' or report.get('base_transform_applied') is not False):raise ValueError('Wrist coordinate contract mismatch')
        for k in (('sample_id','owner_generation','host_received_unix_ns','expires_unix_ns') + (('request_id',) if camera=='left' else ())):
            if pointer.get(k) != report.get(k): raise ValueError('Depth pointer identity mismatch')
        captured = report['host_received_unix_ns']
        if report['expires_unix_ns'] != captured + 1_000_000_000:
            raise ValueError('Depth expiration differs from its original acquisition timestamp')
        if after_ns is not None and captured < after_ns:
            raise _FramePending(camera+' source timestamp precedes the observation boundary')
        if report['expires_unix_ns'] != captured + 1_000_000_000 or not 0 <= time.time_ns() - captured < 1_000_000_000:
            raise ValueError('Live depth is stale/future; acquisition timestamps are never refreshed')
        raw = (folder / 'observation.npz').read_bytes()
        if hashlib.sha256(raw).hexdigest() != report['arrays_sha256']: raise ValueError('Depth payload identity mismatch')
        with np.load(io.BytesIO(raw), allow_pickle=False) as z:
            arrays = {k:z[k] for k in ('rgb','xyz_rgb_m','valid')}
        rgb, xyz, valid = arrays['rgb'], arrays['xyz_rgb_m'], arrays['valid']
        if rgb.shape[:2] != (self.head_shape if camera=='head' else self.left_shape): raise ValueError('Head frame size differs from calibrated native intrinsics')
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[-1] != 3 or xyz.shape != rgb.shape or valid.shape != rgb.shape[:2] or valid.dtype != bool:
            raise ValueError('Malformed bound RGB/depth arrays')
        if not np.isfinite(xyz[valid]).all() or np.any(xyz[...,2][valid] <= 0): raise ValueError('Invalid depth values')
        if time.time_ns() >= report['expires_unix_ns']: raise ValueError('Depth expired during acquisition')
        return arrays, report

    def capture(self, names, require_depth, *, after_ns=None, depth_cameras=None):
        if after_ns is not None and (type(after_ns) is not int or after_ns <= 0):
            raise ValueError('after_ns must be a positive source-clock timestamp')
        wanted=([c for c in names if c=='head'] if require_depth else []) if depth_cameras is None else depth_cameras
        if not isinstance(wanted,list) or any(c not in names or c not in ('head','left') for c in wanted):raise ValueError('Depth cameras must be selected head/left views')
        request_id=None
        if 'left' in wanted:
            if not self.left_depth_root:raise ValueError('Left FFS not configured')
            request_id=uuid.uuid4().hex;root=Path(self.left_depth_root)
            created_ns=time.time_ns()
            tmp=root/(request_id+'.partial');tmp.write_text(json.dumps({'id':request_id,'created_ns':created_ns,
                'after_ns':max(created_ns,after_ns or 0)}));tmp.replace(root/'request.json')
        deadline = time.monotonic() + (5 if 'left' in wanted else 2) if after_ns is not None or 'left' in wanted else None
        captured = []
        # On-demand inference comes first; do not age other RGB while waiting.
        order=([c for c in ('left','head') if c in wanted]+[c for c in names if c not in wanted])
        for name in order:
            while True:
                arrays = None
                try:
                    if name in wanted:
                        arrays, report = self.depth(**({} if after_ns is None else {'after_ns':after_ns}),**({} if name=='head' else {'camera':'left','request_id':request_id})); rgb = arrays['rgb']
                        stamps = [report['host_received_unix_ns']]
                        row = {'binding':'same immutable stereo/RGB packet','source_timestamp_ns':stamps[0],
                               'source':report,'metric_depth':True,'K':self.intrinsics if name=='head' else self.left_intrinsics,
                               'camera_frame':'head_d435_color_optical_frame' if name=='head' else 'left_d435_color_optical_frame',
                               'calibration_sha256':CALIBRATION_SHA if name=='head' else LEFT_CALIBRATION_SHA,
                               'base_transform_available':name=='head'}
                        if name=='head':row['T_base_camera']=self.T.tolist()
                    else:
                        rgb, info = self.owner.camera(name)
                        stamps = [info.get('before',{}).get('timestamp_ns'),info['after']['timestamp_ns']]
                        row = {'binding':info['binding'],'source_timestamp_ns':stamps[1],
                               'source':info,'metric_depth':False}
                        if name=='head':row.update(K=self.intrinsics,T_base_camera=self.T.tolist())
                    if after_ns is not None:
                        if any(type(stamp) is not int for stamp in stamps):
                            raise ValueError('Post-action camera source timestamps are missing')
                        if any(stamp < after_ns for stamp in stamps):
                            raise _FramePending(name+' source timestamp precedes the completed motion')
                        if time.monotonic() >= deadline:
                            raise TimeoutError('Timed out waiting for post-action '+name+' frame')
                    captured.append((name,rgb,row,arrays))
                    break
                except _FramePending as exc:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('Timed out waiting for post-action frame: '+str(exc)) from exc
                    time.sleep(min(.05,remaining))
        # Wait entirely in memory. Rejected old frames and timeouts create no frame directory.
        ident = uuid.uuid4().hex; path = self.frames / ident; path.mkdir(parents=True)
        meta = {'frame_id':ident,'cameras':{},'calibration_sha256':CANDIDATE_SHA,'calibration_provisional':True}
        if after_ns is not None:meta['after_ns'] = after_ns
        images = []
        captured.sort(key=lambda item:names.index(item[0]))
        for name,rgb,row,arrays in captured:
            if arrays is not None:
                # Compression consumed the head frame's remaining1s TTL in R22.
                # Save exact arrays directly for every depth camera, not just wrist.
                np.savez(path / (name+'-depth.npz'), **arrays)
            ok, encoded = cv2.imencode('.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            if not ok: raise ValueError('PNG encoding failed')
            (path / (name + '.png')).write_bytes(encoded.tobytes())
            # Native-size inspection artifact for the main agent. Keep original PNG.
            ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY,95])
            if not ok: raise ValueError('Inspection JPEG encoding failed')
            (path / (name + '.jpg')).write_bytes(jpg.tobytes())
            row['inspection_image'] = f'runtime/frames/{ident}/{name}.jpg'
            row['shape_hw'] = list(rgb.shape[:2]); meta['cameras'][name] = row; images.append((name,rgb))
        meta['captured_at_ns'] = time.time_ns()
        (path / 'meta.json').write_text(json.dumps(meta,indent=2))
        for name,row in meta['cameras'].items():
            expiry=row.get('source',{}).get('expires_unix_ns')
            if row.get('metric_depth') and expiry is not None and time.time_ns()>=expiry:
                # Preserve this saved frame for diagnosis, but never label it live.
                raise ValueError(name+' depth expired during snapshot save; original acquisition timestamp retained')
        return meta, images

    def pixel(self, frame_id, camera, pixels, radius=3, output_frame='left_base_link'):
        if camera not in ('head','left') or output_frame not in ('left_base_link','camera_optical'):raise ValueError('Unsupported camera/output frame')
        if camera=='left' and output_frame!='camera_optical':raise ValueError('Wrist hand-eye unavailable: request camera_optical; never use as a robot target')
        if not isinstance(frame_id,str) or len(frame_id) != 32 or any(c not in '0123456789abcdef' for c in frame_id):
            raise ValueError('Invalid frame id')
        path = within(self.frames, frame_id); meta = json.loads((path / 'meta.json').read_text())
        row = meta['cameras'].get(camera, {})
        if row.get('metric_depth') is not True: raise ValueError('Saved head frame has no bound depth')
        if type(radius) is not int or not 0 <= radius <= 15: raise ValueError('Radius must be 0..15')
        if not isinstance(pixels,list) or not 1 <= len(pixels) <= 64: raise ValueError('1..64 pixels required')
        with np.load(path / (camera+'-depth.npz'),allow_pickle=False) as z:
            xyz, valid = z['xyz_rgb_m'], z['valid']
        h,w = valid.shape; k = np.asarray(row['K']); t = np.asarray(row['T_base_camera']) if output_frame=='left_base_link' else None; results=[]
        for pair in pixels:
            if not isinstance(pair,list) or len(pair)!=2 or any(type(v) not in (int,float) or not np.isfinite(v) for v in pair): raise ValueError('Finite pixel [u,v] required')
            u,v=pair; x,y=int(round(u)),int(round(v))
            if not 0<=x<w or not 0<=y<h: raise ValueError('Pixel outside saved image')
            ys=slice(max(0,y-radius),min(h,y+radius+1)); xs=slice(max(0,x-radius),min(w,x+radius+1))
            depths=xyz[ys,xs,2][valid[ys,xs]]
            if not len(depths): raise ValueError('No valid depth in requested window')
            depth=float(np.median(depths)); p=np.array([(u-k[0,2])/k[0,0]*depth,(v-k[1,2])/k[1,1]*depth,depth])
            results.append({'pixel':[u,v],'depth_m':depth,'valid_samples':int(len(depths)),
                            'position':(t[:3,:3]@p+t[:3,3]).tolist() if t is not None else p.tolist()})
        return {'frame_id':frame_id,'camera':camera,'frame':'left_base_link' if t is not None else row.get('camera_frame',camera+'_color_optical_frame'),'points':results,
                'base_transform_applied':t is not None,'robot_target_usable':t is not None,
                'age_s':(time.time_ns()-row['source_timestamp_ns'])/1e9,'calibration_provisional':True,
                'calibration_sha256':meta['calibration_sha256'] if t is not None else row.get('calibration_sha256'),'motion_permit':False,
                'method':'Franka-style median valid optical depth window; no synthesized depth'}
