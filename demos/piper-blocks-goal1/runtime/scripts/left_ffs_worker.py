#!/usr/bin/env python3
"""On-demand LEFT camera FFS using the installed processor; never opens a camera/CAN."""
import argparse,json,time,sys,shutil
from pathlib import Path


def packet_satisfies_request(packet, request):
    """A new request must be served by acquisition after its original boundary."""
    stamp=packet.get('host_received_unix_ns');created=request.get('created_ns');after=request.get('after_ns',created)
    if type(stamp) is not int or type(created) is not int or type(after) is not int or min(created,after)<=0:
        raise ValueError('Missing wrist acquisition/request boundary')
    return stamp>=max(created,after)


def main():
    p=argparse.ArgumentParser();p.add_argument('--legacy-root',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True);p.add_argument('--manifest',required=True)
    p.add_argument('--packet-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=1800);p.add_argument('--max-disp',type=int,default=384)
    a=p.parse_args()
    if not 30<=a.seconds<=604800:raise ValueError('Bounded runtime required')
    sys.path.insert(0,str(a.legacy_root.resolve()))
    from services.piper.ffs_worker import Processor,atomic
    from services.piper.sensor.ffs_runtime import FastFoundationStereoRunner
    from services.piper.depth import contained
    from services.piper.retention import enroll,retire_expired
    calibration=json.loads(a.calibration.read_text())
    if calibration['role']!='camera_left' or calibration['serial']!='262422071560':raise ValueError('Wrong wrist calibration')
    a.output.mkdir(parents=True,exist_ok=False);enroll(a.output,'result')
    if a.packet_root.exists():raise ValueError('Left packet root must be new before owner startup')
    enrollment_ns=time.time_ns();last_retire=0
    runner=FastFoundationStereoRunner(a.manifest,valid_iters=4,max_disp=a.max_disp,input_scale=1,allow_trusted_pickle=True)
    worker=Processor(calibration,runner);atomic(a.output/'ready.json',{'model_loaded':True,'camera':'left','base_transform_applied':False})
    end=time.monotonic()+a.seconds;warmed=False;seen=set();attempt=0;finished=None
    while time.monotonic()<end:
        if shutil.disk_usage(a.output).free<3*1024**3:raise RuntimeError('Evidence disk reserve')
        try:
            if a.packet_root.exists() and time.monotonic()-last_retire>1:
                marker=a.packet_root/'retention-owner.json'
                if not marker.exists():
                    with marker.open('x') as f:json.dump({'root':str(a.packet_root.resolve()),'kind':'packet','created_ns':enrollment_ns},f)
                retire_expired(a.packet_root,'packet',keep=16,grace_s=20,journal=a.output/'retirement.jsonl')
                last_retire=time.monotonic()
            pointer=json.loads((a.packet_root/'latest.json').read_text());relative=pointer['packet_dir']
            packet=contained(a.packet_root,relative)
            if not warmed:
                worker.process(packet,a.output/'historical-warmup','offline_replay');warmed=True
                atomic(a.output/'live-worker-ready.json',{'camera':'left','warmup_only':True,'hardware_writes':0})
                continue
            if not (a.output/'request.json').exists():time.sleep(.05);continue
            request=json.loads((a.output/'request.json').read_text())
            if request['id']==finished or not 0<=time.time_ns()-request['created_ns']<10_000_000_000:
                time.sleep(.02);continue
            if relative in seen:time.sleep(.01);continue
            metadata=json.loads(contained(packet,'packet.json').read_text())
            if not packet_satisfies_request(metadata,request):
                time.sleep(.01);continue
            seen.add(relative)
            if len(seen)>128:seen={relative}
            attempt+=1;folder=a.output/f'result-{attempt:06}'
            result=worker.process(packet,folder,'live')
            if not packet_satisfies_request(result,request):
                raise ValueError('Wrist processor changed acquisition identity')
            if time.time_ns()>=result['expires_unix_ns']:
                # Do not finish this request with an already unusable result.
                # The same bounded request waits for the next newly acquired packet.
                time.sleep(.01);continue
            result.update(frame='left_d435_color_optical_frame',camera_role='camera_left',request_id=request['id'])
            result['native_geometry']['rgb_coordinate_frame']='left_d435_color_optical_frame'
            atomic(folder/'report.json',result)
            atomic(a.output/'latest.json',dict(result_dir=folder.name,request_id=request['id'],**{k:result[k] for k in ('sample_id','owner_generation','host_received_unix_ns','expires_unix_ns')}))
            finished=request['id']
            atomic(a.output/'producer-health.json',{'published_request':finished,'processing_s':result['elapsed_s'],'valid_fraction':result['valid_fraction'],'max_disp':a.max_disp,'hardware_writes':0})
            retire_expired(a.output,'result',keep=16,grace_s=10,journal=a.output/'retirement.jsonl')
        except (FileNotFoundError,ValueError,RuntimeError,KeyError) as exc:
            atomic(a.output/'last-error.json',{'at_ns':time.time_ns(),'error':str(exc)})
            time.sleep(.1)

if __name__=='__main__':main()
