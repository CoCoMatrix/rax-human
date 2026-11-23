"""Brief same-owner receipt observation reuse; no service or timestamp rewriting."""
import copy,time,json
from scene_checks import same_state,current_image
MAX_REUSE_S=.75

def identity(robot):
    bridge=robot.get('bridge',{});cal=robot.get('camera_calibration',{}).get('sha256')
    owner=robot.get('owner_status',{}).get('generation');urdf=robot.get('urdf_sha256')
    if not bridge.get('instance') or owner is None or not cal or not urdf:return None
    return (bridge['instance'],owner,cal,urdf,json.dumps(robot.get('left_tool_calibration'),sort_keys=True))

def remember(ctx,obs,robot,request_id=None):
    ident=identity(robot)
    if ident is None or not isinstance(obs,dict) or not obs.get('frame_id'):
        ctx._rax_observation=None;return
    ctx._rax_observation={'obs':copy.deepcopy(obs),'identity':ident,
        'request_id':request_id or (robot.get('last_motion') or {}).get('owner_request_id')}

def observe(ctx,robot,*,require_depth=False,force=False):
    cached=getattr(ctx,'_rax_observation',None)
    if cached and not force:
        obs=cached['obs'];fb=robot.get('bridge',{}).get('feedback',{});now=time.time_ns()
        try:
            ready=(cached['identity']==identity(robot) and
                cached['request_id'] is not None and
                cached['request_id']==(robot.get('last_motion') or {}).get('owner_request_id') and
                0<=(now-obs['captured_at_ns'])/1e9<=MAX_REUSE_S and
                type(fb['timestamp_ns']) is int and 0<=now-fb['timestamp_ns']<=300_000_000 and
                fb.get('generation')==cached['identity'][1] and same_state(fb.get('q'),obs['measured14']) and
                (not require_depth or obs['cameras']['head'].get('metric_depth') is True))
            if ready:
                for camera in ('head','left','right'):current_image(obs,camera)
                return copy.deepcopy(obs),'recent_receipt_or_observation'
        except (KeyError,TypeError,ValueError,RuntimeError,OSError):pass
    obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=require_depth)
    remember(ctx,obs,robot)
    return obs,'fresh_capture'
