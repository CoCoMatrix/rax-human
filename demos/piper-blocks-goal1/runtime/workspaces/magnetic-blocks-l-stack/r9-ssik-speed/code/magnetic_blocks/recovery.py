"""Read-only green placement endpoint recovery; never repeats an action."""
import hashlib
from pathlib import Path
from scene_checks import PerceptionUnknown,scene_control,scene_geometry,current_image
from ordinary_checks import placement,after_release


def green_endpoint(ctx,record,path,obs,robot):
    r=record['result'];stage=r.get('stage')
    if stage not in ('green_stack_lower','green_stack_release','green_stack_retreat'):
        raise PerceptionUnknown('Unsupported recovery boundary')
    if not r.get('facts') or not r.get('action_completed') or not r.get('receipts'):
        raise PerceptionUnknown('Unsupported recovery boundary without completed receipts and original path evidence')
    # completed_action_state is called by the controller first. It checks last
    # request, all receipts, planned count, owner, pose and no Hold/uncertainty.
    if stage=='green_stack_retreat':
        if not .065<=obs['measured14'][6]<=.074:raise PerceptionUnknown('Retreat recovery needs completed open jaw')
        fact=after_release(ctx,'green_retreat_recovery',obs,path)
    else:
        original=r['facts'][0];scene=original['scene'];origin=original['payload_origin']
        if origin.get('color')!='green':raise PerceptionUnknown('Recovery cannot change payload identity')
        scene_control(scene,robot);scene_geometry(scene,obs)
        ref=scene.get('stack_reference')
        if not ref:raise PerceptionUnknown('Missing pinned support provenance')
        for item in [ref]+ref.get('history',[]):
            if hashlib.sha256(Path(item['report']).read_bytes()).hexdigest()!=item['sha256']:
                raise PerceptionUnknown('Recovery support/action source changed')
        if stage=='green_stack_lower':
            if r.get('support_required') is not True:raise PerceptionUnknown('Intermediate lowering is not final support')
            if not .048<=obs['measured14'][6]<=.054:raise PerceptionUnknown('Lowering recovery requires unchanged loaded jaw')
        elif stage=='green_stack_release':
            if not .065<=obs['measured14'][6]<=.074:raise PerceptionUnknown('Release recovery needs completed open jaw')
        else:raise PerceptionUnknown('Unsupported green recovery boundary')
        fact=placement(ctx,'green_support_recovery',obs,scene,origin)
    return fact|{'recovery_scope':'Read-only completion check; never replay descent, opening or retreat',
                 'source_stage':stage}
