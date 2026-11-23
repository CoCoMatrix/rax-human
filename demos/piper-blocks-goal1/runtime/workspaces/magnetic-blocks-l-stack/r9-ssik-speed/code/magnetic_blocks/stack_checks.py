"""R8 candidate: fresh base support provenance and green-only stack boundaries.

No independent metrology or force claim. Native IK/CAD/workspace guards stay intact.
"""
import copy
import hashlib
from pathlib import Path
import numpy as np
from scene_checks import (PerceptionUnknown, read_report, completed_action_state,
                          scene_control, scene_geometry, same_state, current_image)
from qualitative import retention_evidence, retention_at_endpoint, assess_images, reuse_support_endpoint

# CALIBRATE: approximately60mm upright blocks, measured this session.
BLOCK_HALF_M = .030
PAYLOAD_HALF_M = .035
MAX_CENTER_ERROR_M = .008
MIN_FINGER_SUPPORT_M = .010
MAX_DESCENT_M = .050


def support_scene(ctx, previous_report, obs, robot, support_report, scene):
    """Bind fresh red/orange depth to the complete green action chain."""
    raw = Path(support_report).read_bytes()
    ref = read_report(support_report)
    rr = ref['result']; root = str(ref['inputs']['previous_report'])
    if rr.get('stage') != 'refresh_stack_support' or not rr.get('ok') or rr.get('executed'):
        raise PerceptionUnknown('Need successful read-only base-support refresh')
    source = rr.get('support_source', {})
    if (source.get('report') != root or not source.get('sha256')
            or hashlib.sha256(Path(root).read_bytes()).hexdigest() != source['sha256']):
        raise PerceptionUnknown('Missing or changed support pregrasp source; legacy refresh requires new evidence')
    old = scene.get('stack_reference')
    digest = hashlib.sha256(raw).hexdigest()
    if old:
        if old['report'] != str(support_report) or old['sha256'] != digest:
            raise PerceptionUnknown('Stack support source changed')
        for item in old['history']:
            if hashlib.sha256(Path(item['report']).read_bytes()).hexdigest() != item['sha256']:
                raise PerceptionUnknown('Retained stack action history changed')
    fresh = rr['support_scene']
    scene_control(fresh, robot); scene_geometry(fresh, obs)
    if rr['start_status']['robot_status']['bridge']['instance'] != robot['bridge']['instance']:
        raise PerceptionUnknown('Stack support owner changed')
    endpoint = obs; path = str(previous_report); seen = set(); chain = []
    for i in range(40):
        resolved = str(Path(path).resolve())
        if resolved in seen: raise PerceptionUnknown('Cyclic stack history')
        seen.add(resolved)
        rec = read_report(path)
        r = completed_action_state(rec, endpoint, robot if i == 0 else None, require_current=i == 0)
        if not r.get('ok'): raise PerceptionUnknown('Unverified stack predecessor')
        chain.append({'report':path,'sha256':hashlib.sha256(Path(path).read_bytes()).hexdigest()})
        if resolved == str(Path(root).resolve()):
            if (r['stage'] != 'approach' or r.get('engage')
                    or rec['inputs'].get('color') != 'green'):
                raise PerceptionUnknown('Support root must be an empty green pregrasp')
            if (not same_state(r['measured']['measured14'], rr['measured']['measured14'])
                or r['measured']['captured_at_ns'] > rr['measured']['captured_at_ns']
                or rr['measured']['captured_at_ns'] > endpoint['captured_at_ns']):
                raise PerceptionUnknown('Support refresh is not between green hover and pickup')
            break
        if r['stage'] not in ('approach','close_grasp','trial_lift','transport',
                              'green_stack_lower','green_stack_release','green_stack_retreat'):
            raise PerceptionUnknown('Unexpected manipulation in support history')
        if r['stage'] in ('approach','close_grasp','trial_lift','transport') and rec['inputs'].get('color') != 'green':
            raise PerceptionUnknown('Stack history includes another payload')
        endpoint = r['initial_observation']; path = str(rec['inputs']['previous_report'])
    else: raise PerceptionUnknown('No support-root action in bounded history')
    if old:
        retained = {str(Path(h['report']).resolve()): h['sha256'] for h in chain}
        if any(retained.get(str(Path(h['report']).resolve())) != h['sha256'] for h in old['history']):
            raise PerceptionUnknown('Stack action history bypasses retained predecessor')
    updated = copy.deepcopy(scene)
    for color in ('red','orange'):updated['blocks'][color] = copy.deepcopy(fresh['blocks'][color])
    updated['stack_reference'] = {'report':str(support_report),'sha256':digest,
        'frame_id':fresh['frame_id'],'history':chain,
        'scope':'Fresh base positions retained through checked green-only motions; invalidate on observed displacement'}
    # Clear visible displacement invalidates retention; occlusion is not new depth.
    from history_scene import reject_visible_displacement
    reject_visible_displacement(dict(updated, blocks={c:updated['blocks'][c] for c in ('red','orange')}), obs)
    return updated


def loaded_state(ctx, previous_report, support_report, obs):
    robot=ctx.value('piper_status');r=completed_action_state(read_report(previous_report),obs,robot)
    if not r.get('ok') or r['stage'] not in ('transport','green_stack_lower'):
        raise PerceptionUnknown('Stack lowering needs verified green transport/lowering')
    fact=r['facts'][0];origin=fact['payload_origin']
    if origin.get('color')!='green':raise PerceptionUnknown('Stack placement accepts retained green only')
    scene=support_scene(ctx,previous_report,obs,robot,support_report,fact['scene'])
    return r,scene,origin


def stack_target(scene, origin, away_m=.003):
    """Place green over red, with a small offset away from the orange neighbor."""
    if not 0 <= away_m <= .005:raise ValueError('Away offset outside0..5mm')
    red=np.array(scene['blocks']['red']['top_center']['surface_position_m'])
    orange=np.array(scene['blocks']['orange']['top_center']['surface_position_m'])
    green=np.array(scene['blocks']['green']['top_center']['surface_position_m'])
    grasp=np.array(origin['grasp_flange']);offset=green[:2]-grasp[:2]
    direction=red[:2]-orange[:2];direction/=np.linalg.norm(direction)
    xy=red[:2]+direction*away_m-offset
    return np.r_[xy,red[2]+origin['flange_to_bottom_m']].tolist()


def check_stack_geometry(ctx, obs, scene, origin, target, quaternion, *, final):
    """One near-vertical descent; model support contact only at the red top."""
    from manipulation_checks import finger_locals
    return planned_stack_geometry(ctx,obs['measured14'],scene,origin,target,quaternion,
                                  finger_locals(ctx,obs),final=final)


def planned_stack_geometry(ctx,start,scene,origin,target,quaternion,locals,*,final):
    """Pure geometry from hypothetical joints; execution still binds actual state."""
    f=ctx.value('piper_fk',joints=start[:6]);T=np.array(f['T_base_flange']);p=np.array(f['position']);target=np.array(target)
    if not .048 <= start[6] <= .054 or T[2,2]>-.995 or abs(np.dot(quaternion,f['quaternion_xyzw']))<.99999:
        raise PerceptionUnknown('Expected preserved near-down loaded green grip')
    if np.linalg.norm(target[:2]-p[:2])>.002 or not .005<=p[2]-target[2]<=MAX_DESCENT_M:
        raise PerceptionUnknown('Stack segment must descend5..50mm vertically')
    red=np.array(scene['blocks']['red']['top_center']['surface_position_m'])
    orange=np.array(scene['blocks']['orange']['top_center']['surface_position_m'])
    green=np.array(scene['blocks']['green']['top_center']['surface_position_m'])
    grip=np.array(origin['grasp_flange']);xy_offset=green[:2]-grip[:2]
    if np.max(np.abs(target[:2]+xy_offset-red[:2]))>MAX_CENTER_ERROR_M:
        raise PerceptionUnknown('Green center not over red support footprint')
    bottom=target[2]-origin['flange_to_bottom_m']
    if bottom<red[2]-.001 or (final and abs(bottom-red[2])>.002):
        raise PerceptionUnknown('Stack bottom target inconsistent with measured red top')
    sol=ctx.value('piper_solve_ik',position=target.tolist(),quaternion_xyzw=quaternion,seed=start[:6]);end=sol['q']
    check=ctx.value('piper_check_path',start14=start,target14=end+start[6:])
    if check.get('cad_clear') is not True or check.get('workspace_clear') is not True:
        raise PerceptionUnknown('Stack native CAD/workspace preflight failed')
    plane=np.array(scene['table']['z_ax_by_c']);minimum=1.;min_orange=1.;rows=[]
    for t in np.linspace(0,1,41):
        F=np.array(ctx.value('piper_fk',joints=(np.array(start[:6])+(np.array(end)-start[:6])*t).tolist())['T_base_flange']);fp=F[:3,3]
        center=fp[:2]+xy_offset;bottom=fp[2]-origin['flange_to_bottom_m']
        if np.linalg.norm(fp[:2]-p[:2])>.003 or np.max(np.abs(center-red[:2]))>MAX_CENTER_ERROR_M:
            raise PerceptionUnknown('Interpolated stack path leaves red support corridor')
        if bottom<red[2]-.001:raise PerceptionUnknown('Interpolated payload goes below red support')
        sep=max(float(np.max(np.abs(center-orange[:2]))-PAYLOAD_HALF_M-BLOCK_HALF_M),float(bottom-orange[2]))
        min_orange=min(min_orange,sep)
        if sep<.001:raise PerceptionUnknown('Green payload envelope overlaps neighboring orange')
        for local in locals:
            c=local@F[:3,:3].T+fp;table=c[:,:2]@plane[:2]+plane[2]
            if np.min(c[:,2]-table)<.012:raise PerceptionUnknown('Stack finger/table clearance below12mm')
            gap=float(c[:,2].min()-max(red[2],orange[2]));minimum=min(minimum,gap)
            if gap<MIN_FINGER_SUPPORT_M:raise PerceptionUnknown('Finger too close to base tops during stack placement')
        rows.append({'fraction':float(t),'flange':fp.tolist(),'green_bottom_m':float(bottom)})
    return {'preflight':check,'end_joints':end,'samples':rows,'min_finger_base_top_m':minimum,'min_green_orange_separation_m':min_orange}


class StackLowerChecks:
    def __init__(self,ctx,previous_report,support_report,final):
        self.ctx=ctx;self.previous=previous_report;self.support=support_report;self.final=final
    def __call__(self,*,phase,before,after,inputs):
        if phase=='stack_path':
            prev,scene,origin=loaded_state(self.ctx,self.previous,self.support,before)
            retain=retention_at_endpoint(self.ctx,prev,before,'green')
            if len(inputs['descent'])!=1 or len(inputs['retreat'])!=1:raise PerceptionUnknown('One descent/retreat required')
            geometry=check_stack_geometry(self.ctx,before,scene,origin,inputs['descent'][0],inputs['quaternion'],final=self.final)
            from task_geometry import retreat_anchor
            down=np.array(inputs['descent'][0]);p=np.array(geometry['samples'][0]['flange'])
            anchor=retreat_anchor(inputs['retreat'][0],p.tolist(),reverse_to_start=True);up=np.array(anchor['position'])
            if np.linalg.norm(up[:2]-down[:2])>.002:
                raise PerceptionUnknown('Stack retreat must reverse this checked segment to its starting pose')
            end=geometry['end_joints'];start=before['measured14']
            # Exact checked joint segment reversal is the recovery path; no new IK branch.
            reverse=self.ctx.value('piper_check_path',start14=end+start[6:],target14=start)
            if reverse.get('cad_clear') is not True or reverse.get('workspace_clear') is not True:
                raise PerceptionUnknown('No checked stack reversal')
            self.placement_context=(scene,origin)
            return {'ok':True,'frame_id':before['frame_id'],'scene':scene,'payload_origin':origin,
                'why':'Verified green retention, fresh support provenance, centered red support corridor and sampled finger/payload clearance',
                'retention':retain,'geometry':geometry,'retreat_preflight':reverse,'retreat_joints':start[:6],'retreat_position':up.tolist(),'retreat_anchor':anchor,
                'support_required':self.final,'limits':'Provisional depth/TCP; endpoint support must be inspected before opening'}
        if phase=='stack_supported':
            if not self.final:
                fact=retention_evidence(self.ctx,before,after,'green')
                return fact|{'frame_id':after['frame_id'],'supported':False}
            from ordinary_checks import placement
            ordinary=placement(self.ctx,phase,after,*self.placement_context)
            if ordinary.get('ok') is not None:return ordinary
            fact=assess_images(self.ctx,phase,before,after,
                'Green has been lowered over the red top to its measured support height. Is green upright and broadly aligned directly on RED, with no obvious hovering, tipping, slipping or bridging? The other base block need not be fully visible for this local support decision. Assess ordinary visible support suitable for opening in place, without demanding hidden underside or exact millimetre gaps.',color='green')
            return fact|{'supported':fact.get('ok') is True}
        raise PerceptionUnknown('Unexpected stack phase')


class StackReleaseChecks:
    def __init__(self,ctx,previous_report,support_report):self.ctx=ctx;self.previous=previous_report;self.support=support_report
    def __call__(self,*,phase,before,after,inputs):
        prev,scene,origin=loaded_state(self.ctx,self.previous,self.support,after)
        if prev['stage']!='green_stack_lower' or not prev.get('support_required') or not (prev.get('recovered_endpoint') or prev['facts'])[-1].get('supported'):
            raise PerceptionUnknown('Green release needs completed verified final red support')
        if not .048<=after['measured14'][6]<=.054:raise PerceptionUnknown('Unexpected loaded jaw')
        fact=reuse_support_endpoint(prev,after,phase)
        if fact is None:
            from ordinary_checks import placement
            fact=placement(self.ctx,phase,after,scene,origin)
        if fact.get('ok') is None:fact=assess_images(self.ctx,phase,prev['measured'],after,
            'Before opening the gripper: is green still upright, broadly centered on and supported by RED, without obvious tipping, hovering or bridging? Small alignment differences and perspective distortion are acceptable. The final lowering already reached the measured red-top support target. Assess current ordinary support, not hidden forces.',color='green')
        return fact|{'scene':scene,'payload_origin':origin,'supported':fact.get('ok') is True}


class StackRetreatChecks:
    def __init__(self,ctx,previous_report):self.ctx=ctx;self.previous=previous_report
    def __call__(self,*,phase,before,after,inputs):
        if phase=='released_and_retreat_path':
            from manipulation_checks import RetreatChecks
            # Reuse the numerical rise checker with explicit green-release stage support.
            return RetreatChecks(self.ctx,self.previous,release_stage='green_stack_release').path(before,inputs)
        if phase=='released_stable_and_clear':
            from ordinary_checks import after_release
            ordinary=after_release(self.ctx,phase,after,self.previous)
            if ordinary.get('ok') is not None:return ordinary
            assessment=assess_images(self.ctx,phase,before,after,
                'The open gripper has withdrawn after releasing green onto red. Are green directly on red and orange beside red, all upright and stable, with green supported by red rather than bridging, and the open gripper no longer carrying or supporting any block? Use the head and wrist perspectives; do not demand hidden contact-force proof.',color='green')
            assessment['ordinary_evidence']=ordinary
            if assessment.get('ok') is None:assessment['why']=ordinary.get('why','Unresolved endpoint')+'; '+assessment['why']
            return assessment
        raise PerceptionUnknown('Unexpected stack-retreat phase')
