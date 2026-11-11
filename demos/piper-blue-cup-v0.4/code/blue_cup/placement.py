"""Ordinary RAX task functions with qualitative scene checks.

Current policy: user-selected TCP depth40mm below the current rim, closure3mm.
The40mm candidate is not R10's measured26mm experience and needs live validation.
These parameters select actions, not success labels; trial lift checks actual pickup.
"""
import json,math,time
from rehearsal import move_once, _options, compact
from perception import OUT

GRASP_WIDTH_M=.003
TRIAL_LIFT_MIN_M=.040  # R10 memory's2026-11-06 operator preference: one lift >=40mm.
TRIAL_LIFT_DEFAULT_M=TRIAL_LIFT_MIN_M


def grasp(ctx, explicit_inputs=None):
    """One bounded wall-closure subgoal; maximum10mm jaw change per dispatch."""
    e=explicit_inputs or {};start=time.monotonic()
    r={'function':'grasp','ok':False,'executed':False,'retention_verified':False,
       'visual_calls':0,'perception_s':0,'planning_s':0,'motion_s':0,'steps':[]}
    try:
        desired=float(e.get('width_m',GRASP_WIDTH_M))
        if not math.isfinite(desired) or not .003<=desired<=.006:raise ValueError('Wall-closure target outside3..6mm characterized candidate range')
        obs=e.get('observation') or ctx.value('piper_observe',cameras=['head','left','right'],require_depth=not bool(e.get('operator_file')))
        width=obs['measured14'][6]
        if not desired<width<=.020:raise ValueError('Closure needs an open jaw above target and <=20mm; no automatic opening/replay')
        for index in range(3):
            target=max(desired,width-.010)
            options=_options(e)
            if index:
                options.pop('operator_file',None)
                options.pop('scene_binding',None)  # Previous jaw pose was intentionally changed.
            options['observation']=obs
            step=move_once(ctx,dict(**options,name='grasp_close_segment',tool='piper_gripper',args={'width_m':target},
                pre='Make one practical qualitative judgment before the operator-selected3mm closure. The current-rim target and measured entry set the nominal wall approach. Check overall cup position and obvious obstructions; do not require RGB to prove hidden inside/outside pad topology or pre-prove retention. Expected occlusion below the rim is acceptable. The40mm trial lift tests whether this attempt actually picked up the cup.',
                pre_facts=({'grasp_setup':'Cup is upright and table-supported, the left jaw is broadly at the intended cup-wall region, and there is no clear gross mis-targeting, tipping or handle/table/cable trapping. Exact hidden pad contact is not a required visual fact'} if index==0 else {}),
                post_facts={}))
            r['steps'].append(compact(step))
            for k in ('visual_calls','wrist_depth_calls','perception_s','planning_s','motion_s'):r[k]=r.get(k,0)+step.get(k,0)
            if step.get('executed') is None:r['executed']=None
            elif step.get('executed') is True:r['executed']=True
            if not step.get('ok'):
                r['reason']=step.get('reason')
                for k in ('next_action','operator_request','operator_file'):
                    if k in step:r[k]=step[k]
                return r
            r['after_frame']=step.get('after_frame');r['motion_arrived']=step.get('motion_arrived')
            if target==desired:
                r.update(ok=True,task_completed=True,closure_target_m=desired,
                    reason='Characterized closure completed; actual pickup remains to be checked by trial_lift',next_action='trial_lift')
                return r
            obs=step['receipt'].get('observation')
            if obs is None:obs=ctx.value('piper_observe',cameras=['head','left','right'],require_depth=False)
            width=obs['measured14'][6]
            if not desired<width<=.020:raise ValueError('Unexpected intermediate jaw feedback; inspect, never replay')
        r['reason']='Closure segment bound exhausted';return r
    except Exception as exc:r['reason']=str(exc);return r
    finally:
        r['total_s']=time.monotonic()-start;path=OUT/(str(time.time_ns())+'-grasp.json');r['report_path']=str(path)
        path.write_text(json.dumps(r,indent=2))
        with (OUT/'timings.jsonl').open('a') as f:f.write(json.dumps(r)+'\n')


def trial_lift(ctx, explicit_inputs=None):
    e=explicit_inputs or {};dz=float(e.get('dz',TRIAL_LIFT_DEFAULT_M))
    if not math.isfinite(dz) or not 0<dz<=.070:return {'ok':False,'executed':False,'reason':'Trial lift must be positive and within the70mm local motion bound'}
    # Select the experienced minimum BEFORE IK, whole-path and visual route checks.
    # Old10/20mm suggestions become a40mm candidate, not a refusal or repeated steps.
    dz=max(TRIAL_LIFT_MIN_M,dz)
    return move_once(ctx,dict(**_options(e),name='trial_lift',tool='piper_move_pose',args={'delta':[0,0,dz]},
        pre='This lift tests the experienced wall grasp; do not require retention to be proved before testing it. Check the entire proposed upward displacement including the held cup, not only its first10mm.',
        pre_facts={} if e.get('execution_mode')=='task_scheduled' else {'lift_route':'The proposed upward route is broadly free of obvious obstacles or entanglement'},
        post=('Compare BEFORE and CURRENT head/left views for a practical overall pickup judgment. '
              'The wrist camera moves with the gripper: a cup wall/rim staying at approximately the same location relative to the orange pads is POSITIVE retention evidence, not evidence that the arm did not lift. '
              'Combine this with the reported upward arrival and available head-view cup/table displacement, changed table features in the wrist view, or base/shadow separation. '
              'These are alternative supporting cues, not a checklist requiring all of them. Shadows are secondary evidence because illumination and the arm can change them. '
              'Broadly stable cup-to-pad position plus consistent upward-motion evidence can support a likely successful pickup even when the base is hidden. '
              'Do not demand direct sight of the entire bottom, a measured gap, identical pad pixels, or proof of no transient slip. '
              'Clear slipping out, dropping, or visible continuing table support contradict success; absence of a visible gap alone does not. '
              'If the combined evidence is genuinely inconclusive, report unknown without inventing a failure or repeating the lift.'),
        post_facts={'cup_lifted_and_held':'Combined before/after cues support that the cup probably followed the upward gripper and remains broadly stably held; direct visibility of the base is not required'}))


def transfer(ctx, explicit_inputs):
    e=explicit_inputs;d=e['delta']
    if len(d)!=3 or d[2]!=0 or not all(math.isfinite(x) for x in d) or math.dist(d,[0,0,0])>.055:
        return {'ok':False,'executed':False,'reason':'Transfer requires horizontal <=55mm step'}
    return move_once(ctx,dict(**_options(e),name='transfer',tool='piper_move_pose',args={'delta':d},
        pre='Judge the overall held cup and intended short route, not exact pad contact or millimetre clearance.',
        pre_facts={'transfer_ready':'Cup appears held clear of support, and the intended payload route has no obvious obstacle or table/coaster collision'},
        post='Check the overall cup outcome after transfer. Small tilt or relative pose change is not alone a failure; dropping, obvious loss of grip or collision is.',
        post_facts={'cup_retained':'Cup followed the gripper and remains held, with no obvious loss of grasp or collision'}))


def lower(ctx, explicit_inputs):
    e=explicit_inputs;dz=float(e['dz'])
    if not math.isfinite(dz) or not -.02<=dz<0:return {'ok':False,'executed':False,'reason':'Lower must be <=20mm downward'}
    options=_options(e); identity=''
    if e.get('target_report'):
        from transport import placement_context
        try:
            bound,identity=placement_context(ctx,e);options.update(bound)
        except Exception as exc:return {'ok':False,'executed':False,'reason':str(exc)}
    facts=({'cup_supported':'Cup appears resting stably on the coaster in the intended placement, not obviously suspended or precariously perched on its edge'}
           if e.get('expect_support') else {'lowering_result':'Cup remains upright and under control in the intended coaster region, with no obvious wrong placement or collision'})
    return move_once(ctx,dict(**options,assess_support=e.get('assess_support',False),current_only_post=True,name='lower',tool='piper_move_pose',args={'delta':[0,0,dz]},
        pre=identity+'Check practical cup/coaster alignment and the next short descent. Exact concentricity and visibility of the entire base perimeter are unnecessary.',
        pre_facts={'lowering_ready':'Held cup is broadly over the intended coaster and the short descent has no obvious obstruction or gross edge misplacement'},
        post=identity+'Judge the overall placement/support state from available visible cues; no millimetre gap or full hidden contact surface proof. Arrival alone does not establish support. '+('Also return support_state: resting, suspended, or unknown, based on combined qualitative cues, not the commanded height.' if e.get('assess_support') else ''),post_facts=facts))


def release(ctx, explicit_inputs):
    e=explicit_inputs;options=_options(e);identity=''
    if e.get('target_report'):
        from transport import placement_context
        bound,identity=placement_context(ctx,e);options.update(bound)
    return move_once(ctx,dict(**options,current_only_post=True,name='release',tool='piper_gripper',args={'width_m':e['width_m']},
        pre=identity+'Use practical visible support cues: cup resting level on coaster, no obvious suspension/edge instability, fingers can open away. Hidden portions of the base need not all be visible; uncertain overall support still cannot authorize release.',
        pre_facts={'release_ready':'Cup appears stably supported by the coaster and opening the fingers has no obvious rim/handle entanglement'},
        post=identity+'Did the cup stay standing on the coaster when the jaw opened? Small settling is allowed; tipping, following the opening fingers or falling is failure.',
        post_facts={'released_on_coaster':'Cup remains upright on the coaster after opening and is no longer carried by the gripper'}))


def retreat(ctx, explicit_inputs):
    e=explicit_inputs;d=e['delta']
    if len(d)!=3 or not all(math.isfinite(x) for x in d) or math.dist(d,[0,0,0])>.07:
        return {'ok':False,'executed':False,'reason':'Retreat exceeds70mm'}
    return move_once(ctx,dict(**_options(e),current_only_post=True,name='retreat',tool='piper_move_pose',args={'delta':d},
        pre='Judge the released cup and short retreat route qualitatively.',
        pre_facts={'retreat_ready':'Cup stands supported on the coaster, jaw is released and retreat has no obvious snag or collision'},
        post='Check the final task outcome, allowing small harmless settling rather than demanding identical pixels.',
        post_facts={'placement_stable':'Cup remains upright on the coaster and the empty gripper has moved away without disturbing it'}))
