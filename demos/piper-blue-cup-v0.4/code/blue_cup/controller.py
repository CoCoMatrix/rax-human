"""Native RAX fn(ctx, inputs) composition. No planning agent, SDK or robot restart.

Candidate only: release/retreat and varied layouts require supervised live tests.
Execute from a fresh, empty-gripper scene. Every failure returns; no replay or home.
"""
import json
import hashlib
import math
import time
import uuid
from pathlib import Path
from rehearsal import compact
from placement import trial_lift, lower
from scheduled import prepare_for_approach, approach, grasp, localize_coaster, transfer_step, release, retreat
from transport import placement_context, placement_identity
from perception import OUT, visual
from checks import assess, context_for, current_depth

MAX_TRANSFER_STEPS=5
MAX_LOWER_STEPS=8
LOWER_STEP_M=.020
CONTACT_STEP_M=.003
SUPPORT_APPROACH_BAND_M=.005
RELEASE_WIDTH_M=.012
RETREAT_SEGMENTS_M=(.080,)
# Distances are policy candidates inherited from rehearsal, not measured clearance.


def _metric(result,key):
    # Parents such as prepare already sum their children. Support checks do not.
    # A reused boundary is explicitly zero, never counted as another model call.
    if key in result:return result[key]
    return sum(_metric(child,key) for child in result.get('steps',[]))


def assess_support(ctx, inputs):
    """Current depth/rim support estimate; Gemini only if ordinary rules cannot tell."""
    start=time.monotonic();bound,identity=placement_context(ctx,inputs)
    obs=current_depth(ctx,bound['observation']);bound['observation']=obs
    rules=context_for(ctx,obs,robot=ctx.value('piper_status'),action='assess_support',inputs=inputs)
    v=assess(ctx,obs,identity+
        'Assess overall placement qualitatively: upright, broadly over the coaster, level base/coaster '
        'relationship, visible edge/shadow and no gross tilt or displacement. A partly hidden base '
        'does not demand a measured air gap or full contact perimeter. '
        'Return support_state as resting, suspended, or unknown. Resting means the combined cues '
        'suggest the coaster already supports the cup. Use suspended for clear visible suspension; '
        'use unknown only if overall support remains genuinely indeterminate. Do not infer support '
        'from gripper width, target height or commanded descent alone.',
        required_facts={'placement_scene':'Cup is upright, broadly at the intended coaster, without obvious obstruction or gross edge misplacement'},context=rules,fallback=visual)
    bound['observation']=v.pop('_checked_observation',obs)
    state=v.get('support_state')
    return {'ok':v.get('ok') is True and state in ('resting','suspended'),
            'function':'assess_support','support_state':state,'measured':v,
            'why':v.get('reason'),'observation':bound['observation'],
            'visual_calls':v.get('visual_calls',0),'perception_s':v.get('perception_s',0),
            'wrist_depth_calls':v.get('wrist_depth_calls',0),
            'total_s':time.monotonic()-start}


def lower_to_support(ctx, inputs):
    """Check BEFORE further descent: R31 may already rest before predicted height.

    Predicted height bounds candidate descent only; it never proves support. Once
    resting is observed return immediately. Do not push below the prediction to
    make an uncertain image pass.
    """
    start=time.monotonic();target=json.loads(Path(inputs['target_report']).read_text())
    if target.get('localization_source')=='initial_same_run_metric_coaster':
        from scheduled import lower_to_height
        return lower_to_height(ctx,inputs)
    previous=inputs['previous_report'];steps=[];check=None
    for _ in range(MAX_LOWER_STEPS):
        if check is None:
            check=assess_support(ctx,{'target_report':inputs['target_report'],'previous_report':previous})
        steps.append({k:v for k,v in check.items() if k!='observation'})
        if not check['ok']:
            return {'ok':False,'function':'lower_to_support','why':'Current qualitative support/alignment unresolved',
                    'steps':steps,'previous_report':previous,'support_state':check.get('support_state')}
        if check['support_state']=='resting':
            return {'ok':True,'function':'lower_to_support','why':'Qualitatively resting; no further downward command',
                    'steps':steps,'previous_report':previous,'support_state':'resting','observation':check['observation'],'total_s':time.monotonic()-start}
        obs=check['observation'];fk=ctx.value('piper_fk',joints=obs['measured14'][:6],arm='left')
        remaining=fk['position'][2]-target['predicted_support_flange_z']
        if not math.isfinite(remaining) or remaining<=.001:
            return {'ok':False,'function':'lower_to_support','why':'At predicted support limit but support unconfirmed',
                    'steps':steps,'previous_report':previous}
        dz=min(CONTACT_STEP_M,remaining) if remaining<=SUPPORT_APPROACH_BAND_M else min(LOWER_STEP_M,remaining-SUPPORT_APPROACH_BAND_M)
        result=lower(ctx,{'dz':-dz,'expect_support':False,'assess_support':True,'target_report':inputs['target_report'],'previous_report':previous,'observation':obs})
        steps.append(compact(result))
        if result.get('ok') is not True:
            return {'ok':False,'function':'lower_to_support','why':result.get('reason'),'steps':steps,'previous_report':previous}
        previous=result['report_path']
        # Same action boundary: its current-only result also answers support.
        # The next motion still acquires fresh state/path/scene preconditions.
        post=result.get('post',{});state=post.get('support_state')
        check={'ok':post.get('ok') is True and state in ('resting','suspended'),
               'support_state':state,'observation':result['receipt']['observation'],
               'measured':post,'visual_calls':0,'reused_from':result['report_path']}
        if state=='resting' and check['ok']:
            return {'ok':True,'function':'lower_to_support','why':'Qualitatively resting after descent',
                    'steps':steps,'previous_report':previous,'support_state':'resting','observation':check['observation'],'total_s':time.monotonic()-start}
    return {'ok':False,'function':'lower_to_support','why':'Bounded descent exhausted','steps':steps,'previous_report':previous}


def main(ctx):
    """Same supplied Context for all steps. Zero main-agent requests.

    Each stage returns a compact checkpoint/report. On failure inspect the saved
    actual state and invoke the named function with freshly bound inputs; never
    rerun main to resume a possibly held object. No automatic uncertain replay.
    """
    run=OUT/('merged-'+uuid.uuid4().hex);run.mkdir(parents=True)
    started=time.monotonic();report={'ok':False,'run_id':run.name,'steps':[],
        'main_agent_calls':0,'held_object_possible':False,'task_success':False,
        'metrics_complete':True,'execution_mode':'compiled_controller_no_agent',
        'code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in sorted(Path(__file__).parent.glob('*.py'))},
        'validation':'offline_candidate_not_end_to_end_live_validated'}
    def save():
        report['elapsed_s']=time.monotonic()-started
        report['auxiliary_visual_calls']=sum(_metric(s,'visual_calls') for s in report['steps'])
        report['wrist_depth_calls']=sum(_metric(s,'wrist_depth_calls') for s in report['steps'])
        report['timings']={key:sum(_metric(s,key) for s in report['steps'])
                           for key in ('perception_s','planning_s','motion_s')}
        path=run/'report.json';tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(report,indent=2));tmp.replace(path)
    def step(name,fn,*args):
        report['active_step']=name;save()
        stage_started=time.monotonic()
        try:value=fn(ctx,*args)
        except Exception as exc:
            report['metrics_complete']=False
            report['steps'].append({'stage':name,'ok':False,
                'stage_elapsed_s':time.monotonic()-stage_started,
                'why':f'{type(exc).__name__}: {exc}'})
            save();raise
        report['steps'].append({'stage':name,**compact(value),
                               'stage_elapsed_s':time.monotonic()-stage_started})
        if value.get('ok') is not True:
            report.update(failed_step=name,why=value.get('why') or value.get('reason'),
                          recovery='inspect_current_state_then_reenter_one_function; never replay main')
            save();raise StopIteration
        save();return value
    try:
        # Per-run boundary reuse must never leak from an older main() invocation.
        if hasattr(ctx,'_rax_condition_boundary'):del ctx._rax_condition_boundary
        # These functions derive every approach target from a new layout, not R31 coordinates.
        prep=step('prepare',prepare_for_approach)
        step('hover',approach,{'phase':'hover'})
        entry=step('entry',approach,{'phase':'entry'})
        report['held_object_possible']=True  # even an ambiguous closure must not auto-open
        step('grasp',grasp)
        lift=step('trial_lift',trial_lift,{'dz':.040,'execution_mode':'task_scheduled'})
        target=step('localize_coaster',localize_coaster,{'preparation_report':prep['report_path'],
            'entry_report':entry['report_path'],'lift_report':lift['report_path']})
        previous=target['report_path']
        for i in range(MAX_TRANSFER_STEPS):
            transfer=step('transfer_'+str(i+1),transfer_step,{'target_report':target['report_path'],'previous_report':previous})
            previous=transfer['report_path']
            if transfer.get('transport_complete'):break
        else:
            report.update(failed_step='transfer',why='Transfer bound exhausted');return report
        supported=step('lower_to_support',lower_to_support,{'target_report':target['report_path'],'previous_report':previous})
        opened=step('release',release,{'width_m':RELEASE_WIDTH_M,'target_report':target['report_path'],
            'previous_report':supported['previous_report'],'observation':supported.get('observation')})
        report['held_object_possible']=False
        previous=opened['report_path']
        for i,dz in enumerate(RETREAT_SEGMENTS_M):
            done=step('retreat_'+str(i+1),retreat,{'delta':[0,0,dz],'target_report':target['report_path'],
                'previous_report':previous,'observe_result':i==len(RETREAT_SEGMENTS_M)-1})
            previous=done['report_path']
        report.update(ok=True,task_success=True,why='Release and retreat verified on this run',active_step='done')
    except StopIteration:pass
    except Exception as exc:
        report.update(failed_step=report.get('active_step','startup'),why=f'{type(exc).__name__}: {exc}',
                      recovery='status_and_fresh_observation_only; no automatic retry/release/home')
    finally:save()
    return report


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true',help='Execute via an already supervised, armed native session; no Agent launched')
    a=p.parse_args()
    if not a.run:p.error('No motion by default. Use --run only inside the authorized robot session.')
    from api.runtime import Context
    with Context() as ctx:print(json.dumps(main(ctx),indent=2))
