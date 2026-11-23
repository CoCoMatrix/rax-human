"""One native RAX function invocation; no server, polling loop or approval files.

Default CLI prints a plan without connecting. --execute uses one outer Context.
An optional checks module must implement real synchronous ordinary perception (or
on-demand Gemini). Missing evidence returns immediately to the main Agent.
"""
import argparse
import hashlib
import importlib
import inspect
import json
import pathlib
import sys
import time
import uuid
import controller

FUNCTIONS = ('green_stack_lower', 'green_stack_release', 'green_stack_retreat', 'refresh_stack_support', 'plan_base_placement', 'green_pregrasp', 'empty_inspection', 'verify_completed_action', 'locate_block', 'verify_approach_boundary', 'open_empty_gripper', 'empty_joint_waypoint', 'empty_joint_transition','empty_joint_batch', 'approach',
             'close_grasp', 'trial_lift', 'transport', 'lower_to_support', 'release', 'retreat')
MOTIONS = ('piper_move_waypoints','piper_gripper_stroke', 'piper_gripper', 'piper_move_pose', 'piper_move_joints')


def save(path, data, *, pretty=True):
    payload = json.dumps(data, ensure_ascii=False, indent=2 if pretty else None, allow_nan=False)
    path = pathlib.Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(payload); tmp.replace(path)


def compact(result):
    obs = result.get('measured') or result.get('review', {}).get('after') or {}
    return {k: result.get(k) for k in ('ok', 'status', 'stage', 'why', 'executed',
            'dispatch_attempted', 'action_completed', 'needs_review', 'next_function', 'elapsed_s')} | {
            'frame_id': obs.get('frame_id'), 'measured14': obs.get('measured14')}


def run_one(ctx, function, inputs, output, *, assess=None, operation=None):
    # Composition uses this same recorder for its read-only input calculations.
    # The ordinary single-function CLI still accepts only FUNCTIONS.
    if function not in FUNCTIONS and operation is None:
        raise ValueError('Choose one rehearsable function, not main or a combined replay')
    # Freeze JSON inputs before adding the runtime callable. Never mutate them.
    args = json.loads(json.dumps(inputs, allow_nan=False))
    if 'assess' in args: raise ValueError('Use the separate synchronous assess argument')
    fn = operation if operation is not None else getattr(controller, function)
    call_args = dict(args)
    if 'assess' in inspect.signature(fn).parameters: call_args['assess'] = assess
    inspect.signature(fn).bind(ctx, **call_args)
    output = pathlib.Path(output)
    if output.exists(): raise FileExistsError('Existing invocation report: inspect it; do not replay')
    root = output.parent; root.mkdir(parents=True, exist_ok=True)
    memory = root.parents[1] / 'memory' if root.name == 'rehearsal' and root.parent.name == 'output' else root
    start = time.monotonic(); rows=[]; original=ctx.value
    # green_pregrasp delegates to approach(color='green', engage=False). Preserve
    # that semantic identity for the existing support/history validators, while
    # retaining the exact public invocation separately. Do not pass extra kwargs.
    recorded_inputs = args | {'color':'green', 'engage':False} if function == 'green_pregrasp' else args
    identity = {'function': function, 'inputs': recorded_inputs, 'invocation_inputs': args,
                'source_sha256': hashlib.sha256(pathlib.Path(controller.__file__).read_bytes()).hexdigest(),
                'code_hashes': {name: hashlib.sha256((pathlib.Path(controller.__file__).parent/name).read_bytes()).hexdigest()
                                for name in ('controller.py','perception.py','scene_checks.py','rehearse_step.py','manipulation_checks.py','green_checks.py','qualitative.py','history_scene.py','placement_plan.py','stack_checks.py','composition.py','ordinary_checks.py','observations.py','recovery.py','timing.py','task_geometry.py','continuous_approach.py','empty_transition.py','batched_entry.py','fk_samples.py','pure_fk.py','native_fk/__init__.py','native_fk/geometry.py','native_fk/assets/dual_piper_x.urdf','assembly_plan.py','assembly_controller.py','payload_replan.py','assembly_batch.py')},
                'started_unix': time.time(), 'report': str(output)}
    save(output, identity | {'status':'in_progress','why':'If interrupted, inspect native status/receipts before another action'})
    save(memory/'REHEARSAL_STATE.json', identity | {'status':'in_progress','never_blindly_replay':True})

    def logged(tool, **kw):
        t=time.monotonic(); row={'tool':tool,'arguments':kw,'started_unix':time.time()}
        # Persist intent before a potentially irreversible dispatch.
        if tool in MOTIONS:
            save(memory/'REHEARSAL_STATE.json', identity | {'status':'motion_dispatch_pending',
                 'tool':tool,'arguments':kw,'never_blindly_replay':True})
        try:
            result=original(tool,**kw);row['result']=result
            return result
        except BaseException as exc:
            row['error']=f'{type(exc).__name__}: {exc}'
            rejection=controller.no_dispatch_error(exc)
            if rejection is not None:row['native_rejection']=rejection;row['no_dispatch_proven']=True
            raise
        finally:
            row['elapsed_s']=time.monotonic()-t;rows.append(row)
            with output.with_suffix('.tools.jsonl').open('a') as stream:
                stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()

    ctx.value=logged
    try:
        result=fn(ctx, **call_args)
    except BaseException as exc:
        # Preserve completed receipts even if Python or a perception callback fails.
        completed=[r['result'] for r in rows if r['tool'] in MOTIONS
                   and isinstance(r.get('result'),dict) and r['result'].get('executed')]
        result={'ok':False,'status':'interrupted' if isinstance(exc,KeyboardInterrupt) else 'error',
                'why':f'{type(exc).__name__}: {exc}','executed':bool(completed),
                'dispatch_attempted':any(r['tool'] in MOTIONS and not r.get('no_dispatch_proven') for r in rows),
                'measured':(completed[-1].get('observation') or {'measured14':completed[-1]['final14']}) if completed else None,
                'never_blindly_replay':True}
        if result['dispatch_attempted']:
            try:result['hold']=ctx.value('piper_hold')
            except BaseException as stop_error:result['hold_error']=str(stop_error)
    finally:
        ctx.value=original
    wall=time.monotonic()-start
    from collections import Counter
    motion_rows = [r for r in rows if r['tool'] in MOTIONS]
    confirmed = lambda r: isinstance(r.get('result'),dict) and all(
        r['result'].get(k) is True for k in ('executed','measured_arrival','measured_settled'))
    record=identity | {'status':'returned','result':result,'wall_s':wall,
           'tool_counts':dict(Counter(r['tool'] for r in rows)),
           'motion_rpc_attempt_count':len(motion_rows),
           'pre_dispatch_rejection_count':sum(r.get('no_dispatch_proven') is True for r in motion_rows),
           'motion_dispatch_count':sum(not r.get('no_dispatch_proven') for r in motion_rows),
           'motion_receipt_count':sum(confirmed(r) for r in motion_rows),
           'ambiguous_dispatch_count':sum(not confirmed(r) and not r.get('no_dispatch_proven') for r in motion_rows),
           'tool_s':sum(r['elapsed_s'] for r in rows),
           'motion_rpc_s':sum(r['elapsed_s'] for r in rows if r['tool'] in MOTIONS),
           'auxiliary_visual_calls':sum(r['tool']=='gemini_generate' for r in rows),
           'agent_wait_inside_function_s':0,'calls':len(rows)}
    # All fields are JSON primitives; the callable never enters identity or record.
    from timing import breakdown
    record['timing']=breakdown(rows,wall)
    record['summary_report']=str(output.with_suffix('.summary.json'))
    save(output, record,pretty=False)
    summary=compact(result)|{'report':str(output),'summary_report':record['summary_report'],
        'wall_s':wall,'tool_counts':record['tool_counts'],'auxiliary_visual_calls':record['auxiliary_visual_calls'],
        'timing':record['timing'],'observation_source':result.get('observation_source'),
        'facts':[{'ok':f.get('ok'),'why':str(f.get('why',''))[:350],'frame_id':f.get('frame_id'),
                  'method':f.get('method'),'reused_support':f.get('reused_support',False)} for f in result.get('facts',[])][-3:]}
    summary['why']=str(summary.get('why',''))[:500]
    save(output.with_suffix('.summary.json'),summary)
    current=identity | compact(result) | {'status':result.get('status','returned'),
                 'wall_s':wall,'agent_wait_inside_function_s':0,
                 'next_action':'Inspect this report/current evidence; do not rerun completed movement'}
    current={k:v for k,v in current.items() if k not in ('inputs','invocation_inputs','code_hashes')}
    current['summary_report']=record['summary_report']
    save(memory/'REHEARSAL_STATE.json',current)
    (memory/'REHEARSAL_CURRENT.md').write_text('# Latest function result (automatic)\n\n'+json.dumps(current,ensure_ascii=False,indent=2)+'\n')
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--function',choices=FUNCTIONS,required=True)
    p.add_argument('--inputs',type=pathlib.Path,required=True)
    p.add_argument('--output',type=pathlib.Path,required=True)
    p.add_argument('--checks',help='Local module:callable for real synchronous perception; optional')
    p.add_argument('--execute',action='store_true')
    a=p.parse_args();args=json.loads(a.inputs.read_text())
    if not a.execute:
        print(json.dumps({'plan_only':True,'function':a.function,'inputs':args,'output':str(a.output)}));return
    assess=None
    if a.checks:
        mod,name=a.checks.split(':',1);assess=getattr(importlib.import_module(mod),name)
    sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]))
    from api.runtime import Context
    with Context() as ctx:
        r=run_one(ctx,a.function,args,a.output,assess=assess)
    print(json.dumps({'report':str(a.output),'wall_s':r['wall_s'],**compact(r['result'])},ensure_ascii=False))


if __name__=='__main__':main()
