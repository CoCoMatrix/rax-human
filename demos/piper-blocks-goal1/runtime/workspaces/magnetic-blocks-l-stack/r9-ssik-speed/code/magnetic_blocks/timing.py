"""Report actual recorded clocks; missing SDK timing is unknown, never zero."""
import math

def motion_timing(row):
    r=row.get('result') or {};events={e.get('stage'):e.get('monotonic_ns') for e in r.get('handoff_timing',[])}
    def span(a,b):
        x,y=events.get(a),events.get(b)
        return (y-x)/1e9 if type(x) is int and type(y) is int and y>=x else None
    ex=r.get('execution_timing',{})
    def exspan(a,b):
        x,y=ex.get(a),ex.get(b)
        return (y-x)/1e9 if type(x) is int and type(y) is int and y>=x else None
    released=ex.get('policy_release_monotonic_ns');staged=events.get('staged_wait_policy_poll')
    return {'request_id':r.get('owner_request_id'),'rpc_s':row.get('elapsed_s'),
        'first_receipt_monotonic_ns':ex.get('first_receipt_seen_monotonic_ns'),
        'last_receipt_monotonic_ns':ex.get('last_receipt_seen_monotonic_ns'),
        'native_preflight_s':span('path_preview_begin','path_preview_end'),
        'previous_action_ack_s':span('owner_initial_state_checked','owner_previous_action_acknowledged'),
        'fresh_feedback_wait_s':span('plan_written_wait_fresh_observation','fresh_observation_ready'),
        'staged_to_policy_release_s':(released-staged)/1e9 if type(released) is int and type(staged) is int and released>=staged else None,
        'policy_release_to_first_receipt_s':exspan('policy_release_monotonic_ns','first_receipt_seen_monotonic_ns'),
        'sdk_receipt_stream_s':exspan('first_receipt_seen_monotonic_ns','last_receipt_seen_monotonic_ns'),
        'last_receipt_to_settled_s':exspan('last_receipt_seen_monotonic_ns','settled_monotonic_ns'),
        'settling_window_s':r.get('settling',{}).get('window_s'),
        'timing_scope':'SDK receipt-observation interval includes journal/polling delay; not motor-only travel. Settling window overlaps last-receipt-to-settled. Null means unavailable.'}

def breakdown(rows,wall):
    motions=('piper_move_pose','piper_move_joints','piper_gripper','piper_gripper_stroke','piper_move_waypoints')
    return {'explicit_fk_s':sum(r['elapsed_s'] for r in rows if r['tool'] in ('piper_fk','piper_fk_batch')),
        'explicit_curve_preview_s':sum(r['elapsed_s'] for r in rows if r['tool']=='piper_preview_waypoints'),
        'explicit_preflight_tool_s':sum(r['elapsed_s'] for r in rows if r['tool'] in ('piper_solve_ik','piper_solve_ik_fast','piper_solve_ik_candidates','piper_check_path')),
        'explicit_observation_s':sum(r['elapsed_s'] for r in rows if r['tool']=='piper_observe'),
        'model_s':sum(r['elapsed_s'] for r in rows if r['tool']=='gemini_generate'),
        'python_and_logging_s':max(0.,wall-sum(r['elapsed_s'] for r in rows)),
        'motions':[motion_timing(r) for r in rows if r['tool'] in motions]}


def inter_motion_gaps(steps):
    motions=[(s['label'],m) for s in steps for m in s.get('timing',{}).get('motions',[])]
    rows=[]
    for (a,x),(b,y) in zip(motions,motions[1:]):
        start=x.get('last_receipt_monotonic_ns');end=y.get('first_receipt_monotonic_ns')
        rows.append({'from_stage':a,'to_stage':b,
            'last_to_first_sdk_receipt_s':(end-start)/1e9 if type(start) is int and type(end) is int and end>=start else None,
            'scope':'Includes final settling, result checks, planning and next dispatch; not pure Python function overhead'})
    return rows
