"""Pure, bounded correction of an unexecuted loaded leg. No dispatch or retries."""
import time,copy
import numpy as np
from assembly_plan import NoPlan


def bind_pickup(execution,actions,start,*,prefer_preview=False):
    """Rebuild the unexecuted departure column at the actual grasp FK.

    End heights and later placement targets stay unchanged. This is a pure
    bounded replan, not extra motion or a relaxed collision corridor.
    """
    from scene_checks import same_state
    e=execution;first=actions[0]
    if not first.get('pickup'):return actions,{'rebound':False}
    reference=list(first['start14']);reference[6:]=start[6:]
    if not same_state(start,reference):raise NoPlan('Pickup start outside approved joint binding')
    if not e.payload or e.held!=first['color']:raise NoPlan('Pickup requires measured held-object relation')
    actual=np.asarray(e.planner.fk(start)['position']);cur=list(start);result=[];departing=True
    if prefer_preview:
        planned=np.asarray(e.planner.fk(first['start14'])['position'])
        if np.linalg.norm(actual-planned)<=.001:
            # The caller MUST check the actual compiled curve with the current
            # payload before dispatch. A refusal falls back to normal rebind.
            rows=copy.deepcopy(actions);cur=list(start)
            for row in rows:
                row['start14']=cur;row['target14'][6:]=start[6:];cur=list(row['target14'])
            return rows,{'rebound':False,'reused_candidate_for_actual_wire_check':True,
                         'grasp_position_delta_m':float(np.linalg.norm(actual-planned)),
                         'actual_wire_check_required':True,'planned_heights_preserved':True}

    for action in actions:
        if action.get('pickup') and departing:
            goal=e.planner.fk(action['target14']);position=list(goal['position']);position[:2]=actual[:2].tolist()
            if position[2]<=e.planner.fk(cur)['position'][2]:raise NoPlan('Planned clearance no longer above measured grasp')
            parts=e.planner.route(cur,position,goal['quaternion_xyzw'],e.scene,
                active=first['color'],payload=e.payload,pickup=True,support=action.get('support'))
            for part in parts:
                result.append({**copy.deepcopy(action),**part,'pickup_bound':True});cur=list(part['target14'])
        else:
            departing=False;row=copy.deepcopy(action);row['start14']=cur.copy()
            row['target14'][6:]=start[6:];result.append(row);cur=row['target14']
    return result,{'rebound':True,'measured_grasp_position_m':actual.tolist(),
                   'original_start14':first['start14'],'measured_start14':start,
                   'departure_segments':sum(bool(a.get('pickup')) for a in result),
                   'collision_margins_unchanged':True,'planned_heights_preserved':True}

def corrected_leg(planner,start,target,scene,*,active,payload):
    if not payload or payload.get('color')!=active:raise NoPlan('Measured payload relation required')
    begin=planner.fk(start);end=planner.fk(target);p=np.asarray(begin['position']);dst=np.asarray(end['position']);q=end['quaternion_xyzw'];began=time.monotonic();failures=[]
    try:
        parts=planner.route(start,dst.tolist(),q,scene,active=active,payload=payload)
        return {'parts':parts,'strategy':'checked_general_route','rejoins_original_endpoint':True,'elapsed_s':time.monotonic()-began,'failed_candidates':[]}
    except NoPlan as exc:failures.append({'strategy':'checked_general_route','why':str(exc)})
    for rise in (.005,.008,.012,.020):
      for divisions in (1,2,4):
        if time.monotonic()-began>15:break
        try:
            a=list(start);parts=[]
            for position,reserve in ((p+[0,0,rise],0.),(dst+[0,0,rise],.003),(dst,0.)):
                origin=np.asarray(planner.fk(a)['position'])
                for u in np.linspace(0,1,divisions+1)[1:]:
                    point=origin+(position-origin)*u
                    b=list(target) if u==1 and np.array_equal(position,dst) else planner.solve(point.tolist(),q,a)
                    b[6:]=a[6:]
                    for part in planner.joint_route(a,b,scene,active=active,payload=payload,reserve_m=reserve):
                        parts.append(part|{'reserve_m':reserve})
                    a=b
            return {'parts':parts,'rise_m':rise,'divisions':divisions,'rejoins_original_endpoint':True,'elapsed_s':time.monotonic()-began,'failed_candidates':failures}
        except NoPlan as exc:failures.append({'rise_m':rise,'divisions':divisions,'why':str(exc)})
    raise NoPlan('No checked correction for measured payload; '+str(failures))
