"""Isolated multi-waypoint planner. No hardware or automatic dispatch.

Single legs retain a straight joint-space path with zero endpoint velocity and
acceleration. Explicitly selected non-contact knots may use checked quintic
blending. Retiming retains the existing velocity, acceleration and sample limits;
every final wire path receives collision/workspace checks before dispatch.
New scenes must supply newly solved joint targets, never historical replay.
"""
import math
import numpy as np

HZ = 30
VELOCITY = math.radians(27.2)
ACCELERATION = 1.5  # Operator-selected candidate cap, rad/s^2; not physical qualification.
WIRE_ANGLE_EPS_RAD = 1e-6  # float32 wire subtraction, not arrival/tracking allowance
JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]


class TrajectoryHorizonExceeded(ValueError):
    reason_code='TRAJECTORY_HORIZON_EXCEEDED'


def compile_arm_path(anchor14, waypoints14, *, check_state, check_path, dwell_s=.2, velocity=VELOCITY, acceleration=ACCELERATION, blend_indices=(), blend_scale=1.):
    a = np.asarray(anchor14, dtype=float)
    w = np.array(waypoints14, dtype=float, copy=True)
    if a.shape != (14,) or w.ndim != 2 or w.shape[1] != 14 or not 1 <= len(w) <= 16:
        raise ValueError('Expected anchor14 and 1..16 waypoints14')
    if not np.isfinite(a).all() or not np.isfinite(w).all():
        raise ValueError('Nonfinite joint value')
    if not math.isfinite(dwell_s) or not .2 <= dwell_s <= 2:
        raise ValueError('Invalid terminal dwell')
    # No closure/release or right-side correction may be folded into an arm chunk.
    fixed = [6, 7, 8, 9, 10, 11, 12, 13]
    if not np.array_equal(w[:, fixed], np.broadcast_to(a[fixed], w[:, fixed].shape)):
        raise ValueError('Jaws and right side must remain exactly fixed')
    if type(blend_scale) not in (int,float) or not math.isfinite(blend_scale) or not 0<=blend_scale<=1:raise ValueError('Invalid blend scale')
    if blend_indices:
        return compile_blended(a,w,check_state=check_state,check_path=check_path,
            dwell_s=dwell_s,velocity=velocity,acceleration=acceleration,blend_indices=blend_indices,blend_scale=blend_scale)
    check_state(a.tolist())
    trajectory = []
    boundaries = [];old_movement_s=0.
    for goal in w:
        delta = goal - a
        if np.max(np.abs(delta[:6])) > math.radians(120) + WIRE_ANGLE_EPS_RAD:
            raise ValueError('Per-segment joint displacement exceeds 120 degrees')
        check_state(goal.tolist())
        distance = float(np.max(np.abs(delta[JOINTS])))
        seconds = max(1., 1.875*distance/(velocity*.999),
                      math.sqrt((10/math.sqrt(3))*distance/(acceleration*.999)))
        old_count=math.ceil(seconds*HZ);old_movement_s+=old_count/HZ
        # Fixed straight geometry, cosine velocity ramps and optional cruise.
        # Endpoint velocity/acceleration remain zero. Caps and >=1s unchanged.
        ramp=min(velocity*.999*math.pi/(2*acceleration*.999),
                 math.sqrt(distance*math.pi/(2*acceleration*.999)))
        seconds=max(1.,2*ramp,distance/(velocity*.999)+ramp)
        count=math.ceil(seconds*HZ);duration=count/HZ
        v=distance/max(duration-ramp,1e-20)
        t=np.arange(1,count+1,dtype=float)/HZ
        if distance<1e-15:progress=np.ones(count)
        else:
            def rise(t):return v*.5*(t-ramp/math.pi*np.sin(math.pi*t/max(ramp,1e-20)))
            x=np.where(t<ramp,rise(t),np.where(t>duration-ramp,distance-rise(duration-t),v*(t-ramp*.5)))
            progress=x/distance
        if v>velocity or (ramp and v*math.pi/(2*ramp)>acceleration):raise ValueError('Straight profile analytic dynamics exceed bounds')
        profile='cosine_velocity'
        if count>=old_count:
            count=old_count;u=np.arange(1,count+1,dtype=float)/count
            progress=10*u**3-15*u**4+6*u**5;profile='quintic'
        samples=(a+progress[:,None]*delta).astype('<f4')
        trajectory.extend(samples.tolist())
        boundaries.append({'sample_end': len(trajectory), 'duration_s': count/HZ,
                           'velocity_at_waypoint': 0, 'acceleration_at_waypoint': 0,'profile':profile})
        a = samples[-1].astype(float)
        # Keep the same wire-representable invariant state through all segments.
        w[:, fixed] = a[fixed]
    trajectory.extend([trajectory[-1].copy() for _ in range(math.ceil(dwell_s*HZ))])
    if len(trajectory) > 600:
        raise TrajectoryHorizonExceeded('TRAJECTORY_HORIZON_EXCEEDED: Merged path exceeds native600-sample horizon; do not truncate')
    wire = np.asarray(trajectory, dtype='<f4').astype(float)
    ext=np.vstack([anchor14,anchor14,wire,wire[-1]])[:,:6]
    wire_v=float(np.max(abs(np.diff(ext,axis=0)))*HZ)
    wire_a=float(np.max(abs(np.diff(ext,n=2,axis=0)))*HZ**2)
    if wire_v>velocity+1e-5 or wire_a>acceleration+1e-4:raise ValueError('Straight wire dynamics exceed original caps')
    for q in wire:
        check_state(q.tolist())
    # Check the actual generated wire path, including initial connector and joints
    # between segments, rather than just comparing its two endpoints.
    reviewed = check_path([list(anchor14), *wire.tolist()])
    if reviewed.get('cad_clear') is not True or reviewed.get('workspace_clear') is not True:
        raise ValueError('Whole wire path lacks collision/workspace clearance')
    return {'schema': 'rax.experimental.multi_waypoint.v1', 'frequency_hz': HZ,
            'anchor14': list(anchor14), 'target14': wire[-1].tolist(), 'targets14': wire.tolist(),
            'waypoints': boundaries, 'duration_s': len(wire)/HZ,
            'terminal_dwell_s': math.ceil(dwell_s*HZ)/HZ,
            'review': reviewed, 'retiming': {'profile':'bounded_faster_of_quintic_and_cosine','initial_movement_s':old_movement_s,'movement_s':sum(x['duration_s'] for x in boundaries),'minimum_leg_s':1.},
            'dynamics':{'wire_peak_velocity_rad_s':wire_v,'wire_peak_acceleration_rad_s2':wire_a}, 'hardware_execution_enabled': False,
            'validation_scope': 'offline_only; native owner integration required'}


def quintic_extrema(coeff, duration):
    """Exact joint extrema for quintics with zero acceleration at both knots."""
    peak_v=peak_a=0.
    for j in range(6):
        _,v0,_,a,b,c=map(float,coeff[:,j])
        def velocity(u):
            return v0+u*u*(3*a+u*(4*b+u*5*c))
        def acceleration(u):
            return u*(6*a+u*(12*b+u*20*c))
        peak_v=max(peak_v,abs(v0),abs(velocity(1.)))
        # Acceleration factors as u*(1-u)*(6a-20c*u).
        if c:
            u=(.3*a)/c
            if 0.<u<1.:peak_v=max(peak_v,abs(velocity(u)))
        # Jerk is quadratic. The q-form avoids cancellation near a knot.
        A,B,C=60*c,24*b,6*a
        if A:
            disc=B*B-4*A*C
            if disc<=0.:continue
            q=-.5*(B+math.copysign(math.sqrt(disc),B))
            roots=(q/A,C/q)
        else:
            roots=(-C/B,) if B else ()
        for u in roots:
            if 0.<u<1.:peak_a=max(peak_a,abs(acceleration(u)))
    return peak_v/duration,peak_a/duration**2


def compile_blended(anchor,waypoints,*,check_state,check_path,dwell_s,velocity,acceleration,blend_indices,blend_scale=1.):
    """C2 quintic interpolation at explicitly non-contact knots only.

    Exact waypoint positions, limited harmonic-mean velocities, zero acceleration
    at knots. Every changed curve is rechecked; no spline through contact boundaries.
    """
    indices=list(blend_indices);n=len(waypoints)
    if any(type(i) is not int or not 0<=i<n-1 for i in indices) or len(set(indices))!=len(indices):
        raise ValueError('Blend indices must identify unique interior waypoint endpoints')
    knots=np.vstack([anchor,waypoints]);d=np.diff(knots,axis=0)
    if np.max(abs(d[:,:6]))>math.radians(120)+WIRE_ANGLE_EPS_RAD:raise ValueError('Segment exceeds120deg')
    if not 0<velocity<=VELOCITY+1e-10 or not 0<acceleration<=ACCELERATION+1e-10:raise ValueError('Invalid dynamics')
    if blend_scale==0.:
        # One native request, no interior dwell; only the waypoint velocity
        # becomes zero. The outer transport still checks the entire wire.
        plan=compile_arm_path(anchor,waypoints,check_state=check_state,check_path=check_path,
                              dwell_s=dwell_s,velocity=velocity,acceleration=acceleration)
        plan['blend_indices']=indices
        return plan
    seconds=np.array([max(1.,1.875*np.max(abs(x[:6]))/(velocity*.999),
        math.sqrt((10/math.sqrt(3))*np.max(abs(x[:6]))/(acceleration*.999))) for x in d])
    counts=np.ceil(seconds*HZ).astype(int)
    tangent_scales=np.full(n+1,.6*blend_scale)
    # Scale the geometric envelope too: saturated knots otherwise produce the
    # same wire at full and reduced speed. Zero tangents trace straight legs.
    departure_limit=.01*blend_scale
    departure_eps=1e-12
    def build(counts,tangent_scales):
        ts=counts/HZ;v=np.zeros_like(knots)
        for i in indices:
            l=d[i,:6]/ts[i];r=d[i+1,:6]/ts[i+1]
            same=l*r>0;v[i+1,:6][same]=tangent_scales[i+1]*(2*l[same]*r[same]/(l[same]+r[same]))
        wires=[];boundaries=[];peak_v=peak_a=max_deviation=0.;leg_deviations=[];leg_peaks=[]
        for i,T in enumerate(ts):
            delta=d[i];v0=v[i]*T;v1=v[i+1]*T
            c=np.array([knots[i],v0,np.zeros(14),10*delta-6*v0-4*v1,-15*delta+8*v0+7*v1,6*delta-3*v0-3*v1])
            local_v,local_a=quintic_extrema(c,T)
            peak_v=max(peak_v,local_v);peak_a=max(peak_a,local_a);leg_peaks.append((local_v,local_a))
            u=np.arange(1,counts[i]+1)/counts[i]
            q=sum(c[k][None,:]*u[:,None]**k for k in range(6))
            # Bound departure from the old joint-line leg; certify changed path below.
            leg=d[i,:6];alpha=np.clip((q[:,:6]-knots[i,:6])@leg/max(float(leg@leg),1e-20),0,1)
            deviation=float(np.max(abs(q[:,:6]-(knots[i,:6]+alpha[:,None]*leg))))
            leg_deviations.append(deviation);max_deviation=max(max_deviation,deviation)
            q[:,6:]=anchor[6:];wires.extend(q.astype('<f4').tolist())
            boundaries.append(dict(sample_end=len(wires),duration_s=float(T),velocity_at_waypoint=v[i+1,:6].tolist(),acceleration_at_waypoint=0,blended=bool(np.any(v[i+1,:6]))))
        return wires,boundaries,peak_v,peak_a,max_deviation,leg_deviations,leg_peaks
    for attempt in range(15):
        wires,boundaries,peak_v,peak_a,max_deviation,leg_deviations,leg_peaks=build(counts,tangent_scales)
        if max_deviation>departure_limit+departure_eps:
            factors=np.ones(n+1)
            for i,deviation in enumerate(leg_deviations):
                if deviation>departure_limit+departure_eps:
                    factors[i]=min(factors[i],.95*departure_limit/deviation)
                    factors[i+1]=min(factors[i+1],.95*departure_limit/deviation)
            tangent_scales*=factors
            continue
        if peak_v<=velocity*.999 and peak_a<=acceleration*.999:break
        factor=max(peak_v/(velocity*.999),math.sqrt(peak_a/(acceleration*.999)))
        counts=np.ceil(counts*factor*1.002).astype(int)
    else:raise ValueError('Spline dynamics did not converge')
    if max_deviation>departure_limit+departure_eps:raise ValueError('Spline departure exceeds0.01rad; no silent fallback')
    # A stop-to-stop duration is only an initial upper bound when interior
    # velocities are nonzero. Try at most two shorter durations per leg,
    # longest first; recompute ALL coupled knots/analytic extrema each time.
    # Keep >=1s per leg, original v/a/departure bounds, exact endpoints, and
    # final float32 dynamics + full geometry validation below.
    original_counts=counts.copy();retime_attempts=0;retime_accepted=0
    def within_limits(candidate):
        if candidate[2]>velocity*.999 or candidate[3]>acceleration*.999 or candidate[4]>departure_limit+departure_eps:
            return False
        wire=np.asarray(candidate[0],dtype='<f4').astype(float)
        ext=np.vstack([anchor,anchor,wire,wire[-1]])[:,:6]
        return (np.max(abs(np.diff(ext,axis=0)))*HZ<=velocity+1e-5
                and np.max(abs(np.diff(ext,n=2,axis=0)))*HZ**2<=acceleration+1e-4)
    for i in np.argsort(-counts):
        if counts[i]<=30:continue
        local_v,local_a=leg_peaks[i]
        ratio=max(local_v/(velocity*.999),math.sqrt(local_a/(acceleration*.999)))
        proposed=max(30,int(math.ceil(counts[i]*max(.75,min(.995,ratio*1.01)))))
        if proposed>=counts[i]:continue
        for wanted in dict.fromkeys((proposed,int(math.ceil((proposed+counts[i])/2)))):
            if wanted>=counts[i]:continue
            trial=counts.copy();trial[i]=wanted;retime_attempts+=1
            candidate=build(trial,tangent_scales)
            if not within_limits(candidate):continue
            counts=trial;wires,boundaries,peak_v,peak_a,max_deviation,leg_deviations,leg_peaks=candidate
            retime_accepted+=1;break
    # Fine retiming after the original pass cannot lengthen any segment.
    # One 30 Hz frame at a time, only when there is analytic headroom.
    for i in np.argsort(-counts):
        if counts[i]<=HZ:continue
        local_v,local_a=leg_peaks[i]
        ratio=max(local_v/(velocity*.999),math.sqrt(local_a/(acceleration*.999)))
        wanted=max(HZ,int(math.ceil(counts[i]*ratio*1.0001)))
        if wanted>=counts[i]:continue
        trial=counts.copy();trial[i]=max(wanted,counts[i]-1)
        candidate=build(trial,tangent_scales);retime_attempts+=1
        if not within_limits(candidate):continue
        counts=trial;wires,boundaries,peak_v,peak_a,max_deviation,leg_deviations,leg_peaks=candidate
        retime_accepted+=1
    retiming=dict(initial_movement_s=float(sum(original_counts)/HZ),movement_s=float(sum(counts)/HZ),
                  attempts=retime_attempts,accepted_legs=retime_accepted,minimum_leg_s=1.)
    wires.extend([wires[-1].copy() for _ in range(math.ceil(dwell_s*HZ))])
    if len(wires)>600:raise TrajectoryHorizonExceeded('TRAJECTORY_HORIZON_EXCEEDED: Merged path exceeds native600-sample horizon')
    wire=np.asarray(wires,dtype='<f4').astype(float)
    # Include actual discrete wire dynamics and rest at both ends, not just ideal polynomial.
    ext=np.vstack([anchor,anchor,wire,wire[-1]])
    actual_v=float(np.max(abs(np.diff(ext[:,:6],axis=0)))*HZ)
    actual_a=float(np.max(abs(np.diff(ext[:,:6],n=2,axis=0)))*HZ**2)
    if actual_v>velocity+1e-5 or actual_a>acceleration+1e-4:raise ValueError('Wire dynamics exceed bounds')
    for q in [anchor,*wire]:check_state(q.tolist())
    review=check_path([anchor.tolist(),*wire.tolist()])
    if review.get('cad_clear') is not True or review.get('workspace_clear') is not True:raise ValueError('Blended wire path rejected')
    return dict(schema='rax.experimental.multi_waypoint.v1',frequency_hz=HZ,anchor14=anchor.tolist(),target14=wire[-1].tolist(),targets14=wire.tolist(),waypoints=boundaries,duration_s=len(wire)/HZ,terminal_dwell_s=math.ceil(dwell_s*HZ)/HZ,review=review,hardware_execution_enabled=False,validation_scope='offline_only; native owner integration required',blend_indices=indices,retiming=retiming,dynamics=dict(analytic_peak_velocity_rad_s=peak_v,analytic_peak_acceleration_rad_s2=peak_a,wire_peak_velocity_rad_s=actual_v,wire_peak_acceleration_rad_s2=actual_a,tangent_scale=float(min(tangent_scales)),tangent_scales=tangent_scales.tolist(),max_joint_line_departure_rad=max_deviation))
