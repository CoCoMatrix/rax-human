"""Isolated multi-waypoint planner. No hardware or automatic dispatch.

Each segment retains the existing straight joint-space path and zero-velocity,
zero-acceleration waypoint boundary. Only intermediate dwell/handshakes disappear.
New scenes must supply newly solved joint targets, never historical replay.
"""
import math
import numpy as np

HZ = 30
VELOCITY = math.radians(17)
ACCELERATION = math.radians(40)
JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]


def compile_arm_path(anchor14, waypoints14, *, check_state, check_path, dwell_s=.2, velocity=VELOCITY, acceleration=ACCELERATION, blend_indices=()):
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
    if blend_indices:
        return compile_blended(a,w,check_state=check_state,check_path=check_path,
            dwell_s=dwell_s,velocity=velocity,acceleration=acceleration,blend_indices=blend_indices)
    check_state(a.tolist())
    trajectory = []
    boundaries = []
    for goal in w:
        delta = goal - a
        if np.max(np.abs(delta[:6])) > math.radians(60) + 1e-9:
            raise ValueError('Per-segment joint displacement exceeds 60 degrees')
        check_state(goal.tolist())
        distance = float(np.max(np.abs(delta[JOINTS])))
        seconds = max(1., 1.875*distance/(velocity*.999),
                      math.sqrt((10/math.sqrt(3))*distance/(acceleration*.999)))
        count = math.ceil(seconds * HZ)
        u = np.arange(1, count+1, dtype=float)/count
        samples = (a + (10*u**3 - 15*u**4 + 6*u**5)[:, None]*delta).astype('<f4')
        trajectory.extend(samples.tolist())
        boundaries.append({'sample_end': len(trajectory), 'duration_s': count/HZ,
                           'velocity_at_waypoint': 0, 'acceleration_at_waypoint': 0})
        a = samples[-1].astype(float)
        # Keep the same wire-representable invariant state through all segments.
        w[:, fixed] = a[fixed]
    trajectory.extend([trajectory[-1].copy() for _ in range(math.ceil(dwell_s*HZ))])
    if len(trajectory) > 600:
        raise ValueError('Merged path exceeds native 600-sample horizon; do not truncate')
    wire = np.asarray(trajectory, dtype='<f4').astype(float)
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
            'review': reviewed, 'hardware_execution_enabled': False,
            'validation_scope': 'offline_only; native owner integration required'}


def compile_blended(anchor,waypoints,*,check_state,check_path,dwell_s,velocity,acceleration,blend_indices):
    """C2 quintic interpolation at explicitly non-contact knots only.

    Exact waypoint positions, limited harmonic-mean velocities, zero acceleration
    at knots. Every changed curve is rechecked; no spline through contact boundaries.
    """
    indices=list(blend_indices);n=len(waypoints)
    if any(type(i) is not int or not 0<=i<n-1 for i in indices) or len(set(indices))!=len(indices):
        raise ValueError('Blend indices must identify unique interior waypoint endpoints')
    knots=np.vstack([anchor,waypoints]);d=np.diff(knots,axis=0)
    if np.max(abs(d[:,:6]))>math.radians(60)+1e-9:raise ValueError('Segment exceeds60deg')
    if not 0<velocity<=VELOCITY+1e-10 or not 0<acceleration<=ACCELERATION+1e-10:raise ValueError('Invalid dynamics')
    seconds=np.array([max(1.,1.875*np.max(abs(x[:6]))/(velocity*.999),
        math.sqrt((10/math.sqrt(3))*np.max(abs(x[:6]))/(acceleration*.999))) for x in d])
    counts=np.ceil(seconds*HZ).astype(int)
    tangent_scale=.3
    for attempt in range(15):
        ts=counts/HZ;v=np.zeros_like(knots)
        for i in indices:
            l=d[i,:6]/ts[i];r=d[i+1,:6]/ts[i+1]
            same=l*r>0;v[i+1,:6][same]=tangent_scale*(2*l[same]*r[same]/(l[same]+r[same]))
        wires=[];boundaries=[];peak_v=peak_a=max_deviation=0.
        for i,T in enumerate(ts):
            delta=d[i];v0=v[i]*T;v1=v[i+1]*T
            c=np.array([knots[i],v0,np.zeros(14),10*delta-6*v0-4*v1,-15*delta+8*v0+7*v1,6*delta-3*v0-3*v1])
            # Exact extrema of velocity and acceleration on the polynomial legs.
            for j in range(6):
                vc=np.array([c[k,j]*k for k in range(1,6)])
                ac=np.array([vc[k]*k for k in range(1,5)])
                jc=np.array([ac[k]*k for k in range(1,4)])
                roots=lambda poly:[float(x.real) for x in np.polynomial.polynomial.polyroots(poly) if abs(x.imag)<1e-8 and 0<x.real<1]
                peak_v=max(peak_v,max(abs(np.polynomial.polynomial.polyval(u,vc))/T for u in [0,1,*roots(ac)]))
                peak_a=max(peak_a,max(abs(np.polynomial.polynomial.polyval(u,ac))/T**2 for u in [0,1,*roots(jc)]))
            u=np.arange(1,counts[i]+1)/counts[i]
            q=sum(c[k][None,:]*u[:,None]**k for k in range(6))
            # Bound departure from the old joint-line leg; certify changed path below.
            leg=d[i,:6];alpha=np.clip((q[:,:6]-knots[i,:6])@leg/max(float(leg@leg),1e-20),0,1)
            max_deviation=max(max_deviation,float(np.max(abs(q[:,:6]-(knots[i,:6]+alpha[:,None]*leg)))))
            q[:,6:]=anchor[6:];wires.extend(q.astype('<f4').tolist())
            boundaries.append(dict(sample_end=len(wires),duration_s=float(T),velocity_at_waypoint=v[i+1,:6].tolist(),acceleration_at_waypoint=0,blended=bool(np.any(v[i+1,:6]))))
        if max_deviation>.01:
            tangent_scale*=.0095/max_deviation
            continue
        if peak_v<=velocity*.999 and peak_a<=acceleration*.999:break
        factor=max(peak_v/(velocity*.999),math.sqrt(peak_a/(acceleration*.999)))
        counts=np.ceil(counts*factor*1.002).astype(int)
    else:raise ValueError('Spline dynamics did not converge')
    if max_deviation>.01:raise ValueError('Spline departure exceeds0.01rad; no silent fallback')
    wires.extend([wires[-1].copy() for _ in range(math.ceil(dwell_s*HZ))])
    if len(wires)>600:raise ValueError('Merged path exceeds native600-sample horizon')
    wire=np.asarray(wires,dtype='<f4').astype(float)
    # Include actual discrete wire dynamics and rest at both ends, not just ideal polynomial.
    ext=np.vstack([anchor,anchor,wire,wire[-1]])
    actual_v=float(np.max(abs(np.diff(ext[:,:6],axis=0)))*HZ)
    actual_a=float(np.max(abs(np.diff(ext[:,:6],n=2,axis=0)))*HZ**2)
    if actual_v>velocity+1e-5 or actual_a>acceleration+1e-4:raise ValueError('Wire dynamics exceed bounds')
    for q in [anchor,*wire]:check_state(q.tolist())
    review=check_path([anchor.tolist(),*wire.tolist()])
    if review.get('cad_clear') is not True or review.get('workspace_clear') is not True:raise ValueError('Blended wire path rejected')
    return dict(schema='rax.experimental.multi_waypoint.v1',frequency_hz=HZ,anchor14=anchor.tolist(),target14=wire[-1].tolist(),targets14=wire.tolist(),waypoints=boundaries,duration_s=len(wire)/HZ,terminal_dwell_s=math.ceil(dwell_s*HZ)/HZ,review=review,hardware_execution_enabled=False,validation_scope='offline_only; native owner integration required',blend_indices=indices,dynamics=dict(analytic_peak_velocity_rad_s=peak_v,analytic_peak_acceleration_rad_s2=peak_a,wire_peak_velocity_rad_s=actual_v,wire_peak_acceleration_rad_s2=actual_a,tangent_scale=tangent_scale,max_joint_line_departure_rad=max_deviation))
