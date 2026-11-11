"""Offline-only Piper adapter. No SDK, socket, CAN or camera transport exists here."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from yourdfpy import URDF

ASSETS = Path(__file__).resolve().parents[2]/'assets/piper-32cm'
URDF_SHA = '12e4e447bffcbbfebca677fb96fd6062d122b1c2987da65187aaa3caa1bb330d'
ORDER = tuple(n for side in ('left','right') for n in [*(f'{side}_joint{i}' for i in range(1,7)),f'{side}_gripper'])
TOOLS = ('piper_status','piper_fk','piper_check','piper_preview_joints','piper_observe','piper_move_joints','piper_hold')
VECTOR = {'type':'array','items':{'type':'number'},'minItems':14,'maxItems':14}

def tool(name, description, properties=None, required=()):
    return {'name':name,'description':description,'inputSchema':{'type':'object','properties':properties or {},'required':list(required),'additionalProperties':False}}

class PiperNoSendBackend:
    mutating = frozenset()  # This backend has no hardware operation, including hold.

    def __init__(self, config):
        if config.backend_options:
            raise ValueError('NO_SEND accepts no transport or hardware options')
        path=ASSETS/'dual_piper_x_description/urdf/dual_piper_x.urdf'
        if hashlib.sha256(path.read_bytes()).hexdigest()!=URDF_SHA:
            raise ValueError('Pinned Piper URDF SHA mismatch')
        self.model=URDF.load(str(path),load_meshes=False,build_scene_graph=True)
        tcp_raw=(ASSETS/'nominal_tcp.json').read_bytes()
        if hashlib.sha256(tcp_raw).hexdigest()!='95a428c5f295a305daf97e692485094a2622d3ed4479ed638a00c898940ae98e':
            raise ValueError('Pinned nominal TCP SHA mismatch')
        self.tcp=json.loads(tcp_raw)
        self.limits=np.array([[self.model.joint_map[n].limit.lower,self.model.joint_map[n].limit.upper] for n in ORDER])
        self.closed=False

    def tools(self):
        return [tool('piper_status','NO_SEND capabilities, units and geometry provenance. No live feedback.'),
                tool('piper_fk','Offline FK in dual_base_link from absolute radians / full jaw width metres.',{'joints':VECTOR},['joints']),
                tool('piper_check','Check model joint limits only. Does NOT certify collisions or hardware safety.',{'joints':VECTOR},['joints']),
                tool('piper_preview_joints','Generate a 30 Hz quintic preview only, without executing or changing measured state.',{'start':VECTOR,'target':VECTOR,'duration_s':{'type':'number','minimum':.1,'maximum':30}},['start','target','duration_s']),
                tool('piper_observe','Unavailable in this stage: no cameras are opened and no synthetic observation substitutes for live RGB.'),
                tool('piper_move_joints','Always refused: hardware dispatch is absent in NO_SEND.',{'joints':VECTOR},['joints']),
                tool('piper_hold','Offline acknowledgement only; cannot stop an independently running physical robot.')]

    def _vector(self, q):
        if not isinstance(q,(list,tuple)) or len(q)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in q):
            raise ValueError('Expected 14 finite real numbers: left 6 rad + jaw metres, right 6 rad + jaw metres')
        a=np.asarray(q,dtype=float)
        if np.any(a<self.limits[:,0]) or np.any(a>self.limits[:,1]):
            raise ValueError('Outside pinned URDF limits; joints use radians, jaws full width metres [0,0.1]')
        return a

    def validate(self,name,args):
        if self.closed: raise ValueError('Backend is closed')
        schemas={t['name']:t['inputSchema'] for t in self.tools()}
        if name not in schemas: raise ValueError('Unknown Piper tool')
        schema=schemas[name]
        if not isinstance(args,dict) or set(args)-set(schema['properties']) or set(schema['required'])-set(args):
            raise ValueError('Unexpected or missing arguments')
        if name=='piper_move_joints': raise ValueError('NO_SEND: hardware dispatch is not implemented, even when operator-armed')
        if name=='piper_observe': raise ValueError('NO_SEND: live observation is unavailable; no cameras were opened')
        if 'joints' in args: self._vector(args['joints'])
        if name=='piper_preview_joints':
            a,b=self._vector(args['start']),self._vector(args['target'])
            d=args['duration_s']
            if type(d) not in (int,float) or not math.isfinite(d) or not .1<=d<=30: raise ValueError('duration_s must be finite 0.1..30')
            # Analytical quintic maxima; conservative offline preview limits, NOT measured owner limits.
            vmax=np.array([.2]*6+[.02]+[.2]*6+[.02])
            amax=np.array([.5]*6+[.05]+[.5]*6+[.05])
            if np.any(1.875*abs(b-a)/d>vmax+1e-12) or np.any((10/np.sqrt(3))*abs(b-a)/d**2>amax+1e-12):
                raise ValueError('Preview exceeds conservative velocity/acceleration envelope; increase duration')

    def status(self):
        return {'robot':'piper_nosend','mode':'NO_SEND','hardware_writes':0,'measured_state':None,
                'joint_order':ORDER,'joint_units':'rad','gripper_units':'m, full inner jaw gap',
                'limits':self.limits.tolist(),'urdf_sha256':URDF_SHA,'base_frame':'dual_base_link',
                'physical_calibration_validated':False,'collision_checked':False,
                'joint6_discrepancy':'URDF +/-120 deg vs owner profile +/-180 deg; use URDF intersection offline',
                'closed':self.closed}

    def call(self,name,args):
        self.validate(name,args)
        if name=='piper_status': return self.status(),[]
        if name=='piper_hold': return {'executed':False,'hardware_stop_sent':False,'hardware_writes':0},[]
        if name=='piper_check': return {'joint_limits_pass':True,'collision_checked':False,'hardware_execution_approved':False},[]
        if name=='piper_fk':
            q=self._vector(args['joints']); self.model.update_cfg(dict(zip(ORDER,q)))
            value={'base_frame':'dual_base_link','physical_calibration_validated':False,'hardware_writes':0}
            for side in ('left','right'):
                T=self.model.get_transform(f'{side}_link6')
                value[f'{side}_link6']=T.tolist()
                value[f'{side}_nominal_tcp']=(T@np.asarray(self.tcp[side]['T_parent_tcp'])).tolist()
            return value,[]
        if name=='piper_preview_joints':
            a,b=self._vector(args['start']),self._vector(args['target']); d=float(args['duration_s'])
            # Nominal 30 Hz, with final endpoint at exact requested duration.
            t=np.r_[np.arange(0,d,1/30),d]; u=t/d; s=10*u**3-15*u**4+6*u**5
            return {'executed':False,'hardware_writes':0,'collision_checked':False,'state_source':'operator supplied offline vectors',
                    'time_s':t.tolist(),'samples':(a+s[:,None]*(b-a)).tolist()},[]
        raise ValueError('Unknown Piper tool')

    def close(self): self.closed=True
