"""Franka-shaped RAX adapter for the existing dual Piper installation.

The adapter does not start/enable a hardware owner. Read-only by default. Live
moves use the independently running XPolicy sole-owner bridge, preserving its
path checks, timing, feedback and receipt validation. No copied task FSM.
"""
import base64
import hashlib
from pathlib import Path
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from services.controller.errors import public_failure
from services import arming
from .owner import OwnerReader, ORDER, SERIALS
from .geometry import Kinematics, vector, URDF_SHA
from .cameras import Cameras, CANDIDATE_SHA
from .tool_calibration import load_calibration, tool_pose
from .moveit import MoveIt, IKNoSolution
from .bridge import OwnerBridge, RejectedBeforeDispatch
from .installed_settling import JOINT_ARRIVAL_TOLERANCE_RAD, OBSERVATION_JOINT_TOLERANCE_RAD, OBSERVATION_JAW_TOLERANCE_M, JOINTS, state_matches

V3 = {'type':'array','items':{'type':'number'},'minItems':3,'maxItems':3}
V6 = {'type':'array','items':{'type':'number'},'minItems':6,'maxItems':6}
QUAT = {'type':'array','items':{'type':'number'},'minItems':4,'maxItems':4}


def tool(name, description, properties=None, required=()):
    return {'name':name,'description':description,'inputSchema':{'type':'object','properties':properties or {},
            'required':list(required),'additionalProperties':False}}


class PostMotionStateError(RuntimeError):
    """A completed command has unsafe/unknown current state, not a camera error."""
    control_uncertain=True


class PiperBackend:
    mutating = frozenset({'piper_move_pose','piper_move_joints','piper_gripper'})

    def __init__(self, config, *, owner=None, cameras=None, solver=None, bridge=None):
        self.config = config; self.closed = False; self.last_motion = None; self.post_motion_fault = None
        opts = config.backend_options
        allowed = {'address','depth_root','left_depth_root','moveit_manifest','motion_enabled','bridge_root','owner_session_root',
                   'agent_image_width','agent_python','protected_paths','workspace_min','workspace_max',
                   'collision_legacy_root','stream_enabled','stream_rgb_hz','stream_robot_hz','stream_depth_hz','stream_window_s'}
        if set(opts) - allowed: raise ValueError('Unknown Piper options: ' + str(sorted(set(opts)-allowed)))
        self.motion_enabled = opts.get('motion_enabled',False)
        if type(self.motion_enabled) is not bool: raise ValueError('motion_enabled must be boolean')
        self.box_min = self.box_max = None
        if 'workspace_min' in opts or 'workspace_max' in opts:
            self.box_min = vector(opts.get('workspace_min'),3)
            self.box_max = vector(opts.get('workspace_max'),3)
            if not np.all(self.box_min < self.box_max):raise ValueError('workspace_min must be below workspace_max')
        if self.motion_enabled and self.box_min is None:
            raise ValueError('Live mode requires operator-defined flange workspace_min/workspace_max')
        self.width = opts.get('agent_image_width',640)
        if type(self.width) is not int or not 320 <= self.width <= 1280: raise ValueError('agent_image_width 320..1280 required')
        self.owner = owner or OwnerReader(opts.get('address','127.0.0.1:8091'))
        self.kin = Kinematics()
        self.cameras = cameras or Cameras(self.owner,Path(config.root)/'agent-workspace/runtime/frames',opts.get('depth_root'),opts.get('left_depth_root'))
        self.solver = solver or MoveIt(Path(config.root)/'moveit',opts.get('moveit_manifest'))
        self.bridge = bridge
        if self.motion_enabled and self.bridge is None:
            if not opts.get('bridge_root') or not opts.get('owner_session_root'):
                raise ValueError('Live mode requires an existing bridge_root and owner_session_root')
            self.bridge = OwnerBridge(opts['bridge_root'],opts['owner_session_root'],config.root)
        self.collision=None
        self.stream_root=Path(config.root)/'stream'
        self.recorder=None
        if type(opts.get('stream_enabled',False)) is not bool:raise ValueError('stream_enabled must be boolean')
        if opts.get('stream_enabled',False):
            from .recording import Recorder
            self.recorder=Recorder(self.owner,self.cameras,self.stream_root,
                rgb_hz=opts.get('stream_rgb_hz',5),robot_hz=opts.get('stream_robot_hz',10),
                depth_hz=opts.get('stream_depth_hz',1),window_s=opts.get('stream_window_s',600))
            self.recorder.start()

    def tools(self):
        cams={'type':'string','enum':['head','left','right']}
        motion_options={
            'duration_s':{'type':'number','minimum':1,'maximum':18,
                          'description':'Omit to let the owner choose a duration within its speed/acceleration limits.'},
            'observe_after':{'type':'boolean','default':False,
                             'description':'After confirmed arrival, save and return current camera images in this call. '
                                           'An observation_error preserves the successful motion receipt: observe again, never replay the motion to recover images.'}}
        return [
            tool('piper_status','Current measured state and adapter limits; no motion.'),
            tool('piper_stream','Read the continuous RGB recording as a contact sheet. Inspection only; use piper_observe for metric depth and action boundaries.',
                 {'camera':cams,'since_ms':{'type':'integer'},'until_ms':{'type':'integer'},'count':{'type':'integer','minimum':1,'maximum':12}},['camera','since_ms','until_ms']),
            tool('piper_check_path','Pure sampled collision and LEFT flange workspace preflight. workspace_check uses the same 41 FK samples and bounds as dispatch; require workspace_clear AND cad_clear. Installed 170mm LEFT finger envelopes; optional environment boxes in dual_base_link. No SDK or motion; empty environment is NOT scene clearance.',
                 {'start14':{'type':'array','items':{'type':'number'},'minItems':14,'maxItems':14},
                  'target14':{'type':'array','items':{'type':'number'},'minItems':14,'maxItems':14},
                  'obstacles':{'type':'array','items':{'type':'object'}}},['start14','target14']),
            tool('piper_observe','Current three cameras. Head FFS is the default depth; explicitly request depth_cameras=[left] for wrist optical depth without hand-eye. Full saved image coordinates in meta.json; preview downscaled.',
                 {'cameras':{'type':'array','items':cams,'minItems':1,'maxItems':3},'require_depth':{'type':'boolean'},'depth_cameras':{'type':'array','items':{'enum':['head','left']},'maxItems':2}}),
            tool('piper_pixel_to_position','Saved full-resolution pixels to metres. Head defaults to provisional left_base_link; LEFT requires output_frame=camera_optical and cannot command a robot target. Does not expire historical analysis or grant motion.',
                 {'frame_id':{'type':'string'},'camera':cams,'pixels':{'type':'array','items':{'type':'array','items':{'type':'number'},'minItems':2,'maxItems':2},'minItems':1,'maxItems':64},'radius':{'type':'integer','minimum':0,'maximum':15},'output_frame':{'enum':['left_base_link','camera_optical']}},
                 ['frame_id','camera','pixels']),
            tool('piper_fk','Pure CAD flange FK, six radians. Base frame is the selected arm base, not dual_base_link; no calibrated grasp TCP.',
                 {'joints':V6,'arm':{'enum':['left','right']}},['joints']),
            tool('piper_solve_ik','Pure MoveIt2 KDL full-pose LEFT flange IK; optional explicit seed for offline use. No approximate solution, no hardware motion, no scene path certification.',
                 {'position':V3,'quaternion_xyzw':QUAT,'seed':V6},['position','quaternion_xyzw']),
            tool('piper_move_pose','Move LEFT flange to a left_base_link pose using KDL and the existing checked joint trajectory. This is a joint-interpolated path, not a guaranteed straight TCP path. Max 400mm per call. Preserve orientation if omitted.',
                 {'position':V3,'quaternion_xyzw':QUAT,**motion_options},['position']),
            tool('piper_move_joints','Move six LEFT joints, preserving right arm and both jaws. Max60deg change; existing owner validates path, speed and measured arrival.',
                 {'joints':V6,**motion_options},['joints']),
            tool('piper_gripper','LEFT full jaw gap 0..75mm in metres; max10mm change per call. Arrival is not grasp/contact proof. Right gripper is unavailable.',
                 {'width_m':{'type':'number','minimum':0,'maximum':.075},**motion_options},['width_m']),
            tool('piper_hold','At tool-call boundaries verify rest or request owner cancellation. Serialized MCP cannot interrupt another running tool: use the independent operator stop or physical emergency stop.')]

    def status(self):
        row={'robot':'piper','mode':'LIVE_BRIDGE' if self.motion_enabled else 'NO_SEND',
             'motion_enabled':self.motion_enabled,'arming':arming.read(self.config.root),'joint_order':ORDER,
             'units':'joints rad; jaws full gap m','urdf_sha256':URDF_SHA,'camera_serials':SERIALS,
             'controlled_link':'left_link6','frame':'left_base_link','grasp_tcp_calibrated':False,
             'workspace_min':None if self.box_min is None else self.box_min.tolist(),
             'workspace_max':None if self.box_max is None else self.box_max.tolist(),
             'camera_calibration':{'sha256':CANDIDATE_SHA,'provisional':True,'loo_max_position_m':.018081505632386675},
             'capabilities':{'left_joint_motion':self.motion_enabled,'left_pose_motion':self.motion_enabled,
                             'right_motion':False,'wrist_metric_depth':bool(getattr(self.cameras,'left_depth_root',None)),
                             'left_wrist_depth_frame':'left_d435_color_optical_frame','wrist_hand_eye':False,'continuous_stream':self.recorder is not None},
             'last_motion':self.last_motion,'closed':self.closed}
        row['left_tool_calibration']=load_calibration()
        table=Path(__file__).resolve().parents[2]/'agent_skills/control/piper/robot-interface/references/tool-characterization.md'
        row['tool_characterization']={
            'reference':'.agents/skills/robot-interface/references/tool-characterization.md',
            'sha256':hashlib.sha256(table.read_bytes()).hexdigest(),
            'acceptance':'partial',
            'joint_arrival_tolerance_rad':JOINT_ARRIVAL_TOLERANCE_RAD,
            'observation_joint_binding_tolerance_rad':OBSERVATION_JOINT_TOLERANCE_RAD,
            'joint_state_tolerance_rad':JOINT_ARRIVAL_TOLERANCE_RAD,
            'observation_jaw_binding_tolerance_m':OBSERVATION_JAW_TOLERANCE_M,
            'unresolved_failure':'PARTIAL: operator approved 0.01rad for all joint state checks (arrival, anchors, binding, rest/recovery) on 2026-11-09 to accommodate reported 0.3deg feedback-change filtering. This supersedes the previous 0.006rad joint-state policy and old 0.003rad requirement; it is not a firmware or motor-feedback repair. Historical J4/3mm and J5 acceptance results remain historical; live validation at the new tolerance is pending. Device motion_status=1 and physical small-step accuracy are not resolved by changing this tolerance. Read the latest tool-table policy before action.',
            'scope':'Left arm and left gripper only; right motion is not required. Retain right-state and whole-robot collision monitoring.',
            'selection':'RAX passed small J1/J4 cases and empty left-jaw 0.3/4/5/10 mm at three repeats each. This does not qualify the failed 3 mm pose, all joints, loaded grasp or moving Stop. Read the measured ranges and timing in the table.'}
        try:
            state=self.owner.state();row['measured_state']=state['robot_state'];row['owner_status']=state['robot_status'];row['cameras']=state['cameras']
        except Exception as exc:row['owner_error']=str(exc)
        if self.bridge:
            try:row['bridge']=self.bridge.status()
            except Exception as exc:row['bridge_error']=str(exc)
        bridge=row.get('bridge',{})
        row['motion_ready']=bool(self.motion_enabled and row['arming']['armed'] and
                                 bridge.get('phase') in ('IDLE','DONE') and
                                 not bridge.get('observation_recovery') and not self.closed and
                                 'owner_error' not in row)
        if self.recorder:row['recording']=self.recorder.status()
        return row

    def validate(self,name,args):
        if self.closed:raise ValueError('Backend closed')
        definitions={x['name']:x['inputSchema'] for x in self.tools()}
        if name not in definitions or not isinstance(args,dict):raise ValueError('Unknown tool/arguments')
        spec=definitions[name]
        if set(args)-set(spec['properties']) or set(spec['required'])-set(args):raise ValueError('Missing or unexpected arguments')
        if 'duration_s' in args and (type(args['duration_s']) not in (int,float) or not np.isfinite(args['duration_s']) or not 1<=args['duration_s']<=18):raise ValueError('Duration 1..18s required')
        for key,count in (('joints',6),('seed',6),('position',3),('quaternion_xyzw',4)):
            if key in args:vector(args[key],count)
        if 'arm' in args and args['arm'] not in ('left','right'):raise ValueError('Unknown arm')
        if 'quaternion_xyzw' in args and abs(np.linalg.norm(args['quaternion_xyzw'])-1)>1e-6:raise ValueError('Unit quaternion required')
        if 'joints' in args:self.kin.check(args['joints'],args.get('arm','left'))
        if 'seed' in args:self.kin.check(args['seed'])
        if 'width_m' in args and (type(args['width_m']) not in (int,float) or not np.isfinite(args['width_m']) or not 0<=args['width_m']<=.075):raise ValueError('Jaw width must be 0..75mm')
        if 'require_depth' in args and type(args['require_depth']) is not bool:raise ValueError('require_depth must be boolean')
        if 'observe_after' in args and type(args['observe_after']) is not bool:raise ValueError('observe_after must be boolean')
        if 'cameras' in args and (not isinstance(args['cameras'],list) or not 1<=len(args['cameras'])<=3 or len(set(args['cameras']))!=len(args['cameras']) or any(c not in SERIALS for c in args['cameras'])):raise ValueError('Select unique known cameras')
        if name=='piper_stream':
            if args['camera'] not in SERIALS:raise ValueError('Unknown camera')
            if any(type(args[k]) is not int for k in ('since_ms','until_ms')) or not 0<=args['since_ms']<=args['until_ms'] or args['until_ms']-args['since_ms']>600_000:raise ValueError('Expected at most 600s interval')
            if type(args.get('count',8)) is not int or not 1<=args.get('count',8)<=12:raise ValueError('count 1..12 required')
        if name=='piper_check_path':
            vector(args['start14'],14);vector(args['target14'],14)
            if not isinstance(args.get('obstacles',[]),list):raise ValueError('obstacles must be a list')
        if name in self.mutating:
            if self.post_motion_fault:raise RejectedBeforeDispatch('Post-motion state fault latched; physical-session recovery required')
            if not self.motion_enabled or self.bridge is None:raise PermissionError('NO_SEND: motion disabled in operator configuration')
            if not arming.read(self.config.root)['armed']:raise PermissionError('RAX session is not operator armed')
            self.bridge.armed()

    def _workspace_path(self,start,target):
        """One shared geometric guard for read-only preflight and actual dispatch."""
        if self.box_min is None or self.box_max is None:
            return {'checked':False,'clear':None,'reason':'No configured flange workspace'}
        a=self.kin.check(list(start));b=self.kin.check(list(target))
        result={'checked':True,'clear':True,'frame':'left_base_link','controlled_link':'left_link6',
                'min_m':self.box_min.tolist(),'max_m':self.box_max.tolist(),'samples':41,'first_violation':None}
        for i,fraction in enumerate(np.linspace(0.,1.,41)):
            p=self.kin.fk((a+fraction*(b-a)).tolist())[:3,3]
            for axis in range(3):
                if p[axis]<self.box_min[axis] or p[axis]>self.box_max[axis]:
                    bound=self.box_min[axis] if p[axis]<self.box_min[axis] else self.box_max[axis]
                    result.update(clear=False,first_violation={'sample_index':i,'fraction':float(fraction),
                        'axis':'xyz'[axis],'position_m':p.tolist(),'boundary_m':float(bound),
                        'excess_m':float(abs(p[axis]-bound))},reason=
                        f'Joint path leaves operator-defined left flange workspace: {"XYZ"[axis]}={p[axis]*1000:.3f} mm, '
                        f'allowed [{self.box_min[axis]*1000:.1f}, {self.box_max[axis]*1000:.1f}] mm (sample {i}/40)')
                    return result
        result['reason']='All sampled flange positions are within workspace'
        return result

    def _execute(self,target,before,duration):
        # Fresh local acquisition after model planning; no age limit is applied to a model's saved analysis.
        # Pose correctness still requires the agent to inspect the latest scene at action boundaries.
        try:
            generation=before['robot_status']['generation'];anchor=vector(before['robot_state']['values'],14)
            # Image acquisition belongs to piper_observe / explicit observe_after.
            # Preserve the existing three-stream identity/heartbeat guard using
            # owner metadata; downloading/decoding three JPEGs proves no scene fact.
            from .owner import check_camera_metadata
            for camera in SERIALS:check_camera_metadata(camera,before.get('cameras',{}).get(camera,{}))
            after=self.owner.state();q=vector(after['robot_state']['values'],14)
            for camera in SERIALS:check_camera_metadata(camera,after.get('cameras',{}).get(camera,{}))
            if after['robot_status']['generation']!=generation or not state_matches(q,anchor):
                raise ValueError('Owner changed during local pre-dispatch check; recompute from fresh state')
            if not np.array_equal(np.asarray(target[7:]),anchor[7:]):raise ValueError('Right arm/jaw must remain fixed')
            # Preserve the newly measured idle right side; do not command encoder-noise correction.
            target=list(target);target[7:]=q[7:].tolist()
            if np.max(abs(np.asarray(target[:6])-q[:6]))>np.deg2rad(60)+1e-9:raise ValueError('Left joint step exceeds60deg')
            if abs(target[6]-q[6])>.010000001:raise ValueError('Jaw step exceeds10mm')
            self.kin.check(target[:6]);self.kin.check(target[7:13],'right')
            # Same geometric path as the owner's monotonic quintic joint interpolation.
            # Like Franka's check, this is sampled FK, not a continuous swept-volume proof.
            workspace=self._workspace_path(q[:6].tolist(),target[:6])
            if workspace['clear'] is not True:raise ValueError(workspace['reason'])
        except Exception as exc:
            raise RejectedBeforeDispatch(str(exc)) from exc
        try:
            self.last_motion=self.bridge.move(target,duration)
        except RejectedBeforeDispatch:
            raise
        except Exception as exc:
            # The bridge cancels on failure. Fresh feedback is diagnostic evidence,
            # not proof that the stop has completed. Never clear control_uncertain.
            try:
                state=self.owner.state()['robot_state']
                measured=vector(state['values'],14)
                indices=[0,1,2,3,4,5,7,8,9,10,11,12]
                errors=abs(measured-np.asarray(target))
                worst=max(indices,key=lambda i:errors[i])
                exc.robot_state={**state,'motion_failure':{
                    'target14':list(target),'error14':errors.tolist(),
                    'max_error_joint':('left' if worst<7 else 'right')+'_j'+str(worst%7+1),
                    'max_joint_error_rad':float(errors[worst]),
                    'physical_stop_confirmed':False}}
            except Exception:
                pass  # Preserve original execution failure if feedback is unavailable.
            raise
        return self.last_motion,[]

    def _observe_after(self, motion):
        # Keep a completed action's receipt even if image acquisition fails. The caller
        # can recover with observe alone; this method never dispatches another motion.
        if motion.get('executed') is not True or motion.get('measured_arrival') is not True:
            raise RuntimeError('Motion completion lacks confirmed measured arrival; no post-motion observation attempted')
        try:
            observation,images=self._observe({},motion=motion)
        except PostMotionStateError:
            raise
        except Exception as exc:
            error=public_failure(exc,operation='piper_observe',stage='observation')
            return {**motion,'observation':None,'observation_error':{
                'type':error['type'],'reason':error['reason'],
                'recovery':'Motion completed with measured arrival. Call piper_observe only; do not replay the successful motion to recover images.'}},[]
        return {**motion,'observation':observation},images

    def _check_post_motion_state(self,motion):
        from .installed_settling import JOINTS
        try:
            state=self.owner.state();q=vector(state['robot_state']['values'],14)
            target=vector(motion['target14'],14);tol=motion['joint_tolerance_rad']
            status=state['robot_status'];meta=state['robot_state']['feedback_metadata']
            stamp=meta['source_timestamp_ns'];age=meta['maximum_age_ms']
            if (type(tol) not in (int,float) or not .001<=tol<=JOINT_ARRIVAL_TOLERANCE_RAD
                    or status['generation']!=motion['owner_generation'] or status.get('lifecycle_state')!='running'):
                raise ValueError('Post-motion owner identity/lifecycle or tolerance invalid')
            if (type(stamp) is not int or type(age) not in (int,float) or not np.isfinite(age)
                    or not 0<=age<=100 or not 0<=time.time_ns()-stamp<=300_000_000-int(age*1e6)):
                raise ValueError('Post-motion feedback stale/future')
            if np.max(abs(q[JOINTS]-target[JOINTS]))>tol or np.max(abs(q[[6,13]]-target[[6,13]]))>.001:
                raise ValueError('Post-motion feedback outside original target tolerance')
            return state
        except Exception as exc:raise PostMotionStateError(str(exc)) from exc

    def _wait_observation_rest(self,motion):
        from .installed_settling import RestWindow
        window=RestWindow();deadline=time.monotonic_ns()+3_000_000_000
        while time.monotonic_ns()<deadline:
            state=self._check_post_motion_state(motion)
            try:
                ready=window.add(state['robot_state']['values'],
                    state['robot_state']['feedback_metadata']['source_timestamp_ns'],time.monotonic_ns())
            except ValueError as exc:raise PostMotionStateError(str(exc)) from exc
            if ready:return window.evidence()
            time.sleep(.05)
        raise PostMotionStateError('Post-motion measured stability timeout')

    def _observe(self,args,*,after_ns=None,motion=None):
        motion=motion or self.last_motion
        if motion is None:return self._capture_observation(args,after_ns=after_ns)
        try:return self._observe_settled(args,motion)
        except PostMotionStateError as exc:
            self.post_motion_fault=str(exc)
            exc.robot_state={'motion_receipt':motion,'physical_stop_confirmed':False}
            if self.bridge is not None:
                try:exc.robot_state['stop_request']=self.bridge.cancel()
                except Exception:pass
            raise

    def _observe_settled(self,args,motion):
        discarded=[]
        for attempt in range(2):
            rest=self._wait_observation_rest(motion)
            observation,images=self._capture_observation(args,after_ns=time.time_ns(),motion=motion)
            self._check_post_motion_state(motion)
            binding=observation['state_binding']
            if not binding['owner_unchanged']:raise PostMotionStateError('Owner changed during post-motion capture')
            if binding['rest_consistent']:
                observation.update(observation_settling=rest,discarded_frame_ids=discarded)
                frames=getattr(self.cameras,'frames',None)
                if frames is not None:
                    import json
                    (Path(frames)/observation['frame_id']/'meta.json').write_text(json.dumps(observation,indent=2))
                return observation,images
            discarded.append(observation['frame_id'])
        raise PostMotionStateError('Post-motion capture drift persisted after one read-only reacquisition')

    def _capture_observation(self,args,*,after_ns=None,motion=None):
        before_state=self._check_post_motion_state(motion) if motion else self.owner.state()
        options={} if after_ns is None else {'after_ns':after_ns}
        if 'depth_cameras' in args:options['depth_cameras']=args['depth_cameras']
        meta,arrays=self.cameras.capture(args.get('cameras',list(SERIALS)),args.get('require_depth',self.config.observation_profile=='rgbd'),**options)
        images=[]
        for camera,rgb in arrays:
            h,w=rgb.shape[:2];preview=cv2.resize(rgb,(min(self.width,w),round(h*min(self.width,w)/w)))
            ok,jpg=cv2.imencode('.jpg',cv2.cvtColor(preview,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,85])
            if not ok:raise ValueError('JPEG encoding failed')
            meta['cameras'][camera]['preview_shape_hw']=list(preview.shape[:2])
            images.append({'type':'image','mimeType':'image/jpeg','data':base64.b64encode(jpg).decode()})
        after_state=self._check_post_motion_state(motion) if motion else self.owner.state();meta['measured14']=after_state['robot_state']['values']
        drift=float(np.max(abs(np.asarray(before_state['robot_state']['values'])-meta['measured14'])))
        delta=abs(np.asarray(before_state['robot_state']['values'])-meta['measured14'])
        joint_drift=float(np.max(delta[JOINTS]));jaw_drift=float(np.max(delta[[6,13]]))
        meta['state_binding']={'kind':'state bracket; cameras are not hardware synchronized',
            'before14':before_state['robot_state']['values'],'after14':meta['measured14'],
            'owner_generation':after_state['robot_status']['generation'],
            'owner_unchanged':before_state['robot_status']['generation']==after_state['robot_status']['generation'],
            'max_drift':drift,'max_joint_drift_rad':joint_drift,'max_jaw_drift_m':jaw_drift,
            'joint_tolerance_rad':OBSERVATION_JOINT_TOLERANCE_RAD,'jaw_tolerance_m':OBSERVATION_JAW_TOLERANCE_M,
            'rest_consistent':joint_drift<=OBSERVATION_JOINT_TOLERANCE_RAD and jaw_drift<=OBSERVATION_JAW_TOLERANCE_M and before_state['robot_status']['generation']==after_state['robot_status']['generation']}
        # Installation identity is independent of RGBD. RGB-only observations must
        # still bind the executed collision asset; they make no projection claim.
        from .finger_geometry import geometry_identity
        meta['finger_geometry']={'geometry_identity':geometry_identity(),
            'physical_clearance_authority':False,'projection_available':False}
        if meta['cameras'].get('head',{}).get('metric_depth') is True:
            from .finger_geometry import inspection_geometry
            meta['finger_geometry']=inspection_geometry(meta['measured14'],meta['cameras']['head'])
        # Persist the same feedback/calibration evidence returned to the controller.
        import json
        frame_path=getattr(self.cameras,'frames',None)
        if frame_path is not None:
            (Path(frame_path)/meta['frame_id']/'meta.json').write_text(json.dumps(meta,indent=2))
        return meta,images

    def call(self,name,args):
        try:
            self.validate(name,args)
        except Exception as exc:
            if name in self.mutating:raise RejectedBeforeDispatch(str(exc)) from exc
            raise
        if name=='piper_status':return self.status(),[]
        if name=='piper_check_path':
            if self.collision is None:
                from .installed_collision import load_checker
                root=self.config.backend_options.get('collision_legacy_root')
                if not root:raise ValueError('No host collision asset installation configured')
                self.collision=load_checker(root)
            workspace=self._workspace_path(args['start14'][:6],args['target14'][:6])
            result=self.collision.path([args['start14'],args['target14']],obstacles=args.get('obstacles',[]))
            result.update(workspace_checked=workspace['checked'],workspace_clear=workspace['clear'],workspace_check=workspace)
            return result,[]
        if name=='piper_stream':
            from services.franka.stream import contact_sheet
            grid,times=contact_sheet(self.stream_root,args['camera'],args['since_ms'],args['until_ms'],args.get('count',8),time.time_ns()//1_000_000)
            images=[]
            if grid is not None:
                ok,jpg=cv2.imencode('.jpg',grid)
                if not ok:raise ValueError('Stream JPEG encoding failed')
                images=[{'type':'image','mimeType':'image/jpeg','data':base64.b64encode(jpg).decode()}]
            return {'camera':args['camera'],'times_ms':times,'inspection_only':True,'metric_depth':False,'executed':False},images
        if name=='piper_fk':
            side=args.get('arm','left');t=self.kin.fk(args['joints'],side)
            return {'T_base_flange':t.tolist(),'position':t[:3,3].tolist(),'quaternion_xyzw':Rotation.from_matrix(t[:3,:3]).as_quat().tolist(),
                    'frame':side+'_base_link','controlled_link':side+'_link6','executed':False,
                    **({'effective_contact_tcp':tool_pose(t)} if side=='left' else {})},[]
        if name=='piper_solve_ik':
            try:
                seed=args.get('seed')
                if seed is None:seed=self.owner.state()['robot_state']['values'][:6]
                return self.solver.solve(args['position'],args['quaternion_xyzw'],seed),[]
            except IKNoSolution:
                raise
            except Exception as exc:
                raise RejectedBeforeDispatch(f'IK query failed; no motion requested or dispatched: {exc}') from exc
        if name=='piper_observe':return self._observe(args)
        if name=='piper_pixel_to_position':return self.cameras.pixel(**args),[]
        if name=='piper_hold':
            if not self.bridge:return {'hardware_stop_sent':False,'reason':'NO_SEND has no motion bridge; cannot stop external hardware'},[]
            result=self.bridge.hold()
            if result.get('owner_idle'):
                before=self.owner.state();time.sleep(.35);after=self.owner.state()
                if before['robot_status']['generation']!=after['robot_status']['generation'] or not state_matches(before['robot_state']['values'],after['robot_state']['values'],jaw_tolerance=.0003):
                    return self.bridge.cancel(),[]
                result['measured_rest']=True
            return result,[]
        try:
            before=self.owner.state();target=list(before['robot_state']['values'])
            if name=='piper_move_joints':target[:6]=args['joints']
            elif name=='piper_gripper':target[6]=args['width_m']
            elif name=='piper_move_pose':
                t=self.kin.fk(target[:6]);p=np.asarray(args['position'])
                if np.linalg.norm(p-t[:3,3])>.4+1e-9:raise ValueError('Flange translation exceeds400mm')
                quat=args.get('quaternion_xyzw',Rotation.from_matrix(t[:3,:3]).as_quat().tolist())
                target[:6]=self.solver.solve(args['position'],quat,target[:6])['q']
        except IKNoSolution:
            raise
        except Exception as exc:
            raise RejectedBeforeDispatch(str(exc)) from exc
        result,images=self._execute(target,before,args.get('duration_s'))
        return self._observe_after(result) if args.get('observe_after',False) else (result,images)

    def close(self):
        if self.closed:return
        self.closed=True
        if self.recorder:self.recorder.close()
        if self.bridge:self.bridge.close()
        self.solver.close()
