"""RAX dual PiPER adapter; no-send by default, explicit bounded joint bridge optional.

The formal XPolicy owner remains the sole hardware writer. The optional bridge
requires private operator arming and verifies receipts plus measured arrival.
"""
from .motion_profiles import LEFT_JAW_COMMAND_MAX_M
import base64
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import time
import uuid
import cv2
import numpy as np
from services import arming
from services.rby1.recorder import Recorder
from services.stream import contact_sheet
from .geometry import PiperNoSendBackend, tool, VECTOR
from .owner import OwnerReader, ORDER
from .depth import DepthReader

CALIBRATION_SHA = '28db0ac28c85f52dc7497ebeac82f46b361b0787d56ad07c791f4b7bfba4f091'


class Camera:
    def __init__(self, owner, name):
        self.owner, self.name = owner, name

    def read(self):
        rgb, meta = self.owner.camera(self.name)
        return meta['after']['timestamp_ns']/1e9, rgb, None


class PiperBackend:
    mutating = frozenset({'piper_move_joints', 'piper_move_eef', 'piper_gripper'})

    def __init__(self, config):
        self.config = config
        options = dict(config.backend_options)
        if set(options) - {'address', 'depth_root', 'record_stream', 'operator_review',
                           'camera_candidate', 'camera_candidate_sha256', 'motion_bridge', 'provisional_calibration',
                           'qualification', 'qualification_sha256'}:
            raise ValueError('Unsupported backend options')
        self.camera_candidate = None
        self.provisional_calibration=options.get('provisional_calibration',False)
        if type(self.provisional_calibration) is not bool:raise ValueError('provisional_calibration must be boolean')
        if 'camera_candidate' in options or 'camera_candidate_sha256' in options:
            if not {'camera_candidate','camera_candidate_sha256'} <= set(options):
                raise ValueError('Camera candidate path and SHA must be supplied together')
            from .extrinsics import CameraCandidate
            self.camera_candidate = CameraCandidate(options['camera_candidate'],options['camera_candidate_sha256'],CALIBRATION_SHA)
        if self.provisional_calibration and self.camera_candidate is None:raise ValueError('provisional calibration requires a pinned candidate')
        self.motion=None
        self.qualification=None
        if 'qualification' in options or 'qualification_sha256' in options:
            from .qualification import load
            if not {'qualification','qualification_sha256'}<=set(options):raise ValueError('qualification path and SHA required together')
            self.qualification=load(options['qualification'],options['qualification_sha256'])
        if 'motion_bridge' in options:
            from .motion_bridge import MotionClient
            self.motion=MotionClient(options['motion_bridge'],config.root)
            self.motion.status()
        self.owner = OwnerReader(options.get('address', '127.0.0.1:8091'))
        self.depth = DepthReader(options.get('depth_root', Path(config.root)/'ffs'), CALIBRATION_SHA)
        self.geometry = PiperNoSendBackend(SimpleNamespace(backend_options={}))
        self.frames = Path(config.root)/'agent-workspace/runtime/frames'
        self.frames.mkdir(parents=True, exist_ok=True)
        self.saved = {}
        self.closed = False
        from .cartesian_motion import CartesianPlanner
        self.planner = CartesianPlanner()
        if self.motion is not None:self.planner.warmup()
        self._collision = None
        self.review = None
        if type(options.get('operator_review',False)) is not bool:
            raise ValueError('operator_review must be boolean')
        if options.get('operator_review',False):
            from .review_bridge import ReviewBridge, TOKEN
            from .controlled_transport import StopOnlyHTTP
            self.review = ReviewBridge(self.owner,self.geometry,self.collision(),stop=StopOnlyHTTP(TOKEN.read_text().strip()))
        record = options.get('record_stream', True)
        if type(record) is not bool:
            raise ValueError('record_stream must be boolean')
        self.recorder = Recorder({name: Camera(self.owner, name) for name in ('head','left','right')},
            self.robot_record, Path(config.root)/'stream', fps=10, robot_hz=10, window_s=120, depth=False, archive=Path(config.root)/'recordings') if record else None
        if self.recorder:
            self.recorder.start()

    def collision(self):
        if self._collision is None:
            from .collision import CollisionChecker
            self._collision = CollisionChecker()
        return self._collision

    def robot_record(self):
        state = self.owner.state()
        return dict(joint_order=ORDER, joints=state['robot_state']['values'],
                    feedback_metadata=state['robot_state'].get('feedback_metadata',{}),
                    owner_generation=state['robot_status']['generation'],
                    owner_hardware_write_count=state['robot_status']['hardware_write_count'], adapter_hardware_writes=0)

    def status(self):
        status = dict(robot='piper', mode='NO_SEND', hardware_writes=0, motion_transport_present=False,
            arming=arming.read(self.config.root), physical_calibration_validated=False,
            controller_characterization_accepted=False, physical_stop_accepted=False,
            collision_acceptance=False, collision_checker='FCL sampled CAD; environment and physical calibration unverified', joint_order=ORDER, joint_units='rad', jaw_units='m full width',
            urdf_sha256=self.geometry.status()['urdf_sha256'],
            blockers=['camera-to-robot and TCP physical calibration', 'whole-path collision acceptance',
                      'physical controller characterization', 'reviewed sole-owner motion transport'], closed=self.closed,operator_review_available=self.review is not None,automatic_release=False)
        status['left_gripper_operating_range']=dict(command_min_m=0,command_max_m=LEFT_JAW_COMMAND_MAX_M,
            observed_physical_max_m=.0756,maximum_confirmed_by_operator=True,contact_force_calibrated=False)
        status['ik_solver']={'name':'moveit2_kdl','persistent':True,'legacy_fallback':False,'frame':'left_base_link','controlled_link':'left_link6'}
        status['capabilities']={'joint_path_preview':True,'left_gripper_preview':True,
            'probe_requests':self.review is not None,'general_physical_dispatch':False,
            'cartesian_solver_exposed':False,'right_gripper_available':False}
        status['camera_calibration_candidate'] = (self.camera_candidate.status()
            if self.camera_candidate is not None else dict(loaded=False,motion_permit=False))
        status['camera_calibration_candidate']['provisional_use']=self.provisional_calibration
        if self.motion is not None:
            status.update(mode='JOINT_COMMISSIONING',motion_transport_present=True,motion_bridge=self.motion.status())
            status['capabilities']['bounded_joint_dispatch']=True
            status['capabilities']['cartesian_solver_exposed']=True
            status['capabilities']['left_gripper_available']=status['motion_bridge'].get('gripper_motion') is True
            if self.qualification is not None:
                limits=status['motion_bridge']['limits'];qualified=self.qualification['limits']
                if any(limits.get(k)!=qualified[k] for k in qualified):
                    raise ValueError('live limits differ from qualified limits')
                if self.camera_candidate is None or self.camera_candidate.sha256!=self.qualification['camera_candidate_sha256']:
                    raise ValueError('qualified calibration identity mismatch')
                if status['motion_bridge'].get('gripper_motion') is not True:
                    raise ValueError('qualified left gripper owner extension missing')
                status.update(mode='SUPERVISED_EXPLORATION',controller_characterization_accepted=True,physical_stop_accepted=True,
                    motion_qualification={'ready':True,'scope':self.qualification['scope'],'limits':qualified,'task_success':False})
                status['capabilities']['general_physical_dispatch']=True
                status['blockers']=['independent camera/TCP accuracy deferred by operator',
                    'scene/contact clearance and object-task success remain to be established during supervised exploration']
        status['recording']={'enabled':self.recorder is not None,
            'target_rgb_fps':10 if self.recorder else None,'target_state_hz':10 if self.recorder else None,
            'archive':str(Path(self.config.root)/'recordings') if self.recorder else None}
        if self.recorder:
            status['recording']['camera_errors']=dict(getattr(self.recorder,'errors',{}))
            archiver=getattr(self.recorder,'archiver',None)
            if archiver:status['recording']['archive_status']=archiver.status()
        try:
            live = self.owner.state()
            status.update(measured_state=live['robot_state'], owner_generation=live['robot_status']['generation'],
                owner_hardware_write_count=live['robot_status']['hardware_write_count'],
                owner_hardware_write_enabled=live['robot_status']['hardware_write_enabled'],
                component_health=live['robot_status']['component_health'], camera_status=live['cameras'])
        except Exception as exc:
            status['owner_error'] = str(exc)
        try:
            _, report = self.depth.read()
            status['depth'] = dict(ready=True, sample_id=report['sample_id'], expires_unix_ns=report['expires_unix_ns'])
        except Exception as exc:
            status['depth'] = dict(ready=False, reason=str(exc), source='FFS required; no D435-depth fallback')
        return status

    def tools(self):
        review_tools = [] if self.review is None else [
            tool('piper_request_joint_probe','Request ONE left-joint air-identification probe <=.005rad for operator review; does NOT execute or auto-approve.',{'joint':{'type':'integer','minimum':1,'maximum':6},'delta_rad':{'type':'number','minimum':-.005,'maximum':.005}},['joint','delta_rad']),
            tool('piper_review_status','Read a requested probe outcome; pending is not execution.',{'request_id':{'type':'string'}},['request_id'])]
        return review_tools + [tool('piper_status','Live owner feedback, exact configured dispatch capabilities and commissioning results.'),
            tool('piper_observe','Fresh saved observation. Head RGB is exactly bound to FFS depth when require_depth=true. RGB-only is explicitly unbound to depth.',
                 {'require_depth':{'type':'boolean'}}),
            tool('piper_pixel_to_position','Analyze exact saved FFS RGB OPTICAL coordinates; returns age/expiry. Historical analysis is allowed, never a motion permit; robot extrinsics unvalidated.',
                 {'frame_id':{'type':'string'}, 'pixels':{'type':'array','minItems':1,'maxItems':32,
                 'items':{'type':'array','items':{'type':'integer'},'minItems':2,'maxItems':2}}}, ['frame_id','pixels']),
            tool('piper_solve_ik','MoveIt2 KDL full-pose LEFT flange IK from current measured seed; pure planning, no motion, no approximate solution or old-solver fallback.', {'position':{'type':'array','items':{'type':'number'},'minItems':3,'maxItems':3},'quaternion_xyzw':{'type':'array','items':{'type':'number'},'minItems':4,'maxItems':4}},['position','quaternion_xyzw']),
            tool('piper_fk','Pinned CAD FK, not physically calibrated.', {'joints':VECTOR}, ['joints']),
            tool('piper_check','CAD joint limits and self-collision clearance, not physical acceptance; excludes only declared mechanical neighbours.', {'joints':VECTOR}, ['joints']),
            tool('piper_check_path','Sampled whole-path dual-arm CAD collision check; no hardware or scene clearance certification.',
                 {'samples':{'type':'array','minItems':1,'maxItems':901,'items':VECTOR}},['samples']),
            tool('piper_nominal_frames','v12-derived 32cm nominal base/camera/TCP frames; all physical calibration gates remain false.'),
            tool('piper_preview_joints','Bounded no-send trajectory starting at fresh measured joints; never changes feedback.',
                 {'target':VECTOR,'duration_s':{'type':'number','minimum':.1,'maximum':30}}, ['target','duration_s']),
            tool('piper_preview_path','Offline left-arm/jaw waypoint preview from measured state; fixed right arm, CAD checks, no execution.',
                 {'waypoints':{'type':'array','minItems':1,'maxItems':8,'items':VECTOR},'durations_s':{'type':'array','minItems':1,'maxItems':8,'items':{'type':'number','minimum':.1,'maximum':30}}},['waypoints','durations_s']),
            tool('piper_preview_gripper','Preview LEFT full jaw gap in metres; no grasp force or contact proof, never executes.',
                 {'side':{'enum':['left']},'width_m':{'type':'number','minimum':0,'maximum':LEFT_JAW_COMMAND_MAX_M},'duration_s':{'type':'number','minimum':.1,'maximum':30}},['side','width_m','duration_s']),
            tool('piper_move_joints','Bounded dual-arm joint motion through formal owner when configured and operator armed. Right jaw fixed; use piper_gripper for left jaw changes. Omit duration_s for automatically timed motion at the live speed/acceleration caps. An explicit duration must respect those caps. Success requires measured stable arrival; see live terminal_dwell_s.', {'joints':VECTOR,'duration_s':{'type':'number','minimum':1,'maximum':18}}, ['joints']),
            tool('piper_move_eef','Bounded LEFT flange pose in left_base_link metres, through checked IK and the formal joint owner. Default preserves orientation. This is left_link6, not a calibrated grasp TCP.', {'arm':{'enum':['left']},'position':{'type':'array','items':{'type':'number'},'minItems':3,'maxItems':3},'quaternion_xyzw':{'type':'array','items':{'type':'number'},'minItems':4,'maxItems':4},'duration_s':{'type':'number','minimum':1,'maximum':18}}, ['arm','position']),
            tool('piper_gripper','LEFT full jaw gap 0..75 mm through the formal owner; max 10 mm change, fixed profile force. The installed gripper is already fully open around measured 75.6 mm; never request 84/93/100 mm. Right jaw unavailable. Arrival is not proof of grasp or contact force.', {'side':{'enum':['left']},'width_m':{'type':'number','minimum':0,'maximum':LEFT_JAW_COMMAND_MAX_M},'duration_s':{'type':'number','minimum':1,'maximum':18}}, ['side','width_m']),
            tool('piper_hold','Hold at the current position, following RAX real-robot hold semantics. Default mode cancel: after completed motion, verify measured rest and retain the running owner for observation/replanning. This does not certify object safety. PiPER active cancellation or explicit stop_command uses owner Stop, latches FAULT and requires operator recovery; never auto-rearm.', {'mode':{'type':'string','enum':['cancel','stop_command']}}),
            tool('piper_stream','Official sliding-window RGB contact sheet from read-only owner feedback.',
                 {'camera':{'enum':['head','left','right']},'seconds':{'type':'integer','minimum':1,'maximum':120}}, ['camera','seconds'])]

    def validate(self, name, args):
        if self.closed:
            raise ValueError('Backend closed')
        schemas = {x['name']:x['inputSchema'] for x in self.tools()}
        if name not in schemas or not isinstance(args,dict):
            raise ValueError('Unknown tool or arguments')
        schema = schemas[name]
        if set(args)-set(schema['properties']) or set(schema['required'])-set(args):
            raise ValueError('Missing or unexpected arguments')
        if name=='piper_move_joints' and self.motion is not None:
            self.geometry._vector(args['joints'])
            duration=args.get('duration_s',5)
            if type(duration) not in (int,float) or not np.isfinite(duration) or not 1<=duration<=18:raise ValueError('duration 1..18 seconds required')
        elif name=='piper_move_eef' and self.motion is not None:
            if args['arm']!='left':raise ValueError('only left flange IK is supported')
            p=np.asarray(args['position'],dtype=float)
            if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('finite XYZ position required')
            duration=args.get('duration_s',5)
            if type(duration) not in (int,float) or not np.isfinite(duration) or not 1<=duration<=18:raise ValueError('duration 1..18 seconds required')
        elif name=='piper_gripper' and self.motion is not None:
            if args['side']!='left' or self.motion.status().get('gripper_motion') is not True:raise PermissionError('left gripper dispatch unavailable')
            width=args['width_m'];duration=args.get('duration_s',2)
            if type(width) not in (int,float) or not np.isfinite(width) or not 0<=width<=LEFT_JAW_COMMAND_MAX_M:raise ValueError('full jaw gap must be 0..75 mm (0.075 m physical command limit)')
            if type(duration) not in (int,float) or not np.isfinite(duration) or not 1<=duration<=18:raise ValueError('duration 1..18 seconds required')
        elif name in self.mutating:
            raise PermissionError('NO_SEND: no motion transport in this build, regardless of operator arming. Physical calibration and reviewed owner dispatch required.')
        if name in ('piper_fk','piper_check'):
            self.geometry.validate(name,args)
        if name=='piper_hold' and args.get('mode','cancel') not in ('cancel','stop_command'):raise ValueError('unknown hold mode')
        if 'require_depth' in args and type(args['require_depth']) is not bool:
            raise ValueError('require_depth must be boolean')
        if name == 'piper_stream' and (args['camera'] not in ('head','left','right') or type(args['seconds']) is not int or not 1<=args['seconds']<=120):
            raise ValueError('Invalid stream request')

    def preview_path(self, waypoints, durations_s):
        if not isinstance(waypoints,list) or not isinstance(durations_s,list) or not 1<=len(waypoints)<=8 or len(waypoints)!=len(durations_s):
            raise ValueError('1..8 waypoints with matching durations required')
        before=self.owner.state();anchor=self.geometry._vector(before['robot_state']['values'])
        current=anchor.tolist();samples=[current];times=[0.0]
        for target,duration in zip(waypoints,durations_s):
            q=self.geometry._vector(target)
            if not np.array_equal(q[7:],anchor[7:]):raise ValueError('right arm/jaw must remain fixed')
            part,_=self.geometry.call('piper_preview_joints',dict(start=current,target=target,duration_s=duration))
            if len(samples)+len(part['samples'])-1>901:raise ValueError('path preview limited to 901 samples')
            times.extend((np.asarray(part['time_s'][1:])+times[-1]).tolist())
            samples.extend(part['samples'][1:]);current=target
        collision=self.collision().path(samples)
        if not collision['cad_clear']:raise ValueError('collision refusal: '+json.dumps(collision.get('contacts')))
        after=self.owner.state()
        if before['robot_status']['generation']!=after['robot_status']['generation']:raise ValueError('owner generation changed during preview')
        if np.max(abs(np.asarray(after['robot_state']['values'])-anchor))>.001:raise ValueError('measured anchor changed during preview')
        result=dict(samples=samples,time_s=times,start=anchor.tolist(),executed=False,hardware_writes=0,
                    owner_generation=before['robot_status']['generation'],collision_report=collision,
                    physical_dispatch_approved=False,physical_calibration_validated=False,
                    state_source='measured anchor; nominal CAD path only',right_arm_frozen=True,
                    limitation='No scene/payload/cable or contact acceptance; no native controller tracking proof')
        from .dispatch import digest
        result['preview_sha256']=digest(result)
        return result,[]

    def fresh_depth(self):
        # Local sensor polling only. No model request/retry or timestamp rewrite.
        deadline = time.monotonic()+1.5
        while True:
            try:
                return self.depth.read()
            except ValueError as exc:
                if 'stale/future' not in str(exc) or time.monotonic() >= deadline:
                    raise
                time.sleep(.025)

    @staticmethod
    def image(rgb, *, jpeg=False):
        ext='.jpg' if jpeg else '.png'
        settings=[cv2.IMWRITE_JPEG_QUALITY,85] if jpeg else [cv2.IMWRITE_PNG_COMPRESSION,1]
        ok, png = cv2.imencode(ext,cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),settings)
        if not ok:
            raise ValueError('Image encode failed')
        return {'type':'image','mimeType':'image/jpeg' if jpeg else 'image/png','data':base64.b64encode(png).decode()}

    def call(self, name, args):
        self.validate(name,args)
        if name == 'piper_status': return self.status(),[]
        if name == 'piper_move_joints':return self.motion.move(**args),[]
        if name == 'piper_solve_ik':
            q=self.owner.state()['robot_state']['values']
            return self.planner.solve(args['position'],args['quaternion_xyzw'],q[:6]),[]
        if name == 'piper_move_eef':
            planner=self.planner;q=self.owner.state()['robot_state']['values']
            limits=self.motion.status()['limits'];cap=limits['max_joint_displacement_rad']
            try:
                plan=planner.plan(q,args['position'],args.get('quaternion_xyzw'),max_joint_delta=cap,max_translation_m=limits.get('max_eef_translation_m',.02))
            except Exception as exc:
                # This boundary precedes MotionClient.move: no request has been staged.
                from .motion_bridge import PlanningRejected
                refusal=PlanningRejected(str(exc));refusal.stage='ik_planning'
                raise refusal from exc
            result=self.motion.move(plan['target14'],args.get('duration_s'))
            actual=planner.kinematics.fk(result['final14'][:6])
            result.update(controlled_link='left_link6',frame='left_base_link',calibrated_tcp=False,
                measured_fk_position_error_m=float(np.linalg.norm(actual[:3,3]-np.asarray(args['position']))),ik=plan)
            return result,[]
        if name == 'piper_gripper':
            q=self.owner.state()['robot_state']['values'];q[6]=args['width_m']
            result=self.motion.move(q,args.get('duration_s'));result.update(side='left',grasp_confirmed=False,force_feedback_validated=False)
            return result,[]
        if name == 'piper_fk': return self.geometry.call(name,args)
        if name == 'piper_check': return self.collision().check(args['joints']),[]
        if name == 'piper_check_path': return self.collision().path(args['samples']),[]
        if name == 'piper_nominal_frames':
            from .calibration import nominal_manifest
            return nominal_manifest(self.geometry),[]
        if name == 'piper_request_joint_probe': return self.review.propose(**args),[]
        if name == 'piper_review_status': return self.review.status(**args),[]
        if name == 'piper_hold':
            if self.motion is not None:return self.motion.hold_current(**args),[]
            if self.review is not None:return self.review.hold(),[]
            return dict(executed=False,hardware_writes=0,hardware_stop_sent=False),[]
        if name == 'piper_preview_path':return self.preview_path(**args)
        if name == 'piper_preview_gripper':
            if args['side']!='left':raise ValueError('right gripper unavailable')
            width=args['width_m']
            if type(width) not in (int,float) or not np.isfinite(width) or not 0<=width<=LEFT_JAW_COMMAND_MAX_M:raise ValueError('full jaw gap must be finite 0..75 mm (0.075 m physical command limit)')
            state=self.owner.state();target=list(state['robot_state']['values']);target[6]=float(width)
            return self.preview_path([target],[args['duration_s']])
        if name == 'piper_preview_joints':return self.preview_path([args['target']],[args['duration_s']])
        if name == 'piper_observe':
            frame_id = uuid.uuid4().hex
            arrays,report = self.fresh_depth() if args.get('require_depth',True) else (None,None)
            rgb,meta = (arrays['rgb'],report) if arrays is not None else self.owner.camera('head')
            directory = self.frames/frame_id
            directory.mkdir()
            encoded = self.image(rgb)
            (directory/'head.png').write_bytes(base64.b64decode(encoded['data']))
            (directory/'head.json').write_text(json.dumps(meta,indent=2))
            if arrays is not None:
                if time.time_ns() >= report['expires_unix_ns']:
                    raise ValueError('FFS expired while saving observation')
                np.savez(directory/'depth.npz',**arrays)
                self.saved[frame_id]=(arrays,report)
                while len(self.saved)>8:
                    self.saved.pop(next(iter(self.saved)))
            if report is not None and time.time_ns() >= report['expires_unix_ns']:
                self.saved.pop(frame_id, None)
                raise ValueError('FFS expired while serializing observation; no live result returned')
            return dict(frame_id=frame_id,path=str(directory),depth_available=arrays is not None,
                        coordinate_frame='global_d435_color_optical_frame' if arrays is not None else None,
                        robot_transform_available=self.provisional_calibration,
                        provisional_calibration=self.provisional_calibration,
                        candidate_transform_available=self.camera_candidate is not None,head=meta),[self.image(rgb,jpeg=True)]
        if name == 'piper_pixel_to_position':
            arrays,report = self.saved.get(args['frame_id'],(None,None))
            if arrays is None: raise ValueError('Unknown/evicted/non-depth frame_id')
            # A saved image remains analyzable after inference latency. Its age
            # is explicit; executing motion still requires independent fresh evidence.
            observation_fresh = time.time_ns() < report['expires_unix_ns']
            pixels = args['pixels']
            if not isinstance(pixels,list) or not 1<=len(pixels)<=32: raise ValueError('Expected 1..32 pixels')
            result=[]
            for pixel in pixels:
                if not isinstance(pixel,list) or len(pixel)!=2 or any(type(x) is not int for x in pixel): raise ValueError('Pixels must be [integer u, integer v]')
                u,v = pixel
                h,w = arrays['valid'].shape
                if not 0<=u<w or not 0<=v<h or not arrays['valid'][v,u]: raise ValueError('Pixel outside image or in depth hole')
                result.append(arrays['xyz_rgb_m'][v,u].tolist())
            return dict(frame_id=args['frame_id'],positions=result,frame='global_d435_color_optical_frame',units='m',
                        robot_positions=self.camera_candidate.points(result) if self.provisional_calibration else None,
                        robot_frame='left_base_link' if self.provisional_calibration else None,
                        provisional_calibration=self.provisional_calibration,
                        candidate_robot_positions=self.camera_candidate.points(result) if self.camera_candidate is not None else None,
                        candidate_frame='left_base_link' if self.camera_candidate is not None else None,
                        candidate_calibration_sha256=self.camera_candidate.sha256 if self.camera_candidate is not None else None,
                        candidate_only=True,confidence_available=False,physical_calibration_validated=False,
                        observation_fresh=observation_fresh,expires_unix_ns=report['expires_unix_ns'],
                        historical_analysis_only=not observation_fresh,motion_permit=False),[]
        if name == 'piper_stream':
            now=int(time.time()*1000)
            sheet, stamps=contact_sheet(Path(self.config.root)/'stream',args['camera'],now-args['seconds']*1000,now,6,now)
            if sheet is None: raise ValueError('No recorded frames yet')
            # The official contact-sheet helper returns BGR.
            return dict(camera=args['camera'],timestamps_ms=stamps,depth_bound=False),[self.image(cv2.cvtColor(sheet,cv2.COLOR_BGR2RGB))]
        raise ValueError('Unsupported tool')

    def close(self):
        if self.closed: return
        self.closed=True
        if self.motion is not None:self.motion.close()
        if self.review is not None:self.review.cancel_pending('backend closed; no delayed approval')
        if self.recorder: self.recorder.close()
        self.geometry.close()
