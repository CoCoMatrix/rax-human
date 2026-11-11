"""Existing installed bridge, vendored for per-request timing (2026-11-10).

No new service/protocol; pinned original retained in verification/rc6-handoff-20261110.
Private RAX requests -> existing PolicyServer -> formal ActionChunk owner.

This process has no SDK/device connection. Start/Prepare/enable remain operator
operations. The independently checked owner lease survives loss of this process.
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid
from types import SimpleNamespace
from services import arming
from services.json_io import write_private_json
from .controlled_transport import StopOnlyHTTP,ReceiptTail,require,vector
from .controlled_policy import ControlledPolicy
from .motion_transport import MotionTransport,ACTIVE
from .motion_lease import read_private
from .geometry import PiperNoSendBackend
from .collision import CollisionChecker


class PlanningRejected(ValueError):
    """The bridge proved this request was rejected before staging any motion."""
    stage="validation"
    executed=False
    dispatch_started=False
    failure_kind="pre_dispatch_rejection"


class MotionClient:
    def __init__(self,root,session_root):
        self.root=Path(root).resolve();self.session_root=Path(session_root).resolve();self.instance=None;self.active=None

    def status(self):
        v=read_private(self.root/'state.json')
        require(v.get('schema')=='rax.piper.motion_bridge.v1' and v.get('session_root')==str(self.session_root),'wrong motion bridge session')
        require(type(v.get('timestamp_ns')) is int and 0<=time.time_ns()-v['timestamp_ns']<=500_000_000,'motion bridge heartbeat stale')
        require(self.instance in (None,v.get('instance')) and isinstance(v.get('instance'),str),'motion bridge instance changed')
        self.instance=v['instance'];return v

    def move(self,joints,duration_s=None):
        s=self.status();require(s['phase'] in ('IDLE','DONE'),'motion bridge busy or faulted')
        require(arming.read(self.session_root).get('armed') is True,'operator disarmed')
        request=uuid.uuid4().hex;self.active=request
        write_private_json(self.root/'requests'/f'{request}.json',dict(id=request,instance=self.instance,
            timestamp_ns=time.time_ns(),joints=joints,duration_s=duration_s))
        deadline=time.monotonic()+35
        try:
            while time.monotonic()<deadline:
                state=self.status()
                path=self.root/'results'/f'{request}.json'
                if path.exists():
                    result=read_private(path);require(result.get('instance')==self.instance,'wrong result instance')
                    require(result.get('id',request)==request,'wrong result request')
                    if not result.get('ok'):
                        if result.get('failure_kind')=='pre_dispatch_rejection' and result.get('executed') is False:
                            # Wait for the rejection's pending slot to be cleared before replanning.
                            if state['phase'] in ('IDLE','DONE') and state.get('request_id') is None:
                                raise PlanningRejected(result.get('reason','planning refused before dispatch'))
                            if state['phase']!='FAULT':
                                time.sleep(.05);continue
                        raise RuntimeError(result.get('reason','motion failed'))
                    require(state['phase']!='FAULT','motion faulted after receipt')
                    if state['phase']=='DONE' and state.get('request_id') is None:
                        return result['result']
                time.sleep(.05)
            raise TimeoutError('motion bridge response timeout')
        except PlanningRejected:
            raise
        except BaseException:
            self.hold();raise
        finally:self.active=None

    def _measure_idle_hold(self):
        """Observe a stationary gap between calls, without Stop/Start/rearming.

        This is deliberately not a cancellation API. Active motion or a fault is
        refused; piper_hold remains the hard-stop tool. Scene/payload safety must
        still be assessed visually before the agent's next normal motion call.
        """
        deadline=time.monotonic()+2;anchor=None;first=None;identity=None;samples=0;last=None
        while time.monotonic()<deadline:
            s=self.status()
            require(s['phase'] in ('IDLE','DONE') and s.get('request_id') is None,'pause requires completed motion; use hold for emergency stop')
            require(arming.read(self.session_root).get('armed') is True,'operator disarmed')
            f=s.get('feedback') or {};o=s.get('latest_policy_observation') or {}
            stamp=f.get('timestamp_ns');now=time.time_ns()
            require(type(stamp) is int and 0<=now-stamp<=300_000_000,'pause feedback stale')
            # Idle NO_ACTION policy polling is ~0.5 s. Rest is measured from
            # the fresh independent CAN feedback, not that slower camera poll.
            # The policy contributes identity only; dispatch keeps its existing
            # strict fresh-observation/anchor requirements in prepare()/stage().
            require(type(o.get('timestamp_ns')) is int and o['timestamp_ns']<=now,'pause policy identity unavailable')
            require(f.get('lifecycle_state')=='running','pause owner is not running')
            require(type(f.get('generation')) is int and f['generation']==o.get('generation') and bool(o.get('policy_session_id')),'pause owner identity missing')
            current=(f['generation'],o['policy_session_id'])
            require(identity in (None,current),'pause owner identity changed')
            identity=current;q=vector(f.get('q'))
            if anchor is None:anchor=q.copy();first=stamp
            require(max(abs(q-anchor))<=.0003,'pause measured drift; inspect state before further motion')
            require(last is None or stamp>=last,'pause feedback time reversed')
            if stamp!=last:samples+=1;last=stamp
            if stamp-first>=300_000_000 and samples>=4:
                return dict(held=True,resumable=True,measured_rest=True,samples=samples,
                    measured14=q.tolist(),hardware_stop_sent=False,automatic_rearm=False,
                    object_success=None,scene_safety_confirmed=False,
                    next_action='Observe fresh scene and continue only with a known safe recovery; normal motion guards remain active.')
            time.sleep(.05)
        raise TimeoutError('pause could not verify stable fresh feedback')

    def hold_current(self,mode='cancel'):
        """Map upstream rby1_hold to the PiPER owner without hiding limitations.

        Between completed calls PiPER already holds its target; confirm measured
        rest and retain that owner. Active cancellation still uses the qualified
        owner Stop, which requires operator recovery on this hardware.
        """
        require(mode in ('cancel','stop_command'),'unknown hold mode')
        if mode=='stop_command':return {**self.hold(),'resumable':False,'operator_recovery_required':True}
        try:
            s=self.status()
            if s['phase'] not in ('IDLE','DONE') or s.get('request_id') is not None:
                return {**self.hold(),'resumable':False,'operator_recovery_required':True}
            return self._measure_idle_hold()
        except BaseException:
            self.hold();raise

    def hold(self):
        write_private_json(self.root/'cancel.json',dict(instance=self.instance,timestamp_ns=time.time_ns()))
        return dict(stop_requested=True,physical_stop_confirmed=False)

    def close(self):
        if self.active:self.hold()


async def run(args):
    root=args.output.resolve();root.mkdir(mode=0o700,parents=True,exist_ok=False)
    for n in ('requests','results','plans'):(root/n).mkdir(mode=0o700)
    session=args.session_root.resolve();instance=uuid.uuid4().hex
    # Only the pre-existing pure wire contract is reused, never its GPT policy.
    sys.path[:0]=[str(args.runtime),'/home/ubuntu/code_junqi/GPT-as-Policy']
    from real_robot.piper.contracts import PiperContract
    from client_server.ws.model_server import PolicyServer,PolicyServerConfig
    token=args.token.read_text().strip();stop=StopOnlyHTTP(token)
    from .motion_profiles import TASK60,transport_options,LEFT_JAW_COMMAND_MAX_M
    limits=(dict(TASK60) if args.motion_profile=='task60' else dict(max_joint_displacement_rad=args.max_joint_step,
        max_joint_velocity_rad_s=.02,max_joint_acceleration_rad_s2=.05,terminal_dwell_s=2,
        max_eef_translation_m=.02,joint_tolerance_rad=args.joint_tolerance_rad,max_left_jaw_step_m=.01))
    limits['max_left_full_closure_start_m']=.020
    t=MotionTransport(PiperNoSendBackend(SimpleNamespace(backend_options={})),CollisionChecker(),
        arm_check=lambda:arming.read(session),stop_client=stop,lease_file=root/'lease.json',
        **transport_options(limits),allow_left_gripper=args.left_gripper)
    policy=ControlledPolicy(PiperContract.from_runtime(args.runtime,horizon=600,action_prefix=600,trajectory_mode=True),t,task=args.task,
        receipt_tail=ReceiptTail(args.journal,max_line_bytes=524288),operator_arm=None,owner_state=stop.state)
    server=PolicyServer(policy,PolicyServerConfig(host='127.0.0.1',port=args.port,compression=None))
    end=asyncio.Event();loop=asyncio.get_running_loop()
    for sig in (signal.SIGINT,signal.SIGTERM):loop.add_signal_handler(sig,end.set)
    handoff_times={}
    def mark(req,stage):
        handoff_times.setdefault(req["id"],[]).append(dict(stage=stage,monotonic_ns=time.monotonic_ns(),wall_ns=time.time_ns()))
    await server.start();watcher=asyncio.create_task(policy.watchdog());seen=set();pending=None;planning=None
    async def finish(req,ok,result=None,reason=None,failure_kind=None):
        await asyncio.to_thread(write_private_json,root/'results'/f'{req["id"]}.json',dict(instance=instance,id=req['id'],ok=ok,result=result,reason=reason,
            handoff_timing=handoff_times.pop(req["id"],[]),failure_kind=failure_kind,executed=False if failure_kind=='pre_dispatch_rejection' else None))
    async def prepare(req):
        mark(req,"prepare_begin")
        require(policy.latest is not None,'fresh formal policy observation unavailable; Prepare/Start by operator required')
        owner=await asyncio.to_thread(stop.state)
        require(owner['robot_status'].get('rax_motion',{}).get('enabled') is True,'formal owner lease extension missing')
        if args.left_gripper:
            require(owner['robot_status']['rax_motion'].get('left_gripper_dispatch') is True,'left gripper owner extension missing')
        require(owner['robot_status'].get('lifecycle_state')=='running','owner not in MODEL running state')
        mark(req,"owner_initial_state_checked")
        acknowledgement_deadline=time.monotonic()+1.5
        while owner['robot_status']['rax_motion'].get('active_request') is not None:
            require(time.monotonic()<acknowledgement_deadline,'owner completion acknowledgement timeout')
            await asyncio.sleep(.03)
            owner=await asyncio.to_thread(stop.state)
            require(owner['robot_status'].get('lifecycle_state')=='running','owner stopped while waiting for acknowledgement')
        mark(req,'owner_previous_action_acknowledged')
        anchor=policy.latest['q'].copy()
        mark(req,'path_preview_begin')
        if req.get('kind')=='waypoints':
            from services.piper.installed_settling import state_matches
            require(state_matches(anchor,req['start14']),'Multi-waypoint planning anchor drift')
            plan=await asyncio.to_thread(t.preview_waypoints,anchor,req['waypoints'],req['obstacles'],req.get('blend_indices',[]))
        elif req.get('kind')=='close_gripper':
            from services.piper.installed_settling import state_matches
            reference=list(req['joints']);reference[6]=anchor[6]
            require(state_matches(anchor,reference),'Closure anchor changed')
            target=list(anchor);target[6]=req['joints'][6]
            plan=await asyncio.to_thread(t.preview,anchor,target,req['duration_s'],full_closure=True)
        else:
            require('kind' not in req,'Unknown motion request kind')
            plan=await asyncio.to_thread(t.preview,anchor,req['joints'],req['duration_s'])
        mark(req,'path_preview_end')
        await asyncio.to_thread(write_private_json,root/'plans'/f'{req["id"]}.json',plan)
        mark(req,'plan_written_wait_fresh_observation')
        deadline=time.monotonic()+1.5
        while policy.latest is None or not 0<=time.time_ns()-policy.latest['timestamp_ns']<=100_000_000:
            require(time.monotonic()<deadline,'fresh observation unavailable after path planning')
            await asyncio.sleep(.01)
        mark(req,'fresh_observation_ready')
        t.stage(plan,policy.latest)
        mark(req,'staged_wait_policy_poll')
    async def publish_status():
        while not end.is_set():
            write_private_json(root/'state.json',dict(schema='rax.piper.motion_bridge.v1',timestamp_ns=time.time_ns(),
                instance=instance,session_root=str(session),phase=t.state,feedback=policy.feedback,
                latest_policy_observation=policy.latest,error=policy.last_error,
                observation_recovery=policy.timing_recovery,last_timing_event=policy.last_timing_event,request_id=None if pending is None else pending['id'],
                trajectory_progress=t.progress(),sdk_receipts=len(t.receipts),result=t.result,stop=t.stop_report,limits=limits,max_left_jaw_width_m=LEFT_JAW_COMMAND_MAX_M,gripper_motion=args.left_gripper,right_gripper_motion=False),durable=False)
            await asyncio.sleep(.03)
    publisher=asyncio.create_task(publish_status())
    try:
        while not end.is_set():
            if watcher.done() or publisher.done():
                raise RuntimeError('motion watchdog/status publisher exited')
            now=time.time_ns()
            cancel=root/'cancel.json'
            if cancel.exists():
                c=read_private(cancel);cancel.unlink()
                if c.get('instance')==instance and 0<=now-c.get('timestamp_ns',0)<=1_000_000_000:
                    was_active=t.state in ACTIVE;t.stop('RAX Hold requested')
                    if not was_active:await asyncio.to_thread(stop)
            if planning is not None and planning.done():
                try:planning.result()
                except Exception as exc:
                    rejected=t.state in ('IDLE','DONE')
                    if not rejected:t.stop('planning/staging failure: '+str(exc))
                    await finish(pending,False,reason=str(exc),failure_kind='pre_dispatch_rejection' if rejected else 'motion_fault');pending=None
                planning=None
            if pending and planning is None and t.state in ('DONE','FAULT'):
                await asyncio.to_thread(write_private_json,root/'plans'/f'{pending["id"]}-receipts.json',dict(receipts=t.receipts,result=t.result,stop=t.stop_report))
                await finish(pending,t.state=='DONE',result=t.result,reason=t.stop_report.get('reason'));pending=None
            for path in sorted((root/'requests').glob('*.json')):
                if path.name in seen:continue
                seen.add(path.name)
                if len(path.stem)!=32 or any(c not in '0123456789abcdef' for c in path.stem):continue
                req={'id':path.stem}
                try:
                    value=read_private(path)
                    require(value.get('id')==path.stem and value.get('instance')==instance,'request identity mismatch')
                    req=value
                    require(type(req.get('timestamp_ns')) is int and 0<=now-req['timestamp_ns']<=1_000_000_000,'expired motion request')
                    require(pending is None and t.state in ('IDLE','DONE'),'motion already active or faulted')
                    require(arming.read(session).get('armed') is True,'operator disarmed')
                    mark(req,'request_dequeued')
                    pending=req;planning=asyncio.create_task(prepare(req))
                except Exception as exc:await finish(req,False,reason=str(exc))
            await asyncio.sleep(.03)
    finally:
        if planning:planning.cancel()
        if t.state in ACTIVE:t.stop('motion bridge shutdown')
        policy.closed=True;watcher.cancel();publisher.cancel();await server.stop()
        write_private_json(root/'EXIT.json',dict(time_ns=time.time_ns(),phase=t.state,stop=t.stop_report))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('runtime','output','journal','token','session-root'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--task',required=True);p.add_argument('--port',type=int,default=19027)
    p.add_argument('--motion-profile',choices=('commissioning','task60'),default='commissioning')
    p.add_argument('--max-joint-step',type=float,default=.02)
    p.add_argument('--joint-tolerance-rad',type=float,default=.001)
    p.add_argument('--left-gripper',action='store_true')
    asyncio.run(run(p.parse_args()))


if __name__=='__main__':main()
