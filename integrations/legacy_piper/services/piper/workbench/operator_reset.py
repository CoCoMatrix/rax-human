"""Explicit operator release through the existing RAX MCP and sole owner.

No model call, SDK, CAN connection, fault clear, or unreviewed arm target.
"""
import json, os, subprocess, time
from pathlib import Path

def jaw_steps(ctx, target, on_receipt=None):
    if target not in (0.,.019):raise ValueError("Only empty-jaw close or supported release allowed")
    status=ctx.value('piper_status')
    q=list(status['measured_state']['values'])
    initial=q[:]; receipts=[]
    while abs(q[6]-target) > .001:
        width=min(target,q[6]+.009) if target>q[6] else max(target,q[6]-.009)
        receipt=ctx.value('piper_gripper',side='left',width_m=width)
        receipts.append(receipt)
        if on_receipt:on_receipt({'initial14':initial,'receipts':receipts[:],
            'executed':receipt.get('executed'),'measured_arrival':receipt.get('measured_arrival'),
            'measured14':receipt.get('final14'),'model_started':False,'arm_motion_requested':False})
        if receipt.get('executed') is not True or receipt.get('measured_arrival') is not True:
            raise RuntimeError('Jaw reset outcome not established; no retry')
        q=list(receipt['final14'])
        if max(abs(q[i]-initial[i]) for i in list(range(6))+list(range(7,14))) > .003:
            raise RuntimeError('Arm/right side changed during jaw-only reset')
        if len(receipts)>3: raise RuntimeError('Jaw reset did not converge; no retry')
    return {'passed':True,'initial14':initial,'measured14':q,'receipts':receipts,
            'model_started':False,'arm_motion_requested':False,'object_success':None}

def release_steps(ctx, target=.019, on_receipt=None):
    if target != .019:raise ValueError("Release target must be19mm")
    return jaw_steps(ctx,target,on_receipt)

def run(core,epoch,body,*,close_empty=False):
    from .common import ROOT, PYTHON, read, write
    from services import arming
    from services.controller.config import Configuration
    from harness.codex_cli import launcher
    from harness.codex_cli.session import Context,wait_frontend
    from services.piper.flow_contract import preflight
    from services.piper.motion_bridge import MotionClient
    if close_empty and body.get('empty_jaw_confirmed') is not True:
        raise ValueError('Empty jaw must be explicitly confirmed')
    if not close_empty and body.get('release_current_object') is not True:
        raise ValueError('Explicit release_current_object confirmation required')
    if core.agent and core.agent.poll() is None:
        raise ValueError('An agent is still active; no second motion producer')
    required_mode='home' if close_empty else 'hold-current'
    if read(core.run/'CONNECTED.json',{}).get('prepare_mode')!=required_mode:
        raise ValueError('Release requires a new hold-current connection')
    if (core.run/'TASK-RECORD.json').exists():
        raise ValueError('Preserve completed run; reconnect hold-current first')
    core._fresh(); core._prepare(epoch); core._check_epoch(epoch)
    target=0. if close_empty else .019
    result_file='RESET-CLOSE-RESULT.json' if close_empty else 'RESET-OPEN-RESULT.json'
    config=core._configure(f'Explicit operator jaw reset to{target}m after confirmed Prepare. No model. No task success.',minutes=1)
    core.task['mode']='operator_reset'
    write(core.run/'TASK-RECORD.json',core.task)
    deployed,socket,host=launcher.prepare(config)
    server=None;result=None
    try:
        with (core.run/'operator-reset-frontend.log').open('x') as log:
            server=subprocess.Popen([str(PYTHON),'-m','services.controller.frontend','--config',str(core.run/'operator-input.json'),'--socket',str(socket),'--started-at',str(time.time())],cwd=ROOT,env={**os.environ,'PYTHONPATH':str(ROOT)},stdout=log,stderr=log)
        wait_frontend(server,socket)
        core._fresh();core._check_epoch(epoch)
        # Arming is owned by the existing trusted operator workbench, never the agent.
        core.owner().command('start')
        deadline=time.monotonic()+12
        while MotionClient(core.live/'bridge',config.root).status().get('latest_policy_observation') is None:
            core._check_epoch(epoch)
            if time.monotonic()>deadline:raise RuntimeError('Reset policy feedback missing')
            time.sleep(.1)
        arming.write(config.root,True,minutes=1,reason='operator explicitly authorized jaw-only reset')
        core.phase='running';core.agent=None
        with Context(str(socket)) as ctx:
            report=preflight(config,ctx,'explore')
            if not report['ready']:raise ValueError('Reset MCP preflight refused')
            result=jaw_steps(ctx,target,on_receipt=lambda r:write(core.run/result_file,r))
            result.update(executed=bool(result['receipts']),measured_arrival=True)
            write(core.run/result_file,result)
            try:result['hold']=ctx.value('piper_hold',mode='cancel')
            except Exception as exc:
                result['post_release_check_error']=str(exc)
                write(core.run/result_file,result)
                raise
        result['operator_text']=body.get('operator_text','')
        write(core.run/result_file,result)
        core.phase='held'
        core.event('operator_reset','已通过正式MCP完成夹爪初始设置；关节保持；没有调用模型',measured14=result['measured14'])
        return result
    finally:
        core.phase='held'
        arming.write(config.root,False,reason='operator reset finished')
        try:core.owner().command('stop')
        finally:
            if server and server.poll() is None:
                server.terminate()
                try:server.wait(timeout=10)
                except subprocess.TimeoutExpired:server.kill();server.wait(timeout=5)
