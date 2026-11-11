"""Native RAX main-Agent software continuation after physical session shutdown.

No robot frontend, device socket, auxiliary vision, task deadline or task call cap.
The existing model relay/sandbox/CLI is reused; this module never writes controller code.
"""
import fcntl
import json
from pathlib import Path
import shutil
import signal
import subprocess
import time

from services.controller.config import Configuration
from services.json_io import write_private_json
from .credentials import load_credentials
from .launcher import codex_config, private_socket, sandbox_command, stop_process
from .relay import ResponsesRelay


PROMPT = '''CURRENT STAGE: SOFTWARE CLOSEOUT ONLY. Hardware has stopped and is NOT
connected to this continuation. Do not call robot/Gemini tools, reacquire a scene,
run a physical function or restart exploration. Older motion instructions/history
are reference evidence only. Read SOFTWARE_CLOSEOUT.md once, the latest progress
and matching experience. You are the RAX main Agent, responsible for experience
refinement, useful fn(ctx, inputs) generation/repair, relevant offline verification,
README and a local git checkpoint. Existing imported Mac candidate code may be
retained after review with provenance. Do the actual file work and verification,
not just a plan. Preserve original physical results and mark untested functions.
This phase is independent of the task's elapsed time and model-call quota: no task
budget is carried forward and no fixed closeout time/call limit is configured.
Use native compaction and concise progress. Finish once the requested artifacts and
factual report are saved. On a real blocker or API error save partial and report;
no unbounded retry loop. Mac supervises and archives; it does not author for you.
'''


def eligible(result):
    if 'KeyboardInterrupt' in result.get('error',''):
        return False, 'explicit task interruption; no automatic model restart'
    stop=result.get('stop') or {}
    if not (stop.get('disarmed') is True and (stop.get('stop_acknowledged') or {}).get('ok') is True
            and (stop.get('rest') or {}).get('measured_rest') is True and not stop.get('errors')):
        return False, 'native stop/disarm/rest not confirmed'
    exits=result.get('process_exit_codes') or {}
    if not exits or any(code is None for code in exits.values()):
        return False, 'task children have not all exited'
    return True, 'stopped physical session'


def prepare(parent, result):
    """Only file preparation; never opens hardware or calls a model."""
    parent=Path(parent)
    allowed,why=eligible(result)
    if not allowed:raise RuntimeError(why)
    cfg=Configuration.load(parent/'configuration.json')
    root=parent/'software-closeout'
    root.mkdir(mode=0o700)  # No overwrite or implicit restart of an earlier closeout.
    workspace=parent/'agent-workspace'
    home=root/'codex-home'
    shutil.copytree(parent/'codex-home', home, symlinks=True)
    (home/'config.toml').write_text(codex_config(cfg,offline=True))
    # Preserve the task-stage working code/memory; neither is overwritten by the host.
    for name in ('code','memory'):
        shutil.copytree(workspace/name,root/'task-checkpoint'/name,symlinks=True)
    report={'phase':'software_closeout','task_root':str(parent),'workspace':str(workspace),
            'robot_connected':False,'task_budget_inherited':False,
            'session_seconds':None,'max_model_calls':None,'status':'prepared',
            'native_resume':any((home/'sessions').rglob('*.jsonl'))}
    write_private_json(root/'phase.json',report)
    (workspace/'references').mkdir(exist_ok=True)
    write_private_json(workspace/'references/physical-session-result.json',result)
    # New future session only, after task shutdown. No edits to an ongoing workspace.
    (workspace/'SOFTWARE_CLOSEOUT.md').write_text(PROMPT+'''
Follow the software deliverables in AGENT_OWNERSHIP.md. Restore no old budget or
motion phase on compaction. Task-stage code/memory was backed up by the host under
software-closeout/task-checkpoint; it is not a new verified controller.
Record software phase time/model calls separately from physical function timing.
Saved task evidence can be inspected; no new robot observations are available.
Keep an unresolved fact unresolved instead of fabricating measurements.
Run offline checks without constructing a live Context; preserve function interfaces
for the next authorized rehearsal. Save output/closeout/SUMMARY.json with stage,
status (partial/candidate_offline_checked/function_rehearsed), source identities,
actual code commit/hash, commands/exit codes/logs, physical outcomes/assistance,
remaining unverified functions and next step. A candidate commit does not promote
it to a hardware-qualified skill. Library publication requiring unavailable MCP is
pending, not successful. End your turn when done, or with a concrete blocker.
''')
    return cfg,root,workspace,home,report


def run(parent, result):
    """Explicit independent phase; automatic only after the supervisor's cleanup."""
    parent=Path(parent)
    # Original RAX launcher must be gone; never race an active conversation.
    with (parent/'launch.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        cfg,root,workspace,home,report=prepare(parent,result)
        settings=json.loads((parent/'codex-relay.json').read_text())
        key,base=load_credentials(settings['key_file'],settings['base_url'])
        sock=private_socket(root,'.model')
        cmd=sandbox_command(cfg,None,sock,prompt=PROMPT,resume=report['native_resume'],
            agent_runtime=settings.get('agent_runtime'),offline=True,workspace=workspace,home=home)
        relay=process=None;started=time.monotonic();code=None;interrupted=False
        def stop(signum,_):
            nonlocal interrupted
            interrupted=True
            raise KeyboardInterrupt
        old={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            relay=ResponsesRelay(sock,key,base,settings['primary_model'],settings['primary_effort'],
                root,None,proxy_url=settings.get('model_http_proxy',settings.get('http_proxy')))
            relay.start()
            report.update(status='running',task_budget_inherited=False)
            write_private_json(root/'phase.json',report)
            with (root/'codex.jsonl').open('x') as log:
                process=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
                # Deliberately no task timer or max-call deadline in this phase.
                code=process.wait()
        except KeyboardInterrupt:
            report['status']='interrupted'
        except Exception as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        finally:
            for sig in old:signal.signal(sig,signal.SIG_IGN)
            try:
                report['agent_stop']=stop_process(process,grace=5,group=True)
                if relay:relay.close()
                report.update(elapsed_s=time.monotonic()-started,exit_code=code,
                              model_calls=relay.calls if relay else 0)
                summary=workspace/'output/closeout/SUMMARY.json'
                if code==0:
                    try:
                        report['agent_summary']=json.loads(summary.read_text())
                        report['status']='reported'  # Inspect evidence; this is NOT hardware qualification.
                    except (OSError,ValueError):report['status']='missing_or_invalid_report'
                elif not interrupted and 'error' not in report:report['status']='failed'
                write_private_json(root/'phase.json',report)
            finally:
                for sig,handler in old.items():signal.signal(sig,handler)
        return report
