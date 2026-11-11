"""Offline native Codex login probe. No RAX session, MCP, SDK, or motion startup.

Use the server's existing CLI login; never export its tokens into the agent
workspace. This verifies account/model/image connectivity, not a live provider
migration of the RAX sandbox.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def probe(output,effort,prompt,images=(),proxy=None):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    codex=shutil.which('codex')
    if not codex:raise RuntimeError('Codex CLI not installed')
    cmd=[codex,'exec','--ignore-user-config','--ignore-rules','--ephemeral',
         '--skip-git-repo-check','--sandbox','read-only','--json',
         '-m','gpt-6-astra','-c',f'model_reasoning_effort="{effort}"',
         '-c','web_search="disabled"','-o',str(output/'answer.txt')]
    for feature in ('shell_tool','unified_exec','plugins','apps','browser_use','computer_use','view_image','multi_agent'):
        cmd+=['--disable',feature]
    identity=[]
    for image in images:
        path=Path(image).resolve(strict=True);cmd+=['--image',str(path)]
        identity.append({'name':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    # End the variadic image argument explicitly before the prompt.
    cmd+=['--',prompt]
    env={k:v for k,v in os.environ.items() if k not in ('OPENAI_API_KEY','OPENAI_BASE_URL')}
    if proxy:env.update(HTTPS_PROXY=proxy,HTTP_PROXY=proxy)
    start=time.monotonic();started=time.time();timed_out=False
    with (output/'events.jsonl').open('w') as out,(output/'stderr.txt').open('w') as err:
        try:
            result=subprocess.run(cmd,cwd=output,env=env,stdin=subprocess.DEVNULL,stdout=out,stderr=err,timeout=180)
            code=result.returncode
        except subprocess.TimeoutExpired:code=None;timed_out=True
    events=[]
    for line in (output/'events.jsonl').read_text().splitlines():
        try:events.append(json.loads(line))
        except ValueError:continue
    tools=[e.get('item',{}).get('type') for e in events if e.get('item',{}).get('type') in
           ('command_execution','file_change','mcp_tool_call','web_search')]
    completed=[e for e in events if e.get('type')=='turn.completed']
    answer=(output/'answer.txt').read_text() if (output/'answer.txt').exists() else ''
    report={'ok':code==0 and bool(completed) and bool(answer.strip()) and not tools,
        'provider':'native_codex_chatgpt_login','model':'gpt-6-astra','effort':effort,
        'started_at':started,'duration_s':time.monotonic()-start,'exit_code':code,'timed_out':timed_out,
        'tool_events':tools,'images':identity,'usage':completed[-1].get('usage') if completed else None,
        'answer':answer,'robot_tools_available':False,'hardware_actions_started':False}
    (output/'RESULT.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--image',type=Path,action='append',default=[])
    p.add_argument('--effort',choices=['high','medium'],required=True);p.add_argument('--proxy')
    p.add_argument('--prompt',default='Offline connectivity test. Use no tools. Reply CODEX_ACCOUNT_OK.')
    a=p.parse_args();r=probe(a.output,a.effort,a.prompt,a.image,a.proxy)
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['ok'] else 1)
