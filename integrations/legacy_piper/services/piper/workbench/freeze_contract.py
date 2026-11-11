"""Freeze only the exact controller that completed an evidenced full round."""
import ast,json,re
from pathlib import Path

def entry_source(source,entrypoint):
    if not isinstance(entrypoint,str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*:main',entrypoint):
        raise ValueError('Explicit module:main entry required')
    path=Path(source)/(entrypoint.split(':')[0]+'.py')
    tree=ast.parse(path.read_text())
    fn=next((n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main'),None)
    if fn is None or [a.arg for a in fn.args.args]!=['ctx'] or fn.args.vararg or fn.args.kwarg:
        raise ValueError('Full-round entry must have signature main(ctx)')
    return path

def completion_evidence(workspace,digest):
    workspace=Path(workspace).resolve()
    record=workspace/'output/round-result.json'
    if not record.is_file():raise ValueError('Complete the exploration round before freezing')
    result=json.loads(record.read_text())
    if not all(result.get(k) is True for k in ['full_round','task_success','retreat_verified']):
        raise ValueError('Full round, placement and retreat evidence required')
    if result.get('controller_sha256')!=digest:raise ValueError('Successful round used different source/config')
    if result.get('mixed_versions') or result.get('source_revisions'):
        raise ValueError('Mixed-version exploration is not an unchanged candidate test; consolidate and run a fresh full round')
    actions=[x['action'] for x in result.get('executed_actions',[])]
    cursor=0
    for action in ['approach','close','trial_lift','carry','lower','release','retreat']:
        try:cursor=actions.index(action,cursor)+1
        except ValueError:raise ValueError('Missing ordered full-round motion evidence: '+action)
    frames=result.get('checkpoint_frames',[])
    if len(set(frames))<2:raise ValueError('At least two final camera views required')
    for name in frames:
        p=(workspace/name).resolve()
        if not p.is_relative_to(workspace) or not p.is_file() or not p.stat().st_size:
            raise ValueError('Final frame missing or outside workspace')
    return result


def verify_receipts(workspace,result,digest):
    """Match candidate reports to the trusted host MCP journal, in execution order."""
    workspace=Path(workspace).resolve();audit=workspace.parent/'mcp.jsonl'
    if not audit.is_file():raise ValueError('Trusted MCP execution journal missing')
    trusted=[]
    for line in audit.read_text().splitlines():
        try:row=json.loads(line)
        except ValueError:continue
        if row.get('event')!='mcp_result':continue
        receipt=row.get('result',{})
        if isinstance(receipt,str):
            try:receipt=json.loads(receipt)
            except ValueError:continue
        if isinstance(receipt,dict) and receipt.get('executed') is True and receipt.get('measured_arrival') is True:
            trusted.append((row.get('tool'),receipt))
    allowed={'open':'piper_gripper','close':'piper_gripper','release':'piper_gripper',
             'approach':'piper_move_joints','adjust':'piper_move_eef','trial_lift':'piper_move_eef',
             'carry':'piper_move_eef','lower':'piper_move_eef','retreat':'piper_move_eef'}
    cursor=0;seen=set()
    for entry in result.get('executed_actions',[]):
        if entry.get('controller_sha256')!=digest:raise ValueError('Action source identity differs')
        name=entry.get('report','');path=(workspace/name).resolve()
        if not name or not path.is_relative_to(workspace) or path in seen or not path.is_file():
            raise ValueError('Action report missing, reused or outside workspace')
        seen.add(path);report=json.loads(path.read_text());action=entry['action']
        if report.get('action')!=action or report.get('executed') is not True or report.get('measured_arrival') is not True:
            raise ValueError('Action report does not establish measured execution')
        receipt=report.get('motion_receipt')
        for index in range(cursor,len(trusted)):
            if trusted[index]==(allowed.get(action),receipt):cursor=index+1;break
        else:raise ValueError('Action receipt absent from ordered trusted MCP journal')
    return {'verified_actions':len(seen),'trusted_receipts':len(trusted)}
