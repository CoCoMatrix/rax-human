"""Small file-only handoff for separate Piper physical sessions; never sends actions."""
from __future__ import annotations
from collections import Counter
import json
from pathlib import Path
import re
import shutil
import time

MOTION = {'piper_move_pose', 'piper_move_joints', 'piper_gripper'}


def read_json(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else default


def checkpoint(run):
    """Derive historical facts from tool outcomes and saved frame metadata, not prose."""
    run = Path(run); session = run/'rax-session'; active = {}; actions = []; errors = []; frames = []; malformed = 0
    ledger = session/'mcp.jsonl'
    if ledger.exists():
        for line in ledger.read_text().splitlines():
            try: row = json.loads(line)
            except ValueError: malformed += 1; continue
            ident = row.get('call_id')
            if row.get('event') == 'mcp_request': active[ident] = row
            elif row.get('event') in ('mcp_result', 'mcp_error'):
                request = active.pop(ident, {})
                tool = request.get('tool', row.get('tool', row.get('operation')))
                item = {'call_id': ident, 'tool': tool, 'requested_at': request.get('time'), 'completed_at': row.get('time')}
                if row['event'] == 'mcp_error':
                    item.update(reason=row.get('reason'), control_uncertain=row.get('control_uncertain'),
                                no_action_executed=row.get('no_action_executed'))
                    errors.append(item); continue
                raw = row.get('result', {})
                try: value = json.loads(raw) if isinstance(raw, str) else raw
                except ValueError: malformed += 1; continue
                if not isinstance(value, dict): continue
                if tool in MOTION:
                    item['arguments'] = request.get('arguments', {})
                    item['result'] = {k: value[k] for k in ('executed','measured_arrival','sdk_completed_steps','owner_request_id','final14','object_success') if k in value}
                    actions.append(item)
                observed = value if tool == 'piper_observe' else (value.get('observation') or {})
                if re.fullmatch('[0-9a-f]{32}', str(observed.get('frame_id', ''))):
                    ident = observed['frame_id']; meta = read_json(session/'agent-workspace/runtime/frames'/ident/'meta.json', {})
                    frames.append({'frame_id': ident, 'tool_completed_at': row.get('time'),
                                   'captured_at_ns': meta.get('captured_at_ns'),
                                   'source_timestamps_ns': {k:v.get('source_timestamp_ns') for k,v in meta.get('cameras',{}).items()}})
    # Include an operator startup observation even if the model never made a tool call.
    frame_root = session/'agent-workspace/runtime/frames'
    known = {f['frame_id'] for f in frames}
    if frame_root.is_dir():
        for folder in frame_root.iterdir():
            if folder.is_symlink() or not re.fullmatch('[0-9a-f]{32}',folder.name) or folder.name in known: continue
            meta = read_json(folder/'meta.json', {})
            if meta.get('frame_id') == folder.name:
                frames.append({'frame_id':folder.name,'tool_completed_at':None,'captured_at_ns':meta.get('captured_at_ns'),
                               'source_timestamps_ns':{k:v.get('source_timestamp_ns') for k,v in meta.get('cameras',{}).items()}})
    frames.sort(key=lambda f:f.get('captured_at_ns') or 0)
    stop = read_json(run/'STOP-RESULT.json', {})
    result = read_json(run/'RESULT.json', {})
    life = read_json(session/'codex-lifecycle.json', {})
    return {'source_run': str(run.resolve()), 'historical_only': True,
            'current_pose_requires_fresh_feedback_and_images': True,
            'successful_actions': actions, 'action_counts': dict(Counter(x['tool'] for x in actions)),
            'tool_errors': errors, 'unresolved_motion_requests': [v for v in active.values() if v.get('tool') in MOTION],
            'malformed_ledger_rows': malformed, 'last_frames': frames[-3:],
            'stop': stop, 'agent_exit_code': result.get('agent_exit_code'),
            'recorded_elapsed_s': result.get('elapsed_s'), 'model_lifecycle_elapsed_s': life.get('elapsed_s'),
            'last_gripper_action': next((a for a in reversed(actions) if a['tool']=='piper_gripper'), None),
            'held_cup_possible': True,
            'task_success': 'Not inferred from process exit, jaw width, or arrival; inspect current scene and task criteria.'}


def validate_source(run, commissioning):
    run = Path(run).resolve(); commissioning = Path(commissioning).resolve()
    if not run.is_relative_to(commissioning) or run == commissioning:
        raise ValueError('Continuation must refer to an existing commissioning run in this repository')
    for rel in ('RESULT.json', 'STOP-RESULT.json', 'rax-session/mcp.jsonl', 'rax-session/agent-workspace/memory'):
        p = run/rel
        if not p.exists() or p.is_symlink(): raise ValueError('Missing or linked continuation record: '+rel)
    result = read_json(run/'RESULT.json')
    if not result.get('control_archived') or not result.get('sensor_archived'):
        raise ValueError('Continuation source has not finished archival; do not take over an active session')
    return run


def copy_regular_tree(source, destination):
    source = Path(source); destination = Path(destination)
    if source.is_symlink(): raise ValueError('Linked memory/code is not imported')
    size = 0; count = 0
    for item in source.rglob('*'):
        if item.is_symlink(): raise ValueError('Linked memory/code is not imported: '+str(item.relative_to(source)))
        if item.is_file(): size += item.stat().st_size; count += 1
        if size > 8*1024**2 or count > 1000: raise ValueError('Historical memory/code exceeds small handoff limit (8 MiB / 1000 files)')
    shutil.copytree(source, destination, dirs_exist_ok=True)


def copy_memory_evidence(source_workspace, destination, *, memory_dir=None):
    """Copy native evidence.json references unchanged; no summary or skill promotion.

    Existing workspace-copy plumbing only: missing evidence and conflicting bytes
    are reported instead of leaving links that the next sandbox cannot open.
    Calling with identical roots performs a read-only reference check.
    """
    import hashlib
    source=Path(source_workspace).resolve();destination=Path(destination).resolve()
    memory=Path(memory_dir) if memory_dir is not None else source/'memory'
    refs=set()
    notes=[memory/name for name in ('INDEX.md','TIPS.md','PROGRESS.md','RUNS.md','OPERATOR_NOTES.md')]
    notes+=list(memory.glob('skills/**/SKILL.md'))
    for note in notes:
        if not note.exists():continue
        if note.is_symlink():raise ValueError('Linked memory note is not imported')
        refs.update(re.findall(r'(?:references|audit|runtime/frames)/[A-Za-z0-9_./-]+\.(?:md|jsonl|json|png|jpg)',note.read_text()))
    for evidence in sorted(memory.glob('skills/**/evidence.json')):
        if evidence.is_symlink():raise ValueError('Linked memory evidence is not imported')
        data=read_json(evidence)
        if not isinstance(data,dict) or not isinstance(data.get('examples'),list):
            raise ValueError('Malformed native evidence: '+str(evidence))
        for row in data['examples']:
            if not isinstance(row,dict) or not isinstance(row.get('frames',[]),list):
                raise ValueError('Malformed native evidence example')
            refs.update(row.get('frames',[]))
            if row.get('complete_ledger'):refs.add(row['complete_ledger'])
    copies=[]
    for name in sorted(refs):
        if not isinstance(name,str):raise ValueError('Evidence reference must be a path')
        rel=Path(name)
        if (rel.is_absolute() or '..' in rel.parts or not rel.parts
                or rel.parts[0] not in ('runtime','audit','references')):
            raise ValueError('Evidence must be a workspace-relative frame/audit/reference: '+name)
        src=source/rel;dst=destination/rel
        if any(p.is_symlink() for p in [src,*src.parents] if p!=source and p.is_relative_to(source)):
            raise ValueError('Linked source evidence: '+name)
        if not src.is_file():raise ValueError('Missing memory evidence: '+name)
        if not dst.resolve().is_relative_to(destination) or any(p.is_symlink() for p in [dst,*dst.parents] if p!=destination and p.is_relative_to(destination)):
            raise ValueError('Linked destination evidence: '+name)
        if dst.exists() and hashlib.sha256(src.read_bytes()).digest()!=hashlib.sha256(dst.read_bytes()).digest():
            raise ValueError('Memory evidence conflict: '+name)
        copies.append((src,dst))
    # Validate every path before copying anything.
    for src,dst in copies:
        if src!=dst and not dst.exists():
            dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    return {'references_checked':len(copies),'references':[str(dst.relative_to(destination)) for _,dst in copies]}


def import_memory(memory_dir, workspace):
    """Preserve RAX's native memory and its linked evidence in a new workspace."""
    memory=Path(memory_dir)
    if memory.is_symlink() or any(p.is_symlink() for p in memory.rglob('*')):
        raise ValueError('Linked memory is not imported')
    report=copy_memory_evidence(memory.parent,workspace,memory_dir=memory)
    copy_regular_tree(memory,Path(workspace)/'memory')
    for item in [Path(workspace)/'memory',*(Path(workspace)/'memory').rglob('*')]:
        item.chmod(item.stat().st_mode | (0o700 if item.is_dir() else 0o600))
    return report


def describe(data):
    last = data['last_gripper_action']
    jaw = (data.get('stop', {}).get('rest') or {}).get('measured14')
    return '\n'.join([
        'Historical session checkpoint — not current robot state.',
        'Source: '+data['source_run'],
        'Recorded wall time seconds: '+str(data['recorded_elapsed_s'])+'; model lifecycle seconds: '+str(data['model_lifecycle_elapsed_s'])+'. Do not infer remaining budget from old prose.',
        'Completed tool actions: '+json.dumps(data['action_counts'])+'. Full targets/results are in the adjacent checkpoint JSON; they are records, never a replay queue.',
        'Last gripper tool: '+json.dumps(last, ensure_ascii=False),
        'Stop measured14 (historical): '+json.dumps(jaw),
        'Last saved frame IDs (historical): '+', '.join(f['frame_id'] for f in data['last_frames']),
        'A held cup remains possible. First inspect fresh feedback and all three views in the NEW physical session. Until current state is known, do not initialize/open the jaw, home, regrasp, or replay old coordinates. If the cup is still held, continue only the remaining placement. If current evidence clearly shows an empty gripper and a supported cup, complete the authorized task with freshly planned grasp/placement; old targets are not replay commands.',
        'Continue the remaining placement only if the current scene supports it. A jaw command or process exit does not prove grasp, support, release, or task success.',
        'On unknown control or unresolved outcome, Stop and report; no automatic replay or automatic release.',
    ])+'\n'


def import_continuation(source, workspace, audit=None):
    source = Path(source); workspace = Path(workspace); references = workspace/'references/prior-session'
    references.mkdir(parents=True, exist_ok=True)
    old_workspace = source/'rax-session/agent-workspace'
    import_memory(old_workspace/'memory', workspace)
    if (old_workspace/'code').is_dir(): copy_regular_tree(old_workspace/'code', references/'code')
    progress = workspace/'memory/PROGRESS.md'
    if progress.exists(): shutil.copy2(progress, references/'historical-PROGRESS.md')
    data = checkpoint(source)
    (references/'checkpoint.json').write_text(json.dumps(data, indent=2))
    for row in data['last_frames']:
        ident = row['frame_id']; src = old_workspace/'runtime/frames'/ident
        for name in ('meta.json','head.png','left.png','right.png'):
            file = src/name
            if file.is_file() and not file.is_symlink():
                if file.stat().st_size > 5*1024**2: raise ValueError('Historical preview exceeds 5 MiB; keep original evidence in place')
                dst = references/'frames'/ident/name; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(file,dst)
    correction = ''
    if audit and Path(audit).is_file():
        value = read_json(audit)
        if value.get('source_run') != str(source.resolve()): raise ValueError('Audit refers to a different prior run')
        shutil.copy2(audit, references/'independent-audit.json')
        correction = ('Independent audit of the PRIOR run: '+json.dumps(value.get('outcome',{}),ensure_ascii=False)+
                      '\nIt supports cup lift/retention, not completed placement; it is not a current-scene measurement.\n')
    text = describe(data)+correction+'\nRead references/prior-session/checkpoint.json and the saved prior frames only as historical evidence. Current startup state is separately saved in references/current-session-start.json.\n'
    progress.write_text(text)
    (workspace/'memory/SESSION_HANDOFF.md').write_text(text)
    return data


def save_handoff(out, result):
    """After children exit/Stop, persist a machine-derived checkpoint even on timeout."""
    out = Path(out); data = checkpoint(out)
    data.update(generated_at_ns=time.time_ns(), stop=result.get('stop', read_json(out/'STOP-RESULT.json',{})),
                agent_exit_code=result.get('agent_exit_code'), recorded_elapsed_s=result.get('elapsed_s'),
                process_exit_codes=result.get('process_exit_codes'),
                shutdown_errors={k:v for k,v in result.items() if 'error' in k},
                continued_from=result.get('continued_from'),
                no_automatic_release=True)
    (out/'HANDOFF.json').write_text(json.dumps(data, indent=2))
    (out/'HANDOFF.md').write_text(describe(data)+'\nStop/disarm and shutdown evidence: HANDOFF.json, STOP-RESULT.json, RESULT.json.\n')
    return data
