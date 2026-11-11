"""Read-once RAX context entry. File receipts are not motion/scene permission."""
import hashlib,json,os,time,fcntl
from pathlib import Path
from session_status import status

INITIAL=('TASK.md','MCP_SESSION.md','.agents/skills/robot-interface/SKILL.md',
         '.agents/skills/manual-robot-control/SKILL.md','memory/INDEX.md','skill-library/INDEX.md')

def _enter_unlocked(workspace=Path('.')):
    workspace=Path(workspace);path=workspace/'memory/CONTEXT_READS.json'
    ledger=json.loads(path.read_text()) if path.exists() else {'schema':'rax.context_reads.v1','files':{}}
    if ledger.get('schema')!='rax.context_reads.v1':raise ValueError('Invalid read ledger')
    changed={};current={}
    for name in INITIAL:
        p=workspace/name
        if not p.is_file():raise ValueError('Required context missing: '+name)
        data=p.read_bytes();digest=hashlib.sha256(data).hexdigest();current[name]=digest
        if ledger['files'].get(name)!=digest:changed[name]=data.decode()
    progress=workspace/'memory/PROGRESS.md'
    result={'entry':'initial_or_changed_references' if changed else 'resume',
        'read_now':changed,'unchanged_references':[n for n in INITIAL if n not in changed],
        'progress':progress.read_text() if progress.exists() else '', 'state':status(workspace),
        'next':'Continue the recorded next unfinished step. Read only relevant indexed experience or changed code. Check live status; acquire fresh views AFTER analysis. Do not replay startup, planning, segmentation or executed actions.',
        'motion_permit':False,'live_status_required':True}
    # Record only contents delivered by this helper, never task completion or scene facts.
    ledger.setdefault('files',{}).update(current)
    ledger.update(emitted_ns=time.time_ns())
    path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(ledger,indent=2));os.replace(tmp,path)
    return result

def enter(workspace=Path('.')):
    workspace=Path(workspace);memory=workspace/'memory';memory.mkdir(exist_ok=True)
    fd=os.open(memory/'.context.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        return _enter_unlocked(workspace)

if __name__=='__main__':print(json.dumps(enter(),ensure_ascii=False))
