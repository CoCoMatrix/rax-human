"""Seed concise, operator-curated RAX task memory without replacing agent notes."""
from pathlib import Path

FILES = ('INDEX.md', 'TIPS.md', 'PROGRESS.md',
         'skills/cup-wall-pinch/SKILL.md', 'skills/cup-wall-pinch/evidence.json')

def seed_memory(workspace, source):
    source = Path(source)
    if not source.is_dir():
        return []
    written = []
    for name in FILES:
        p = source / name
        if p.is_symlink() or not p.is_file():
            raise ValueError('Curated memory file missing or linked: ' + name)
        data = p.read_bytes()
        if name == 'PROGRESS.md':
            data = ("# Current session\n\nCurrent state is output/runs/current-round/ROUND.json and memory/AUTO_STATE.json when present.\n"
                    "A new session does not inherit a pose or permission from historical notes.\n"
                    "Reconcile receipts and acquire fresh observations before motion.\n").encode()
        if len(data) > 10000:
            raise ValueError('Curated memory must remain concise: ' + name)
        dst = Path(workspace) / 'memory' / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        try:
            with dst.open('xb') as stream:
                stream.write(data)
        except FileExistsError:
            continue
        written.append(name)
    return written

# Historical task notes survive workbench session replacement; live state never does.
def persist_memory(workspace, destination, run_id):
    import hashlib, json, uuid
    memory=Path(workspace)/'memory'
    if not memory.is_dir(): return None
    files={}
    for p in sorted(memory.rglob('*')):
        rel=p.relative_to(memory)
        if 'prior-attempt' in rel.parts or p.is_symlink() or not p.is_file(): continue
        if p.suffix not in ('.md','.json') or p.stat().st_size>10000: continue
        if len(files)>=64: break
        files[str(rel)]=p.read_bytes()
    if not files:return None
    destination=Path(destination); ident=uuid.uuid4().hex
    folder=destination/ident;folder.mkdir(parents=True)
    manifest={'run':run_id,'historical_only':True,'files':{}}
    for name,data in files.items():
        p=folder/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        manifest['files'][name]=hashlib.sha256(data).hexdigest()
    (folder/'MANIFEST.json').write_text(json.dumps(manifest,indent=2))
    temp=destination/(ident+'.tmp');temp.write_text(json.dumps({'id':ident}));temp.replace(destination/'LATEST.json')
    return manifest

def restore_history(workspace, source):
    import hashlib,json,re
    source=Path(source);pointer=source/'LATEST.json'
    if not pointer.is_file():return []
    ident=json.loads(pointer.read_text())['id']
    if not re.fullmatch('[0-9a-f]{32}',ident):raise ValueError('Invalid memory snapshot identity')
    folder=source/ident; manifest=json.loads((folder/'MANIFEST.json').read_text());written=[]
    for name,digest in manifest['files'].items():
        rel=Path(name)
        if rel.is_absolute() or '..' in rel.parts:raise ValueError('Memory path escapes snapshot')
        p=folder/rel
        if p.is_symlink() or not p.resolve().is_relative_to(folder.resolve()):raise ValueError('Linked memory refused')
        data=p.read_bytes()
        if len(data)>10000 or hashlib.sha256(data).hexdigest()!=digest:raise ValueError('Memory identity mismatch')
        dst=Path(workspace)/'memory/prior-attempt'/rel;dst.parent.mkdir(parents=True,exist_ok=True)
        dst.write_bytes(data);written.append(name)
    index=Path(workspace)/'memory/INDEX.md'
    text=index.read_text() if index.exists() else '# Task memory\n'
    link='\nPrevious attempt notes: [prior-attempt/TIPS.md](prior-attempt/TIPS.md), if present; retrieve only matching lessons. All prior-attempt state and replies are HISTORICAL, never live permissions.\n'
    if 'prior-attempt/' not in text:index.write_text(text+link)
    return written
