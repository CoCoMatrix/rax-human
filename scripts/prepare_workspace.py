"""Create a NEW archival source + writable copy, never a controller or hardware qualification."""
import argparse,hashlib,json,re,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def prepare(source,name,*,apply=False,speed='standard'):
    source=Path(source).resolve()
    if not re.fullmatch(r'[a-z0-9][a-z0-9_/-]*',name) or '..' in name.split('/'):
        raise ValueError('Use a new relative lowercase task/version name')
    code=source/'code' if (source/'code').is_dir() else source
    if not any(code.rglob('*.py')):raise ValueError('No candidate Python code')
    dest=ROOT/'workspaces'/name;frozen=ROOT/'versions'/name
    if dest.exists() or frozen.exists():raise FileExistsError('Never overwrite working or frozen versions')
    report={'source':str(code),'workspace':str(dest),'archive':str(frozen),'hardware_started':False,'qualification':'imported candidate; source evidence governs qualification','apply':apply}
    if not apply:return report
    for p in code.rglob('*'):
        if p.is_symlink():raise ValueError('Candidate contains symlinks')
    frozen.mkdir(parents=True)
    shutil.copytree(code,frozen/'code',ignore=shutil.ignore_patterns('.git','__pycache__','*.pyc'))
    manifest={str(p.relative_to(frozen)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (frozen/'code').rglob('*') if p.is_file()}
    (frozen/'SHA256SUMS.json').write_text(json.dumps(manifest,indent=2))
    dest.mkdir(parents=True);shutil.copytree(frozen/'code',dest/'code')
    for d in ('memory','calibration'):(dest/d).mkdir()
    if (source/'memory').is_dir():
        from piper_handoff import import_memory
        import_memory(source/'memory',dest)
    else:
        (dest/'memory/INDEX.md').write_text('# Imported candidate\nNo inherited robot state. Acquire fresh observations.\n')
        (dest/'memory/PROGRESS.md').write_text('Imported code only. Not rehearsed in this new workspace.\n')
    if (source/'calibration').is_dir():shutil.copytree(source/'calibration',dest/'calibration',dirs_exist_ok=True)
    if (source/'run_no_agent.py').is_file():
        shutil.copy2(source/'run_no_agent.py',dest/'run_no_agent.py')
    subprocess.run(['git','init','-q',str(dest/'code')],check=True)
    subprocess.run(['git','-C',str(dest/'code'),'add','.'],check=True)
    subprocess.run(['git','-C',str(dest/'code'),'-c','user.name=RAX import','-c','user.email=import@localhost','commit','-qm','Import candidate source; no new hardware qualification'],check=True)
    commit=subprocess.check_output(['git','-C',str(dest/'code'),'rev-parse','HEAD'],text=True).strip()
    demo=json.loads((source/'DEMO.json').read_text()) if (source/'DEMO.json').exists() else {}
    metadata={'code_commit':commit,'candidate_version':demo.get('version',name),'original_code_commit':demo.get('original_code_commit'),'writable':True,'source_version':str(frozen.relative_to(ROOT)),
      'source_manifest_sha256':hashlib.sha256((frozen/'SHA256SUMS.json').read_bytes()).hexdigest(),
      'motion_speed_profile':speed,'qualification':report['qualification'],'source':str(source)}
    (dest/'WORKSPACE.json').write_text(json.dumps(metadata,indent=2))
    (dest/'HANDOFF.md').write_text('Imported candidate. Preserve source identity; new observations and authorization required. No inherited current pose.\n')
    for p in frozen.rglob('*'):p.chmod(0o555 if p.is_dir() else 0o444)
    frozen.chmod(0o555)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--name',required=True);p.add_argument('--speed-profile',choices=['standard','faster'],default='standard');p.add_argument('--apply',action='store_true');a=p.parse_args()
    print(json.dumps(prepare(a.source,a.name,apply=a.apply,speed=a.speed_profile),indent=2))
