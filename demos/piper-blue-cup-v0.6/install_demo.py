"""Copy demo into a NEW workspace of an already adapted runtime. No hardware calls."""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path
from verify_demo import HERE,verify


def install(root,name='blue-cup/v0.6-continuity-demo',apply=False):
    root=Path(root).resolve();meta=json.loads((HERE/'DEMO.json').read_text())
    verify(runtime=root)
    base=root/meta['original_workspace'];dest=root/'workspaces'/name
    if Path(name).is_absolute() or '..' in Path(name).parts or not name:raise ValueError('Expected a new workspace-relative name')
    if dest.exists() or dest.is_symlink():raise ValueError('Destination exists; never overwrite a controller')
    if not dest.resolve().is_relative_to((root/'workspaces').resolve()):raise ValueError('Destination outside workspaces')
    for path in ('scripts/piper_explore.py','services/piper/backend.py','services/piper/installed_settling.py'):
        if not (root/path).is_file():raise ValueError('Missing adapted runtime dependency: '+path)
    settings=json.loads((base/'WORKSPACE.json').read_text())
    verify(installed_code=base/'code/blue_cup')
    origin=(root/settings['source_version']).resolve()
    if not origin.is_relative_to(root/'versions') or settings.get('writable') is not True:raise ValueError('Invalid frozen origin')
    manifest=origin/'SHA256SUMS.json'
    if hashlib.sha256(manifest.read_bytes()).hexdigest()!=settings['source_manifest_sha256']:raise ValueError('Frozen identity mismatch')
    for name_,digest in json.loads(manifest.read_text()).items():
        f=(origin/name_).resolve()
        if not f.is_relative_to(origin) or hashlib.sha256(f.read_bytes()).hexdigest()!=digest:raise ValueError('Frozen file mismatch')
    for p in base.rglob('*'):
        if p.is_symlink():raise ValueError('Baseline contains symlink: '+str(p))
    report={'will_install':apply,'destination':str(dest),'controller_sha256':meta['files_sha256']['code/blue_cup/controller.py'],'hardware_started':False}
    if not apply:return report
    # Preserve host-only memory/calibration/evidence locally. Nothing is uploaded.
    # Existing source and frozen versions are never written.
    shutil.copytree(base,dest,ignore=shutil.ignore_patterns('.git','__pycache__','*.pyc'))
    shutil.rmtree(dest/'code/blue_cup')
    shutil.copytree(HERE/'code/blue_cup',dest/'code/blue_cup')
    shutil.copy2(HERE/'run_no_agent.py',dest/'run_no_agent.py')
    code=dest/'code/blue_cup'
    def git(*args):return subprocess.check_output(['git','-C',str(code),*args],text=True).strip()
    git('init','-q');git('add','.')
    git('-c','user.name=RAX Demo Import','-c','user.email=demo@localhost','commit','-qm','Import unchanged tested v0.6.0-continuity-rc1 controller')
    settings.update(code_commit=git('rev-parse','HEAD'),demo_original_code_commit=meta['original_code_commit'],candidate_version='v0.6.0-continuity-rc1-demo',current_authorization={'hardware_task_motion_authorized':False},stage='packaged historical demo; new scene must be located at run time')
    (dest/'WORKSPACE.json').write_text(json.dumps(settings,indent=2)+'\n')
    shutil.copy2(HERE/'DEMO.json',dest/'DEMO.json')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--name',default='blue-cup/v0.6-continuity-demo');p.add_argument('--apply',action='store_true');a=p.parse_args()
    print(json.dumps(install(a.root,a.name,a.apply),indent=2))
