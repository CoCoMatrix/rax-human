"""Verify the exported controller/adapter and historical identities; no devices."""
import argparse,ast,hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent

def verify(root=HERE,installed_code=None,runtime=None):
    root=Path(root).resolve();meta=json.loads((root/'DEMO.json').read_text())
    for name,digest in meta['files_sha256'].items():
        p=root/name
        if p.is_symlink() or not p.resolve().is_relative_to(root):raise ValueError('Unsafe snapshot path: '+name)
        if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise ValueError('Snapshot changed: '+name)
        if p.suffix=='.py':ast.parse(p.read_text(),filename=name)
        if installed_code is not None and name.startswith('code/blue_cup/'):
            q=Path(installed_code)/name.removeprefix('code/blue_cup/')
            if hashlib.sha256(q.read_bytes()).hexdigest()!=digest:raise ValueError('Installed source differs: '+name)
    expected={Path(k).name:v for k,v in meta['files_sha256'].items() if k.startswith('code/blue_cup/') and Path(k).suffix=='.py' and len(Path(k).parts)==3}
    runs=json.loads((root/'validation-runs.json').read_text())['runs'];matching=[x for x in runs if x['version']==meta['version']]
    for row in matching:
        if row['commit']!=meta['original_code_commit'] or row['source_report_code_sha256']!=expected:raise ValueError('Run/code identity mismatch')
    if not matching:raise ValueError('No matching run evidence')
    host_count=0
    if runtime is not None:
        runtime=Path(runtime)
        for name,digest in meta['required_host_sha256'].items():
            if hashlib.sha256((runtime/name).read_bytes()).hexdigest()!=digest:raise ValueError('Host adapter differs: '+name)
            host_count+=1
        owner=runtime/'owner-runtime'
        if (owner/'XPolicyLab').resolve()!=owner.resolve():raise ValueError('Isolated XPolicyLab self-alias points outside owner copy')
    return dict(ok=True,files_verified=len(meta['files_sha256']),matching_runs=len(matching),successful_runs=sum(x['outcome']=='success' for x in matching),host_files_verified=host_count,hardware_started=False)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--installed-code',type=Path);p.add_argument('--runtime',type=Path);a=p.parse_args()
    print(json.dumps(verify(installed_code=a.installed_code,runtime=a.runtime),indent=2))
