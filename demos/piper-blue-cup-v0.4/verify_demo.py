"""Hash, syntax and recorded-evidence verification. Never imports robot code."""
import argparse,ast,hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent

def verify(root=HERE,installed_code=None):
    root=Path(root);meta=json.loads((root/'DEMO.json').read_text())
    for name,expected in meta['files_sha256'].items():
        path=root/name
        if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):raise ValueError('Invalid source path: '+name)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('Snapshot changed: '+name)
        if path.suffix=='.py':ast.parse(path.read_text(),filename=name)
        if installed_code is not None and name.startswith('code/blue_cup/'):
            other=Path(installed_code)/name.removeprefix('code/blue_cup/')
            if hashlib.sha256(other.read_bytes()).hexdigest()!=expected:raise ValueError('Installed source differs: '+name)
    evidence=root/meta['validation_summary']
    if hashlib.sha256(evidence.read_bytes()).hexdigest()!=meta['validation_summary_sha256']:raise ValueError('Evidence summary changed')
    expected={Path(k).name:v for k,v in meta['files_sha256'].items() if k.startswith('code/blue_cup/') and Path(k).suffix=='.py' and len(Path(k).parts)==3}
    runs=json.loads(evidence.read_text())['runs']
    for r in runs:
        if r['commit']!=meta['original_code_commit'] or r['source_report_code_sha256']!=expected:raise ValueError('Run/code identity mismatch')
    return {'ok':True,'files_verified':len(meta['files_sha256']),'historical_matching_runs':len(runs),'historical_successes':sum(r['outcome']=='success' for r in runs),'hardware_started':False}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--installed-code',type=Path);a=p.parse_args()
    print(json.dumps(verify(installed_code=a.installed_code),indent=2))
