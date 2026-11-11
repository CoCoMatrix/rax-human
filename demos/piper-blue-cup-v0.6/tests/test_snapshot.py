import importlib.util,sys,json,shutil,hashlib
from pathlib import Path
import pytest
D=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(D))
from verify_demo import verify
from install_demo import install

def test_evidence_matches_exported_controller():
    r=verify();assert r['matching_runs']==2 and r['successful_runs']==2 and not r['hardware_started']

def test_tamper_is_detected(tmp_path):
    dst=tmp_path/'demo';shutil.copytree(D,dst,ignore=shutil.ignore_patterns('__pycache__'))
    p=dst/'code/blue_cup/controller.py';p.write_text(p.read_text()+'\n# changed\n')
    with pytest.raises(ValueError,match='Snapshot changed'):verify(dst)

@pytest.fixture
def runtime(tmp_path,monkeypatch):
    root=tmp_path/'runtime';meta=json.loads((D/'DEMO.json').read_text());base=root/meta['original_workspace']
    shutil.copytree(D/'code',base/'code')
    for n in ('memory','calibration'):(base/n).mkdir()
    origin=root/'versions/fixture';origin.mkdir(parents=True);m=origin/'SHA256SUMS.json';m.write_text('{}')
    (base/'WORKSPACE.json').write_text(json.dumps(dict(source_version='versions/fixture',source_manifest_sha256=hashlib.sha256(m.read_bytes()).hexdigest(),writable=True)))
    shutil.copytree(D/'host-overlay',root,dirs_exist_ok=True)
    (root/'scripts/piper_explore.py').write_text('# fixture: never executed\n')
    # Host has a private Owner source; package tests do not fabricate matching bytes.
    import install_demo
    original=install_demo.verify
    def verify_controller_only(**kw):
        kw.pop('runtime',None);return original(**kw)
    monkeypatch.setattr(install_demo,'verify',verify_controller_only)
    return root,base

def test_install_copy_only_and_refuses_overwrite(runtime):
    root,base=runtime;before=(base/'WORKSPACE.json').read_bytes()
    d=install(root);assert not Path(d['destination']).exists()
    d=install(root,apply=True);dest=Path(d['destination']);verify(installed_code=dest/'code/blue_cup')
    assert (base/'WORKSPACE.json').read_bytes()==before and not d['hardware_started']
    assert json.loads((dest/'WORKSPACE.json').read_text())['current_authorization']['hardware_task_motion_authorized'] is False
    with pytest.raises(ValueError,match='exists'):install(root,apply=True)

@pytest.mark.parametrize('name',['../escape','/tmp/escape'])
def test_install_rejects_outside_workspace(runtime,name):
    with pytest.raises(ValueError):install(runtime[0],name=name,apply=True)

def test_wrong_host_fails_before_copy(tmp_path):
    with pytest.raises((ValueError,FileNotFoundError)):install(tmp_path,apply=True)
    assert not (tmp_path/'workspaces').exists()
