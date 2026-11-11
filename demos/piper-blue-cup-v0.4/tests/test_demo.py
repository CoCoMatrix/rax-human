import importlib,sys,json,hashlib
from pathlib import Path
import pytest

DEMO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(DEMO))
from verify_demo import verify
from install_demo import install


def test_snapshot_and_all_historical_identities():
    result=verify()
    assert result['historical_matching_runs']==4
    assert result['historical_successes']==3
    assert result['hardware_started'] is False


@pytest.fixture
def controller(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.syspath_prepend(str(DEMO/'code/blue_cup'))
    m=importlib.import_module('controller')
    out=tmp_path/'output';out.mkdir(exist_ok=True)
    monkeypatch.setattr(m,'OUT',out)
    return m


def run_fake(monkeypatch,m,fail=None):
    calls=[]
    def fn(name):
        def invoke(ctx,*args):
            calls.append(name)
            if name=='trial_lift':assert args[0]=={'dz':.040,'execution_mode':'task_scheduled'}
            return {'ok':name!=fail,'why':'synthetic test','report_path':'synthetic.json','previous_report':'synthetic.json','transport_complete':True,'visual_calls':0,'steps':[]}
        return invoke
    for name in ('prepare_for_approach','approach','grasp','trial_lift','localize_coaster','transfer_step','lower_to_support','release','retreat'):
        monkeypatch.setattr(m,name,fn(name))
    return m.main(object()),calls


def test_main_order_and_zero_model_calls(controller,monkeypatch):
    report,calls=run_fake(monkeypatch,controller)
    assert report['ok'] and report['task_success']
    assert calls==['prepare_for_approach','approach','approach','grasp','trial_lift','localize_coaster','transfer_step','lower_to_support','release','retreat']
    assert report['main_agent_calls']==report['auxiliary_visual_calls']==0
    assert report['held_object_possible'] is False
    assert all(s['stage_elapsed_s']>=0 for s in report['steps'])


def test_first_failure_never_grasps(controller,monkeypatch):
    report,calls=run_fake(monkeypatch,controller,fail='approach')
    assert not report['ok'] and report['failed_step']=='hover'
    assert calls==['prepare_for_approach','approach']


def test_lift_failure_never_releases_or_replays(controller,monkeypatch):
    report,calls=run_fake(monkeypatch,controller,fail='trial_lift')
    assert not report['ok'] and report['held_object_possible']
    assert calls[-1]=='trial_lift' and calls.count('grasp')==1
    assert 'release' not in calls and 'transfer_step' not in calls


@pytest.fixture
def runtime(tmp_path):
    import shutil
    root=tmp_path/'runtime';base=root/'workspaces/blue-cup/20261109-v0.4-merged-candidate'
    shutil.copytree(DEMO/'code',base/'code')
    for name in ('memory','calibration'):(base/name).mkdir()
    origin=root/'versions/fixture';origin.mkdir(parents=True)
    manifest=origin/'SHA256SUMS.json';manifest.write_text('{}')
    (base/'WORKSPACE.json').write_text(json.dumps({'source_version':'versions/fixture','source_manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),'writable':True}))
    for name in ('scripts/piper_explore.py','services/piper/backend.py','services/piper/installed_settling.py'):
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('# fixture never executed\n')
    return root,base


def test_install_dry_run_and_new_copy_only(runtime):
    root,base=runtime;before=(base/'WORKSPACE.json').read_bytes()
    r=install(root);assert not Path(r['destination']).exists()
    r=install(root,apply=True);dest=Path(r['destination'])
    assert dest.is_dir() and r['hardware_started'] is False
    verify(installed_code=dest/'code/blue_cup')
    assert (base/'WORKSPACE.json').read_bytes()==before
    meta=json.loads((dest/'WORKSPACE.json').read_text())
    assert meta['candidate_version']=='v0.4.0-rc10-demo'
    assert meta['current_authorization']['hardware_task_motion_authorized'] is False
    with pytest.raises(ValueError,match='exists'):install(root,apply=True)


@pytest.mark.parametrize('name',['../outside','/tmp/outside'])
def test_installer_rejects_escape(runtime,name):
    with pytest.raises(ValueError):install(runtime[0],name=name,apply=True)


def test_installer_rejects_changed_original(runtime):
    root,base=runtime
    (base/'code/blue_cup/controller.py').write_text('# different\n')
    with pytest.raises(ValueError,match='differs'):install(root,apply=True)
    assert not (root/'workspaces/blue-cup/v0.4-rc10-demo').exists()
