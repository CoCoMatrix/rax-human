import importlib.util,json,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import site_paths
import prepare_workspace

def test_site_paths_do_not_read_credentials(tmp_path,monkeypatch):
    p=tmp_path/'site.json';p.write_text(json.dumps({'key_file':str(tmp_path/'missing-key'),'legacy_root':'integrations/legacy_piper'}))
    monkeypatch.setenv('RAX_SITE_CONFIG',str(p))
    assert site_paths.path('key_file','unused')==tmp_path/'missing-key'
    assert site_paths.path('legacy_root','unused')==ROOT/'integrations/legacy_piper'

def test_workspace_copy_is_new_and_never_qualifies_candidate(tmp_path,monkeypatch):
    monkeypatch.setattr(prepare_workspace,'ROOT',tmp_path)
    s=tmp_path/'source';(s/'code/task').mkdir(parents=True)
    (s/'code/task/controller.py').write_text('def step(ctx): return {}\n')
    planned=prepare_workspace.prepare(s,'task/v1')
    assert not planned['hardware_started'] and not (tmp_path/'workspaces').exists()
    result=prepare_workspace.prepare(s,'task/v1',apply=True)
    assert 'candidate' in result['qualification']
    meta=json.loads((Path(result['workspace'])/'WORKSPACE.json').read_text())
    assert len(meta['code_commit'])==40
    assert (Path(result['workspace'])/'code/.git').is_dir()
    with pytest.raises(FileExistsError):prepare_workspace.prepare(s,'task/v1',apply=True)

@pytest.mark.parametrize('name',['../escape','/absolute','x/../../z','x;rm'])
def test_workspace_rejects_bad_paths(tmp_path,name):
    with pytest.raises(ValueError):prepare_workspace.prepare(tmp_path,name)

def test_task_templates_start_with_empty_memory():
    for name in ('blue-cup','magnetic-blocks-l-stack'):
        p=ROOT/'examples/tasks'/name/'session.json';v=json.loads(p.read_text())
        assert v['seed_memory']=='seed-memory'
        assert (p.parent/v['seed_memory']/'INDEX.md').is_file()
