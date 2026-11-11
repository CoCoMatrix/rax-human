"""Continuation is a new physical session, with no replay/zero-pose dependency."""
import importlib
import json
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
explore = importlib.import_module('piper_explore')
handoff = importlib.import_module('piper_handoff')


def test_ready_announcement_waits_for_complete_lease_status():
    from types import SimpleNamespace
    initial = {'robot_status': {'status': 'READY_LIVE_TAKEOVER'}}
    full = {'robot_status': {'status': 'READY_LIVE_TAKEOVER',
                            'rax_motion': {'enabled': True, 'left_gripper_dispatch': True}}}
    states = iter([initial, full])
    reader = SimpleNamespace(state=lambda: next(states))
    assert explore.complete_ready_state(reader) is None
    assert explore.complete_ready_state(reader) is full


def test_wrist_packet_archive_preserves_original_data_and_times(tmp_path):
    import hashlib
    source=tmp_path/'volatile/left-packets/sample';source.mkdir(parents=True)
    (source/'packet.json').write_text('{"host_received_unix_ns":123}')
    (source/'images.npz').write_bytes(b'original-stereo-data')
    out=tmp_path/'run';out.mkdir()
    result=explore.archive_left_packets(out,tmp_path/'volatile')
    assert result['verified'] and result['files']==2
    assert (source/'packet.json').exists()  # legacy cleanup happens only after success
    assert (out/'left-packets/sample/images.npz').read_bytes()==b'original-stereo-data'
    archive=json.loads((out/'LEFT-SENSOR-ARCHIVE.json').read_text())
    assert archive['files']['sample/images.npz']==hashlib.sha256(b'original-stereo-data').hexdigest()


def test_wrist_archive_failure_retains_source(tmp_path,monkeypatch):
    source=tmp_path/'volatile/left-packets';source.mkdir(parents=True);(source/'packet.json').write_text('original')
    out=tmp_path/'run';out.mkdir();original=explore.shutil.copytree
    def corrupt(src,dest):
        original(src,dest);(dest/'packet.json').write_text('corrupt')
    monkeypatch.setattr(explore.shutil,'copytree',corrupt)
    with pytest.raises(ValueError,match='hash mismatch'):explore.archive_left_packets(out,tmp_path/'volatile')
    assert (source/'packet.json').read_text()=='original'
    assert not (out/'LEFT-SENSOR-ARCHIVE.json').exists()


def test_ready_does_not_hide_disabled_lease_or_bypass_reader_validation():
    from types import SimpleNamespace
    disabled = {'robot_status': {'status': 'READY_LIVE_TAKEOVER', 'rax_motion': {'enabled': False}}}
    assert explore.complete_ready_state(SimpleNamespace(state=lambda: disabled)) is disabled
    def stale():
        raise ValueError('Actual CAN feedback stale')
    with pytest.raises(ValueError, match='stale'):
        explore.complete_ready_state(SimpleNamespace(state=stale))


def test_writable_rehearsal_source_validates_frozen_origin_without_controller(tmp_path):
    import hashlib
    frozen=tmp_path/'versions/v03'; frozen.mkdir(parents=True)
    (frozen/'maneuvers.py').write_text('original')
    hashes={'maneuvers.py':hashlib.sha256(b'original').hexdigest()}
    manifest=frozen/'SHA256SUMS.json';manifest.write_text(json.dumps(hashes))
    candidate=tmp_path/'workspaces/v03';(candidate/'code/blue_cup').mkdir(parents=True)
    (candidate/'code/blue_cup/maneuvers.py').write_text('editable')
    (candidate/'WORKSPACE.json').write_text(json.dumps({'source_version':'versions/v03','source_manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),'writable':True}))
    assert explore.validate_rehearsal_source(candidate,tmp_path)['kind']=='working_copy'
    (frozen/'maneuvers.py').write_text('corrupt')
    with pytest.raises(ValueError,match='Frozen'):
        explore.validate_rehearsal_source(candidate,tmp_path)


def test_v03_prompt_does_not_assume_v02_functions_or_completed_perception():
    text=explore.working_rehearsal_prompt('start')
    assert 'v0.3' in text and 'implement' in text
    assert 'timings.jsonl' in text and 'SUMMARY.json' in text
    assert 'jaw_check' not in text and 'WALL_OFFSET' not in text
    assert 'one function' in text and 'Do not execute main(ctx)' in text


def prior_run(tmp_path):
    run=tmp_path/'commissioning/old'; w=run/'rax-session/agent-workspace'
    (w/'memory').mkdir(parents=True); (w/'code').mkdir()
    (w/'memory/PROGRESS.md').write_text('OLD: 19 minutes remain; go to historical target')
    (w/'memory/TIPS.md').write_text('conditional grasp experience')
    (w/'code/test.py').write_text('raise RuntimeError("must never execute imported code")')
    (run/'RESULT.json').write_text(json.dumps({'control_archived':True,'sensor_archived':True,'elapsed_s':1813,'agent_exit_code':124}))
    (run/'STOP-RESULT.json').write_text(json.dumps({'rest':{'measured14':[0]*6+[.0046]+[0]*6+[.0021]}}))
    frame='a'*32; folder=w/'runtime/frames'/frame;folder.mkdir(parents=True)
    (folder/'meta.json').write_text(json.dumps({'frame_id':frame,'captured_at_ns':42,'cameras':{'head':{'source_timestamp_ns':40}}}))
    (folder/'head.png').write_bytes(b'fixture')
    rows=[{'event':'mcp_request','call_id':'g','tool':'piper_gripper','time':1,'arguments':{'width_m':.004}},
          {'event':'mcp_result','call_id':'g','tool':'piper_gripper','time':2,'result':json.dumps({'executed':True,'measured_arrival':True,'sdk_completed_steps':81,'final14':[0]*6+[.0046]+[0]*7})},
          {'event':'mcp_request','call_id':'ik','tool':'piper_solve_ik','time':3,'arguments':{}},
          {'event':'mcp_error','call_id':'ik','operation':'piper_solve_ik','time':4,'reason':'MOVEIT_NO_IK_SOLUTION','control_uncertain':False},
          {'event':'mcp_request','call_id':'pending','tool':'piper_move_pose','time':5,'arguments':{'position':[.2,-.2,.25]}}]
    (run/'rax-session/mcp.jsonl').write_text('\n'.join(json.dumps(x) for x in rows))
    return run


def test_checkpoint_distinguishes_ik_from_actions_and_keeps_pending(tmp_path):
    data=handoff.checkpoint(prior_run(tmp_path))
    assert data['action_counts']=={'piper_gripper':1}
    assert data['tool_errors'][0]['tool']=='piper_solve_ik'
    assert data['unresolved_motion_requests'][0]['call_id']=='pending'
    assert data['last_frames'][0]['frame_id']=='a'*32
    assert data['historical_only'] and data['held_cup_possible']
    assert data['recorded_elapsed_s']==1813


def test_import_preserves_memory_but_overwrites_stale_progress_without_running_code(tmp_path):
    run=prior_run(tmp_path); workspace=tmp_path/'new-workspace'
    handoff.import_continuation(run,workspace)
    progress=(workspace/'memory/PROGRESS.md').read_text()
    assert '19 minutes remain' not in progress and 'Historical session' in progress
    assert (workspace/'memory/TIPS.md').read_text()=='conditional grasp experience'
    assert (workspace/'references/prior-session/historical-PROGRESS.md').read_text().startswith('OLD:')
    assert (workspace/'references/prior-session/code/test.py').exists()
    assert not (workspace/'code/test.py').exists()
    assert (workspace/'references/prior-session/frames'/('a'*32)/'head.png').exists()


def test_import_rejects_symlinks_and_unfinished_source(tmp_path):
    run=prior_run(tmp_path)
    (run/'rax-session/agent-workspace/memory/link').symlink_to(tmp_path/'outside')
    with pytest.raises(ValueError,match='Linked'):handoff.import_continuation(run,tmp_path/'new')
    (run/'RESULT.json').write_text('{}')
    with pytest.raises(ValueError,match='not finished'):handoff.validate_source(run,tmp_path/'commissioning')


def test_current_nonzero_held_pose_is_allowed_without_zero_baseline():
    q=[-.806446834,1.599472086,-1.297669752,1.355125991,0,-.80855868,.0046,0,0,0,0,0,0,.0021]
    options={'workspace_min':[.055,-.4,.17],'workspace_max':[.36,.045,.36]}
    actual=explore.checked_current_pose({'robot_state':{'values':q}},options)
    assert actual.tolist()==q
    with pytest.raises(ValueError,match='outside'):
        explore.checked_current_pose({'robot_state':{'values':q}},{'workspace_min':[0,0,0],'workspace_max':[.1,.1,.1]})


def test_continuation_prompt_forbids_regrasp_jaw_test_and_requests_current_views():
    text=explore.task_prompt('start',True)
    assert 'Do not perform the fresh-task jaw-opening self-test' in text
    assert 'CURRENT images and feedback' in text
    assert 'First check left jaw' not in text
    assert 'actual remaining budget' in text
    assert 'permits a fresh grasp and placement based on current geometry' in text
    assert 'If held, do not regrasp' in text


def test_handoff_saved_after_timeout_without_task_success_claim(tmp_path):
    run=prior_run(tmp_path)
    handoff.save_handoff(run,{'agent_exit_code':124,'elapsed_s':900,'stop':{'rest':{'measured14':[0]*14}}})
    data=json.loads((run/'HANDOFF.json').read_text())
    assert data['recorded_elapsed_s']==900 and data['agent_exit_code']==124
    assert data['held_cup_possible'] and data['no_automatic_release']
    assert 'Not inferred' in data['task_success']


def test_no_run_plan_reads_finished_source_without_devices(monkeypatch,tmp_path,capsys):
    from test_piper_minimal_live import graph_source
    import yaml
    run=prior_run(tmp_path); (tmp_path/'examples').mkdir()
    (tmp_path/'examples/blue-cup-exploration.json').write_text(json.dumps({'workspace_min':[.055,-.4,.17],'workspace_max':[.36,.045,.36]}))
    graph=tmp_path/'source.yml';graph.write_text(yaml.safe_dump(graph_source()))
    owner=tmp_path/'owner';profile=owner/'real_world/robots/piper_x_dual/profile.yml';profile.parent.mkdir(parents=True);profile.write_text('live_takeover:\n  gripper_write_enabled: false\n  speed_percent: 10\n')
    monkeypatch.setattr(explore,'ROOT',tmp_path);monkeypatch.setattr(explore,'GRAPH',graph);monkeypatch.setattr(explore,'OWNER',owner)
    monkeypatch.setattr(explore,'no_existing_devices',lambda:pytest.fail('no-run must not inspect/start hardware'))
    monkeypatch.setattr(explore,'sensor_transport',lambda:pytest.fail('no-run must not create runtime'))
    monkeypatch.setattr(sys,'argv',['piper_explore','--output',str(tmp_path/'commissioning/new'),'--continue-from',str(run),'--seconds','900'])
    explore.main(); plan=json.loads(capsys.readouterr().out)
    assert not plan['starts_devices'] and plan['continue_from']==str(run)
    assert plan['session_seconds']==900 and '1020s' in plan['commands']['graph']
    assert 'candidate' not in plan and not (tmp_path/'commissioning/new').exists()


def test_fresh_exploration_can_write_code_and_never_delegates_zero_return():
    text=explore.task_prompt('start',False)
    assert 'task code directory starts empty' in text
    assert 'create and revise short Python Context helpers' in text
    assert 'consolidate the understood maneuvers' in text
    with pytest.raises(ValueError,match='XPolicy'):
        explore.task_prompt('start',return_to_zero_first=True)


def test_rehearsal_prompt_uses_existing_functions_without_full_run_or_stale_confirmation():
    text=' '.join(explore.rehearsal_prompt('start').split())
    assert '.agents/skills/autonomous-control/SKILL.md' in text
    assert 'one function' in text and 'Do not execute main(ctx)' in text
    assert '19 mm' in text and 'before approaching the cup' in text
    assert 'are not current permission' in text and 'new physical session invalidates them' in text
    assert 'SAME request' in text and 'request_id=...' in text
    assert 'observation_recovery only read status/observe' in text
    assert 'output/rehearsal/SUMMARY.json' in text
    assert 'XPolicy' in text and 'actual remaining budget' in text
    assert 'task code directory starts empty' not in text


def test_rehearsal_agent_requires_candidate_before_device_access(monkeypatch,tmp_path):
    monkeypatch.setattr(explore,'ROOT',tmp_path)
    monkeypatch.setattr(explore,'no_existing_devices',lambda:pytest.fail('must reject before hardware'))
    monkeypatch.setattr(sys,'argv',['piper_explore','--output',str(tmp_path/'commissioning/new'),'--rehearsal-agent'])
    with pytest.raises(SystemExit):explore.main()


@pytest.mark.parametrize('with_agent',[False,True])
def test_rehearsal_dry_run_preserves_candidate_and_selects_model(monkeypatch,tmp_path,capsys,with_agent):
    import hashlib,yaml
    from test_piper_minimal_live import graph_source
    candidate=tmp_path/'candidates/v02'; code=candidate/'code/blue_cup/controller.py'
    code.parent.mkdir(parents=True);code.write_text('raise RuntimeError("must not execute in plan")')
    (candidate/'CANDIDATE_SHA256SUMS.json').write_text(json.dumps({'code/blue_cup/controller.py':hashlib.sha256(code.read_bytes()).hexdigest()}))
    (tmp_path/'examples').mkdir()
    (tmp_path/'examples/blue-cup-exploration.json').write_text(json.dumps({'workspace_min':[.055,-.4,.17],'workspace_max':[.36,.045,.36]}))
    graph=tmp_path/'source.yml';graph.write_text(yaml.safe_dump(graph_source()))
    owner=tmp_path/'owner';profile=owner/'real_world/robots/piper_x_dual/profile.yml';profile.parent.mkdir(parents=True);profile.write_text('live_takeover:\n  gripper_write_enabled: false\n  speed_percent: 10\n')
    monkeypatch.setattr(explore,'ROOT',tmp_path);monkeypatch.setattr(explore,'GRAPH',graph);monkeypatch.setattr(explore,'OWNER',owner)
    monkeypatch.setattr(explore,'no_existing_devices',lambda:pytest.fail('dry run touched hardware'))
    argv=['piper_explore','--output',str(tmp_path/'commissioning/new'),'--rehearsal-candidate',str(candidate)]
    if with_agent:argv.append('--rehearsal-agent')
    monkeypatch.setattr(sys,'argv',argv)
    explore.main();plan=json.loads(capsys.readouterr().out)
    assert plan['main_planning_model_started'] is with_agent
    assert plan['rehearsal_agent'] is with_agent
    assert not plan['starts_devices'] and not (tmp_path/'commissioning/new').exists()


def test_memory_import_copies_all_declared_evidence_not_only_last_frames(tmp_path):
    run=prior_run(tmp_path);src=run/'rax-session/agent-workspace'
    skill=src/'memory/skills/test';skill.mkdir(parents=True)
    (skill/'SKILL.md').write_text('Learned conditional maneuver')
    (skill/'evidence.json').write_text(json.dumps({'schema_version':1,'examples':[{
        'frames':['runtime/frames/older/head.png'],'complete_ledger':'audit/motion.jsonl'}]}))
    for name in ('runtime/frames/older/head.png','audit/motion.jsonl'):
        p=src/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'evidence')
    dst=tmp_path/'next';handoff.import_continuation(run,dst)
    assert (dst/'runtime/frames/older/head.png').read_bytes()==b'evidence'
    assert (dst/'audit/motion.jsonl').read_bytes()==b'evidence'
    assert not (dst/'code/test.py').exists()


def test_missing_evidence_is_not_silently_imported(tmp_path):
    src=tmp_path/'source';p=src/'memory/skills/test/evidence.json';p.parent.mkdir(parents=True)
    p.write_text(json.dumps({'examples':[{'frames':['runtime/frames/missing/head.png']}]}))
    with pytest.raises(ValueError,match='Missing'):handoff.copy_memory_evidence(src,tmp_path/'next')


@pytest.mark.parametrize('ref',['../secret','/etc/passwd','code/controller.py'])
def test_memory_evidence_cannot_import_code_or_escape_workspace(tmp_path,ref):
    src=tmp_path/'source';p=src/'memory/skills/test/evidence.json';p.parent.mkdir(parents=True)
    p.write_text(json.dumps({'examples':[{'frames':[ref]}]}))
    with pytest.raises(ValueError):handoff.copy_memory_evidence(src,tmp_path/'next')


def test_memory_evidence_conflict_does_not_overwrite_existing_frame(tmp_path):
    src=tmp_path/'source';p=src/'memory/skills/test/evidence.json';p.parent.mkdir(parents=True)
    p.write_text(json.dumps({'examples':[{'frames':['runtime/frames/a/head.png']}]}))
    dst=tmp_path/'next'
    for root,data in [(src,b'original'),(dst,b'other')]:
        f=root/'runtime/frames/a/head.png';f.parent.mkdir(parents=True);f.write_bytes(data)
    with pytest.raises(ValueError,match='conflict'):handoff.copy_memory_evidence(src,dst)
    assert (dst/'runtime/frames/a/head.png').read_bytes()==b'other'


def test_rehearsal_and_fresh_memory_use_same_native_evidence_paths(tmp_path):
    src=tmp_path/'working';p=src/'memory/skills/test/evidence.json';p.parent.mkdir(parents=True)
    p.write_text(json.dumps({'examples':[{'frames':['runtime/frames/a/head.png']}]}))
    f=src/'runtime/frames/a/head.png';f.parent.mkdir(parents=True);f.write_bytes(b'frame')
    (src/'memory/INDEX.md').write_text('skills/test/SKILL.md')
    dst=tmp_path/'new'
    handoff.import_memory(src/'memory',dst)
    assert (dst/'memory/INDEX.md').exists() and (dst/'runtime/frames/a/head.png').read_bytes()==b'frame'


def test_native_notes_keep_their_linked_failure_evidence(tmp_path):
    src=tmp_path/'source';(src/'memory').mkdir(parents=True)
    (src/'memory/TIPS.md').write_text('Failure: `references/history/run/PROGRESS.md`. Current geometry: [notes](references/geometry.md).')
    for name in ('references/history/run/PROGRESS.md','references/geometry.md'):
        p=src/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('original evidence')
    dst=tmp_path/'next';handoff.import_memory(src/'memory',dst)
    assert (dst/'references/history/run/PROGRESS.md').read_text()=='original evidence'
    assert (dst/'references/geometry.md').is_file()


def test_imported_native_memory_remains_writable(tmp_path):
    src=tmp_path/'source';p=src/'memory/TIPS.md';p.parent.mkdir(parents=True);p.write_text('observed lesson');p.chmod(0o444)
    dst=tmp_path/'new';handoff.import_memory(src/'memory',dst)
    assert (dst/'memory/TIPS.md').stat().st_mode & 0o200


def test_live_owner_receives_same_arrival_policy_as_bridge(tmp_path):
    from services.piper.installed_settling import JOINT_ARRIVAL_TOLERANCE_RAD
    source={'nodes':[{'id':name,'args':'python -m example.'+name,'inputs':{'tick':'timer'}}
                     for name in ['realsense_multi','piper_operator_ui','piper_x_dual_owner','piper_control_mux','piper_x_dual_policy']]}
    graph=explore.live_graph(source,tmp_path,tmp_path/'control',tmp_path/'sensor')
    owner=next(n for n in graph['nodes'] if n['id']=='piper_x_dual_owner')
    assert float(owner['env']['RAX_MAX_JOINT_ARRIVAL_TOLERANCE_RAD'])==JOINT_ARRIVAL_TOLERANCE_RAD
    policy=next(n for n in graph['nodes'] if n['id']=='piper_x_dual_policy')
    assert policy['args'].endswith('/scripts/piper_policy.py')
    assert policy['env']['MODEL_RETRY_SECONDS']=='0.5'
    assert policy['env']['PIPER_CONTINUOUS_REVIEW']=='1'
    assert policy['env']['ACTION_RTC_MODE']=='disabled'
    assert policy['env']['POLICY_PREFETCH_ENABLED']=='0'


def test_shutdown_covers_left_ffs_after_camera_owner():
    import ast
    tree=ast.parse(Path(explore.__file__).read_text())
    sorter=next(n for n in ast.walk(tree) if isinstance(n,ast.Lambda) and isinstance(n.body,ast.Subscript) and isinstance(n.body.value,ast.Dict) and any(isinstance(k,ast.Constant) and k.value=='ffs' for k in n.body.value.keys))
    key=eval(compile(ast.Expression(sorter),'<shutdown-order>','eval'))
    names=['left-ffs','ffs','graph','bridge','agent','frontend']
    ordered=sorted([(n,None) for n in names],key=key)
    assert [n for n,_ in ordered].index('graph')<[n for n,_ in ordered].index('left-ffs')


def test_rehearsal_stage_precedes_base_prompt_and_current_dependencies_are_explicit():
    text=explore.working_rehearsal_prompt('BASE_START')
    assert text.startswith('CURRENT STAGE:')
    assert text.index('prepare_for_approach')<text.index('BASE_START')
    assert 'NO placement.py or maneuvers.py dependency' in text
    assert 'Do not separately call status before snapshot' in text
    assert 'manual-robot-control only if actually switching' in text
    assert 'not a file-access restriction' in text
    readme=(ROOT/'workspaces/blue-cup/20261104-v0.3-r12-working/code/blue_cup/README.md').read_text()
    assert len(readme)<6500
    assert 'Preparation contract and source dependencies' in readme


def test_packet_budget_matches_installed_ffs_and_rejects_before_devices(monkeypatch,tmp_path):
    assert explore.packet_budget(1800)==7200
    assert explore.packet_budget(3000)==9360
    assert explore.packet_budget(3213)==9999
    with pytest.raises(ValueError,match='10000'):explore.packet_budget(3600)
    monkeypatch.setattr(explore,'ROOT',tmp_path)
    monkeypatch.setattr(explore,'no_existing_devices',lambda:pytest.fail('Invalid budget must not start devices'))
    monkeypatch.setattr(sys,'argv',['piper_explore','--output',str(tmp_path/'commissioning/new'),'--seconds','3600','--run'])
    with pytest.raises(SystemExit):explore.main()
    assert not (tmp_path/'commissioning/new').exists()


def test_left_packet_archive_preserves_original_acquisition_data(tmp_path):
    sensor=tmp_path/'sensor';source=sensor/'left-packets/packet-1';source.mkdir(parents=True)
    (source/'packet.json').write_text('{"host_received_ns":1234}')
    (source/'left.png').write_bytes(b'original stereo bytes')
    out=tmp_path/'run';out.mkdir()
    result=explore.archive_left_packets(out,sensor)
    assert result['verified'] and result['files']==2
    assert (out/'left-packets/packet-1/left.png').read_bytes()==(source/'left.png').read_bytes()
    assert json.loads((out/'left-packets/packet-1/packet.json').read_text())['host_received_ns']==1234
    assert (out/'LEFT-SENSOR-ARCHIVE.json').exists()


def test_left_packet_archive_error_preserves_tmpfs_source(tmp_path,monkeypatch):
    sensor=tmp_path/'sensor';source=sensor/'left-packets';source.mkdir(parents=True)
    (source/'packet.json').write_text('original')
    out=tmp_path/'run';out.mkdir();original=explore.shutil.copytree
    def corrupt(src,dst):
        original(src,dst);(dst/'packet.json').write_text('broken')
    monkeypatch.setattr(explore.shutil,'copytree',corrupt)
    with pytest.raises(ValueError,match='hash mismatch'):explore.archive_left_packets(out,sensor)
    assert (source/'packet.json').read_text()=='original'
    assert not (out/'left-packets').exists()
