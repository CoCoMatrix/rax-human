import importlib.util
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('piper_minimal_live',ROOT/'scripts/piper_minimal_live.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def graph_source():
    return {'nodes':[
        {'id':'realsense_multi','args':'python -m XPolicyLab.real_world.cameras.realsense_dora_node','inputs':{'tick':'dora/timer/millis/10'}},
        {'id':'piper_x_dual_owner','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.dora_owner_node','inputs':dict(tick='tick',camera_status='camera',button_command='mux',model_action='policy',model_status='status',transition_command='transition',takeover='takeover')},
        {'id':'piper_operator_ui','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.operator_ui_node','inputs':{'tick':'tick','policy_status':'policy'}},
        {'id':'piper_x_dual_policy','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.policy_node','inputs':{}},
        {'id':'piper_control_mux','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.control_mux_node','inputs':{}}]}


def test_live_graph_scope_and_private_control_bindings(tmp_path):
    control=tmp_path/'control';sensor=tmp_path/'sensors'
    graph=probe.live_graph(graph_source(),tmp_path,control,sensor)
    assert len(graph['nodes'])==5
    for node in graph['nodes']:
        e=node['env']
        assert e['ENABLE_WRITE']=='1' and e['PIPER_PREPARE_HOLD_CURRENT']=='1'
        assert e['RAX_PREPARE_LEFT_HOME']=='0' and e['RAX_LEFT_GRIPPER_DISPATCH']=='0'
        assert e['PIPER_INITIALIZE_LEFT_GRIPPER_WIDTH_M']==e['PIPER_RELEASE_SOFTWARE_STOP_ARM']==e['PIPER_CALIBRATE_CLOSED_GRIPPER']==''
        assert e['REALSENSE_STEREO_EXPORT_DIR']==e['PIPER_DEPLOYMENT']==''
        assert e['RAX_MOTION_LEASE']==str(control/'bridge/lease.json')
        assert e['RAX_ARMING_FILE']==str(control/'operator/armed.json')
        assert e['PIPER_REVIEW_RECEIPT_JOURNAL']==str(control/'receipts.jsonl')
    owner=next(n for n in graph['nodes'] if n['id']=='piper_x_dual_owner')
    assert set(owner['inputs'])=={'tick','camera_status','button_command','model_action','model_status'}


def test_disarm_attempts_both_files_when_one_write_fails(monkeypatch,tmp_path):
    from services.franka import arming
    calls=[]
    def write(root,*args,**kwargs):
        calls.append(root.name)
        if root.name=='session':raise OSError('injected write failure')
    monkeypatch.setattr(arming,'write',write)
    with pytest.raises(RuntimeError,match='injected'):probe.disarm(tmp_path)
    assert calls==['session','rax-session']


def test_independent_stop_disarms_before_broken_http(monkeypatch,tmp_path):
    from services.franka import arming
    for root in (tmp_path/'session',tmp_path/'rax-session'):arming.write(root,True,minutes=1)
    def broken_operator(out):
        assert all(not arming.read(out/p)['armed'] for p in ('session','rax-session'))
        raise OSError('UI unavailable')
    monkeypatch.setattr(probe,'Operator',broken_operator)
    with pytest.raises(RuntimeError,match='UI unavailable'):probe.stop_session(tmp_path)
    assert (tmp_path/'STOP-RESULT.json').is_file()
