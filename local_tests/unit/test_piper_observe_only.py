import importlib.util
from pathlib import Path
import pytest

SCRIPT=Path(__file__).resolve().parents[2]/'scripts/piper_observe_only.py'
spec=importlib.util.spec_from_file_location('piper_observe_only',SCRIPT)
observe=importlib.util.module_from_spec(spec);spec.loader.exec_module(observe)


def source_graph():
    return {'nodes':[
        {'id':'realsense_multi','args':'python -m XPolicyLab.real_world.cameras.realsense_dora_node','inputs':{'tick':'dora/timer/millis/10'},'outputs':['status']},
        {'id':'piper_x_dual_owner','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.dora_owner_node','inputs':{'tick':'dora/timer/millis/10','camera_status':'realsense_multi/status','button_command':'piper_control_mux/button_command','model_action':'piper_x_dual_policy/action_robot','transition_command':'piper_operator_ui/transition_command'}},
        {'id':'piper_operator_ui','args':'python -m XPolicyLab.real_world.robots.piper_x_dual.operator_ui_node','inputs':{'tick':'dora/timer/millis/50','robot_status':'piper_x_dual_owner/status','policy_status':'piper_x_dual_policy/status','model_info':'piper_x_dual_policy/model_info'}},
        {'id':'piper_x_dual_policy','args':'policy'}, {'id':'piper_control_mux','args':'mux'}]}


def test_observation_graph_has_no_command_action_enable_routes(tmp_path):
    original=source_graph();graph=observe.make_graph(original,tmp_path)
    assert {n['id'] for n in graph['nodes']}=={'realsense_multi','piper_x_dual_owner','piper_operator_ui'}
    owner=next(n for n in graph['nodes'] if n['id']=='piper_x_dual_owner')
    assert set(owner['inputs'])=={'tick','camera_status'}
    for node in graph['nodes']:
        assert node['env']['ENABLE_WRITE']=='0'
        assert node['env']['PYTHONDONTWRITEBYTECODE']=='1'
        assert 'policy_status' not in node['inputs'] and 'model_action' not in node['inputs']
        assert node['env']['REALSENSE_STEREO_EXPORT_DIR']==str(tmp_path/'packets')
        assert node['env']['TASK']==observe.OBSERVATION_TASK
        assert node['env']['PIPER_TASK_CATALOG_PATH']==str(tmp_path/'tasks.json')
    assert 'button_command' in original['nodes'][1]['inputs'] # Source graph unmodified.


def test_plan_commands_keep_all_outputs_in_new_directory(tmp_path):
    dora,worker=observe.commands(tmp_path,120)
    assert dora[-1]==str(tmp_path/'graph.yml')
    assert worker[worker.index('--output')+1]==str(tmp_path/'depth')
    assert worker[worker.index('--packet-root')+1]==str(tmp_path/'packets')
    assert '--run' not in worker and 'start' not in dora


def test_write_count_reports_only_observed_counts():
    result={'hardware_writes':None}
    with pytest.raises(PermissionError):observe.record_write_status(result,{})
    assert result['hardware_writes'] is None
    observe.record_write_status(result,{'hardware_write_enabled':False,'hardware_write_count':0})
    assert result['hardware_writes']==0
    with pytest.raises(PermissionError):
        observe.record_write_status(result,{'hardware_write_enabled':True,'hardware_write_count':3})
    assert result['hardware_writes']==3
