import json
import pytest
from test_piper_action_observation import rig,value

def test_nested_receipt_budget_matches_and_observation_is_bracketed(tmp_path):
    f,bridge,events=rig(tmp_path);f.started_at=__import__('time').time()
    try:
        r=value(f.call('piper_gripper',{'width_m':.02,'observe_after':True}))
        assert r['observation']['wall_budget']==r['wall_budget']
        assert r['observation']['state_binding']['rest_consistent']
    finally:f.close()

@pytest.mark.parametrize('unknown',[False,True])
def test_read_only_error_is_no_dispatch_but_preserves_prior_unknown(tmp_path,unknown):
    f,bridge,events=rig(tmp_path);f.control_uncertain=unknown
    try:
        r=value(f.call('piper_pixel_to_position',{'frame_id':'a'*32,'camera':'left','pixels':[[1,2]]}))['error']
        assert r['request_no_action_executed'] and r['dispatch_started'] is False
        assert r['episode_ended'] is False and r['control_uncertain'] is unknown
        assert not bridge.moves
    finally:f.close()

@pytest.mark.parametrize('phase,ready',[('IDLE',True),('DONE',True),('MOVING',False)])
def test_ready_contract_includes_completed_bridge(tmp_path,phase,ready):
    f,bridge,events=rig(tmp_path)
    bridge.status=lambda:{'phase':phase,'observation_recovery':False}
    # Real Owner publishes camera metadata; this fake has it as an empty object.
    try:assert f.backend.status()['motion_ready'] is ready
    finally:f.close()

def test_workspace_contract_matches_backend_tools(tmp_path):
    from services.workspace.setup import ROBOT_FACTS
    f,_,_=rig(tmp_path)
    try:assert set(ROBOT_FACTS['piper']['tools'])=={t['name'] for t in f.backend.tools()}
    finally:f.close()
