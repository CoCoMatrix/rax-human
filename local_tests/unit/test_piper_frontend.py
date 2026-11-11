"""A rejected target must not be reported as an executed or measured-stop motion."""
import json
from conftest import frontend


def test_pre_dispatch_refusal_has_no_execution_or_measured_hold(tmp_path):
    from services.piper.bridge import RejectedBeforeDispatch
    f = frontend(tmp_path)
    f.backend.mutating = {'mock_move'}
    f.backend.validate = lambda *args: None
    def reject(*args):
        raise RejectedBeforeDispatch('No full-pose IK solution')
    f.backend.call = reject
    try:
        result = f.call('mock_move', {})
        error = json.loads(result['content'][0]['text'])['error']
        assert result['isError'] is True
        assert error['stage'] == 'validation'
        assert error['no_action_executed'] is True
        assert error['control_uncertain'] is False
        assert 'holds at robot_state' not in error['recovery']
        # Refusing a later command cannot clear an earlier genuinely unknown execution.
        f.control_uncertain = True
        error = json.loads(f.call('mock_move', {})['content'][0]['text'])['error']
        assert error['control_uncertain'] is True
    finally:
        f.close()
