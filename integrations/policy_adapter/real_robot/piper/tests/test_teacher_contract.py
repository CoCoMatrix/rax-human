import copy
import pytest
from real_robot.piper.teacher_contract import response_schema, normalize_response

BINDING = dict(request_id='r1', observation_sequence=1, observation_timestamp_ns=1234,
               task='Pick up the bottle.', proposal_sha256='a'*64, step_id=0)


def response(mode='student'):
    value = dict(binding=dict(BINDING), mode=mode, steps=1, reason='The new proposal approaches the bottle.',
        coordinate_frame=None, edit=None, target=None, assessment=dict(
        task_progress=dict(verified_completed=[], currently_attempting='Approach the bottle.', remaining=['Grasp.']),
        current_subgoal='Approach.', execution_status='not_started', execution_evidence='No action has executed.',
        expected_next_intent='Approach the bottle.', predicted_next_intent='Approach the bottle.',
        intent_status='aligned', intent_evidence='The flange trajectory moves toward the visible bottle.'))
    if mode == 'stop':
        value['steps'] = None
    if mode in ('edit', 'eef'):
        value['coordinate_frame'] = 'dual_base_link'
        value['assessment']['intent_status'] = 'misaligned'
        field = 'edit' if mode == 'edit' else 'target'
        value[field] = {arm: dict(link=link, gripper='keep', **(
            dict(delta_position_m=[0, 0, 0.01], delta_rotation_vector_rad=[0, 0, 0]) if mode == 'edit'
            else dict(position_m=[0.2, 0.1, 0.3], quaternion_wxyz=[1, 0, 0, 0])))
            for arm, link in (('left_arm', 'left_link6'), ('right_arm', 'right_link6'))}
    return value


@pytest.mark.parametrize('mode', ['student', 'edit', 'eef', 'stop'])
def test_modes_only_drop_unused_nulls(mode):
    original = response(mode)
    before = copy.deepcopy(original)
    result = normalize_response(original, BINDING)
    assert result['binding'] == BINDING and result['mode'] == mode
    assert all(value is not None for value in result.values())
    assert original == before
    assert ('edit' in result) == (mode == 'edit')
    assert ('target' in result) == (mode == 'eef')
    assert ('steps' in result) == (mode != 'stop')


def test_strict_schema_requires_every_declared_object_property():
    def walk(schema):
        if schema.get('type') == 'object':
            assert schema['additionalProperties'] is False
            assert set(schema['properties']) == set(schema['required'])
            for value in schema['properties'].values(): walk(value)
        for item in schema.get('anyOf', []): walk(item)
    walk(response_schema(BINDING))


@pytest.mark.parametrize('mutation,reason', [
    (lambda r: r['binding'].update(observation_sequence=True), 'binding differs'),
    (lambda r: r.update(steps=True), 'step count'),
    (lambda r: r.update(edit={'unsafe': 1}), 'wrong-mode'),
    (lambda r: r.update(coordinate_frame='environment_origin'), 'coordinate frame'),
    (lambda r: r.update(unexpected=1), 'exactly'),
    (lambda r: r['assessment'].update(execution_evidence=''), 'assessment text'),
])
def test_wrong_api_shapes_are_not_silently_repaired(mutation, reason):
    r = response(); mutation(r)
    with pytest.raises(ValueError, match=reason): normalize_response(r, BINDING)


@pytest.mark.parametrize('mode', ['edit', 'eef'])
def test_reject_arx_grippers_and_nonfinite_vectors(mode):
    field = 'edit' if mode == 'edit' else 'target'
    r = response(mode); r[field]['left_arm']['gripper'] = True
    with pytest.raises(ValueError, match='width_m'): normalize_response(r, BINDING)
    r = response(mode); key = 'delta_position_m' if mode == 'edit' else 'position_m'
    r[field]['left_arm'][key][0] = float('nan')
    with pytest.raises(ValueError, match='finite numbers'): normalize_response(r, BINDING)
    r = response(mode); r['steps'] = 6
    with pytest.raises(ValueError, match='mode cap'): normalize_response(r, BINDING)


def test_schema_is_not_a_gate_or_motion_authorization():
    r = response('edit'); r['assessment']['intent_status'] = 'uncertain'
    # Existing PiperReviewer will reject uncertainty-only takeover. Parsing it
    # does not substitute for review or authorize a fallback/student command.
    result = normalize_response(r, BINDING)
    assert result['assessment']['intent_status'] == 'uncertain'
    assert 'dispatch_authorized' not in result
