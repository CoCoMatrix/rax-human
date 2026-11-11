"""Strict API-output shape for the existing proposal-bound PiperReviewer.

This is serialization only: it neither chooses an action nor grants dispatch.
Nullable placeholders satisfy a strict structured-output API; only unused null
placeholders are removed. Non-null wrong-mode controls are always rejected.
The existing PiperReviewer remains authoritative for outcome/intent gating,
request binding, freshness, PiPER FK/IK, candidate bounds and v9 limits.
"""
from __future__ import annotations

from collections.abc import Mapping
import copy
import json
import math

from .contracts import require


def _obj(properties):
    return {'type': 'object', 'additionalProperties': False,
            'properties': properties, 'required': list(properties)}


def _nullable(schema):
    return {'anyOf': [schema, {'type': 'null'}]}


def response_schema(binding):
    """Return the PiPER schema, not the incompatible ARX boolean-gripper schema."""
    binding = dict(binding)
    required = {'request_id', 'observation_sequence', 'observation_timestamp_ns',
                'task', 'proposal_sha256', 'step_id'}
    require(set(binding) == required, 'expected complete proposal review binding')
    properties = {}
    for key, value in binding.items():
        require(type(value) in (str, int), 'binding fields must be strings/integers')
        properties[key] = {'type': 'integer' if type(value) is int else 'string', 'enum': [value]}
    text = {'type': 'string'}
    vector = lambda size: {'type': 'array', 'items': {'type': 'number'},
                           'minItems': size, 'maxItems': size}
    gripper = {'anyOf': [{'type': 'string', 'enum': ['keep']},
                         _obj({'width_m': {'type': 'number', 'minimum': 0}})]}
    def arms(mode):
        return _obj({arm: _obj({'link': {'type': 'string', 'enum': [link]},
                 **({'delta_position_m': vector(3), 'delta_rotation_vector_rad': vector(3)}
                    if mode == 'edit' else {'position_m': vector(3), 'quaternion_wxyz': vector(4)}),
                 'gripper': gripper}) for arm, link in
                 (('left_arm', 'left_link6'), ('right_arm', 'right_link6'))})
    assessment = _obj({
        'task_progress': _obj({'verified_completed': {'type': 'array', 'items': text},
                              'currently_attempting': text,
                              'remaining': {'type': 'array', 'items': text}}),
        'current_subgoal': text,
        'execution_status': {'type': 'string', 'enum':
            ['not_started', 'progressing', 'failed', 'uncertain', 'recovered']},
        'execution_evidence': text, 'expected_next_intent': text, 'predicted_next_intent': text,
        'intent_status': {'type': 'string', 'enum': ['aligned', 'misaligned', 'uncertain']},
        'intent_evidence': text})
    return _obj({'binding': _obj(properties),
                 'mode': {'type': 'string', 'enum': ['student', 'edit', 'eef', 'stop']},
                 'steps': _nullable({'type': 'integer', 'minimum': 1, 'maximum': 16}),
                 'reason': text, 'assessment': assessment,
                 'coordinate_frame': _nullable({'type': 'string', 'enum': ['dual_base_link']}),
                 'edit': _nullable(arms('edit')), 'target': _nullable(arms('eef'))})


def normalize_response(response, binding):
    """Reject malformed API results; strip only explicit unused null placeholders.

    Pass the returned mapping into PiperReviewer.review(...). This function
    alone provides no action acceptance or motion authorization.
    """
    schema = response_schema(binding)
    require(isinstance(response, Mapping) and set(response) == set(schema['properties']),
            'teacher response requires exactly the declared fields')
    try:
        actual_binding = json.dumps(response['binding'], sort_keys=True, allow_nan=False)
        expected_binding = json.dumps(dict(binding), sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError('invalid teacher binding') from error
    require(actual_binding == expected_binding, 'teacher binding differs from reviewed proposal')
    mode = response['mode']
    require(isinstance(mode, str) and mode in ('student', 'edit', 'eef', 'stop'), 'unknown teacher mode')
    require(isinstance(response['reason'], str) and response['reason'].strip(), 'visible reason required')
    assessment = response['assessment']
    require(isinstance(assessment, Mapping) and set(assessment) == set(
        schema['properties']['assessment']['properties']), 'assessment fields differ from outcome/intent contract')
    progress = assessment['task_progress']
    require(isinstance(progress, Mapping) and set(progress) ==
            {'verified_completed', 'currently_attempting', 'remaining'}, 'invalid task progress fields')
    require(isinstance(progress['currently_attempting'], str) and progress['currently_attempting'].strip(),
            'currently attempting text required')
    for field in ('verified_completed', 'remaining'):
        require(isinstance(progress[field], list) and all(isinstance(item, str) and item.strip()
                for item in progress[field]), 'progress entries must be nonempty strings')
    for field in ('current_subgoal', 'execution_evidence', 'expected_next_intent',
                  'predicted_next_intent', 'intent_evidence'):
        require(isinstance(assessment[field], str) and assessment[field].strip(), 'visible assessment text required')
    require(assessment['execution_status'] in ('not_started', 'progressing', 'failed', 'uncertain', 'recovered')
            and assessment['intent_status'] in ('aligned', 'misaligned', 'uncertain'), 'invalid assessment status')
    if mode == 'stop':
        require(response['steps'] is None, 'stop steps must be null')
    else:
        require(type(response['steps']) is int and 1 <= response['steps'] <= (16 if mode == 'student' else 5),
                'teacher step count exceeds mode cap')
    selected = {'edit': 'edit', 'eef': 'target'}.get(mode)
    for field in ('edit', 'target'):
        if field != selected:
            require(response[field] is None, 'wrong-mode controls must be null')
    if selected is None:
        require(response['coordinate_frame'] is None, 'non-correction coordinate frame must be null')
    else:
        require(response['coordinate_frame'] == 'dual_base_link', 'explicit PiPER base frame required')
        targets = response[selected]
        require(isinstance(targets, Mapping) and set(targets) == {'left_arm', 'right_arm'},
                'both PiPER arms must be explicit')
        vectors = {'delta_position_m': 3, 'delta_rotation_vector_rad': 3} if mode == 'edit' else {
                   'position_m': 3, 'quaternion_wxyz': 4}
        for arm, link in (('left_arm', 'left_link6'), ('right_arm', 'right_link6')):
            item = targets[arm]
            require(isinstance(item, Mapping) and set(item) == set(vectors) | {'link', 'gripper'}
                    and item['link'] == link, 'expected explicit PiPER flange controls')
            for field, size in vectors.items():
                value = item[field]
                require(isinstance(value, list) and len(value) == size and all(type(x) in (int, float)
                        and math.isfinite(x) for x in value), 'pose vectors require finite numbers')
            grip = item['gripper']
            require(grip == 'keep' or isinstance(grip, Mapping) and set(grip) == {'width_m'}
                    and type(grip['width_m']) in (int, float) and math.isfinite(grip['width_m'])
                    and grip['width_m'] >= 0, 'gripper must be keep or explicit width_m')
    result = copy.deepcopy(dict(response))
    for field in ('steps', 'coordinate_frame', 'edit', 'target'):
        if result[field] is None:
            result.pop(field)
    return result
