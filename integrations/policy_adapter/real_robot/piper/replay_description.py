"""Re-evaluate saved shadow observations/actions with the supplied v9 geometry.

Offline only: no policy connection, operator API, camera, CAN or action return.
The original capture timestamp is passed as the validation reference time; this
is a historical data check, never a claim that saved images are current.
"""
import argparse
import json
from pathlib import Path
import numpy as np

from .contracts import PiperContract, CAMERA_ROLES, vector_schema
from .description import PiperDescription
from .description_config import mast_configuration
from .backend import PiperXPolicyBackend, write_json


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime', type=Path, required=True)
    p.add_argument('--source-shadow', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--description-assets', type=Path,
        default=Path(__file__).resolve().parents[2] / 'assets/robots/piper/global-d435-fixed-v12')
    p.add_argument('--mast-joint-rad', type=float, default=None)
    args = p.parse_args()
    old_run = json.loads((args.source_shadow / 'run.json').read_text())
    contract = PiperContract.from_runtime(args.runtime)
    description = PiperDescription(contract, args.description_assets)
    mast = mast_configuration(args.mast_joint_rad)
    backend = PiperXPolicyBackend(contract, None, args.output, task=old_run['task'],
        identity=old_run['student_identity'], description=description,
        mast_joint_rad=mast['joint_position_rad'], mode='offline_replay', mast_metadata=mast)
    reports = []
    previous_sequence = None
    for folder in sorted(args.source_shadow.glob('proposal_*')):
        record = json.loads((folder / 'proposal.json').read_text())
        with np.load(folder / 'observation.npz', allow_pickle=False) as arrays:
            wire = {'data_format_version': 'xpolicylab.real_world.v1',
                'instruction': record['task'],
                'state': {'vector': arrays['state14'], 'schema': vector_schema()},
                'vision': {role: {'color': arrays[role], 'encoding': 'rgb8'} for role in CAMERA_ROLES},
                'additional_info': {'robot_type': 'piper_x_dual', 'robot_id': contract.robot_id,
                    'timestamp_ns': record['observation_timestamp_ns'],
                    'sequence': record['observation_sequence']}}
            observation = contract.validate_observation(wire, max_age_ms=1,
                now_ns=record['observation_timestamp_ns'], previous_sequence=previous_sequence)
        previous_sequence = observation.sequence
        with np.load(folder / 'actions.npz', allow_pickle=False) as arrays:
            actions = arrays['raw_actions']
        diagnostics, _ = backend.diagnostics(observation, actions)
        report = {'mode': 'offline_replay', 'source': str(folder),
                  'observation_sequence': observation.sequence, 'original_capture_timestamp_ns': observation.timestamp_ns,
                  'hardware_access': False, 'model_inference_called': False, **diagnostics}
        write_json(args.output / f'{folder.name}.json', report)
        reports.append(report)
    summary = {'mode': 'offline_replay', 'source_shadow': str(args.source_shadow),
        'records': len(reports), 'identity': dict(description.identity),
        'mast_configuration': mast, 'hardware_access': False,
        'model_inference_called': False,
        'v9_intersection_preview_valid_count': sum(r['v9_intersection_preview_valid'] for r in reports),
        'complete': bool(reports) and all(r['effective_preview_exact_limits_valid']
                                        and r['v9_intersection_preview_valid'] for r in reports)}
    write_json(args.output / 'summary.json', summary)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
