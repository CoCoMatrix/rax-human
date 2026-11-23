"""Run saved-evidence/fake-context tests with live Context and sockets blocked.

Run from the workspace root: python code/magnetic_blocks/verify_offline.py
No live runtime is imported or constructed. Hardware and model calls are not tests.
"""
import argparse
import pathlib
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


def forbidden(*args, **kwargs):
    raise AssertionError('Offline verification forbids live Context/network access')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    choice=parser.add_mutually_exclusive_group()
    choice.add_argument('--portable',action='store_true',help='Inherited self-contained tests; does not qualify missing historical fixture tests')
    choice.add_argument('--composition',action='store_true',help='Composition contract tests only, with synthetic scene/vision checks')
    args=parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    if pathlib.Path.cwd().resolve() != root:
        raise SystemExit('Run from workspace root so saved evidence paths resolve')
    (root / 'tmp').mkdir(exist_ok=True)
    tempfile.tempdir = str(root / 'tmp')
    runtime = types.ModuleType('api.runtime')
    runtime.Context = forbidden
    with patch.dict(sys.modules, {'api.runtime': runtime}), \
         patch('socket.socket', side_effect=forbidden), \
         patch('socket.create_connection', side_effect=forbidden):
        suite = (unittest.defaultTestLoader.loadTestsFromNames(['test_composition']) if args.composition else
            unittest.defaultTestLoader.loadTestsFromNames([
            'test_workflow','test_native_budget','test_perception','test_scene_checks',
            'test_occlusion_repair','test_geometry_reuse','test_r8_stack','test_closeout_r8']) if args.portable else
            unittest.defaultTestLoader.discover(str(pathlib.Path(__file__).parent), pattern='test_*.py'))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
