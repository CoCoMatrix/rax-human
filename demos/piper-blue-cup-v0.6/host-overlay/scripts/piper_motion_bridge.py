"""Run the existing bridge with this repository's corrected passive policy.

The installed lease/SDK owner remain unchanged; settling is configured locally. No server starts on
import. Used only by piper_explore --run, never by Prepare or offline tests.
"""
import importlib.util
from pathlib import Path
import sys
import types
import importlib


def load_bridge(legacy, speed_profile='standard'):
    if speed_profile not in ('standard','faster'):
        raise ValueError('Unknown Piper motion speed profile')
    sys.path.insert(0,str(legacy))
    directory=Path(__file__).resolve().parents[1]/'services/piper'
    spec=importlib.util.spec_from_file_location('services.piper.motion_bridge_runtime',directory/'motion_bridge_runtime.py')
    motion_bridge=importlib.util.module_from_spec(spec);spec.loader.exec_module(motion_bridge)
    package=types.ModuleType('_rax_installed_piper')
    package.__path__=[str(directory)]
    sys.modules[package.__name__]=package
    settling=importlib.import_module(package.__name__+'.installed_settling')
    sys.modules['services.piper.installed_settling']=settling
    for name in ('motion_transport','controlled_policy'):
        spec=importlib.util.spec_from_file_location('services.piper.'+name,directory/(name+'.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        sys.modules[spec.name]=module
        if name=='motion_transport':motion_bridge.MotionTransport=settling.transport_class(module.MotionTransport)
        else:motion_bridge.ControlledPolicy=module.ControlledPolicy
    adapter=importlib.import_module(package.__name__+'.installed_collision')
    motion_bridge.CollisionChecker=adapter.checker_class(motion_bridge.CollisionChecker)
    profiles=importlib.import_module('services.piper.motion_profiles')
    profiles.TASK60={**profiles.TASK60,'joint_tolerance_rad':settling.JOINT_STATE_TOLERANCE_RAD,'max_eef_translation_m':.4}
    if speed_profile=='faster':
        import math
        # Operator-selected candidate, not a hardware qualification. Keep the
        # tracking reserve, jaw bounds, displacement bounds and settling intact.
        profiles.TASK60.update(max_joint_velocity_rad_s=math.radians(20),
            max_joint_acceleration_rad_s2=math.radians(40))
    return motion_bridge


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--speed-profile',choices=('standard','faster'),default='standard')
    selected,remaining=parser.parse_known_args()
    sys.argv=[sys.argv[0]]+remaining
    from piper_observe_only import OLD
    load_bridge(OLD,selected.speed_profile).main()
