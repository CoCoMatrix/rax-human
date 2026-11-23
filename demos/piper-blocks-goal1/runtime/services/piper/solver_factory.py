"""Piper-local selection; unselected candidates keep the existing KDL implementation."""
from .moveit import MoveIt
def make_solver(log_root, options):
    name=options.get('ik_solver','kdl')
    if name=='kdl':
        solver=MoveIt(log_root,options.get('moveit_manifest'));solver.name='kdl';return solver
    if name=='ssik_rank1':
        from .ssik_solver import SSIK
        return SSIK(log_root)
    raise ValueError('Unsupported Piper IK solver: '+str(name))
