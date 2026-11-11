"""Pinned Piper CAD FK, both arms; metres, radians, flange (not grasp TCP)."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ASSETS = Path(__file__).parent / 'assets'
URDF_SHA = '12e4e447bffcbbfebca677fb96fd6062d122b1c2987da65187aaa3caa1bb330d'


def vector(value, count):
    if not isinstance(value, (list, tuple)) or len(value) != count or any(type(v) not in (float, int) for v in value):
        raise ValueError(f'Expected {count} finite numbers')
    a = np.asarray(value, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Non-finite vector')
    return a


class Kinematics:
    def __init__(self):
        raw = (ASSETS / 'dual_piper_x.urdf').read_bytes()
        if hashlib.sha256(raw).hexdigest() != URDF_SHA:
            raise ValueError('Piper CAD identity changed')
        root = ET.fromstring(raw)
        self.chains = {}
        for side in ('left', 'right'):
            chain = []
            for i in range(1, 7):
                j = root.find(f"joint[@name='{side}_joint{i}']")
                origin = j.find('origin'); t = np.eye(4)
                t[:3, 3] = np.fromstring(origin.attrib['xyz'], sep=' ')
                t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.attrib['rpy'], sep=' ')).as_matrix()
                chain.append((t, np.fromstring(j.find('axis').attrib['xyz'], sep=' '),
                              float(j.find('limit').attrib['lower']), float(j.find('limit').attrib['upper'])))
            self.chains[side] = chain

    def check(self, q, side='left'):
        if side not in self.chains: raise ValueError('Unknown arm')
        a = vector(q, 6)
        if any(not lo <= v <= hi for v, (_, _, lo, hi) in zip(a, self.chains[side])):
            raise ValueError('Joint target outside CAD limits')
        return a

    def fk(self, q, side='left'):
        q = self.check(q, side); t = np.eye(4)
        for angle, (origin, axis, _, _) in zip(q, self.chains[side]):
            r = np.eye(4); r[:3, :3] = Rotation.from_rotvec(angle * axis).as_matrix()
            t = t @ origin @ r
        return t
