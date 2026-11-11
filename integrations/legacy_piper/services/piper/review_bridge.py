"""RAX -> operator-reviewed single-joint probe inbox. No motion or auto-approval."""
import hashlib
import json
import math
import time
import uuid
from pathlib import Path

ROOT = Path('/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/commissioning/review-queue-20261026')
TOKEN = Path('/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025/commissioning/manual-web-20261026/private/writable-owner-token')


class ReviewBridge:
    def __init__(self, owner, geometry, collision, *, root=ROOT, stop=None):
        self.owner, self.geometry, self.collision = owner, geometry, collision
        self.root = Path(root)
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.stop = stop

    def propose(self, joint, delta_rad):
        if type(joint) is not int or not 1 <= joint <= 6:
            raise ValueError('Only left joints 1..6 supported')
        if type(delta_rad) not in (int,float) or not math.isfinite(delta_rad) or not 0 < abs(delta_rad) <= .005:
            raise ValueError('One native probe, nonzero delta <= .005 rad')
        for p in self.root.glob('*/request.json'):
            if not (p.parent/'result.json').exists() and not (p.parent/'cancelled.json').exists():
                raise ValueError('An operator review is already pending; no repeated requests')
        state = self.owner.state()
        if state['robot_status']['hardware_write_enabled']:
            raise ValueError('Proposal stage requires no-write owner')
        anchor = state['robot_state']['values']
        target = list(anchor); target[joint-1] += float(delta_rad)
        from .controlled_transport import ControlledTransport
        transport = ControlledTransport(self.geometry,self.collision,
            arm_check=lambda: {'armed':False},stop_client=lambda: {})
        plan = transport.preview(anchor,target,3.0,native_single_target=True)
        after = self.owner.state()
        if (after['robot_status']['generation'] != state['robot_status']['generation']
                or after['robot_status']['hardware_write_enabled'] is not False
                or max(abs(a-b) for a,b in zip(after['robot_state']['values'],anchor)) > .001):
            raise ValueError('Owner or measured anchor changed during probe preview')
        request_id = uuid.uuid4().hex
        directory = self.root/request_id; directory.mkdir(mode=0o700)
        record = dict(request_id=request_id,created_ns=time.time_ns(),kind='left_joint_identification',
                      joint=joint,delta_rad=delta_rad,owner_generation=state['robot_status']['generation'],
                      plan=plan,executed=False,operator_approved=False,physical_calibration_validated=False,
                      requirements=['fresh scene review','same measured anchor','native owner preview',
                                    'private short-lived approval','measured arrival and stop receipt'])
        (directory/'request.json').write_text(json.dumps(record,indent=2))
        return dict(request_id=request_id,status='AWAITING_OPERATOR_REVIEW',executed=False,
                    automatic_release=False,target14=target,plan_sha256=plan['sha256'])

    def status(self, request_id):
        if not isinstance(request_id,str) or len(request_id)!=32 or any(c not in '0123456789abcdef' for c in request_id):
            raise ValueError('Invalid request identity')
        directory=self.root/request_id
        if not (directory/'request.json').exists():raise ValueError('Unknown request')
        for name in ('result.json','cancelled.json'):
            p=directory/name
            if p.exists():return json.loads(p.read_text())
        return dict(request_id=request_id,status='AWAITING_OPERATOR_REVIEW',executed=False)

    def cancel_pending(self, reason):
        for p in self.root.glob('*/request.json'):
            terminal=p.parent/'cancelled.json'
            if not (p.parent/'result.json').exists() and not terminal.exists():
                with terminal.open('x') as stream:
                    json.dump(dict(status='CANCELLED',executed=False,reason=reason,time_ns=time.time_ns()),stream)
        return dict(pending_requests_cancelled=True)

    def hold(self):
        self.cancel_pending('operator Hold')
        if self.stop is None:
            raise RuntimeError('Operator stop channel is not configured')
        return dict(self.stop(),pending_requests_cancelled=True)
