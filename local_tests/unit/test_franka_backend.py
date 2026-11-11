"""Franka backend logic against a simulated bridge: nothing here touches hardware."""
import json
from pathlib import Path

import cv2 as cv
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from services.controller.config import Configuration
from services.franka import arming
from services.franka.backend import HOME_Q, FrankaBackend
from services.franka.bridges import TargetRejected
from services.franka.calibration import project
from services.franka.kinematics import flange_pose, link8_to_tcp, tcp_pose
from services.franka.servo import MotionStopped, PoseServo, ServoSettings

START_Q = np.array([-0.0169, -0.6314, -0.0054, -2.3977, -0.0217, 1.7756, 0.7776])


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class FakeArm:
    """First-order tracking that settles `sag` short of the command, like the DROID IK."""

    def __init__(self, q=START_Q, sag=(0.002, -0.003, -0.005), on_step=None):
        F = flange_pose(q)
        self.q = np.asarray(q, dtype=float)
        self.pos = F[:3, 3].copy()
        self.rot = Rotation.from_matrix(F[:3, :3])
        self.sag = np.asarray(sag)
        self.commands = 0
        self.holds = 0
        self.on_step = on_step
        self.reject = False

    def state(self):
        return {'arm_pos': self.pos.copy(), 'arm_quat': self.rot.as_quat(), 'arm_joint_position': self.q.copy(),
                'policy_running': True}

    def set_target(self, position, quaternion):
        if self.reject:
            raise TargetRejected('IK target rejected')
        self.commands += 1
        goal = np.asarray(position) + self.sag
        step = goal - self.pos
        self.pos = self.pos + step * min(1.0, 0.0025 / max(np.linalg.norm(step), 1e-9)) \
            if np.linalg.norm(step) > 0.0025 else goal  # <= 50 mm/s at 20 Hz
        delta = (Rotation.from_quat(quaternion) * self.rot.inv()).as_rotvec()
        self.rot = Rotation.from_rotvec(delta * 0.3 + Rotation.from_rotvec([0, 0, 0.002]).as_rotvec()) * self.rot
        if self.on_step:
            self.on_step(self)

    def hold(self):
        self.holds += 1

    def move_joints(self, positions, time_to_go):
        self.q = np.asarray(positions, dtype=float)
        F = flange_pose(self.q)
        self.pos, self.rot = F[:3, 3].copy(), Rotation.from_matrix(F[:3, :3])


class FakeGripper:
    def __init__(self):
        self.width = 0.08
        self.grasped = False
        self.calls = []

    def state(self):
        return {'width': self.width, 'max_width': 0.0801, 'is_moving': False, 'is_grasped': self.grasped,
                'stale': False}

    def move(self, width, speed, force=0.1, blocking=True):
        self.calls.append(('move', width))
        self.width = width
        return self.state()

    def grasp(self, width, speed, force, blocking=True):
        self.calls.append(('grasp', width, force))
        self.width, self.grasped = 0.05, True
        return self.state()


class FakeCameras:
    def close(self):
        pass


@pytest.fixture
def make_backend(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    extrinsic = tmp_path/'cam.yaml'
    yaml.safe_dump({'matrix': np.eye(4).tolist()}, extrinsic.open('w'))

    def build(arm=None, **options):
        root = tmp_path/'sessions'/'s1'
        root.mkdir(parents=True, exist_ok=True)
        (root/'operator-input.json').write_text('{}')
        cameras = [{'name': 'left', 'serial': 1, 'mount': 'fixed', 'extrinsics': str(extrinsic)}]
        config = Configuration(root=root, task='t', robot='franka', backend_options={'cameras': cameras, **options})
        arm = arm or FakeArm()
        clock = Clock()
        servo = PoseServo(arm, ServoSettings(), clock=clock, sleep=clock.sleep)
        return FrankaBackend(config, arm=arm, gripper=FakeGripper(), cameras=FakeCameras(), servo=servo), arm, root
    return build


def test_cameras_come_from_the_operator_configuration(make_backend, tmp_path, monkeypatch):
    """No built-in cameras: a session without them is refused; the shipped example, with its
    relative extrinsics resolved under FRANKA_CALIBRATION_DIR, builds every camera model."""
    with pytest.raises(ValueError, match="option 'cameras' is required"):
        make_backend(cameras=None)
    example = json.loads((Path(__file__).resolve().parents[2]/'examples/franka.json').read_text())
    for spec in example['cameras']:
        assert not Path(spec['extrinsics']).is_absolute()
    with pytest.raises(ValueError, match='needs FRANKA_CALIBRATION_DIR'):
        make_backend(cameras=example['cameras'])
    calibration = tmp_path/'calibration'
    calibration.mkdir()
    shift = np.eye(4)
    shift[:3, 3] = [0.1, 0.2, 0.3]
    for spec in example['cameras']:
        yaml.safe_dump({'matrix': shift.tolist()}, (calibration/spec['extrinsics']).open('w'))
    monkeypatch.setenv('FRANKA_CALIBRATION_DIR', str(calibration))
    options = {k: v for k, v in example.items() if k != 'cameras'}
    backend, _, _ = make_backend(cameras=example['cameras'], **options)
    assert {name: (model.serial, model.mount) for name, model in backend.models.items()} == {
        'left': (11111111, 'fixed'), 'right': (22222222, 'fixed'), 'wrist': (33333333, 'tcp')}
    assert np.allclose(backend.models['left'].base_from_camera()[:3, 3], [-0.1, -0.2, -0.3])
    assert backend.options['protected_paths'] == [] and backend.options['camera_cpus'] == '0-15'


def test_launcher_options_are_accepted_and_unknown_ones_refused(make_backend):
    backend, _, _ = make_backend(agent_python='/opt/agent-env/bin/python', protected_paths=['/opt/robot'])
    assert backend.options['agent_python'] == '/opt/agent-env/bin/python'
    with pytest.raises(ValueError, match="unknown franka backend options: \\['camera_serials'\\]"):
        make_backend(camera_serials=[1])


def tcp_now(arm):
    return link8_to_tcp(arm.pos, arm.rot.as_quat())


def test_kinematics_round_trip():
    T = tcp_pose(START_Q)
    p, q = link8_to_tcp(flange_pose(START_Q)[:3, 3], Rotation.from_matrix(flange_pose(START_Q)[:3, :3]).as_quat())
    assert np.allclose(p, T[:3, 3])
    # The hand is 0.1034 m beyond the flange along the flange z axis.
    assert np.isclose(np.linalg.norm(T[:3, 3] - flange_pose(START_Q)[:3, 3]), 0.1034)


def test_motion_refused_until_armed(make_backend):
    backend, arm, root = make_backend()
    with pytest.raises(PermissionError, match='not armed'):
        backend.validate('franka_move_pose', {'position': [0, 0, 0.01], 'relative': True})
    assert arm.commands == 0
    arming.write(root, True)
    backend.validate('franka_move_pose', {'position': [0, 0, 0.01], 'relative': True})
    arming.write(root, True, minutes=-1)
    assert arming.read(root)['armed'] is False


def test_integral_servo_removes_steady_sag(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    start, quat = tcp_now(arm)
    goal = start + [0.05, -0.03, 0.02]
    value, images = backend.call('franka_move_pose', {'position': goal.tolist()})
    reached, _ = tcp_now(arm)
    assert value['reached'] and np.linalg.norm(reached - goal) < 0.0015
    assert value['rotation_error_deg'] < 0.6  # orientation held despite the drift
    assert images == [] and arm.holds >= 1


def test_targets_outside_box_or_too_far_are_refused(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    start, _ = tcp_now(arm)
    with pytest.raises(ValueError, match='outside the workspace box'):
        backend.validate('franka_move_pose', {'position': [start[0], start[1], 0.05]})
    with pytest.raises(ValueError, match='per call'):
        backend.validate('franka_move_pose', {'position': [0.75, 0.40, 0.65]})
    assert arm.commands == 0


def test_disarm_stops_a_running_move(make_backend):
    holder = {}

    def disarm_midway(arm):
        if arm.commands == 10:
            arming.write(holder['root'], False)
    backend, arm, root = make_backend(arm=FakeArm(on_step=disarm_midway))
    holder['root'] = root
    arming.write(root, True)
    with pytest.raises(MotionStopped, match='disarmed'):
        backend.call('franka_move_pose', {'position': [0, 0, 0.1], 'relative': True})
    assert arm.commands == 10


def test_rejected_target_is_a_wall(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    arm.reject = True
    with pytest.raises(TargetRejected, match='wall') as caught:
        backend.call('franka_move_pose', {'position': [0, 0, 0.01], 'relative': True})
    assert arm.holds >= 1
    assert caught.value.control_certain and caught.value.robot_state['distance_to_target_mm'] > 5


def test_joint_path_checked_by_fk(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    backend.validate('franka_move_joints', {'preset': 'home'})
    low = START_Q.copy()
    low[1] += 1.2  # shoulder forward: the hand dives into the table (TCP z ~0.03)
    with pytest.raises(ValueError, match='outside the workspace box|link to z'):
        backend.validate('franka_move_joints', {'positions': low.tolist()})
    with pytest.raises(ValueError, match='per call'):
        backend.validate('franka_move_joints', {'positions': (START_Q + [1.6, 0, 0, 0, 0, 0, 0]).tolist()})
    value, _ = backend.call('franka_move_joints', {'preset': 'home'})
    assert np.allclose(arm.q, HOME_Q) and value['max_joint_error_rad'] == 0


def test_gripper_requires_arming_and_grasps(make_backend):
    backend, arm, root = make_backend()
    with pytest.raises(PermissionError):
        backend.call('franka_gripper', {'action': 'open'})
    arming.write(root, True)
    value, _ = backend.call('franka_gripper', {'action': 'grasp', 'width': 0.0, 'force': 30})
    assert value['is_grasped'] and backend.gripper.calls == [('grasp', 0.0, 30.0)]


def test_pixel_to_position_inverts_projection(make_backend):
    backend, arm, root = make_backend()
    frame = backend.frames_root/'abc123'
    frame.mkdir(parents=True)
    K = [[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]]
    T_base_cam = np.eye(4)
    T_base_cam[:3, 3] = [0.5, 0.0, 1.0]
    T_base_cam[:3, :3] = Rotation.from_euler('x', 180, degrees=True).as_matrix()  # looking down
    point = np.array([0.55, 0.05, 0.12])
    u, v = project(point, T_base_cam, K)
    depth = np.zeros((720, 1280), np.uint16)
    depth[:] = round((1.0 - 0.12) * 1000)
    cv.imwrite(str(frame/'left_depth.png'), depth)
    (frame/'meta.json').write_text(json.dumps({'cameras': {'left': {
        'depth': 'left_depth.png', 'K': K, 'width': 1280, 'height': 720, 'T_base_cam': T_base_cam.tolist()}}}))
    value, _ = backend.call('franka_pixel_to_position', {'frame_id': 'abc123', 'camera': 'left',
                                                         'pixels': [[u, v], [5000, 1]]})
    assert np.allclose(value['points'][0]['position'], point, atol=1e-3)
    assert value['points'][1]['error'] == 'outside the image'
    with pytest.raises(ValueError):
        backend.validate('franka_pixel_to_position', {'frame_id': '../x', 'camera': 'left', 'pixels': [[1, 1]]})


def test_unknown_options_are_rejected(make_backend):
    with pytest.raises(ValueError, match='unknown franka backend options'):
        make_backend(max_speed=1)


def test_ik_precheck_refuses_poses_too_close_to_a_joint_limit(make_backend):
    backend, arm, root = make_backend(arm=FakeArm(q=HOME_Q, sag=(0, 0, 0)))
    arming.write(root, True)
    start, quat = tcp_now(arm)
    # Low and close to the base: the elbow (J4) folds into its lower limit.
    with pytest.raises(ValueError, match='J4 elbow'):
        backend.validate('franka_move_pose', {'position': [0.33, 0.0, 0.15], 'quaternion': quat.tolist()})
    backend.validate('franka_move_pose', {'position': [0.45, 0.0, 0.15], 'quaternion': quat.tolist()})
    assert arm.commands == 0


def write_stream(stream, now_ms, seconds=30, fps=2):
    """Synthetic recording: left frames whose brightness encodes their age, plus a robot log."""
    from services.franka.camera_server import minute_name
    for k in range(seconds * fps):
        t = now_ms - int(k * 1000 / fps)
        folder = stream/'left'/minute_name(t / 1000)
        folder.mkdir(parents=True, exist_ok=True)
        cv.imwrite(str(folder/f'{t}.jpg'), np.full((72, 128, 3), min(255, k * 4), np.uint8))
    (stream/'robot').mkdir(parents=True, exist_ok=True)
    for k in range(seconds * 10):
        t = now_ms - k * 100
        with open(stream/'robot'/f'{minute_name(t / 1000)}.jsonl', 'a') as log:
            log.write(json.dumps({'t_ms': t, 'tcp_position': [0.4, 0.0, 0.3 + k * 0.001], 'gripper_width': 0.08}) + '\n')


def test_stream_tool_returns_frames_from_the_past(make_backend):
    import time as clock
    backend, arm, root = make_backend()
    backend.stream = root/'stream'
    now_ms = int(clock.time() * 1000)
    write_stream(backend.stream, now_ms)
    value, images = backend.call('franka_stream', {'cameras': ['left'], 'seconds_ago': [0, 10]})
    assert len(images) == 2 and [f['seconds_ago'] for f in value['frames']] == [0, 10]
    old = value['frames'][1]
    assert abs(old['images'][0]['age_s'] - 10) < 0.6 and old['images'][0]['file'].startswith('runtime/stream/left/')
    assert abs(old['robot']['tcp_position'][2] - 0.4) < 0.002  # the arm 10 s ago (100 records back)
    value, images = backend.call('franka_stream', {'cameras': ['left'], 'sheet': {'seconds': 20, 'count': 8}})
    assert len(images) == 1 and len(value['sheets'][0]['frame_ages_s']) == 8
    with pytest.raises(ValueError, match='only the last'):
        backend.call('franka_stream', {'seconds_ago': [5000]})
    assert arm.commands == 0  # read-only


def test_stream_minutes_older_than_the_window_are_pruned(tmp_path):
    import time as clock
    from services.franka.camera_server import Cameras, minute_name
    cameras = Cameras.__new__(Cameras)
    cameras.stream, cameras.window_s = tmp_path, 600
    now = clock.time()
    for age in (0, 300, 700, 1300):
        (tmp_path/'left'/minute_name(now - age)).mkdir(parents=True, exist_ok=True)
    (tmp_path/'robot').mkdir()
    for age in (0, 1300):
        (tmp_path/'robot'/f'{minute_name(now - age)}.jsonl').write_text('{}\n')
    cameras.prune(now)
    kept = sorted(p.name for p in (tmp_path/'left').iterdir())
    assert kept == sorted({minute_name(now - 300), minute_name(now)})
    assert [p.stem for p in (tmp_path/'robot').iterdir()] == [minute_name(now)]
