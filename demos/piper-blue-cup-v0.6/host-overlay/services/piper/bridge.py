"""Client only for the installed XPolicy sole-owner bridge; never starts/arms it."""
import json
import fcntl
from pathlib import Path
import threading
import time
import uuid
from services import arming
from .geometry import vector
from .installed_settling import settling_valid, SETTLING_WINDOW_S, SETTLING_MIN_SAMPLES


class RejectedBeforeDispatch(ValueError):
    executed = False
    dispatch_started = False


class OwnerBridge:
    def __init__(self, root, owner_session, session_root):
        self.root = Path(root).resolve(); self.owner_session = Path(owner_session).resolve()
        self.session_root = Path(session_root).resolve(); self.instance = None
        self.lock = threading.Lock(); self.active = False
        # Separate from the full motion lock: close must not wait for arrival.
        self._lifecycle = threading.RLock()
        self.task_lock = None
        self.closed = False

    def _claim(self):
        """Keep one new RAX motion client per bridge for its entire lifetime."""
        with self._lifecycle:
            if self.closed:
                raise ValueError('Motion client is closed')
            if self.task_lock is not None:
                return
            handle = (self.root / 'rax-client.lock').open('a')
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BaseException:
                handle.close()
                raise ValueError('Another RAX session owns this motion bridge') from None
            self.task_lock = handle

    def _release(self):
        with self._lifecycle:
            if self.task_lock is not None:
                self.task_lock.close()
                self.task_lock = None

    def status(self):
        row = json.loads((self.root / 'state.json').read_text())
        if row.get('schema') != 'rax.piper.motion_bridge.v1' or row.get('session_root') != str(self.owner_session):
            raise ValueError('Wrong sole-owner bridge session')
        stamp = row.get('timestamp_ns')
        if type(stamp) is not int or not 0 <= time.time_ns() - stamp <= 500_000_000:
            raise ValueError('Owner bridge heartbeat stale')
        instance = row.get('instance')
        if not isinstance(instance, str) or not instance or self.instance not in (None, instance):
            raise ValueError('Owner bridge instance changed')
        self.instance = instance
        return row

    def armed(self):
        for root in (self.session_root, self.owner_session):
            if not arming.read(root)['armed']: raise PermissionError('Both RAX and sole owner must be operator armed')

    @staticmethod
    def _write(path, value):
        # Queue ownership is the existing bridge's; unique atomic files avoid partial requests.
        tmp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex)
        try:
            with tmp.open('x') as f:
                tmp.chmod(0o600); json.dump(value, f, allow_nan=False)
            tmp.replace(path)
        finally:
            tmp.unlink(missing_ok=True)

    def move(self, q, duration_s=None):
        vector(q, 14)
        return self._move_request({'joints':q,'duration_s':duration_s})

    def close_gripper(self,q,duration_s=None):
        vector(q,14)
        return self._move_request({'kind':'close_gripper','joints':q,'duration_s':duration_s})

    def move_waypoints(self,start,waypoints,obstacles,blend_indices=None):
        return self._move_request({'kind':'waypoints','start14':start,'waypoints':waypoints,'obstacles':obstacles,'blend_indices':blend_indices or []})

    def _move_request(self,payload):
        with self.lock:
            try:
                self._claim()
                row = self.status(); self.armed()
            except Exception as exc:
                raise RejectedBeforeDispatch(str(exc)) from exc
            if row['phase'] not in ('IDLE', 'DONE') or row.get('request_id') is not None:
                raise RejectedBeforeDispatch('Owner busy or faulted')
            if row.get('observation_recovery'):
                raise RejectedBeforeDispatch('Owner observation recovery pending; read status/observe before retrying')
            if row.get('error') or row.get('stop'):
                raise RejectedBeforeDispatch('Owner reports an error or Stop; physical-session recovery required')
            request = uuid.uuid4().hex
            with self._lifecycle:
                if self.closed:
                    raise RejectedBeforeDispatch('Motion client is closed')
                self._write(self.root / 'requests' / (request + '.json'),
                            {'id': request, 'instance': self.instance, 'timestamp_ns': time.time_ns(),
                             **payload})
                self.active = True
            deadline = time.monotonic() + 35
            try:
                while time.monotonic() < deadline:
                    self.armed(); state = self.status()
                    path = self.root / 'results' / (request + '.json')
                    if path.exists():
                        reply = json.loads(path.read_text())
                        if reply.get('id') != request or reply.get('instance') != self.instance:
                            raise RuntimeError('Owner reply identity mismatch')
                        if not reply.get('ok'):
                            if reply.get('failure_kind') == 'pre_dispatch_rejection' and reply.get('executed') is False:
                                if state['phase'] in ('IDLE', 'DONE') and state.get('request_id') is None:
                                    raise RejectedBeforeDispatch(reply.get('reason', 'Rejected'))
                            elif state['phase'] == 'FAULT':
                                raise RuntimeError(reply.get('reason', 'Owner fault'))
                        elif state['phase'] == 'DONE' and state.get('request_id') is None:
                            result = reply['result']; vector(result.get('final14'), 14)
                            if result.get('executed') is not True or result.get('measured_arrival') is not True or result.get('sdk_completed_steps', 0) < 1:
                                raise RuntimeError('Owner completion lacks measured arrival or SDK receipts')
                            settling=result.get('settling') or {}
                            if (result.get('measured_settled') is not True
                                    or not isinstance(settling.get('window_s'),(int,float))
                                    or not SETTLING_WINDOW_S<=settling['window_s']<=4
                                    or settling.get('samples',0)<SETTLING_MIN_SAMPLES
                                    or not settling_valid(settling)):
                                raise RuntimeError('Owner completion lacks measured settling; update/review bridge, never replay')
                            vector(result.get('target14'),14)
                            return {**result, 'owner_request_id': request, 'object_success': None,
                                    'handoff_timing': reply.get('handoff_timing', [])}
                    if state['phase'] == 'FAULT': raise RuntimeError('Owner faulted during motion')
                    time.sleep(.05)
                raise TimeoutError('Unknown motion outcome; never replay automatically')
            except RejectedBeforeDispatch: raise
            except BaseException:
                self.cancel(); raise
            finally:
                with self._lifecycle:
                    self.active = False
                    if self.closed:
                        self._release()

    def cancel(self):
        self._write(self.root / 'cancel.json', {'instance': self.instance, 'timestamp_ns': time.time_ns()})
        return {'stop_requested': True, 'physical_stop_confirmed': False, 'operator_recovery_required': True}

    def hold(self):
        row = self.status()
        if row['phase'] in ('IDLE', 'DONE') and row.get('request_id') is None:
            # Between calls the owner already holds. The backend checks measured rest separately.
            return {'hardware_stop_sent': False, 'owner_idle': True, 'resumable': True}
        return self.cancel()

    def close(self):
        with self._lifecycle:
            self.closed = True
            if self.active:
                self.cancel()
            else:
                self._release()
