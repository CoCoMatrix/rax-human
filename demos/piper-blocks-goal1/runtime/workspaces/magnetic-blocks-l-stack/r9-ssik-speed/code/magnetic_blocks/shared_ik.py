"""One-preflight exact IK memo. No geometry, paths, state or hardware in this store."""
import copy
import json
import os
import threading
import time
import uuid
from collections import Counter
from multiprocessing.managers import BaseManager


class ExactIKStore:
    def __init__(self):
        self.values = {}
        self.pending = {}
        self.condition = threading.Condition()
        self.counts = Counter()
        self.hit_keys = set()
        self.wait_seconds = 0.0

    def acquire(self, key):
        begun = time.monotonic()
        waited = False
        with self.condition:
            while True:
                if key in self.values:
                    self.counts['hits'] += 1
                    self.hit_keys.add(key)
                    self.counts['success_hits' if self.values[key].get('ok') is True else 'negative_hits'] += 1
                    if waited:
                        self.wait_seconds += time.monotonic() - begun
                    return {'hit': True, 'value': copy.deepcopy(self.values[key])}
                now = time.monotonic()
                old = self.pending.get(key)
                if old is None or now - old[1] >= 2.0:
                    if old is not None:
                        self.counts['expired_claims'] += 1
                    token = uuid.uuid4().hex
                    self.pending[key] = (token, now)
                    self.counts['misses'] += 1
                    if waited:
                        self.wait_seconds += now - begun
                    return {'hit': False, 'token': token}
                if not waited:
                    self.counts['waiters'] += 1
                    waited = True
                self.condition.wait(min(.05, max(.001, 2.0 - (now-old[1]))))

    def finish(self, key, token, value):
        with self.condition:
            if self.pending.get(key, (None,))[0] != token:
                self.counts['stale_publications'] += 1
                return False
            del self.pending[key]
            if value is not None:
                self.values[key] = copy.deepcopy(value)
                self.counts['published'] += 1
            else:
                self.counts['not_cached'] += 1
            self.condition.notify_all()
            return True

    def stats(self):
        with self.condition:
            return dict(self.counts, entries=len(self.values), pending=len(self.pending),
                        wait_s=self.wait_seconds, pid=os.getpid())

    def verification_sample(self):
        with self.condition:
            # Bounded post-run evidence, no extra file writes on the hot path.
            keys=sorted(self.hit_keys)
            if len(keys)>64:
                keys=[keys[i*(len(keys)-1)//63] for i in range(64)]
            return [{'key':k,'value':copy.deepcopy(self.values[k])} for k in keys]


_store = None
def get_store():
    global _store
    if _store is None:
        _store = ExactIKStore()
    return _store


class IKManager(BaseManager):
    pass


IKManager.register('cache', callable=get_store)


def start_cache(folder, pool):
    address = str(folder/'shared-ik.sock')
    auth = os.urandom(32)
    manager = IKManager(address=address, authkey=auth)
    manager.start()
    os.chmod(address, 0o600)
    identity = {'pool_id': pool['pool_id'], 'solver': pool['lanes'][0]['solver'],
                'model': pool['lanes'][0]['geometry_identity']}
    packet = {'address': address, 'auth': auth.hex(), 'identity': identity}
    return manager, manager.cache(), packet


def cache_key(identity, arguments):
    # JSON round-trips full binary64 precision. No rounding or pose-only matching.
    return json.dumps([identity, arguments], sort_keys=True, separators=(',', ':'), allow_nan=False)


def stable_result(value, arguments, identity):
    if (value.get('executed') is not False or value.get('hardware_writes') != 0
            or value.get('scene_collision_checked') is not False
            or value.get('mode') != arguments['mode']
            or value.get('urdf_sha256') != identity['model']['urdf_sha256']):
        return None
    rows = value.get('attempts', [])
    seeds = arguments['seeds']
    if not rows or len(rows) > len(seeds):
        return None
    for i, row in enumerate(rows):
        if row.get('seed') != seeds[i] or row.get('executed') is not False:
            return None
        if row.get('ok') is True:
            if i != len(rows)-1 or value.get('ok') is not True:
                return None
        elif (row.get('ok') is not False or row.get('reason') != 'MOVEIT_NO_IK_SOLUTION'
              or row.get('solver_reason') != 'SSIK_NO_IK_SOLUTION'):
            # Budget expiry/transport/unknown refusals must remain unresolved.
            return None
    if value.get('ok') is False and (len(rows) != len(seeds) or rows[-1]['ok'] is not False):
        return None
    result = copy.deepcopy(value)
    for key in ('wall_budget', 'operator_updates', 'operator_updates_error'):
        result.pop(key, None)
    return result


def attach_cache(ctx, packet):
    manager = IKManager(address=packet['address'], authkey=bytes.fromhex(packet['auth']))
    manager.connect()
    cache = manager.cache()
    original = ctx.value
    counts = Counter()

    def value(name, **arguments):
        if name != 'piper_solve_ik_candidates':
            return original(name, **arguments)
        key = cache_key(packet['identity'], arguments)
        claim = cache.acquire(key)
        if claim['hit']:
            counts['hits'] += 1
            counts['seed_attempts_avoided'] += len(claim['value']['attempts'])
            return claim['value']
        counts['misses'] += 1
        try:
            result = original(name, **arguments)
        except BaseException:
            cache.finish(key, claim['token'], None)
            raise
        cache.finish(key, claim['token'], stable_result(result, arguments, packet['identity']))
        return result

    ctx.value = value
    ctx._shared_ik_manager = manager
    ctx._shared_ik_counts = counts
