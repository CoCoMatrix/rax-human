"""Agent -> operator requests (help, scene reset) answered from the dashboard or the operator CLI.

Files live in the session root, outside the agent's writable workspace:

    <session>/operator/requests/<id>.json         written by the robot service
    <session>/operator/requests/<id>.reply.json   written by the operator tools

The `request_operator` tool blocks until the reply file appears or the wait times out;
the robot does nothing meanwhile (the arm holds its last pose).
"""
import json
import os
from pathlib import Path
import re
import time
import uuid
import fcntl
from contextlib import contextmanager

KINDS = ('help', 'reset')
REQUEST_ID = re.compile(r'[0-9a-f]{12}')


def directory(session_root):
    return Path(session_root)/'operator'/'requests'


def _write(path, value):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}')
    temporary.write_text(json.dumps(value, indent=1))
    temporary.chmod(0o600)
    os.replace(temporary, path)


def _read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def get_request(session_root, request_id):
    if not isinstance(request_id, str) or not REQUEST_ID.fullmatch(request_id):
        raise ValueError('invalid request id')
    request = _read(directory(session_root)/f'{request_id}.json')
    if not request or request.get('id') != request_id:
        raise ValueError(f'unknown request {request_id}')
    return request


@contextmanager
def _lock(session_root, request_id):
    get_request(session_root, request_id)
    with (directory(session_root)/f'.{request_id}.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def open_request(session_root, kind, message, *, context=None):
    if kind not in KINDS:
        raise ValueError(f'kind must be one of {KINDS}')
    if not isinstance(message, str) or not message.strip() or len(message) > 4000:
        raise ValueError('message: 1..4000 characters')
    request = {'id': uuid.uuid4().hex[:12], 'kind': kind, 'message': message.strip(), 'created': time.time(),
               'status': 'pending', 'context': context}
    _write(directory(session_root)/f"{request['id']}.json", request)
    return request


def reply(session_root, request_id, text='', *, done=True, by=None):
    """Record an actual operator answer, including a late one; never rearm/move."""
    if type(done) is not bool or not isinstance(text, str) or not text.strip() or len(text)>4000:
        raise ValueError('A nonempty answer and boolean done are required')
    with _lock(session_root, request_id):
        request = get_request(session_root, request_id)
        path = directory(session_root)/f'{request_id}.reply.json'
        if path.exists() or request.get('status') == 'answered':
            raise ValueError('request already answered; never overwrite operator evidence')
        if request.get('status') not in ('pending', 'expired'):
            raise ValueError('request is not awaiting an answer')
        answer = {'reply': text, 'done': done, 'by': by or os.environ.get('USER'),
                  'time': time.time(), 'late': request.get('status') == 'expired'}
        _write(path, answer)
        _write(directory(session_root)/f'{request_id}.json',
               {**request, 'status': 'answered', 'answered': answer['time']})
        return answer


def recent(session_root, limit=5):
    """Read-only delivery via status, so late answers do not disappear on timeout."""
    folder=directory(session_root)
    requests=[_read(p) for p in folder.glob('????????????.json')]
    requests=sorted((r for r in requests if r), key=lambda r:r['created'], reverse=True)
    return [{'request_id':r['id'], 'kind':r['kind'], 'message':r['message'],
             'status':r['status'], 'created':r['created'], 'context':r.get('context'),
             'timed_out_at':r.get('timed_out_at'),
             'answer':_read(folder/f"{r['id']}.reply.json"),
             'requires_fresh_observation':True, 'motion_authorized':False}
            for r in requests[:limit]]


def pending(session_root):
    folder = directory(session_root)
    requests = [_read(p) for p in sorted(folder.glob('*.json')) if not p.name.endswith('.reply.json')] \
        if folder.is_dir() else []
    return [r for r in requests if r and r.get('status') == 'pending'
            and not (folder/f"{r['id']}.reply.json").exists()]


def wait(session_root, request, timeout_s, *, poll_s=1.0, clock=time.monotonic, sleep=time.sleep):
    """Block until the operator answers; returns the tool result."""
    folder = directory(session_root)
    started = clock()
    while True:
        # Deadline and answer are serialized. A reply at the deadline cannot be lost.
        with _lock(session_root, request['id']):
            current=get_request(session_root, request['id'])
            answer = _read(folder/f"{request['id']}.reply.json")
            base={'request_id':request['id'], 'question':current['message'],
                  'context':current.get('context'), 'requires_fresh_observation':True,
                  'motion_authorized':False, 'waited_s':round(clock()-started,1)}
            if answer is not None:
                return {**base, 'status':'answered', 'reply':answer['reply'],
                        'done':answer['done'], 'late':answer.get('late',False),
                        'answered_at':answer['time'], 'by':answer.get('by')}
            if clock()-started >= timeout_s:
                _write(folder/f"{request['id']}.json", {**current, 'status':'expired',
                       'timed_out_at':current.get('timed_out_at',time.time())})
                return {**base, 'status':'timeout', 'reply':None, 'done':False,
                        'note':'No confirmation received. Do not perform the dependent motion. '
                               'Query status or request_operator with this request_id for a late reply; '
                               'do not create a duplicate question or infer permission from timeout.'}
        sleep(min(poll_s,max(0,timeout_s-(clock()-started))))
