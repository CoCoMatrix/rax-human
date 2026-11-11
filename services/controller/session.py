"""Durable session owner: one robot owner per session, the Gemini dollar ledger and the
MCP audit log. Frontends must never instantiate a second owner."""
from __future__ import annotations

import fcntl
import json
import os
import threading
import time

from services.json_io import write_private_json
from .billing import NANODOLLARS, SESSION_LIMIT, nanodollars

AUDIT_LIMIT_BYTES = 2 * 1024**3  # Beyond this the audit log stops growing and tools refuse.


class Session:
    def __init__(self, config):
        self.config = config
        root = config.root
        root.mkdir(parents=True, mode=0o700, exist_ok=True)
        self._owner = (root/'owner.lock').open('a')
        try:
            fcntl.flock(self._owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self._owner.close()
            raise RuntimeError('Session already has a robot owner')
        self._lock = threading.RLock()
        self.state_path = root/'state.json'
        if self.state_path.exists():
            self.state = json.loads(self.state_path.read_text())
        else:
            self.state = {'schema': 1, 'created_at': time.time(), 'audit_bytes': 0,
                          'gemini_spend': {'reported': 0, 'reserved': 0, 'blocked': False},
                          'gemini_usage': {'calls': 0, 'tokens': 0, 'reported_tokens': 0, 'cost_usd': 0.0}}
            self._save()

    def _save(self):
        with self._lock:
            write_private_json(self.state_path, self.state)

    def audit(self, event):
        raw = (json.dumps({'time': time.time(), **event}, allow_nan=False, default=str) + '\n').encode()
        with self._lock:
            if self.state['audit_bytes'] + len(raw) > AUDIT_LIMIT_BYTES:
                raise RuntimeError('Session audit budget exhausted')
            self.state['audit_bytes'] += len(raw)
            self._save()
            with (self.config.root/'mcp.jsonl').open('ab') as stream:
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())

    def budget(self):
        with self._lock:
            usage, spend = self.state['gemini_usage'], self.state['gemini_spend']
            return {'calls_used': usage['calls'], 'tokens_charged': usage['tokens'],
                    'tokens_reported': usage['reported_tokens'], 'reported_cost_usd': usage['cost_usd'],
                    'session_cost_limit_usd': SESSION_LIMIT / NANODOLLARS,
                    'session_cost_reported_usd': spend['reported'] / NANODOLLARS,
                    'session_cost_reserved_usd': spend['reserved'] / NANODOLLARS,
                    'session_cost_estimated_upper_bound_usd': spend.get('bounded', 0) / NANODOLLARS,
                    'session_cost_remaining_usd': (0 if spend['blocked'] else
                        max(0, SESSION_LIMIT - spend['reported'] - spend['reserved'] - spend.get('bounded', 0))) / NANODOLLARS,
                    'session_cost_blocked': spend['blocked']}

    def charge(self, tokens, cost_reservation):
        with self._lock:
            if type(cost_reservation) is not int or cost_reservation < 1:
                raise ValueError('Invalid dollar reservation')
            spend = self.state['gemini_spend']
            remaining = max(0, SESSION_LIMIT - spend['reported'] - spend['reserved'] - spend.get('bounded', 0))
            if spend['blocked'] or cost_reservation > remaining:
                raise RuntimeError('Gemini session budget exhausted: '
                                   f'required=${cost_reservation / NANODOLLARS:.6f}, '
                                   f'remaining=${(0 if spend["blocked"] else remaining) / NANODOLLARS:.6f}, '
                                   f'blocked={spend["blocked"]}; no provider request sent')
            usage = self.state['gemini_usage']
            usage['calls'] += 1
            usage['tokens'] += tokens
            spend['reserved'] += cost_reservation
            self._save()

    def refund(self, tokens, reported, cost_reservation, *, cost_upper_bound=None):
        with self._lock:
            usage, spend = self.state['gemini_usage'], self.state['gemini_spend']
            usage['tokens'] -= tokens
            usage['reported_tokens'] += reported['total_tokens']
            try:
                cost = nanodollars(reported.get('cost'))
            except ValueError:
                if cost_upper_bound is not None:
                    bound = nanodollars(cost_upper_bound)
                    spend['reserved'] -= cost_reservation
                    spend['bounded'] = spend.get('bounded', 0) + bound
                    if bound > cost_reservation:
                        spend['blocked'] = True
                    self._save()
                    if spend['blocked']:
                        raise RuntimeError('Estimated provider cost exceeded reservation; Gemini session closed')
                    return
                # Successful text without trustworthy billing is not a free call.
                self._save()
                return
            spend['reserved'] -= cost_reservation
            spend['reported'] += cost
            usage['cost_usd'] += cost / NANODOLLARS
            if cost > cost_reservation:
                spend['blocked'] = True  # Provider violated the enforced price/token bound.
            self._save()
            if spend['blocked']:
                raise RuntimeError('Provider cost exceeded reservation; Gemini session closed')

    def close(self):
        self._owner.close()
