"""Retire only expired immutable samples within an explicitly enrolled live root."""
import hashlib
import json
import re
import time
from pathlib import Path


def enroll(root, kind):
    root = Path(root)
    if kind not in ('packet', 'result') or root.is_symlink():
        raise ValueError('Invalid retention root')
    root.mkdir(exist_ok=True)
    # Must enroll before any producer sample; never adopt historical datasets.
    if any(root.iterdir()):
        raise ValueError('Retention root must be new and empty')
    marker = dict(root=str(root.resolve()), kind=kind, created_ns=time.time_ns())
    (root/'retention-owner.json').write_text(json.dumps(marker))


def retire_expired(root, kind, *, keep, grace_s, journal, now_ns=None):
    root = Path(root)
    if not root.exists():
        return []
    if root.is_symlink() or keep < 16 or grace_s < 10:
        raise ValueError('Invalid retirement boundary')
    marker = json.loads((root/'retention-owner.json').read_text())
    if marker['root'] != str(root.resolve()) or marker['kind'] != kind:
        raise ValueError('Retention owner mismatch')
    pointer = root/'latest.json'
    if not pointer.exists():
        return []
    latest = json.loads(pointer.read_text())[kind+'_dir']
    pattern = r'result-\d{6,}' if kind == 'result' else r'packet-\d{6,}-[a-f0-9]{32}'
    samples = sorted((p for p in root.iterdir() if re.fullmatch(pattern, p.name)),
                     key=lambda p: int(p.name.split('-')[1]))
    now = time.time_ns() if now_ns is None else now_ns
    retired = []
    for sample in samples[:-keep]:
        if sample.name == latest or sample.is_symlink():
            continue
        metadata = 'report.json' if kind == 'result' else 'packet.json'
        names = {metadata, 'observation.npz' if kind == 'result' else 'images.npz'}
        files = list(sample.iterdir())
        if {p.name for p in files} != names or any(p.is_symlink() or not p.is_file() for p in files):
            continue
        report = json.loads((sample/metadata).read_text())
        stamp = report['host_received_unix_ns']
        if type(stamp) is not int or now-stamp < grace_s*1_000_000_000:
            continue
        # Enrolment is a further boundary: no pre-existing sample may be retired.
        if stamp < marker['created_ns']:
            continue
        record = dict(root=str(root.resolve()), sample=sample.name, kind=kind,
                      sample_id=report['sample_id'], received_ns=stamp, retired_ns=now,
                      files={p.name:dict(bytes=p.stat().st_size,
                             sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files})
        with Path(journal).open('a') as stream:
            stream.write(json.dumps(record)+'\n')
        for p in files:
            p.unlink()
        sample.rmdir()
        retired.append(sample.name)
    return retired
