"""Private, bounded live sensor transport; disk archival only after all producers stop.

Receipt timestamps, immutable sample hashes and all freshness gates are unchanged.
A disk journal stall must not block the live packet/result/multiview handoff.
"""
import hashlib
import fcntl
from contextlib import contextmanager
import json
import os
from pathlib import Path
import secrets
import shutil
from .volatile_control import require_tmpfs

def bindings(out, root):
    return [(Path(out)/'depth',root/'depth'),
            (Path(out)/'packets',root/'packets'),
            (Path(out)/'session/stream/multiview',root/'multiview')]

def create(out, *, base=None, validate=require_tmpfs, minimum_free_bytes=4*1024**3):
    out=Path(out).resolve()
    base=Path(base) if base is not None else Path('/run/user')/str(os.getuid())
    validate(base)
    if shutil.disk_usage(base).free < minimum_free_bytes:
        raise ValueError('Insufficient private tmpfs sensor reserve')
    for link,_ in bindings(out,base):
        if link.exists() or link.is_symlink():
            raise ValueError('Sensor binding must be new')
    root=base/('rax-sensors-'+secrets.token_hex(12));root.mkdir(mode=0o700)
    (root/'multiview').mkdir(mode=0o700)
    for link,source in bindings(out,root):
        link.parent.mkdir(parents=True,exist_ok=True)
        link.symlink_to(source,target_is_directory=True)
    (out/'SENSOR-RUNTIME.json').write_text(json.dumps(
        dict(root=str(root),transport='private_tmpfs',archive_after_stop=True,
             max_bytes=6*1024**3,original_timestamps_preserved=True),indent=2))
    return root

def configure(root, worker, env):
    root=Path(root)
    worker[worker.index('--packet-root')+1]=str(root/'packets')
    worker[worker.index('--output')+1]=str(root/'depth')
    env['REALSENSE_STEREO_EXPORT_DIR']=str(root/'packets')

def check_capacity(root):
    root=Path(root)
    if shutil.disk_usage(root).free < 3*1024**3:
        raise RuntimeError('Live sensor tmpfs reserve exhausted')
    size=0
    for p in root.rglob('*'):
        try:
            if p.is_file():size+=p.stat().st_size
        except FileNotFoundError:
            pass  # Producer retirement may remove an expired immutable sample.
    if size>6*1024**3:
        raise RuntimeError('Live sensor memory budget exhausted')

def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while block:=f.read(1024*1024):h.update(block)
    return h.hexdigest()

def _archive_locked(out, root):
    """Caller must stop owner, camera and FFS before invoking this function.

    All originals and symlink bindings survive any copy/verification failure.
    """
    out=Path(out).resolve();root=Path(root).resolve();pairs=bindings(out,root)
    for link,source in pairs:
        if not link.is_symlink() or link.resolve()!=source:
            raise ValueError('Sensor binding changed; preserve memory evidence')
    size=sum(p.stat().st_size for p in root.rglob('*') if p.is_file())
    if shutil.disk_usage(out).free < size+64*1024**2:
        raise RuntimeError('Archive disk reserve insufficient; memory evidence retained')
    records={}
    for link,source in pairs:
        dest=link.with_name(link.name+'-sensor-archive')
        if source.exists():shutil.copytree(source,dest)
        else:dest.mkdir()
        original={str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file()} if source.exists() else {}
        copied={str(p.relative_to(dest)):digest(p) for p in dest.rglob('*') if p.is_file()}
        if original!=copied:raise ValueError('Sensor archive integrity mismatch; original retained')
        records[str(link.relative_to(out))]=original
    (out/'SENSOR-ARCHIVE.json').write_text(json.dumps(
        dict(verified=True,source_root=str(root),files=records),indent=2))
    for link,source in pairs:
        dest=link.with_name(link.name+'-sensor-archive')
        link.unlink();dest.rename(link)
    shutil.rmtree(root)


@contextmanager
def writer(destination):
    destination=Path(destination)
    if not destination.is_symlink():
        yield
        return
    root=destination.resolve().parent
    if not root.name.startswith('rax-sensors-'):
        raise ValueError('Unexpected live sensor writer binding')
    with (root/'writers.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if (root/'closed.json').exists():
            raise ValueError('Live sensor transport closed for archive')
        yield

def archive(out, root):
    root=Path(root).resolve()
    with (root/'writers.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        (root/'closed.json').write_text('{"closed":true}')
        return _archive_locked(out,root)
