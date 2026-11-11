"""Session-owned RAM control files, archived only after the owner stops."""
import os
from pathlib import Path
import secrets
import shutil


def require_tmpfs(path):
    path=Path(path).resolve()
    mounts=[]
    for line in Path('/proc/self/mountinfo').read_text().splitlines():
        left,right=line.split(' - ',1)
        mount=Path(left.split()[4])
        if path.is_relative_to(mount):mounts.append((len(str(mount)),right.split()[0]))
    if not mounts or max(mounts)[1]!='tmpfs':
        raise ValueError('control runtime must be on tmpfs')
    if path.stat().st_uid!=os.getuid():raise ValueError('control runtime owner mismatch')


def create(out, *, base=None, validate=require_tmpfs):
    out=Path(out).resolve()
    base=Path(base) if base is not None else Path('/run/user')/str(os.getuid())
    validate(base)
    root=base/('rax-motion-'+secrets.token_hex(12))
    root.mkdir(mode=0o700)
    (root/'operator').mkdir(mode=0o700)
    (out/'session').mkdir(exist_ok=True)
    (out/'session/operator').symlink_to(root/'operator',target_is_directory=True)
    # Bridge creates its own new directory; the launcher merely binds it.
    (out/'bridge').symlink_to(root/'bridge',target_is_directory=True)
    return root


def archive(out, root):
    """Caller must first stop all hardware/bridge children; never live-migrate."""
    out=Path(out).resolve();root=Path(root).resolve()
    for link,source in ((out/'bridge',root/'bridge'),(out/'session/operator',root/'operator')):
        if not link.is_symlink() or link.resolve()!=source:
            raise ValueError('volatile control binding changed; preserve evidence')
        archived=link.with_name(link.name+'-archive')
        if source.exists():shutil.copytree(source,archived)
        else:archived.mkdir(mode=0o700)
        link.unlink();archived.rename(link)
    if (root/'receipts.jsonl').exists():shutil.copy2(root/'receipts.jsonl',out/'owner-receipts.jsonl')
    shutil.rmtree(root)
