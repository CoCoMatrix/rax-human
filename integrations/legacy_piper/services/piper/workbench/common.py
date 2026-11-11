"""Local paths and atomic records; never imports a robot SDK."""
import hashlib
import json
import os
from pathlib import Path
import uuid

ROOT = Path(__file__).resolve().parents[3]
DATA = ROOT / 'commissioning/workbench'
PYTHON = ROOT / '.venv/bin/python'
AGENT_PYTHON = Path('/home/ubuntu/code_junqi/rax-piper-agent-isolated/bin/python')
QUALIFICATION = ROOT / 'commissioning/task60-20261029/QUALIFICATION.json'
GUARD = 'junqi-rax-clock-guard-workbench.service'
HARDWARE_UNIT = 'junqi-rax-workbench-hardware.service'
WEB_UNIT = 'junqi-rax-workbench.service'
BASELINE = ROOT / 'commissioning/task60-20261029/restore-5eaccef-audit-20261029/workspace/code/cup_to_coaster'


def read(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex)
    try:
        temp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
        temp.chmod(0o600)
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def source_manifest(path):
    """Only source/config, with no symlinks or external paths admitted."""
    path = Path(path).resolve()
    if not path.is_dir():
        raise ValueError('程序目录不存在')
    rows = {}
    for item in sorted(path.rglob('*')):
        rel = item.relative_to(path)
        if any(part in ('.git', '__pycache__', '.pytest_cache') for part in rel.parts):
            continue
        if item.is_symlink():
            raise ValueError('冻结源码不能包含符号链接：' + str(rel))
        if item.is_file():
            if item.stat().st_size > 8 * 1024 * 1024:
                raise ValueError('源码包含超过 8 MiB 的文件：' + str(rel))
            rows[str(rel)] = hashlib.sha256(item.read_bytes()).hexdigest()
    if not rows:
        raise ValueError('程序目录为空')
    digest = hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()
    return {'sha256': digest, 'files': rows}
