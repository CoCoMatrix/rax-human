"""goal1 Coal pre-filter, step 2: load the vendored Coal only if every vendored file matches the
sealed services/piper/coal_vendor_manifest.json (like ssik_vendor_manifest.json). Any mismatch
or problem leaves the plain FCL check. Run in the runtime root after patch_coal.py."""
from pathlib import Path

p = Path('services/piper/installed_collision_math.py'); s = p.read_text()
old = ("    if os.environ.get('RAX_COAL_PREFILTER','1')=='0' or not _VENDOR.is_dir():return None\n"
       "    if str(_VENDOR) not in sys.path:sys.path.append(str(_VENDOR))\n")
new = ("    if os.environ.get('RAX_COAL_PREFILTER','1')=='0' or not _VENDOR.is_dir():return None\n"
       "    try:\n"
       "        manifest=json.loads(Path(__file__).with_name('coal_vendor_manifest.json').read_text())\n"
       "        root=_VENDOR.parents[3]  # vendor/coal-3.0.3\n"
       "        present={str(f.relative_to(root)) for f in root.rglob('*') if f.is_file() and '__pycache__' not in f.parts}\n"
       "        if not manifest or present!=set(manifest):return None\n"
       "        for rel,digest in manifest.items():\n"
       "            if hashlib.sha256((root/rel).read_bytes()).hexdigest()!=digest:return None\n"
       "    except (OSError,ValueError):return None\n"
       "    if str(_VENDOR) not in sys.path:sys.path.append(str(_VENDOR))\n")
assert s.count(old) == 1
s = s.replace(old, new)
assert s.count("import os\n") == 1
s = s.replace("import os\n", "import hashlib\nimport json\nimport os\n")
p.write_text(s)
print('coal vendor identity check added')
