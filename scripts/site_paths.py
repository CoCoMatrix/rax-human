"""Deployment paths only. Importing this module never opens hardware or credentials."""
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]

def settings():
    location=Path(os.environ.get('RAX_SITE_CONFIG',ROOT/'site.local.json')).expanduser()
    if not location.exists():return {}
    data=json.loads(location.read_text())
    if not isinstance(data,dict):raise ValueError('Site configuration must be a JSON object')
    return data

def value(name,default):
    item=settings().get(name,default)
    if not isinstance(item,str):raise ValueError('Site value must be text: '+name)
    return item

def path(name,default):
    p=Path(value(name,str(default))).expanduser()
    return p if p.is_absolute() else (ROOT/p).resolve()
