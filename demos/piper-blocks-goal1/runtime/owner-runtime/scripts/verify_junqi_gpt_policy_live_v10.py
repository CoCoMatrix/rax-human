import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'contract.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='3d838562082dce70a809757b5045b78b5b68f8893ad9cecf2ba5ab254d409dff', 'live contract changed'
d=json.loads(p.read_text())
for name,sha in d['files'].items():
 assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha, 'asset changed: '+name
print(json.dumps({'contract_valid':True,'checkpoint':None,'physical_task_verified':False}))
