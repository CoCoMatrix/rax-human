import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'contract.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='726b620ea1b33b6b36531b5c6a7ac037cac64f84e2cccb957f1476b1fce5a64f', 'contract changed'
d=json.loads(p.read_text())
for name,sha in d['files'].items():
 assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha, 'asset changed: '+name
print(json.dumps({'contract_valid':True,'checkpoint':None,'physical_task_verified':False,'perception':'rgb'}))
