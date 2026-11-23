import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'contract.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='9544a8938652e870f437d9c2b5a2a47aa72737b6ee62377c56d8569f6a54f55f', 'contract changed'
d=json.loads(p.read_text())
for name,sha in d['files'].items():
 assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha, 'asset changed: '+name
print(json.dumps({'contract_valid':True,'checkpoint':None,'physical_task_verified':False,'perception':'rgb'}))
