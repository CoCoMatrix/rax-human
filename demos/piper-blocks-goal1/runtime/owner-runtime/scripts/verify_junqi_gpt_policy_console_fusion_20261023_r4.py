import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'contract.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='fb032c630e33656e51b4cee0c95891978bf2a9f51b8fc3d42b358a73ff7272b6', 'contract changed'
d=json.loads(p.read_text())
for name,sha in d['files'].items():
 assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha, 'asset changed: '+name
print(json.dumps({'contract_valid':True,'checkpoint':None,'physical_task_verified':False,'perception':'rgb'}))
