import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/'contract.json'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='e7a8a7a01d249fed16d1016733ec7483d890377f548f6f3543892ee7bf381a4a', 'contract changed'
d=json.loads(p.read_text())
for name,sha in d['files'].items():
 assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha, 'asset changed: '+name
print(json.dumps({'contract_valid':True,'checkpoint':None,'physical_task_verified':False,'perception':'rgb'}))
