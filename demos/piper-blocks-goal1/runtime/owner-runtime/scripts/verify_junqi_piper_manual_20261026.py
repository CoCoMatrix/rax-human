import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='68cec6c03a7d6c193c7902eebf925b46201b63b61e43b78a0a85cc2c82808306'
d=json.loads(p.read_text())
for n,s in d["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==s,"asset changed: "+n
print(json.dumps({"contract_valid":True,"physical_motion_verified":False}))
