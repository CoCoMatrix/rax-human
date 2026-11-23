import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='2175833e4162041ba950b4316b5e9313d14af49679ca3e015f135627f007edbd', "contract identity mismatch"
d=json.loads(p.read_text())
for n,h in d["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==h, "asset changed: "+n
print(json.dumps({"contract_valid":True,"physical_task_verified":False}))
