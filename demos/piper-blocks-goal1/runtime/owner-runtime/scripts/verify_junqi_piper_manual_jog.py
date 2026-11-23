import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='b6191316cd012b0d71fcc338802241b8a1091db340440371c674b5f30d6b79e5'
d=json.loads(p.read_text())
for n,h in d["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==h,n
print("contract_valid")
