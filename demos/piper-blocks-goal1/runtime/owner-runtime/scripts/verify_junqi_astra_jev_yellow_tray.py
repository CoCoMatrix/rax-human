import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='e151c37a806c14a664acfecac8b33533889e62b0af23127114986c076acd6c84'
d=json.loads(p.read_text())
for n,h in d["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==h,n
print("contract_valid")
