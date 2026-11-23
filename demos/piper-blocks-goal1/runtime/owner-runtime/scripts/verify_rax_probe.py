import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='7c01230e89141adae1c71de9c625d65c229efadb057b737d4ef8f31d8674e3b8'
d=json.loads(p.read_text())
for n,s in d["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==s,n
print("PROBE_CONTRACT_OK")
