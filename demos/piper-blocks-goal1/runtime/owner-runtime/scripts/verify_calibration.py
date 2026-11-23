import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='61d9145cc6fe259c6f5ee71ae0880febc2681385be27813b5132fd4e91a3083f'
for n,s in json.loads(p.read_text())["files"].items():
 assert hashlib.sha256(Path(n).read_bytes()).hexdigest()==s,n
print("calibration software contract valid; hardware unverified")
