import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1])/"contract.json"
assert hashlib.sha256(p.read_bytes()).hexdigest()=='3c5c041b5cd5cd25c89de543560f0ac507de61cd5b167c42c15679df23fb9438', "GPT-Policy observation contract identity changed"
print(json.dumps({"contract_valid": True,"hardware_write_allowed": False}))
