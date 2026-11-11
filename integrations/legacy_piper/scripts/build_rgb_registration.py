"""Explicit personal-workspace C++ build, no camera or robot side effects."""
import hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from services.piper.sensor.rgb_registration import SOURCE,BUILD,digest
BUILD.mkdir(parents=True,exist_ok=True);key=digest(SOURCE);dest=BUILD/(key+'.so');tmp=dest.with_suffix('.partial')
command=['g++','-O3','-std=c++17','-fPIC','-shared',str(SOURCE),'-o',str(tmp)]
subprocess.run(command,check=True);tmp.replace(dest)
receipt={'source_sha256':key,'binary_sha256':digest(dest),'command':command,'hardware_writes':0}
dest.with_suffix('.json').write_text(json.dumps(receipt,indent=2));print(str(dest))
