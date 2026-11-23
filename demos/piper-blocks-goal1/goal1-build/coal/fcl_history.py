"""Plain FCL only (pre-filter off): is the reported distance history dependent?"""
import sys,json
from pathlib import Path
sys.argv=['x',sys.argv[1],'/home/ubuntu/code_junqi/rax-piper-real-ffs-20261025','/dev/null',sys.argv[2]]
import verify_coal as V
V.R=Path(sys.argv[1]);V.RUNS=[Path(sys.argv[4])]
V.init();ck=V.CK;ck._coal_prefilter=None
work=V.jobs();T='025-green-to-yellow-empty-transition-22.tools.jsonl:1'
item=[w for w in work if w[1]==T][0]
def val(): 
    _,s,_=V.run_mode(item[2],item[3],None);return json.loads(s[20])['tolerated_clearances'][0]['distance_m']
a=val()                      # fresh process, nothing before
for w in work:               # every earlier preview of this run, FCL only
    if w[1]==T:break
    V.run_mode(w[2],w[3],None)
b=val()
c=val()                      # immediately again
print(repr(a),repr(b),repr(c))
