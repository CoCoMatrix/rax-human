import json,sys,glob,os
sys.path.insert(0,'/home/ubuntu/code_junqi/rax-piper-franka-20261104/experiments/blocks-r9-ssik-sharedik-portfolio-inward1-blend1-carry2-gap68-start1-goal1-web-20261119/runtime')
from services.dashboard import goal_spec as gs
goals=json.load(open('goals.json'));d=sys.argv[1];rows={}
for f in sorted(x for x in glob.glob(d+'/*.json') if '.lane' not in x):
    g,run=os.path.basename(f)[:-5].split('__');j=json.load(open(f));p=j['plan'];spec=gs.check(goals[g])['spec']
    if p['ok']:
        pos=p['goal'];cells={}
        xs=sorted({round(v[0],3) for v in pos.values()});ys=sorted({round(v[1],3) for v in pos.values()})
        for c,v in pos.items():cells.setdefault((len(ys)-1-ys.index(round(v[1],3)),xs.index(round(v[0],3))),[]).append((v[2],c))
        got=gs.check({'stacks':[{'col':k[0],'row':k[1],'blocks':[c for _,c in sorted(v)]} for k,v in cells.items()],'spacing':spec['spacing']})
        same=got['ok'] and gs.rotation_key(got['spec'])==gs.rotation_key(spec)
        last=p['moves'][-1]['actions'][-1]
        gaps=sorted({round(abs(a-b)*1000) for a in xs for b in xs if a<b}|{round(abs(a-b)*1000) for a in ys for b in ys if a<b})
        txt='OK %5.1fs side=%s order=%s structure=%s pitch_mm=%s end=%s'%(p.get('wall_s',p.get('elapsed_s')),p['orange_side'],'>'.join(c[0] for c in p['order']),'match' if same else 'MISMATCH',gaps,'released_in_place' if last[7] else last[0])
    else:txt='FAIL %5.1fs %s | %s'%(p.get('wall_s',p.get('elapsed_s')),p['error'][:70],p.get('screens',[])[:3])
    rows.setdefault(g,[]).append(run[18:24]+' '+txt)
for g,r in rows.items():
    print(g,'|',gs.describe(gs.check(goals[g])['spec']))
    for x in r:print('    ',x)
