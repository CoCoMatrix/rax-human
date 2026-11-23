"""Pure FK batches; preserve every geometric sample and all motion checks."""
import numpy as np

def fk_many(ctx,joints):
    points=[[float(x) for x in q] for q in joints]
    if not getattr(ctx,'_blocks_batch_fk',False):
        return (ctx.value('piper_fk',joints=q) for q in points)
    out=[]
    for i in range(0,len(points),512):
        chunk=points[i:i+512]
        result=ctx.value('piper_fk_batch',joints_batch=chunk)
        if result.get('executed') is not False or result.get('frame')!='left_base_link':
            raise ValueError('Invalid pure FK batch contract')
        matrices=result.get('T_base_flange',[])
        if len(matrices)!=len(chunk):raise ValueError('Incomplete FK batch')
        for matrix in matrices:
            t=np.asarray(matrix,float)
            if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('Malformed FK batch')
            out.append({'T_base_flange':matrix,'position':t[:3,3].tolist()})
    return out

def fk_line(ctx,start,end,fractions):
    times=list(fractions);a=np.asarray(start[:6]);b=np.asarray(end[:6])
    return zip(times,fk_many(ctx,[a+(b-a)*u for u in times]))
