import numpy as np
from ssik.subproblems import sp6
from ssik.subproblems._aux import _quadratic_real_roots
original=sp6.solve_two_ellipse_numeric
stats={'rank_one_calls':0,'fallback_calls':0}
def rank_one_conic(xm1,xn1,xm2,xn2,policy):
 for a,N,b,M in ((xm1,xn1,xm2,xn2),(xm2,xn2,xm1,xn1)):
  U,S,Vt=np.linalg.svd(N)
  if S[0]>1e-12 and S[1]<=1e-12*S[0]:
   stats['rank_one_calls']+=1
   # Change only the null-space coordinate basis. Do not round URDF axes or offsets.
   # a + N*V*[u,v]; rank one means this conic fixes u.
   V=Vt.T;Nrot=N@V;Mrot=M@V;d=Nrot[:,0]
   us=_quadratic_real_roots(float(d@d),float(2*a@d),float(a@a-1),policy.subproblem_degeneracy)
   out=[]
   for u in us:
    c=b+Mrot[:,0]*u;e=Mrot[:,1]
    vs=_quadratic_real_roots(float(e@e),float(2*c@e),float(c@c-1),policy.subproblem_degeneracy)
    for v in vs:
     z=V@np.array([u,v])
     # Preserve original equations and tight residual; all full-FK gates also remain unchanged.
     if max(abs(np.linalg.norm(xm1+xn1@z)-1),abs(np.linalg.norm(xm2+xn2@z)-1))<=1e-9:
      out.append(tuple(map(float,z)))
   return out
 stats['fallback_calls']+=1
 return original(xm1,xn1,xm2,xn2,policy)
