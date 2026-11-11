// Source-pixel footprint rasterization; no camera access or robot commands.
// Algorithm reference: librealsense align.cpp (pixel corners + z buffer).
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

static bool project(const double* p,const double* k,int model,double& u,double& v) {
 if(!std::isfinite(p[2])||p[2]<=0)return false;
 double x=p[0]/p[2],y=p[1]/p[2],r=x*x+y*y;
 if(model){
  double f=1+k[4]*r+k[5]*r*r+k[8]*r*r*r;
  if(model==2){x*=f;y*=f;}
  double nx=(model==2?x:x*f)+2*k[6]*x*y+k[7]*(r+2*x*x);
  double ny=(model==2?y:y*f)+2*k[7]*x*y+k[6]*(r+2*y*y);
  x=nx;y=ny;
 }
 u=x*k[0]+k[2];v=y*k[1]+k[3];return std::isfinite(u)&&std::isfinite(v);
}
static void transform(const double* p,const double* t,double* q){
 for(int a=0;a<3;++a)q[a]=t[4*a]*p[0]+t[4*a+1]*p[1]+t[4*a+2]*p[2]+t[4*a+3];
}
extern "C" void register_v1(int h,int w,int H,int W,const double* xyz,const uint8_t* valid,
 const double* t,const double* ks,const double* kr,int model,double span,int footprint,
 double* native_xyz,double* uv,int32_t* direct,int32_t* surface,uint8_t* support,uint8_t* visible){
 const double nan=std::numeric_limits<double>::quiet_NaN();int n=h*w,m=H*W;
 std::fill(direct,direct+m,-1);std::fill(surface,surface+m,-1);
 std::fill(support,support+n,0);std::fill(visible,visible+n,0);
 std::fill(native_xyz,native_xyz+3*n,nan);std::fill(uv,uv+2*n,nan);
 for(int i=0;i<n;++i){
  const double* p=xyz+3*i;double* q=native_xyz+3*i;
  if(!valid[i]||!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[2]<=0)continue;
  transform(p,t,q);double u,v;if(!project(q,kr,model,u,v))continue;
  uv[2*i]=u;uv[2*i+1]=v;
  if(u<-.5||v<-.5||u>=W-.5||v>=H-.5)continue;
  int j=int(std::floor(v+.5))*W+int(std::floor(u+.5));int old=direct[j];
  if(old<0||q[2]<native_xyz[3*old+2])direct[j]=i;
 }
 std::copy(direct,direct+m,surface);
 for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x){
  int i=y*w+x;double lowz=1e300,highz=-1e300;bool ok=true;
  for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
   int k=(y+dy)*w+x+dx;const double* p=xyz+3*k;
   if(!valid[k]||!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2])||p[2]<=0)ok=false;
   lowz=std::min(lowz,p[2]);highz=std::max(highz,p[2]);
  }
  if(!ok||highz-lowz>span)continue;support[i]=1;
  const double* p=xyz+3*i;double minU=1e300,minV=1e300,maxU=-1e300,maxV=-1e300;
  for(double dx:{-.5,.5})for(double dy:{-.5,.5}){
   double a[3]={p[0]+dx*p[2]/ks[0],p[1]+dy*p[2]/ks[1],p[2]},q[3],u,v;
   transform(a,t,q);if(!project(q,kr,model,u,v)){ok=false;continue;}
   minU=std::min(minU,u);maxU=std::max(maxU,u);minV=std::min(minV,v);maxV=std::max(maxV,v);
  }
  // Validate floating bounds before casting, including enormous/invalid projections.
  if(!ok||minU<-.5||minV<-.5||maxU>=W-.5||maxV>=H-.5)continue;
  int x0=int(std::floor(minU+.5)),x1=int(std::floor(maxU+.5)),y0=int(std::floor(minV+.5)),y1=int(std::floor(maxV+.5));
  if(x1-x0+1>footprint||y1-y0+1>footprint)continue;
  double z=native_xyz[3*i+2];
  for(int yy=y0;yy<=y1;++yy)for(int xx=x0;xx<=x1;++xx){
   int j=yy*W+xx,old=surface[j];double oz=old<0?1e300:native_xyz[3*old+2];
   if(old<0||z<oz-1e-12||(std::abs(z-oz)<=1e-12&&i<old))surface[j]=i;
  }
 }
 // Visibility is checked against the footprint z-buffer, not just centre collisions.
 for(int i=0;i<n;++i){
  double u=uv[2*i],v=uv[2*i+1];
  if(!std::isfinite(u)||!std::isfinite(v)||u<-.5||v<-.5||u>=W-.5||v>=H-.5)continue;
  int j=int(std::floor(v+.5))*W+int(std::floor(u+.5)),win=surface[j];
  if(win>=0&&native_xyz[3*i+2]<=native_xyz[3*win+2]+.001)visible[i]=1;
 }
}
