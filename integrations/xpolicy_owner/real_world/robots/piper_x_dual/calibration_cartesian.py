"""Bounded offline IK for operator jogs. No hardware transport or camera frame.

Translations use left_base_link axes and preserve left_link6 orientation.
All rigidly attached points share that translation; no calibrated TCP assumed.
"""
import hashlib
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

URDF_SHA = '12e4e447bffcbbfebca677fb96fd6062d122b1c2987da65187aaa3caa1bb330d'
TRANSLATION_M = .002
STEP_MM = (2, 5, 10)
MAX_JOINT_DELTA_RAD = np.deg2rad(2.)
DIRECTIONS = {'forward': (1,0,0), 'backward': (-1,0,0),
              'left': (0,1,0), 'right': (0,-1,0), 'up': (0,0,1), 'down': (0,0,-1)}


class CartesianJog:
    def __init__(self, urdf=None):
        if urdf is None:
            packaged=Path(__file__).with_name('description')/'calibration_kinematics.urdf'
            urdf=packaged if packaged.exists() else Path(__file__).resolve().parents[2]/'assets/piper-32cm/dual_piper_x_description/urdf/dual_piper_x.urdf'
        raw=Path(urdf).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=URDF_SHA:
            raise ValueError('标定点动运动学模型指纹不符')
        root=ET.fromstring(raw);self.origins=[];self.axes=[];self.limits=[]
        for i in range(1,7):
            joint=root.find(f"joint[@name='left_joint{i}']")
            if joint is None or joint.attrib['type']!='revolute':
                raise ValueError('Expected six pinned left-arm revolute joints')
            parent='left_base_link' if i==1 else f'left_link{i-1}'
            if joint.find('parent').attrib['link']!=parent or joint.find('child').attrib['link']!=f'left_link{i}':
                raise ValueError('Unexpected left-arm chain')
            origin=joint.find('origin');T=np.eye(4)
            T[:3,3]=np.fromstring(origin.attrib['xyz'],sep=' ')
            T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.attrib['rpy'],sep=' ')).as_matrix()
            self.origins.append(T);self.axes.append(np.fromstring(joint.find('axis').attrib['xyz'],sep=' '))
            self.limits.append([float(joint.find('limit').attrib[k]) for k in ('lower','upper')])
        self.limits=np.array(self.limits)

    def vector(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite measured left joint angles')
        if np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):
            raise ValueError('实测关节角超出固定模型范围；不能生成点动目标')
        return q

    def fk(self,q):
        q=self.vector(q);T=np.eye(4)
        for origin,axis,angle in zip(self.origins,self.axes,q):
            A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
            T=T@origin@A
        return T

    def fk_jacobian(self,q):
        q=self.vector(q);T=np.eye(4);pivots=[];axes=[]
        for origin,axis,angle in zip(self.origins,self.axes,q):
            T=T@origin;pivots.append(T[:3,3].copy());axes.append(T[:3,:3]@axis)
            A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix();T=T@A
        J=np.zeros((6,6))
        for i,(p,w) in enumerate(zip(pivots,axes)):
            J[:3,i]=np.cross(w,T[:3,3]-p);J[3:,i]=w
        return T,J

    @staticmethod
    def residual_jacobian(T,J,goal):
        r=Rotation.from_matrix(goal[:3,:3].T@T[:3,:3]).as_rotvec()
        x,y,z=r;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        theta=np.linalg.norm(r)
        a=1/12+theta**2/720 if theta<1e-4 else (1-.5*theta/np.tan(.5*theta))/theta**2
        inverse_left=np.eye(3)-.5*K+a*K@K
        residual=np.r_[(T[:3,3]-goal[:3,3])*10,r]
        jac=np.vstack((J[:3]*10,inverse_left@goal[:3,:3].T@J[3:]))
        return residual,jac

    def solve(self,q,direction,step_mm=2):
        if direction not in DIRECTIONS:raise ValueError('Unknown base-frame direction')
        if type(step_mm) is not int or step_mm not in STEP_MM:
            raise ValueError('平移步长只能选择 2、5、10 mm')
        step_m=step_mm/1000
        q=self.vector(q);start=self.fk(q);goal=start.copy()
        goal[:3,3]+=step_m*np.asarray(DIRECTIONS[direction])
        lower,upper=self.limits.T
        # Search the real joint range; a 2 mm Cartesian request is not a
        # 2 degree joint limit. Seeds are calculations only, never waypoints.
        seeds=[q]
        for degrees in (10,-10,25,-25):
            seeds.append(np.clip(q+np.deg2rad([0,degrees,-2*degrees,degrees,0,0]),lower,upper))
        seeds.append((lower+upper)/2)
        started=time.monotonic();best=None;path_rejected=False;attempts=0
        deadline=started+.20
        class SolveTimeout(Exception):pass
        cached_q=None;cached=None
        def evaluate(v):
            nonlocal cached_q,cached
            if time.monotonic()>deadline:raise SolveTimeout
            if cached_q is None or not np.array_equal(v,cached_q):
                T,J=self.fk_jacobian(v);cached=self.residual_jacobian(T,J,goal);cached_q=v.copy()
            return cached
        for seed in seeds:
            try:
                result=least_squares(lambda v:evaluate(v)[0],np.clip(seed,lower+1e-9,upper-1e-9),
                    jac=lambda v:evaluate(v)[1],bounds=(lower,upper),max_nfev=80,
                    ftol=1e-10,xtol=1e-10,gtol=1e-10)
            except SolveTimeout:break
            attempts+=1
            if best is None or np.linalg.norm(result.fun)<np.linalg.norm(best.fun):best=result
            position_error=float(np.linalg.norm(result.fun[:3])/10)
            angle_error=float(np.linalg.norm(result.fun[3:]))
            if not result.success or position_error>.00005 or angle_error>.0005:continue
            target=result.x
            # Check a denser nominal joint-interpolated path for larger joint
            # changes. This is not obstacle checking or a firmware guarantee.
            dq=float(np.max(np.abs(target-q)))
            samples=max(9,int(np.ceil(dq/np.deg2rad(.25)))+1)
            if samples>361:
                path_rejected=True;continue
            valid=True
            for fraction in np.linspace(0,1,samples):
                if time.monotonic()>deadline:valid=False;break
                T=self.fk(q+fraction*(target-q))
                if (np.linalg.norm(T[:3,3]-(start[:3,3]+fraction*(goal[:3,3]-start[:3,3])))>.00025 or
                        Rotation.from_matrix(start[:3,:3].T@T[:3,:3]).magnitude()>.002):
                    valid=False;break
            if not valid:path_rejected=True;continue
            return target.tolist(),dict(frame='left_base_link',controlled_point='left_link6',
                direction=direction,step_m=step_m,orientation_preserved=True,
                solver='scipy_trf_analytic_jacobian_multiseed',attempts=attempts,
                elapsed_ms=(time.monotonic()-started)*1000,search_bounds='physical_joint_limits',
                path_samples=samples,position_residual_m=position_error,orientation_residual_rad=angle_error,
                maximum_joint_delta_rad=dq,urdf_sha256=URDF_SHA,collision_checked=False,physical_validated=False)
        if path_rejected:
            raise ValueError('已找到目标逆解，但途中直线／朝向检查未通过；未下发动作，需先调整机械臂姿态')
        if best is None:raise ValueError('逆解计算超时，未下发动作；请重新 Hold 后检查')
        active=[f'J{i+1}'+('下限' if abs(v-lo)<1e-5 else '上限')
                for i,(v,(lo,hi)) in enumerate(zip(best.x,self.limits)) if min(abs(v-lo),abs(v-hi))<1e-5]
        why='限位附近未找到保持朝向的解（'+','.join(active)+'）' if active else '搜索未收敛到满足位置和朝向的解'
        raise ValueError(f'当前姿态无法完成 {step_mm} mm：{why}；最佳位置残差 {np.linalg.norm(best.fun[:3])*100:.3f} mm，朝向误差 {np.rad2deg(np.linalg.norm(best.fun[3:])):.3f}°；未下发动作')
