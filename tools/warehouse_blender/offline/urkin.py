import numpy as np
from scipy.spatial.transform import Rotation as R
def T(xyz=(0,0,0), rpy=(0,0,0)):
    M=np.eye(4); M[:3,:3]=R.from_euler('xyz',rpy).as_matrix(); M[:3,3]=xyz; return M
def Rz(q): return T(rpy=(0,0,q))
pi=np.pi
PED=0.70
# (joint origin xyz, rpy) parent->child for 6 revolute joints (axis z)
J=[((0,0,0.1807),(0,0,0)),((0,0,0),(1.570796327,0,0)),((-0.6127,0,0),(0,0,0)),
   ((-0.57155,0,0.17415),(0,0,0)),((0,-0.11985,0),(1.570796327,0,0)),((0,0.11655,0),(1.570796326589793,pi,pi))]
TOOL = T(rpy=(0,-pi/2,-pi/2)) @ T(rpy=(pi/2,0,pi/2))   # wrist_3 -> flange -> tool0
TCP_OFF = T((0,0,0.12+0.03))                             # tool0 -> suction_tcp (cup face)
def link_frames(q):
    M = T((0,0,PED)) @ T(rpy=(0,0,pi))  # world->base_link->base_link_inertia
    out=[M.copy()]
    for (xyz,rpy),qi in zip(J,q):
        M = M @ T(xyz,rpy) @ Rz(qi); out.append(M.copy())
    return out  # [base_inertia, shoulder, upper, fore, w1, w2, w3]
def tcp(q): return link_frames(q)[-1] @ TOOL @ TCP_OFF

from scipy.optimize import least_squares
HOME=np.array([-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0.0])
def target_R(yaw):  # tool z down, tool x along world yaw
    return R.from_euler('xyz',(pi,0,0)).as_matrix() @ R.from_euler('z',-yaw).as_matrix()
def ik(p, yaw, seed=None):
    Rt = R.from_euler('z',yaw).as_matrix() @ np.diag([1,-1,-1])
    if seed is None:
        seed = HOME.copy(); seed[0] = np.arctan2(p[1],p[0]) - np.arctan2(-0.691,0.174) + HOME[0]
    def res(q):
        M=tcp(q); e1=M[:3,3]-p; e2=(M[:3,:3]-Rt).ravel()*0.3
        return np.concatenate([e1,e2])
    s=least_squares(res, seed, xtol=1e-12, ftol=1e-12)
    q=s.x; q=(q+pi)%(2*pi)-pi
    M=tcp(q); err=np.linalg.norm(M[:3,3]-p); oerr=np.linalg.norm(M[:3,:3]-Rt)
    return q, err, oerr

def ik_best(p, yaw, prev=None):
    p=np.array(p,float); best=None
    pan0=np.arctan2(p[1],p[0])
    seeds=[]
    for dp in (-pi/2, pi/2):
        for el in (1.2,1.8,2.4):
            for lift in (-2.2,-1.6,-1.0):
                seeds.append(np.array([pan0+dp, lift, el, -pi/2-lift-el, -pi/2, 0.0]))
    if prev is not None: seeds.insert(0,np.array(prev))
    for s in seeds:
        q,e,oe=ik(p,yaw,s)
        if e>1e-4 or oe>1e-3: continue
        F=link_frames(q)
        if F[3][2,3] < F[2][2,3]+0.05: continue  # elbow above shoulder
        cost = 0 if prev is None else np.sum(np.abs(((q-np.array(prev))+pi)%(2*pi)-pi))
        if best is None or cost<best[1]: best=(q,cost)
    return None if best is None else best[0]
