"""Small numeric FK/IK for the UR10e + suction gripper, in the world frame.

Used by the v0 tests to place suction_tcp at a pose without MoveIt. Later phases use
MoveIt; this stays as a lightweight test utility.

DH parameters: Universal Robots' published UR10e values (consistent with
ur_description/config/ur10e/default_kinematics.yaml). The DH base frame is the ROS
`base` frame, i.e. `base_link` rotated by pi about z.
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

UR10E_DH = {  # d, a, alpha
    'd': [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
    'a': [0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0],
    'alpha': [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
}


def _tf(xyz=(0, 0, 0), rot=None):
    T = np.eye(4)
    T[:3, 3] = xyz
    if rot is not None:
        T[:3, :3] = rot
    return T


class UrKinematics:

    def __init__(self, cell):
        ped = cell['robot']['pedestal']
        g = cell['gripper']
        # world -> base (DH frame 0)
        self.T_world_base = _tf(
            (ped['xy'][0], ped['xy'][1], ped['size'][2]),
            Rotation.from_euler('z', np.pi).as_matrix())
        # flange (DH frame 6, = tool0) -> suction_tcp
        self.T_flange_tcp = _tf((0, 0, g['body_length'] + g['cup_length']))

    def fk(self, q):
        """World pose (4x4) of suction_tcp for joint vector q."""
        T = self.T_world_base.copy()
        for i, th in enumerate(q):
            d, a, al = UR10E_DH['d'][i], UR10E_DH['a'][i], UR10E_DH['alpha'][i]
            ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
            T = T @ np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1]])
        return T @ self.T_flange_tcp

    @staticmethod
    def down_pose(xyz, yaw=0.0):
        """TCP pose with the cup pointing straight down (tool z = -world z)."""
        R = Rotation.from_euler('z', yaw).as_matrix() @ Rotation.from_euler('x', np.pi).as_matrix()
        return _tf(xyz, R)

    def ik(self, T_target, seed, tol=1e-4):
        """Joint vector reaching T_target (world frame), closest to seed. None if unreachable."""
        seed = np.asarray(seed, dtype=float)

        def residual(q):
            T = self.fk(q)
            dp = T[:3, 3] - T_target[:3, 3]
            dr = Rotation.from_matrix(T_target[:3, :3].T @ T[:3, :3]).as_rotvec()
            # small regularizer keeps the solution near the seed (same arm configuration)
            return np.concatenate([dp, 0.3 * dr, 1e-3 * (q - seed)])

        best = None
        for s in [seed] + [seed + np.random.default_rng(i).normal(0, 0.3, 6) for i in range(8)]:
            r = least_squares(residual, s, bounds=(-2 * np.pi, 2 * np.pi))
            err = np.linalg.norm(r.fun[:6])
            if best is None or err < best[0]:
                best = (err, r.x)
            if err < tol:
                break
        return best[1] if best[0] < tol else None
