"""NOVA guidance: where the drone should go and which way it should face.

Pure maths, with no MuJoCo or ROS, so it can move into its own node before any
hardware flight. Ported from NOVA_PAMPC/sim_world.py.

p_T must come from the UKF, never from ground truth.
"""
import numpy as np


class Guidance:

    def __init__(self, fx, cx, initial_yaw, n=20, d_safe=1.0, z_offset=1.0, t_ramp=1.0):
        self.fx = fx
        self.cx = cx
        self.n = n
        self.d_safe = d_safe
        self.z_offset = z_offset
        self.t_ramp = t_ramp
        self._frozen_yaw = initial_yaw
        self._last_dir = np.array([1.0, 0.0])

    def shifted_goal(self, p_T):
        """NOVA eq. 19, with Z held at a fixed height above the target."""
        xy_norm = np.hypot(p_T[0], p_T[1])
        if xy_norm > 1e-3:
            self._last_dir = np.array([p_T[0], p_T[1]]) / xy_norm
        direction = self._last_dir
        return np.array([self.d_safe * direction[0], self.d_safe * direction[1], self.z_offset])

    def blended_goal(self, p_T, t):
        """NOVA eq. 20: fades from the drone's position to the shifted goal over t_ramp."""
        s = max(0.0, 1.0 - t / self.t_ramp)
        a_xy = s * s
        a_z = s
        shifted = self.shifted_goal(p_T)
        xy = a_xy * p_T[:2] + (1.0 - a_xy) * shifted[:2]
        z = a_z * p_T[2] + (1.0 - a_z) * shifted[2]
        return np.array([xy[0], xy[1], z])

    def yaw_to_target(self, psi, box_center_u):
        """NOVA eq. 22. box_center_u is None when there is no detection: yaw freezes."""
        if box_center_u is not None:
            self._frozen_yaw = psi - np.arctan((box_center_u - self.cx) / self.fx)
        return self._frozen_yaw

    def horizon(self, p_T, yaw, t):
        """NOVA eq. 21: the same goal on every node. Returns positions, velocities, yaws."""
        goal = self.blended_goal(p_T, t)
        positions = np.tile(goal, (self.n + 1, 1))
        velocities = np.zeros((self.n + 1, 3))
        yaws = np.full(self.n + 1, yaw)
        return positions, velocities, yaws
