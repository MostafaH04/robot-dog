"""Kinematics helpers for the physical closed-chain leg geometry."""

import numpy as np


class LegKin:
    """Solve leg IK and map physical angles onto the simplified URDF joints."""

    def __init__(self):
        self.shoulder_len = 0.038
        self.a_len = 0.1059
        self.b_len = 0.0245
        self.c_len = 0.047434
        self.d_len = 0.063725
        self.e_len = 0.11058

    def leg_ik(self, relative_x, relative_y, relative_z, right):
        """Return three physical joint angles for a Cartesian foot target."""
        theta_0, target_z = self._yz_plane_ik(relative_y, relative_z, right)
        theta_1, theta_2 = self._xz_plane_ik(relative_x, target_z)
        return theta_0, theta_1, theta_2

    def leg_control_conversion(self, theta_0, theta_1, theta_2):
        """Map physical closed-chain angles to the simplified URDF joints."""
        theta_1_prime = theta_1 - np.pi / 2
        upper_inner = np.pi - theta_1 + theta_2
        alpha = np.sqrt(
            self.a_len ** 2
            + self.b_len ** 2
            - 2 * self.a_len * self.b_len * np.cos(upper_inner)
        )
        beta_upper = np.arccos(
            (self.b_len ** 2 - self.a_len ** 2 - alpha ** 2)
            / (-2 * self.a_len * alpha)
        )
        beta_lower = np.arccos(
            (self.e_len ** 2 - self.c_len ** 2 - alpha ** 2)
            / (-2 * self.c_len * alpha)
        )
        theta_2_prime = np.pi - (beta_upper + beta_lower)
        return theta_0, theta_1_prime, theta_2_prime

    def leg_control_conv(self, theta_0, theta_1, theta_2):
        """Preserve the original short name for downstream callers."""
        return self.leg_control_conversion(theta_0, theta_1, theta_2)

    def _yz_plane_ik(self, relative_y, relative_z, right):
        relative_dist_sqr = relative_y ** 2 + relative_z ** 2
        target_len = np.sqrt(relative_dist_sqr - self.shoulder_len ** 2)
        if relative_z < 0:
            target_len *= -1

        phi_0 = np.arccos(self.shoulder_len / np.sqrt(relative_dist_sqr))
        phi = np.arctan2(relative_z, relative_y if right else -relative_y)
        if phi < 0:
            phi += 2 * np.pi

        theta = phi_0 - (phi - np.pi)
        return theta, target_len

    def _xz_plane_ik(self, relative_x, relative_z):
        relative_dist_sqr = relative_x ** 2 + relative_z ** 2
        relative_dist = np.sqrt(relative_dist_sqr)
        combined_lower_len = self.c_len + self.d_len

        phi_prime = np.arccos(
            (combined_lower_len ** 2 - self.a_len ** 2 - relative_dist_sqr)
            / (-2 * self.a_len * relative_dist)
        )
        beta = np.arccos(
            (relative_dist_sqr - self.a_len ** 2 - combined_lower_len ** 2)
            / (-2 * self.a_len * combined_lower_len)
        )
        phi = np.arctan2(relative_z, relative_x)
        if phi < 0:
            phi += 2 * np.pi

        theta_1 = phi - np.pi - phi_prime
        internal_len_sqr = (
            self.a_len ** 2
            + self.c_len ** 2
            - 2 * self.a_len * self.c_len * np.cos(beta)
        )
        internal_len = np.sqrt(internal_len_sqr)
        phi_1_prime = np.arccos(
            (self.e_len ** 2 - internal_len_sqr - self.b_len ** 2)
            / (-2 * internal_len * self.b_len)
        )
        correction = np.arccos(
            (self.d_len ** 2 - relative_dist_sqr - internal_len_sqr)
            / (-2 * relative_dist * internal_len)
        )
        theta_2 = (2 * np.pi - phi) - phi_1_prime + correction
        return theta_1, -theta_2

    @staticmethod
    def _length(point):
        return np.hypot(point[0], point[1])
