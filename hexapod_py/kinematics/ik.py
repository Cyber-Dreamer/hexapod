import numpy as np
from typing import List, Optional

class HexapodKinematics:

    def __init__(self, segment_lengths, hip_positions, joint_limits):
        self.segment_lengths = segment_lengths
        self.hip_positions = hip_positions
        self.joint_limits = joint_limits
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        self.hip_base_angles = np.arctan2(np.array(self.hip_positions)[:, 1], np.array(self.hip_positions)[:, 0])
        
    def leg_ik(self, coordinate: np.ndarray, knee_direction: int = -1) -> Optional[List[float]]:
        """
        Calculates the joint angles for a single leg to reach a target coordinate.

        Args:
            coordinate (np.ndarray): The target (x, y, z) position for the foot.
            knee_direction (int):    The desired knee bend direction.
                                     -1 for "knee down" (standard walking).
                                     +1 for "knee up". Defaults to -1.

        Returns:
            A list of 3 joint angles [gamma, alpha, beta] in radians,
            or None if the position is unreachable.
        """
        x, y, z = coordinate
        l_coxa, l_femur, l_tibia = self.segment_lengths

        # 1. Coxa Angle (gamma) - Constrained to forward-facing plane
        gamma = np.arctan2(y, abs(x))

        # 2. Reduce to 2D side-view problem
        x_in_leg_frame = x * np.cos(gamma) + y * np.sin(gamma)
        x_prime = x_in_leg_frame - l_coxa
        z_prime = z
        
        dist_femur_to_foot = np.sqrt(x_prime**2 + z_prime**2)

        # 3. Reachability Check
        epsilon = 1e-6
        if (dist_femur_to_foot > l_femur + l_tibia + epsilon) or \
           (dist_femur_to_foot < abs(l_femur - l_tibia) - epsilon):
            return None

        # 4. Law of Cosines to find internal angles
        cos_beta_arg = (l_femur**2 + l_tibia**2 - dist_femur_to_foot**2) / (2 * l_femur * l_tibia)
        beta_inner = np.arccos(np.clip(cos_beta_arg, -1.0, 1.0))
        
        cos_alpha_arg = (dist_femur_to_foot**2 + l_femur**2 - l_tibia**2) / (2 * dist_femur_to_foot * l_femur)
        alpha_inner = np.arccos(np.clip(cos_alpha_arg, -1.0, 1.0))

        phi = np.arctan2(z_prime, x_prime)

        # 5. Set final angles using the knee_direction parameter
        # This replaces the old "if z >= 0" logic with a direct calculation.
        alpha = phi - (knee_direction * alpha_inner)
        beta = knee_direction * (np.pi - beta_inner)

        # 6. Joint Limit Check
        angles = np.array([gamma, alpha, beta])
        limits = np.array(self.joint_limits)
        if np.any(np.abs(angles) > limits):
            return None

        return angles.tolist()

    # The body_ik function remains unchanged as its job is only to calculate coordinates.
    def body_ik(self, translation, rotation, coxa_positions, default_foot_positions):
        roll, pitch, yaw = rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        new_foot_positions = np.zeros_like(default_foot_positions)
        for i in range(6):
            hip_pos_new = R @ coxa_positions[i] + translation
            foot_vector_world = default_foot_positions[i] - hip_pos_new
            foot_vector_body = R.T @ foot_vector_world
            hip_base_angle = self.hip_base_angles[i]
            R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                                  [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                                  [0,                      0,                      1]])
            new_foot_positions[i] = R_hip_inv @ foot_vector_body
        return new_foot_positions