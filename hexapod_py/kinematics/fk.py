"""
Hexapod Forward Kinematics Module
=================================

This module implements the forward kinematics for the hexapod, calculating
the 3D positions of each joint given a set of joint angles.
"""

import numpy as np

class HexapodForwardKinematics:
    """
    Calculates the 3D world positions of all joints of the hexapod.
    """

    def __init__(self, leg_lengths, hip_positions):
        """
        Initializes the forward kinematics calculator.

        Args:
            leg_lengths (list[float]): A list of three floats representing the
                                       lengths of the coxa, femur, and tibia.
            hip_positions (np.ndarray): A 6x3 array where each row is the (x, y, z)
                                        position of a coxa joint relative to the
                                        body's center.
        """
        self.l_coxa, self.l_femur, self.l_tibia = leg_lengths
        self.hip_positions = np.array(hip_positions)
        # Pre-calculate the base angle for each hip from its position
        self.hip_base_angles = np.arctan2(self.hip_positions[:, 1], self.hip_positions[:, 0])

    def leg_fk(self, angles, leg_idx, body_translation=np.array([0, 0, 0]), body_rotation=np.array([0, 0, 0])):
        """
        Calculates the world positions of the joints for a single leg.

        Args:
            angles (tuple[float, float, float]): The joint angles (gamma, alpha, beta)
                                                 for the coxa, femur, and tibia in radians.
            leg_idx (int): The index of the leg (0-5).
            body_translation (np.ndarray): A 3-element numpy array (tx, ty, tz)
                                           representing the body's translation.
            body_rotation (np.ndarray): A 3-element numpy array (roll, pitch, yaw)
                                        representing the body's rotation in radians.

        Returns:
            np.ndarray: A 4x3 array containing the (x, y, z) coordinates of the
                        hip, coxa-femur, femur-tibia, and foot tip joints.
        """
        gamma, alpha, beta = angles
        hip_pos = self.hip_positions[leg_idx]

        # --- Apply body transformation ---
        roll, pitch, yaw = body_rotation
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        # Combined rotation matrix (Yaw -> Pitch -> Roll)
        R = Rz @ Ry @ Rx

        # 1. Calculate joint positions in the leg's local coordinate system,
        #    relative to the hip attachment point. This logic is the mathematical
        #    inverse of the leg_ik function.

        # The total rotation of the coxa joint is its base mounting angle plus the joint's own rotation (gamma).
        total_gamma = self.hip_base_angles[leg_idx] + gamma

        # Position of the femur joint relative to the hip
        p1_local = np.array([self.l_coxa * np.cos(total_gamma),
                             self.l_coxa * np.sin(total_gamma),
                             0])

        # Position of the tibia joint relative to the hip
        p2_local = p1_local + np.array([self.l_femur * np.cos(alpha) * np.cos(total_gamma),
                                        self.l_femur * np.cos(alpha) * np.sin(total_gamma),
                                        self.l_femur * np.sin(alpha)])

        # Position of the foot tip relative to the hip
        p3_local = p2_local + np.array([self.l_tibia * np.cos(alpha + beta) * np.cos(total_gamma),
                                        self.l_tibia * np.cos(alpha + beta) * np.sin(total_gamma),
                                        self.l_tibia * np.sin(alpha + beta)])

        # 3. Transform the entire leg to the world frame.
        #    First, rotate the hip's base position and the local joint vectors.
        #    Then, translate them by the body's translation vector.
        rotated_hip_pos = R @ hip_pos + body_translation
        return np.array([rotated_hip_pos,
                         rotated_hip_pos + R @ p1_local,
                         rotated_hip_pos + R @ p2_local,
                         rotated_hip_pos + R @ p3_local])