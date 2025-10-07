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

    def leg_fk(self, angles):
        """
        Calculates the local positions of the joints for a single leg, assuming
        the coxa joint is at the origin (0,0,0). This is the direct inverse
        of the leg_ik function.

        Args:
            angles (tuple[float, float, float]): The joint angles (gamma, alpha, beta)
                                                 for the coxa, femur, and tibia in radians.

        Returns:
            np.ndarray: A 4x3 array containing the (x, y, z) coordinates of the
                        hip, coxa-femur, femur-tibia, and foot tip joints in the
                        leg's local coordinate frame.
        """
        gamma, alpha, beta = angles

        p0 = np.array([0.0, 0.0, 0.0])

        # Position of the femur joint relative to the hip
        p1 = p0 + np.array([self.l_coxa * np.cos(gamma),
                            self.l_coxa * np.sin(gamma),
                            0])

        # Position of the tibia joint relative to the hip
        p2 = p1 + np.array([self.l_femur * np.cos(alpha) * np.cos(gamma),
                                   self.l_femur * np.cos(alpha) * np.sin(gamma),
                                   self.l_femur * np.sin(alpha)])

        # Position of the foot tip relative to the hip
        p3 = p2 + np.array([self.l_tibia * np.cos(alpha + beta) * np.cos(gamma),
                                   self.l_tibia * np.cos(alpha + beta) * np.sin(gamma),
                                   self.l_tibia * np.sin(alpha + beta)])
        
        return np.array([p0, p1, p2, p3])

    def body_fk(self, all_leg_angles, body_translation=np.array([0, 0, 0]), body_rotation=np.array([0, 0, 0])):
        """
        Calculates the world positions of all joints for all six legs.

        Args:
            all_leg_angles (list[tuple]): A list of 6 angle tuples (gamma, alpha, beta).
            body_translation (np.ndarray): The body's translation vector.
            body_rotation (np.ndarray): The body's rotation vector (roll, pitch, yaw).

        Returns:
            list[np.ndarray]: A list of 6 arrays, where each array is a 4x3 matrix
                              of joint coordinates in the world frame for one leg.
        """
        # --- Create Body Rotation Matrix ---
        roll, pitch, yaw = body_rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R_body = Rz @ Ry @ Rx

        world_leg_points = []
        for i in range(6):
            angles = all_leg_angles[i]
            hip_pos_world = R_body @ self.hip_positions[i] + body_translation
            if angles is None:
                # If angles are None, just transform the hip position
                world_leg_points.append(np.array([hip_pos_world]))
                continue

            # 1. Get the joint positions in the leg's local frame
            local_points = self.leg_fk(angles)

            # 2. Create rotation matrix for the leg's base mounting angle
            hip_base_angle = self.hip_base_angles[i]
            R_hip = np.array([[np.cos(hip_base_angle), -np.sin(hip_base_angle), 0],
                              [np.sin(hip_base_angle),  np.cos(hip_base_angle), 0],
                              [0,                      0,                     1]])

            # 3. Transform local leg points to world frame
            # Rotate local points by R_hip, then R_body, and add the final hip position
            # Note: local_points[0] is the origin, so we just add hip_pos_world
            leg_world = np.zeros_like(local_points)
            leg_world[0] = hip_pos_world
            for j in range(1, local_points.shape[0]):
                leg_world[j] = hip_pos_world + R_body @ (R_hip @ local_points[j])
            
            world_leg_points.append(leg_world)
            
        return world_leg_points