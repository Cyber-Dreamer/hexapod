"""
Hexapod Kinematics Module
=========================

This module provides the HexapodKinematics class, which is responsible for all
kinematic calculations of the hexapod robot. This includes both forward and
inverse kinematics.

Coordinate Frames:
- World Frame: A fixed global frame of reference.
- Body Frame: A frame attached to the center of the robot's body.
- Leg Frame: A frame attached to each coxa joint.

Forward Kinematics: Calculates the 3D position of the leg joints given the
joint angles.

Inverse Kinematics: Calculates the required joint angles to move the foot of a
leg to a specific 3D position.
"""

import numpy as np

class HexapodKinematics:
    """
    The HexapodKinematics class encapsulates the kinematic calculations for a
    hexapod robot. It requires the leg segment lengths (coxa, femur, tibia)
    and the initial positions of the leg attachments to the body.
    """
    def __init__(self, leg_lengths, leg_positions, coxa_initial_rpys=None):
        """
        Initializes the HexapodKinematics object.

        :param leg_lengths: A list or tuple of the leg segment lengths
                            [coxa, femur, tibia].
        :param leg_positions: A list of tuples, where each tuple is the (x, y, z)
                              position of the coxa joint attachment in the body frame.
        :param coxa_initial_rpys: A list of [roll, pitch, yaw] for each coxa joint's
                                  initial orientation relative to the body frame.
        """
        self.leg_lengths = leg_lengths
        self.leg_positions = leg_positions
        self.coxa_initial_rpys = coxa_initial_rpys if coxa_initial_rpys is not None else [[0.0, 0.0, 0.0]] * len(leg_positions)

    def inverse_kinematics(self, foot_pos_local, knee_direction=1):
        """
        Calculates the inverse kinematics for a single leg in its local frame.
        Uses a standard Z-up, CCW angle convention.

        :param foot_pos_local: The desired (x, y, z) position of the foot
                               relative to the coxa joint.
        :param knee_direction: The desired direction of the knee bend.
                               1 for up/forward, -1 for down/backward.
        :return: A list of the three joint angles [coxa, femur, tibia] in radians,
                 or None if the position is unreachable.
        """
        coxa_len, femur_len, tibia_len = self.leg_lengths
        x, y, z = foot_pos_local

        # Coxa angle (theta1) - Top-down view, CCW from x-axis
        theta1 = np.arctan2(y, x)

        # Solve for femur and tibia angles in the leg's 2D plane
        # Project foot position onto the leg's vertical plane
        l1 = np.sqrt(x**2 + y**2)
        l_eff = l1 - coxa_len  # Effective horizontal distance from femur joint

        # Distance from femur joint to foot
        l3_sq = l_eff**2 + z**2
        l3 = np.sqrt(l3_sq)

        # Check if the target is reachable
        if l3 > femur_len + tibia_len or l3 < abs(femur_len - tibia_len):
            return None  # Position is unreachable

        # Angle of the tibia joint (theta3) using Law of Cosines
        # beta is the angle at the knee, inside the femur-tibia-l3 triangle
        cos_beta = (femur_len**2 + tibia_len**2 - l3_sq) / (2 * femur_len * tibia_len)
        beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))

        # theta3 is angle of tibia relative to femur. 0 is straight.
        # A positive value corresponds to a "knee-up" bend.
        theta3 = knee_direction * (np.pi - beta)

        # Angle of the femur joint (theta2)
        # alpha is the angle between the femur link and the line to the foot
        cos_alpha = (femur_len**2 + l3_sq - tibia_len**2) / (2 * femur_len * l3)
        alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))

        # phi is the angle of the foot position vector in the leg plane
        phi = np.arctan2(z, l_eff)

        # theta2 is angle of femur from horizontal.
        theta2 = phi - knee_direction * alpha

        return [theta1, theta2, theta3]

    def body_ik(self, translation, rotation, foot_positions, knee_direction=1):
        """
        Calculates the inverse kinematics for the entire body.

        This function computes the required joint angles for all legs to achieve
        a desired body posture, assuming the feet are planted on the ground.

        :param translation: Desired [x, y, z] translation of the body in the world frame.
        :param rotation: Desired [roll, pitch, yaw] rotation of the body.
        :param foot_positions: A list of the current (x, y, z) positions of the feet
                               in the world frame.
        :param knee_direction: The desired direction of the knee (1 for up, -1 for down).
        :return: A list of lists, where each inner list contains the joint angles
                 for a leg.
        """
        joint_angles_list = []
        
        roll, pitch, yaw = rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        
        T = np.array(translation)

        for i in range(6):
            # Position of the coxa joint in the body frame
            p_coxa_body = np.array(self.leg_positions[i])
            
            # Calculate the coxa joint's position in the world frame
            p_coxa_world = R @ p_coxa_body + T
            
            # Vector from the coxa joint to the foot in the world frame
            v_foot_world = np.array(foot_positions[i]) - p_coxa_world
            
            # Transform the vector into the leg's local frame (which is the rotated body frame)
            v_foot_local = R.T @ v_foot_world
            
            # Apply inverse of initial coxa rotation to v_foot_local
            # This is to align the local frame with the kinematic model's assumption (X-axis along coxa when angle is 0)
            roll_offset, pitch_offset, yaw_offset = self.coxa_initial_rpys[i]

            # Create inverse rotation matrices for roll, pitch, yaw
            Rx_inv = np.array([[1, 0, 0], [0, np.cos(-roll_offset), -np.sin(-roll_offset)], [0, np.sin(-roll_offset), np.cos(-roll_offset)]])
            Ry_inv = np.array([[np.cos(-pitch_offset), 0, np.sin(-pitch_offset)], [0, 1, 0], [-np.sin(-pitch_offset), 0, np.cos(-pitch_offset)]])
            Rz_inv = np.array([[np.cos(-yaw_offset), -np.sin(-yaw_offset), 0], [np.sin(-yaw_offset), np.cos(-yaw_offset), 0], [0, 0, 1]])

            # Apply inverse rotation in reverse order (extrinsic ZYX -> intrinsic XYZ)
            # Or, if RPY is extrinsic ZYX, then inverse is R_inv = Rx_inv @ Ry_inv @ Rz_inv
            # Assuming the RPY in URDF is extrinsic ZYX (common)
            R_coxa_inv = Rx_inv @ Ry_inv @ Rz_inv
            v_foot_local_adjusted = R_coxa_inv @ v_foot_local

            # Calculate the joint angles for the leg
            joint_angles = self.inverse_kinematics(v_foot_local_adjusted, knee_direction)
            joint_angles_list.append(joint_angles)
            
        return joint_angles_list

    def forward_kinematics(self, leg_index, joint_angles):
        """
        Computes the foot position for a single leg using forward kinematics.
        Uses a standard Z-up, CCW angle convention.
        
        :param leg_index: The index of the leg (0-5).
        :param joint_angles: [coxa, femur, tibia] angles in radians.
        :return: The (x, y, z) position of the foot in the body frame.
        """
        coxa_len, femur_len, tibia_len = self.leg_lengths
        theta1, theta2, theta3 = joint_angles
        
        coxa_pos = self.leg_positions[leg_index]

        # Horizontal distance covered by femur and tibia
        l_horiz = femur_len * np.cos(theta2) + tibia_len * np.cos(theta2 + theta3)
        
        # Total horizontal distance from coxa axis
        l_total = coxa_len + l_horiz
        
        # Position of the foot relative to the coxa attachment point
        x_local = l_total * np.cos(theta1)
        y_local = l_total * np.sin(theta1)
        z_local = femur_len * np.sin(theta2) + tibia_len * np.sin(theta2 + theta3)
        
        # Position of the foot in the body frame
        foot_pos_body = np.array(coxa_pos) + np.array([x_local, y_local, z_local])
        
        return foot_pos_body.tolist()