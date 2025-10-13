"""
Gait Base Class
===============

This module defines the abstract base class for all gait implementations.
"""

import numpy as np

class Gait:
    """
    Abstract base class for hexapod gaits.
    """
    def __init__(self, kinematics, step_height):
        """
        Initializes the Gait object.

        :param kinematics: The HexapodKinematics object.
        :param step_height: The height of a step.
        """
        self.kinematics = kinematics
        self.step_height = step_height
        self.gait_phase = 0.0

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, default_joint_angles, body_height, step_height, max_step_length):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, roll, pitch, default_foot_positions, default_joint_angles, max_step_length, body_height, step_height):
        # Linear velocity component
        linear_step = np.array([vx, vy, 0]) * max_step_length

        # Rotational velocity component
        # Get the vector from body center to the leg's default position
        hip_pos = self.kinematics.hip_positions[leg_idx]
        # The rotational movement is perpendicular to this vector
        rotational_step = np.array([-hip_pos[1], hip_pos[0], 0]) * omega * max_step_length

        # Total step vector is the sum of linear and rotational parts
        total_step = linear_step + rotational_step
        
        # Swing phase (leg is in the air, moving to the start of the next step)
        if phase < 0.5:
            swing_phase = phase * 2
            # Parabolic trajectory for the foot lift (0 -> 1 -> 0)
            z_swing = step_height * (1 - (2 * swing_phase - 1)**2)

            # Foot moves from its rearmost point to its foremost point
            swing_offset = total_step * (swing_phase - 0.5)
            
            target_pos = default_foot_positions[leg_idx] + swing_offset
            target_pos[2] += z_swing
        # Stance phase (leg is on the ground, pushing the body)
        else:
            stance_phase = (phase - 0.5) * 2
            # Foot moves from its foremost point to its rearmost point
            stance_offset = total_step * (0.5 - stance_phase)

            target_pos = default_foot_positions[leg_idx] + stance_offset

        # Apply body roll and pitch rotation
        if abs(roll) > 0.001 or abs(pitch) > 0.001:
            # Rotation matrix for roll (around X-axis)
            Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
            # Rotation matrix for pitch (around Y-axis)
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            
            # Combine rotations (Roll first, then Pitch) and apply to the target position
            R_body = Ry @ Rx
            target_pos = R_body @ target_pos

        # Calculate the vector from the hip to the target in the body's coordinate frame
        v_foot_body = target_pos - self.kinematics.hip_positions[leg_idx]

        # Rotate the body-frame vector into the leg's local coordinate frame.
        # This is the same crucial step that was needed for body_ik to work correctly.
        hip_base_angle = self.kinematics.hip_base_angles[leg_idx]
        R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                              [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                              [0,                       0,                      1]])
        v_foot_local = R_hip_inv @ v_foot_body

        angles = self.kinematics.leg_ik(v_foot_local)

        if angles is None:
            # If the target is unreachable, instantly fall back to the pre-calculated neutral angles for this leg.
            # This is extremely fast and avoids expensive real-time calculations.
            return default_joint_angles[leg_idx]

        return angles
