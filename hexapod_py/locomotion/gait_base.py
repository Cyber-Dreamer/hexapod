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
    def __init__(self, kinematics, step_height, knee_direction=-1):
        """
        Initializes the Gait object.

        :param kinematics: The HexapodKinematics object.
        :param step_height: The height of a step.
        """
        self.knee_direction = knee_direction
        self.kinematics = kinematics
        self.step_height = step_height
        self.gait_phase = 0.0

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, last_known_angles, body_height, step_height, rotation_scale_factor=1.0):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, roll, pitch, default_foot_positions, last_known_angles, max_step_length, body_height, step_height, rotation_scale_factor=1.0):
        # vx, vy are now target linear velocities (mm/s)
        # omega is now target angular velocity (rad/s)

        # For a simple gait (e.g., tripod), stance phase is 50% of the cycle.
        # The step length needs to be twice the displacement that occurs during the stance phase.
        # However, the phase calculation is normalized. The `speed` parameter in `run_gait`
        # controls the cycle frequency. A simpler approach is to scale the velocity by a factor
        # to get a reasonable step length. Let's use a factor that makes `max_step_length`
        # correspond to the max velocity.
        step_translation_factor = 0.5 # This factor can be tuned.
        
        # Linear velocity component
        # This is calculated in the BODY frame.
        linear_step = np.array([vx, vy, 0]) * step_translation_factor
        # Rotational velocity component
        hip_pos = self.kinematics.hip_positions[leg_idx]
        rotational_step = np.array([-hip_pos[1], hip_pos[0], 0]) * omega * rotation_scale_factor
        # Total step vector is the sum of linear and rotational parts
        total_step = linear_step + rotational_step
        
        # Swing phase (leg is in the air, moving to the start of the next step)
        if phase < 0.5:
            swing_phase = phase * 2
            # Parabolic trajectory for the foot lift (0 -> 1 -> 0)
            z_lift = step_height * (1 - (2 * swing_phase - 1)**2)

            # Foot moves from its rearmost point to its foremost point
            swing_offset = total_step * (swing_phase - 0.5)
            
            # To prevent IK failures at the edge of the workspace, the swing motion
            # is performed closer to the body. We define a "swing center" that is
            # horizontally retracted from the default foot position.
            swing_retraction_factor = 0.7 # 0.0 = no retraction, 1.0 = retracts to hip. 0.7 is a good start.
            swing_center = default_foot_positions[leg_idx].copy()
            swing_center[0] *= (1.0 - swing_retraction_factor)
            swing_center[1] *= (1.0 - swing_retraction_factor)

            target_pos = swing_center + swing_offset
            target_pos[2] += z_lift
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
            
            # Combine rotations (Roll first, then Pitch)
            R_body = Ry @ Rx
            # Calculate the new hip position after body rotation
            hip_pos_rotated = R_body @ self.kinematics.hip_positions[leg_idx]
            # The vector to the foot is from the new hip position to the original target
            v_foot_body = target_pos - hip_pos_rotated 
        else:
            v_foot_body = target_pos - self.kinematics.hip_positions[leg_idx]
        # Rotate the vector from the body frame into the leg's local coordinate frame.
        # This is the same crucial step that was needed for body_ik to work correctly.
        hip_base_angle = self.kinematics.hip_base_angles[leg_idx]
        R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                              [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                              [0,                       0,                      1]])
        v_foot_local = R_hip_inv @ v_foot_body

        angles = self.kinematics.leg_ik(v_foot_local, knee_direction=self.knee_direction)

        if angles is None:
            # If the target is unreachable, fall back to the last known good angles for this leg.
            return last_known_angles[leg_idx]

        return angles
