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
    def __init__(self, kinematics, step_height, knee_direction):
        """
        Initializes the Gait object.

        :param kinematics: The HexapodKinematics object.
        :param step_height: The height of a step.
        :param knee_direction: The knee bend direction.
        """
        self.kinematics = kinematics
        self.step_height = step_height
        self.knee_direction = knee_direction
        self.gait_phase = 0.0

    def run(self, vx, vy, omega, pitch, speed, default_foot_positions, body_height, step_height):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, pitch, default_foot_positions, step_length, body_height, step_height):
        # Determine the movement vector for this step
        step_dir = np.array([vx, vy, 0])
        
        # Swing phase (leg is in the air, moving to the start of the next step)
        if phase < 0.5:
            swing_phase = phase * 2
            # Parabolic trajectory for the foot lift
            z_swing = step_height * (1 - (2 * swing_phase - 1)**2)
            
            # Foot moves from its end-point to its start-point (against movement direction)
            swing_offset = -step_dir * step_length * (1 - swing_phase)
            
            target_pos = default_foot_positions[leg_idx] + swing_offset
            target_pos[2] = -body_height + z_swing
        # Stance phase (leg is on the ground, pushing the body)
        else:
            stance_phase = (phase - 0.5) * 2
            
            # Foot moves from its start-point to its end-point (with movement direction)
            stance_offset = step_dir * step_length * (1 - stance_phase)
            
            target_pos = default_foot_positions[leg_idx] + stance_offset
            target_pos[2] = -body_height

        if abs(pitch) > 0.001:
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            target_pos = Ry @ target_pos

        v_foot_local = target_pos - self.kinematics.hip_positions[leg_idx]
        return self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
