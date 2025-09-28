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

    def run(self, vx, vy, omega, pitch, speed, default_foot_positions):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, pitch, default_foot_positions, step_length, body_height):
        # Swing phase (leg is in the air)
        if phase < 0.5:
            swing_phase = phase * 2
            z = self.step_height * (1 - (2 * swing_phase - 1)**2)
            x_swing = -step_length / 2 * (1 - swing_phase)
            y_swing = 0
            target_pos = default_foot_positions[leg_idx] + np.array([x_swing, y_swing, 0])
            target_pos[2] = -body_height + z
        else:
            stance_phase = (phase - 0.5) * 2
            x_stance = step_length / 2 * (1 - stance_phase)
            y_stance = 0
            x_stance -= vx * step_length * stance_phase
            y_stance -= vy * step_length * stance_phase
            if abs(omega) > 0.01:
                rot_z = -omega * step_length * stance_phase
                start_pos = default_foot_positions[leg_idx]
                c, s = np.cos(rot_z), np.sin(rot_z)
                rot_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                rotated_pos = rot_matrix @ start_pos
                x_stance += rotated_pos[0] - start_pos[0]
                y_stance += rotated_pos[1] - start_pos[1]
            target_pos = default_foot_positions[leg_idx] + np.array([x_stance, y_stance, 0])
            target_pos[2] = -body_height
        if abs(pitch) > 0.001:
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            target_pos = Ry @ target_pos
        v_foot_local = target_pos - self.kinematics.leg_positions[leg_idx]
        return self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
