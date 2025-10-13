"""
Ripple Gait Implementation
==========================
"""

from .gait_base import Gait

class RippleGait(Gait):
    def __init__(self, kinematics, step_height, max_step_length, knee_direction=-1):
        super().__init__(kinematics, step_height, knee_direction=knee_direction)
        self.max_step_length = max_step_length
        # The sequence of legs lifting in a ripple gait, following the new
        # clockwise numbering scheme. Starts with a rear leg.
        self.ripple_sequence = [5, 1, 3, 0, 4, 2] # RR, ML, FR, RL, MR, FL

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, default_joint_angles, body_height, step_height, rotation_scale_factor=1.0):
        self.gait_phase = (self.gait_phase + speed) % 1.0
        joint_angles = [None] * 6
        num_legs = len(self.ripple_sequence)
        for i, leg_idx in enumerate(self.ripple_sequence):
            phase = (self.gait_phase + (i / num_legs)) % 1.0
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase, vx, vy, omega, roll, pitch, default_foot_positions, default_joint_angles, self.max_step_length, body_height, step_height, rotation_scale_factor)
        return joint_angles
