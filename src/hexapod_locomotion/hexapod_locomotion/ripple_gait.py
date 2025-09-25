"""
Ripple Gait Implementation
==========================
"""

from .gait_base import Gait

class RippleGait(Gait):
    """
    Implements the Ripple Gait, where legs move in a wave-like sequence.
    This gait is slower but can be more stable on uneven terrain.
    """
    def __init__(self, node, kinematics, step_height, step_length, knee_direction):
        super().__init__(node, kinematics, step_height, knee_direction)
        self.step_length = step_length
        self.ripple_sequence = [0, 3, 1, 4, 2, 5] # FR, FL, MR, ML, RR, RL

    def run(self, vx, vy, omega, pitch, speed, default_foot_positions):
        self.gait_phase = (self.gait_phase + speed) % 1.0
        joint_angles = [None] * 6
        num_legs = len(self.ripple_sequence)
        
        for i, leg_idx in enumerate(self.ripple_sequence):
            # Each leg is offset in phase
            phase = (self.gait_phase + (i / num_legs)) % 1.0
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase, vx, vy, omega, pitch, default_foot_positions, self.step_length)
            
        return joint_angles