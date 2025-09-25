"""
Tripod Gait Implementation
==========================
"""

from .gait_base import Gait

class TripodGait(Gait):
    """
    Implements the Tripod Gait, where three legs move at a time.
    This provides a very stable walking motion.
    """
    def __init__(self, node, kinematics, step_height, step_length, knee_direction):
        super().__init__(node, kinematics, step_height, knee_direction)
        self.step_length = step_length
        self.tripod_legs_1 = [0, 2, 4] # FR, RR, ML
        self.tripod_legs_2 = [1, 3, 5] # MR, FL, RL

    def run(self, vx, vy, omega, pitch, speed, default_foot_positions):
        self.gait_phase = (self.gait_phase + speed) % 1.0
        joint_angles = [None] * 6
        
        phase_1 = self.gait_phase
        phase_2 = (self.gait_phase + 0.5) % 1.0

        # Move Tripod 1
        for leg_idx in self.tripod_legs_1:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_1, vx, vy, omega, pitch, default_foot_positions, self.step_length)

        # Move Tripod 2
        for leg_idx in self.tripod_legs_2:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_2, vx, vy, omega, pitch, default_foot_positions, self.step_length)
            
        return joint_angles