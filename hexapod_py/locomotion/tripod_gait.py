"""
Tripod Gait Implementation
==========================
"""

from .gait_base import Gait

class TripodGait(Gait):
    def __init__(self, kinematics, step_height, step_length, knee_direction):
        super().__init__(kinematics, step_height, knee_direction)
        self.step_length = step_length
        # Tripod 1: Front-Right (0), Rear-Right (2), Middle-Left (4)
        self.tripod_legs_1 = [0, 2, 4]
        # Tripod 2: Front-Left (3), Rear-Left (5), Middle-Right (1)
        self.tripod_legs_2 = [3, 5, 1]

    def run(self, vx, vy, omega, pitch, speed, default_foot_positions, body_height, step_height):
        self.gait_phase = (self.gait_phase + speed) % 1.0
        joint_angles = [None] * 6

        # In a tripod gait, the two groups of legs are exactly half a phase apart.
        # One group is in stance (on the ground) while the other is in swing (in the air).
        phase_tripod_1 = self.gait_phase
        phase_tripod_2 = (self.gait_phase + 0.5) % 1.0

        for leg_idx in self.tripod_legs_1:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_tripod_1, vx, vy, omega, pitch, default_foot_positions, self.step_length, body_height, step_height)
        for leg_idx in self.tripod_legs_2:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_tripod_2, vx, vy, omega, pitch, default_foot_positions, self.step_length, body_height, step_height)

        return joint_angles
