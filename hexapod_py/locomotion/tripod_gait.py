from .gait_base import Gait

class TripodGait(Gait):
    def __init__(self, kinematics, step_height, max_step_length, knee_direction=-1):
        super().__init__(kinematics, step_height, knee_direction=knee_direction)
        self.max_step_length = max_step_length
        # Tripod 1: Front-Right (3), Rear-Right (5), Middle-Left (1)
        self.tripod_legs_1 = [3, 5, 1]
        # Tripod 2: Front-Left (2), Rear-Left (0), Middle-Right (4)
        self.tripod_legs_2 = [2, 0, 4]

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, last_known_angles, body_height, step_height):
        self.gait_phase = (self.gait_phase + speed) % 1.0
        joint_angles = [None] * 6

        # In a tripod gait, the two groups of legs are exactly half a phase apart.
        # One group is in stance (on the ground) while the other is in swing (in the air).
        phase_tripod_1 = self.gait_phase
        phase_tripod_2 = (self.gait_phase + 0.5) % 1.0

        for leg_idx in self.tripod_legs_1:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_tripod_1, vx, vy, omega, roll, pitch, default_foot_positions, last_known_angles, body_height, step_height)
        for leg_idx in self.tripod_legs_2:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_tripod_2, vx, vy, omega, roll, pitch, default_foot_positions, last_known_angles, body_height, step_height)

        return joint_angles
