"""
Hexapod Locomotion Module
=========================
This module implements the high-level locomotion logic for the hexapod robot.
It acts as a manager for different gait patterns, loading and running the
selected gait. The actual gait logic is implemented in separate classes.
"""

import numpy as np
from hexapod_py.kinematics.ik import HexapodKinematics
from .tripod_gait import TripodGait
from .ripple_gait import RippleGait

class HexapodLocomotion:
    def __init__(self, step_height=0.05, step_length=0.1, knee_direction=-1, gait_type='tripod', body_height=0.20, standoff_distance=0.25):
        # --- Robot Dimensions ---
        leg_positions = [
            [ 0.13163, -0.07600853, 0.0],  # Leg 0: Front-Right
            [ 0.0,     -0.15201706, 0.0],  # Leg 1: Middle-Right
            [-0.13163, -0.07600853, 0.0],  # Leg 2: Rear-Right
            [ 0.13163,  0.07600853, 0.0],  # Leg 3: Front-Left
            [ 0.0,      0.15201706, 0.0],  # Leg 4: Middle-Left
            [-0.13163,  0.07600853, 0.0]   # Leg 5: Rear-Left
        ]
        leg_lengths = [0.078346, 0.19180174, 0.28496869]  # [coxa, femur, tibia]
        calculated_coxa_initial_rpys = []
        for pos in leg_positions:
            x, y, _ = pos
            yaw = np.arctan2(y, x)
            calculated_coxa_initial_rpys.append([0.0, 0.0, yaw])
        self.kinematics = HexapodKinematics(leg_lengths=leg_lengths, leg_positions=leg_positions, coxa_initial_rpys=calculated_coxa_initial_rpys)
        self.knee_direction = knee_direction
        self.available_gaits = {
            'tripod': TripodGait(self.kinematics, step_height, step_length, knee_direction),
            'ripple': RippleGait(self.kinematics, step_height, step_length, knee_direction)
        }
        self.set_gait(gait_type)
        self.body_height = body_height
        self.standoff_distance = standoff_distance
        self.recalculate_stance()

    def set_gait(self, gait_type):
        self.gait_type = gait_type
        self.current_gait = self.available_gaits.get(gait_type)
        if self.current_gait:
            self.current_gait.gait_phase = 0.0

    def update_knee_direction(self, new_direction):
        self.knee_direction = new_direction
        for gait in self.available_gaits.values():
            gait.knee_direction = new_direction
        return self.set_body_pose([0, 0, 0], [0, 0, 0])

    def set_body_pose(self, translation, rotation):
        self.recalculate_stance()
        return self.kinematics.body_ik(translation, rotation, self.foot_positions, self.knee_direction)

    def run_gait(self, vx, vy, omega, pitch=0.0, speed=0.02):
        if self.current_gait:
            self.recalculate_stance()
            return self.current_gait.run(vx, vy, omega, pitch, speed, self.default_foot_positions, self.body_height)
        else:
            return [None] * 6

    def recalculate_stance(self):
        self.default_foot_positions = []
        for pos in self.kinematics.leg_positions:
            v = np.array([pos[0], pos[1], 0])
            if np.linalg.norm(v) > 0:
                v_norm = v / np.linalg.norm(v)
            else:
                v_norm = v
            default_pos_xy = v + v_norm * self.standoff_distance
            self.default_foot_positions.append(np.array([default_pos_xy[0], default_pos_xy[1], -self.body_height]))
        self.foot_positions = np.array(self.default_foot_positions)
