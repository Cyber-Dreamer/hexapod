"""
Hexapod Locomotion Module
=========================
This module implements the high-level locomotion logic for the hexapod robot.
It acts as a manager for different gait patterns, loading and running the
selected gait. The actual gait logic is implemented in separate classes
within the 'gaits' sub-package.
"""

import numpy as np
from hexapod_kinematics.kinematics import HexapodKinematics
from .tripod_gait import TripodGait
from .ripple_gait import RippleGait

class HexapodLocomotion:
    """
    The HexapodLocomotion class orchestrates the movement of the hexapod.
    It generates foot trajectories for a given gait and uses inverse kinematics
    to translate these into joint angles.
    """
    def __init__(self, node, step_height=0.05, step_length=0.1, knee_direction=-1, gait_type='stationary'):
        """
        Initializes the HexapodLocomotion object.

        :param node: The ROS2 node that this class is associated with.
        :param knee_direction: The desired direction of the knee (1 for up, -1 for down).
        :param gait_type: The type of gait to use ('tripod' or 'ripple').
        """
        self.node = node
        
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

        # --- Gait Management ---
        self.knee_direction = knee_direction
        self.available_gaits = {
            'tripod': TripodGait(node, self.kinematics, step_height, step_length, knee_direction),
            'ripple': RippleGait(node, self.kinematics, step_height, step_length, knee_direction)
        }
        self.set_gait(gait_type)

        self.recalculate_stance()

    def set_gait(self, gait_type):
        """Sets the active gait."""
        self.gait_type = gait_type
        self.current_gait = self.available_gaits.get(gait_type)
        if self.current_gait:
            self.current_gait.gait_phase = 0.0

    def update_knee_direction(self, new_direction):
        """Updates the knee direction for all gaits and recalculates the current pose."""
        self.knee_direction = new_direction
        for gait in self.available_gaits.values():
            gait.knee_direction = new_direction
        return self.set_body_pose([0, 0, 0], [0, 0, 0])

    def set_body_pose(self, translation, rotation):
        """
        Calculates joint angles to achieve a desired body pose.
        """
        self.recalculate_stance() 
        return self.kinematics.body_ik(translation, rotation, self.foot_positions, self.knee_direction)

    def run_gait(self, vx, vy, omega, pitch=0.0, speed=0.02): # Add default speed
        """
        Generates joint angles based on the selected gait type.
        :param pitch: Desired body pitch angle in radians.
        """
        if self.current_gait:
            self.recalculate_stance() # Ensure stance is up-to-date with params
            return self.current_gait.run(vx, vy, omega, pitch, speed, self.default_foot_positions)
        else:
            self.node.get_logger().warn(f"Gait '{self.gait_type}' not running or not found. Robot is stationary.")
            return [None] * 6

    def recalculate_stance(self):
        """
        Recalculates the default foot positions based on the current ROS parameters.
        """
        standoff_distance = self.node.get_parameter('standoff_distance').get_parameter_value().double_value
        body_height = self.node.get_parameter('body_height').get_parameter_value().double_value

        self.default_foot_positions = []
        for pos in self.kinematics.leg_positions:
            v = np.array([pos[0], pos[1], 0])
            if np.linalg.norm(v) > 0:
                v_norm = v / np.linalg.norm(v)
            else:
                v_norm = v
            default_pos_xy = v + v_norm * standoff_distance
            self.default_foot_positions.append(np.array([default_pos_xy[0], default_pos_xy[1], -body_height]))
        self.foot_positions = np.array(self.default_foot_positions)
