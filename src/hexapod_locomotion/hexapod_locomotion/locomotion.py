"""
Hexapod Locomotion Module
=========================

This module implements the high-level locomotion logic for the hexapod robot.
It uses the HexapodKinematics class to generate coordinated movements for walking.

The primary implementation is a tripod gait, where the robot keeps three feet
on the ground at all times, providing a stable stance. The gait is implemented
as a state machine that alternates between shifting the body and lifting the legs.
"""

import numpy as np
from hexapod_kinematics.kinematics import HexapodKinematics

class HexapodLocomotion:
    """
    The HexapodLocomotion class orchestrates the movement of the hexapod.
    It generates foot trajectories for a given gait and uses inverse kinematics
    to translate these into joint angles.
    """
    def __init__(self, node, step_height=0.05, step_length=0.1, knee_direction=-1, gait_type='tripod'):
        """
        Initializes the HexapodLocomotion object.

        :param node: The ROS2 node that this class is associated with.
        :param step_height: The maximum height of a foot during the swing phase.
        :param step_length: The length of a single step.
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

        # --- Gait Parameters ---
        self.step_height = step_height
        self.step_length = step_length
        self.knee_direction = knee_direction
        self.gait_type = gait_type

        # Gait definitions
        self.tripod_legs_1 = [0, 2, 4] # FR, RR, ML
        self.tripod_legs_2 = [1, 3, 5] # MR, FL, RL
        self.ripple_sequence = [0, 3, 1, 4, 2, 5] # FR, FL, MR, ML, RR, RL

        self.gait_phase = 0.0  # Overall gait phase (0 to 1)
        self.leg_phases = np.zeros(6)

        self.recalculate_stance()

    def set_body_pose(self, translation, rotation):
        """
        Calculates joint angles to achieve a desired body pose.
        """
        self.recalculate_stance() 
        return self.kinematics.body_ik(translation, rotation, self.foot_positions, self.knee_direction)

    def run_gait(self, vx, vy, omega, speed=0.02):
        """
        Generates joint angles based on the selected gait type.
        """
        self.gait_phase = (self.gait_phase + speed) % 1.0

        if self.gait_type == 'tripod':
            return self._run_tripod_gait(vx, vy, omega)
        elif self.gait_type == 'ripple':
            return self._run_ripple_gait(vx, vy, omega)
        else:
            self.node.get_logger().error(f"Unsupported gait type: {self.gait_type}")
            return [None] * 6

    def _run_tripod_gait(self, vx, vy, omega):
        """
        Tripod Gait: 3 legs move at a time.
        """
        joint_angles = [None] * 6
        
        # Phase for each tripod group (0 to 1)
        phase_1 = self.gait_phase
        phase_2 = (self.gait_phase + 0.5) % 1.0

        # Move Tripod 1
        for leg_idx in self.tripod_legs_1:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_1, vx, vy, omega)

        # Move Tripod 2
        for leg_idx in self.tripod_legs_2:
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase_2, vx, vy, omega)
            
        return joint_angles

    def _run_ripple_gait(self, vx, vy, omega):
        """
        Ripple Gait: 3 legs on the ground at a time, moving in a sequence.
        """
        joint_angles = [None] * 6
        num_legs = len(self.ripple_sequence)
        
        for i, leg_idx in enumerate(self.ripple_sequence):
            # Each leg is offset in phase
            phase = (self.gait_phase + (i / num_legs)) % 1.0
            joint_angles[leg_idx] = self._calculate_leg_ik(leg_idx, phase, vx, vy, omega)
            
        return joint_angles

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega):
        """
        Calculates the IK for a single leg based on its phase in the gait cycle.
        """
        body_height = self.node.get_parameter('body_height').get_parameter_value().double_value

        # Swing phase (leg is in the air)
        if phase < 0.5:
            swing_phase = phase * 2
            z = self.step_height * (1 - (2 * swing_phase - 1)**2)
            
            # Move foot from back to front
            x_swing = -self.step_length / 2 * (1 - swing_phase)
            y_swing = 0 # Simplified, no sideways swing for now

            target_pos = self.default_foot_positions[leg_idx] + np.array([x_swing, y_swing, 0])
            target_pos[2] = -body_height + z

        # Stance phase (leg is on the ground)
        else:
            stance_phase = (phase - 0.5) * 2
            
            # Move foot from front to back
            x_stance = self.step_length / 2 * (1 - stance_phase)
            y_stance = 0

            # Incorporate velocity commands into stance movement
            x_stance -= vx * self.step_length * stance_phase
            y_stance -= vy * self.step_length * stance_phase
            
            # Incorporate rotation command (omega)
            # This requires rotating the foot position around the body center
            if abs(omega) > 0.01:
                rot_z = -omega * self.step_length * stance_phase
                start_pos = self.default_foot_positions[leg_idx]
                c, s = np.cos(rot_z), np.sin(rot_z)
                rot_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                rotated_pos = rot_matrix @ start_pos
                x_stance += rotated_pos[0] - start_pos[0]
                y_stance += rotated_pos[1] - start_pos[1]

            target_pos = self.default_foot_positions[leg_idx] + np.array([x_stance, y_stance, 0])
            target_pos[2] = -body_height

        v_foot_local = target_pos - self.kinematics.leg_positions[leg_idx]
        return self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)

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

    def update_knee_direction(self, new_direction):
        """
        Updates the knee direction and recalculates the current pose.
        """
        self.knee_direction = new_direction
        # Recalculate the current stance with the new knee direction
        return self.set_body_pose([0,0,0], [0,0,0])
