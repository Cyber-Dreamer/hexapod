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
from rcl_interfaces.msg import ParameterDescriptor

class HexapodLocomotion:
    """
    The HexapodLocomotion class orchestrates the movement of the hexapod.
    It generates foot trajectories for a given gait and uses inverse kinematics
    to translate these into joint angles.
    """
    def __init__(self, node, step_height=0.04, step_length=0.08, knee_direction=-1, gait_type='tripod'):
        """
        Initializes the HexapodLocomotion object.

        :param node: The ROS2 node that this class is associated with.
        :param step_height: The maximum height of a foot during the swing phase.
        :param step_length: The length of a single step.
        :param knee_direction: The desired direction of the knee (1 for up, -1 for down).
        :param gait_type: The type of gait to use ('tripod' or 'ripple').
        """
        self.node = node
        
        # Declare parameters
        

        # --- Robot Dimensions ---
        # Symmetrical leg positions for a standard hexapod layout.
        # The order is: Front-Right, Middle-Right, Rear-Right, Front-Left, Middle-Left, Rear-Left
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

        # Tripod gait definition (group of legs that move together)
        # Tripod 1: FR, RR, ML
        # Tripod 2: FL, RL, MR
        self.tripod1 = [0, 2, 4]
        self.tripod2 = [3, 5, 1]
        
        # Ripple gait definition (legs move sequentially)
        self.ripple_sequence = [0, 1, 2, 3, 4, 5] # Example sequence

        self.swing_legs = []
        self.stance_legs = []
        self._initialize_gait_legs()

        # --- State Machine ---
        self.gait_phase = 0.0  # 0.0 to 1.0 for a full sub-cycle (lift or shift)
        self.state = "SHIFT"

        self.recalculate_stance()

    def _initialize_gait_legs(self):
        if self.gait_type == 'tripod':
            self.swing_legs = self.tripod1
            self.stance_legs = self.tripod2
        elif self.gait_type == 'ripple':
            # For ripple, initially all legs are stance, one will lift first
            self.swing_legs = []
            self.stance_legs = list(range(6))
        else:
            self.node.get_logger().warn(f"Unknown gait type: {self.gait_type}. Defaulting to tripod.")
            self.gait_type = 'tripod'
            self.swing_legs = self.tripod1
            self.stance_legs = self.tripod2

    def set_body_pose(self, translation, rotation):
        """
        Calculates joint angles to achieve a desired body pose (translation and rotation)
        while keeping the foot positions fixed on the ground.

        :param translation: Desired [x, y, z] translation of the body in the world frame.
        :param rotation: Desired [roll, pitch, yaw] rotation of the body.
        :return: A list of lists, where each inner list contains the joint angles
                 for a leg, or None if the pose is unreachable.
        """
        # Ensure foot_positions are up-to-date for the current stance
        self.recalculate_stance() 
        return self.kinematics.body_ik(translation, rotation, self.foot_positions, self.knee_direction)

    def _run_tripod_gait(self, vx, vy, omega, speed):
        """
        Generates joint angles for the tripod gait using a state machine.

        :param vx: Forward velocity command (e.g., from -1 to 1).
        :param vy: Sideways velocity command.
        :param omega: Angular velocity command.
        :param speed: The speed of the gait cycle.
        :return: A list of lists, containing the joint angles for each leg.
        """
        # Recalculate stance if parameters have changed
        self.recalculate_stance()

        self.gait_phase = (self.gait_phase + speed) % 1.0

        if self.state == "SHIFT":
            # --- Body Shifting Phase ---
            translation = np.array([-vx, -vy, 0]) * self.step_length * 0.5
            rotation = np.array([0, 0, -omega]) * self.step_length * 0.5

            current_translation = translation * self.gait_phase
            current_rotation = rotation * self.gait_phase

            joint_angles = self.kinematics.body_ik(current_translation, current_rotation, self.default_foot_positions, self.knee_direction)

            if self.gait_phase >= 0.99:
                self.gait_phase = 0.0
                self.state = "LIFT"
                for i in self.stance_legs:
                    self.foot_positions[i] = self.default_foot_positions[i] - translation

        elif self.state == "LIFT":
            # --- Leg Lifting Phase ---
            joint_angles = []
            body_height = self.node.get_parameter('body_height').get_parameter_value().double_value
            for i in range(6):
                if i in self.swing_legs:
                    z = self.step_height * (1 - (2 * self.gait_phase - 1)**2)
                    start_pos = self.foot_positions[i]
                    end_pos = self.default_foot_positions[i]
                    x = start_pos[0] + (end_pos[0] - start_pos[0]) * self.gait_phase
                    y = start_pos[1] + (end_pos[1] - start_pos[1]) * self.gait_phase
                    target_pos = np.array([x, y, -body_height + z])
                    v_foot_local = target_pos - self.kinematics.leg_positions[i]
                    angles = self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
                    joint_angles.append(angles)
                else: # Stance leg
                    target_pos = self.foot_positions[i]
                    v_foot_local = target_pos - self.kinematics.leg_positions[i]
                    angles = self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
                    joint_angles.append(angles)

            if self.gait_phase >= 0.99:
                self.gait_phase = 0.0
                self.state = "SHIFT"
                self.swing_legs, self.stance_legs = self.stance_legs, self.swing_legs

        return joint_angles

    def _run_ripple_gait(self, vx, vy, omega, speed):
        """
        Generates joint angles for a ripple gait.
        Moves one leg at a time in a sequence.

        :param vx: Forward velocity command.
        :param vy: Sideways velocity command.
        :param omega: Angular velocity command.
        :param speed: The speed of the gait cycle.
        :return: A list of lists, containing the joint angles for each leg.
        """
        self.recalculate_stance()

        joint_angles = [None] * 6
        body_height = self.node.get_parameter('body_height').get_parameter_value().double_value

        # Determine which leg is currently swinging
        # The gait phase will cycle through 0 to 1 for each leg in the ripple sequence
        leg_cycle_duration = 1.0 / len(self.ripple_sequence)
        current_leg_index_in_sequence = int(self.gait_phase / leg_cycle_duration)
        current_leg_id = self.ripple_sequence[current_leg_index_in_sequence]

        # Calculate phase for the current leg's movement (0 to 1)
        leg_phase = (self.gait_phase % leg_cycle_duration) / leg_cycle_duration

        # All legs are stance except the current swinging leg
        self.swing_legs = [current_leg_id]
        self.stance_legs = [i for i in range(6) if i != current_leg_id]

        # --- Body Shift (continuous for stance legs) ---
        # The body continuously shifts based on velocity commands
        # This is a simplified continuous shift, more advanced gaits might have discrete shifts
        translation_body_shift = np.array([-vx, -vy, 0]) * self.step_length * 0.5
        rotation_body_shift = np.array([0, 0, -omega]) * self.step_length * 0.5

        # Apply body shift to all stance legs
        for i in self.stance_legs:
            # For stance legs, their foot position in the world frame is fixed.
            # The body moves, so the relative position of the foot to the body changes.
            # We need to calculate the new foot position relative to the body's new pose.
            # This is implicitly handled by body_ik if we pass the current foot_positions
            # and the body's desired translation/rotation.
            # For now, just calculate IK for stance legs based on their current foot_positions.
            target_pos = self.foot_positions[i] # Foot position in world frame
            v_foot_local = target_pos - self.kinematics.leg_positions[i]
            angles = self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
            joint_angles[i] = angles

        # --- Leg Lifting and Swinging Phase (for current_leg_id) ---
        if current_leg_id in self.swing_legs:
            # Lift phase (first half of leg_phase)
            if leg_phase < 0.5:
                z = self.step_height * (leg_phase * 2) # Linear lift up
            # Lower phase (second half of leg_phase)
            else:
                z = self.step_height * (1 - (leg_phase - 0.5) * 2) # Linear lower down

            # Swing phase (move foot forward/sideways)
            # The foot moves from its current position to a new target position
            # For simplicity, let's assume it moves from its current foot_position
            # to a point slightly ahead/sideways based on vx, vy, omega
            # This is a very basic swing trajectory.
            start_pos = self.foot_positions[current_leg_id]
            # Target position for the foot after the swing
            # This should be relative to the body's movement
            end_pos = self.default_foot_positions[current_leg_id] + np.array([vx, vy, 0]) * self.step_length

            x = start_pos[0] + (end_pos[0] - start_pos[0]) * leg_phase
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * leg_phase
            target_pos_swing = np.array([x, y, -body_height + z])

            v_foot_local = target_pos_swing - self.kinematics.leg_positions[current_leg_id]
            angles = self.kinematics.inverse_kinematics(v_foot_local, self.knee_direction)
            joint_angles[current_leg_id] = angles

            # Update foot position after the leg completes its swing
            if leg_phase >= 0.99:
                self.foot_positions[current_leg_id] = end_pos # Update foot position in world frame

        # Advance gait phase for the next iteration
        self.gait_phase = (self.gait_phase + speed) % 1.0

        return joint_angles

    def recalculate_stance(self):
        """
        Recalculates the default foot positions based on the current ROS parameters.
        """
        standoff_distance = self.node.get_parameter('standoff_distance').get_parameter_value().double_value
        body_height = self.node.get_parameter('body_height').get_parameter_value().double_value

        # --- Foot Positions ---
        # Calculate a stable default stance position for the feet
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

    def run_gait(self, vx, vy, omega, speed=0.02):
        """
        Generates joint angles based on the selected gait type.

        :param vx: Forward velocity command (e.g., from -1 to 1).
        :param vy: Sideways velocity command.
        :param omega: Angular velocity command.
        :param speed: The speed of the gait cycle.
        :return: A list of lists, containing the joint angles for each leg.
        """
        if self.gait_type == 'tripod':
            return self._run_tripod_gait(vx, vy, omega, speed)
        elif self.gait_type == 'ripple':
            return self._run_ripple_gait(vx, vy, omega, speed)
        else:
            self.node.get_logger().error(f"Unsupported gait type: {self.gait_type}")
            return None
