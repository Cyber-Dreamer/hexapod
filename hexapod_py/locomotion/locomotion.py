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
    def __init__(self, step_height=40, max_step_length=180, gait_type='tripod', body_height=200, standoff_distance=200, knee_direction=-1, gait_speed_factor=0.03, rotation_scale_factor=0.7, max_linear_velocity=300, max_angular_velocity=np.pi/2):
        
        # Measure of the joint in mm
        center_to_HipJoint = 152.024
        hipJoint_to_femurJoint = 92.5
        femurJoint_to_tibiaJoint = 191.8
        tibiaJoint_to_tipFoot = 284.969
        
        # Standard leg numbering:
        # New Clockwise Numbering:
        # 0: Rear-Left, 1: Middle-Left, 2: Front-Left
        # 3: Front-Right, 4: Middle-Right, 5: Rear-Right
        # Angles are measured counter-clockwise from the positive X-axis. The
        # order of angles must match the leg numbering scheme.
        self.hip_angles = np.deg2rad([210, 270, 330, 30, 90, 150])

        hip_positions = [
            tuple(coord) 
            for coord in np.column_stack(
                (center_to_HipJoint * np.cos(self.hip_angles),
                 center_to_HipJoint * np.sin(self.hip_angles),
                 np.zeros_like(self.hip_angles))).tolist()
        ]
        
        leg_lengths = [
            hipJoint_to_femurJoint,
            femurJoint_to_tibiaJoint,
            tibiaJoint_to_tipFoot
        ]
        


        self.kinematics = HexapodKinematics(segment_lengths=leg_lengths, hip_positions=hip_positions, joint_limits=np.deg2rad([90, 110, 120]))
        self.knee_direction = knee_direction
        self.available_gaits = {
            # max_step_length is now a fallback/limit, not the primary driver of step size
            'tripod': TripodGait(self.kinematics, step_height, max_step_length, self.knee_direction),
            'ripple': RippleGait(self.kinematics, step_height, max_step_length, self.knee_direction)
        }
        self.set_gait(gait_type)
        self.body_height = body_height
        self.step_height = step_height
        self.gait_speed_factor = gait_speed_factor
        self.rotation_scale_factor = rotation_scale_factor
        self.max_linear_velocity = max_linear_velocity  # mm/s
        self.max_angular_velocity = max_angular_velocity # rad/s

        if standoff_distance is None:
            # To achieve a 90-degree angle at the knee, the horizontal distance
            # from the femur joint to the foot tip must be equal to the femur length.
            # The total horizontal distance (standoff) is this plus the coxa length.
            standoff_distance = self.kinematics.segment_lengths[0] + self.kinematics.segment_lengths[1]
        self.standoff_distance = standoff_distance
        self.recalculate_stance()

    def set_gait(self, gait_type):
        self.gait_type = gait_type
        self.current_gait = self.available_gaits.get(gait_type)
        if self.current_gait:
            self.current_gait.gait_phase = 0.0

    def set_body_pose(self, translation, rotation):
        """
        Calculates the leg joint angles required to achieve a given body pose.

        This is a wrapper around the body_ik function that uses the robot's
        default standing positions as the reference.
        """
        # 1. Calculate the target position for each foot in its local leg coordinate system.
        #    We pass self.default_foot_positions, which are the fixed world
        #    coordinates for the feet to stay planted on.
        new_foot_targets_local = self.kinematics.body_ik(translation, rotation, self.kinematics.hip_positions, self.default_foot_positions)

        # 2. For each leg, calculate the joint angles to reach its new target position.
        all_angles = []
        for i in range(6):
            angles = self.kinematics.leg_ik(new_foot_targets_local[i], knee_direction=self.knee_direction)
            all_angles.append(angles if angles is not None else self.default_joint_angles[i])

        return all_angles
    
    def run_gait(self, vx, vy, omega, roll=0.0, pitch=0.0, step_height=None):
        if self.current_gait:
            # Update gait parameters if new values are provided from the UI
            if step_height is not None:
                self.current_gait.step_height = step_height
    
            # Calculate the magnitude of the movement command
            command_magnitude = max(np.linalg.norm([vx, vy]), abs(omega))
            
            # Dynamically calculate gait cycle speed. If there's no movement, the gait pauses.
            speed = command_magnitude * self.gait_speed_factor

            # Calculate the duration of one full gait cycle (T_cycle) in seconds.
            # This assumes the simulation runs at a rate that makes `speed` meaningful.
            # A smaller gait_speed_factor means a longer cycle.
            # We assume a nominal simulation rate (e.g., 240Hz) and that `speed` advances the phase each step.
            # T_cycle = (1 / (speed * sim_rate)) if speed > 0 else float('inf')
            # For simplicity, we'll tie step length directly to velocity input, not cycle time.
            
            # Convert normalized inputs (-1 to 1) to physical velocities
            target_vx = vx * self.max_linear_velocity
            target_vy = vy * self.max_linear_velocity
            target_omega = omega * self.max_angular_velocity

            # Note: recalculate_stance() is intentionally not called here for performance.
            # It's only called when standoff or body_height are changed.
            return self.current_gait.run(target_vx, target_vy, target_omega, roll, pitch, speed, self.default_foot_positions, self.default_joint_angles, self.body_height, self.current_gait.step_height, self.rotation_scale_factor)
        else:
            return [None] * 6

    def recalculate_stance(self, standoff_distance=None):
        if standoff_distance is None:
            standoff_distance = self.standoff_distance
        self.standoff_distance = standoff_distance
        self.default_foot_positions = []
        for pos in self.kinematics.hip_positions:
            v = np.array([pos[0], pos[1], 0])
            if np.linalg.norm(v) > 0:
                v_norm = v / np.linalg.norm(v)
            else:
                v_norm = v
            default_pos_xy = v + v_norm * standoff_distance
            self.default_foot_positions.append(np.array([default_pos_xy[0], default_pos_xy[1], -self.body_height]))
        self.foot_positions = np.array(self.default_foot_positions)

        # Pre-calculate and cache the joint angles for the default neutral stance.
        # This is much more efficient than recalculating them on-the-fly during gait.
        self.default_joint_angles = []
        new_foot_targets_local = self.kinematics.body_ik(np.zeros(3), np.zeros(3), self.kinematics.hip_positions, self.foot_positions)
        for i in range(6):
            angles = self.kinematics.leg_ik(new_foot_targets_local[i], knee_direction=self.knee_direction)
            if angles is None:
                angles = np.zeros(3) # Should not happen with a valid stance
            self.default_joint_angles.append(angles)

    def calculate_sit_angles(self):
        """
        Calculates the joint angles for a "sitting" position.
        Legs are tucked underneath the body. This is a good intermediate
        pose before standing up.
        """
        sit_angles = []
        # Define a target position in the local frame for each leg
        # This is forward of the hip and significantly raised up
        l_coxa, l_femur, l_tibia = self.kinematics.segment_lengths
        target_pos_local = np.array([l_coxa, 0, -l_femur])

        for i in range(6):
            angles = self.kinematics.leg_ik(target_pos_local, knee_direction=self.knee_direction)
            sit_angles.append(angles if angles is not None else [0, 0, 0])

        return sit_angles
