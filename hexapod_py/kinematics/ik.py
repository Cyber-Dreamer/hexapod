import numpy as np
from typing import List, Optional

class HexapodKinematics:
    """
    Inverse Kinematics logic for the hexapod
    """

    def __init__(self, segment_lengths, hip_positions, joint_limits):
        self.segment_lengths = np.array(segment_lengths)
        self.hip_positions = np.array(hip_positions)
        self.joint_limits = np.array(joint_limits)
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        self.hip_base_angles = np.arctan2(np.array(self.hip_positions)[:, 1], np.array(self.hip_positions)[:, 0])


    def leg_ik(self, coordinate: np.ndarray, knee_direction: int = -1) -> Optional[List[float]]:
        """
        Calculates the joint angles for a leg to reach a target coordinate. Coordinates are based on the hip position.
        """
        x, y, z = coordinate
        l_coxa, l_femur, l_tibia = self.segment_lengths

        # Coxa (hip) angle. Use abs(x) to prevent flipping when reaching under the body.
        coxa_angle = np.arctan2(y, abs(x))

        # Convert to a 2D problem for the femur and tibia.
        # l_horizontal is the distance from the coxa pivot to the foot target in the XY plane.
        l_horizontal = np.sqrt(x**2 + y**2)
        femur_to_foot_x = l_horizontal - l_coxa
        femur_to_foot_z = z
        
        dist_femur_to_foot = np.sqrt(femur_to_foot_x**2 + femur_to_foot_z**2)

        # Reachability check and clamping.
        epsilon = 1e-6
        max_reach = l_femur + l_tibia
        min_reach = abs(l_femur - l_tibia)

        if not (min_reach - epsilon <= dist_femur_to_foot <= max_reach + epsilon):
            # If unreachable, clamp the distance to the nearest boundary.
            clamped_dist = np.clip(dist_femur_to_foot, min_reach, max_reach)
            
            # Scaling the vector from the femur joint to the foot to find a reachable coordinate in the boundary.
            scale = clamped_dist / dist_femur_to_foot
            
            # Update the local variables for the IK calculation
            femur_to_foot_x *= scale
            femur_to_foot_z *= scale
            dist_femur_to_foot = clamped_dist

        # Use the Law of Cosines to find internal angles.
        
        ## Angle at the knee joint, between the femur and tibia.
        cos_arg_knee = (l_femur**2 + l_tibia**2 - dist_femur_to_foot**2) / (2 * l_femur * l_tibia)
        knee_angle = np.arccos(np.clip(cos_arg_knee, -1.0, 1.0))
        
        ## Angle at the femur joint, between the femur and the line to the foot tip.
        cos_arg_femur = (dist_femur_to_foot**2 + l_femur**2 - l_tibia**2) / (2 * dist_femur_to_foot * l_femur)
        femur_angle_offset = np.arccos(np.clip(cos_arg_femur, -1.0, 1.0))

        phi = np.arctan2(femur_to_foot_z, femur_to_foot_x)

        # Set final angles using the knee_direction parameter
        femur_angle = phi - (knee_direction * femur_angle_offset)
        tibia_angle = knee_direction * (np.pi - knee_angle)

        # Joint Limit Check
        angles = np.array([coxa_angle, femur_angle, tibia_angle])

        # Ensure a valid output by clamping to joint limits.
        clamped_angles = np.clip(angles, -self.joint_limits, self.joint_limits)

        return clamped_angles.tolist()


    def body_ik(self, translation, rotation, coxa_positions, default_foot_positions):
        """
        Calculates the the joint angles in order to have the body at a certain translation and rotation.
        """
        roll, pitch, yaw = rotation
        
        # Creating body rotation matrix from the roll, pitch, and yaw angles.
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        new_foot_positions = np.zeros_like(default_foot_positions)

        # Transforming foot positions to the new body frame
        for i in range(6):
            hip_pos_new = R @ coxa_positions[i] + translation
            foot_vector_world = default_foot_positions[i] - hip_pos_new
            foot_vector_body = R.T @ foot_vector_world
            hip_base_angle = self.hip_base_angles[i]

            # Inverse rotation to leg frame
            R_hip_inv = np.array([
                [ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                [0, 0, 1]
            ])
            
            new_foot_positions[i] = R_hip_inv @ foot_vector_body
        return new_foot_positions