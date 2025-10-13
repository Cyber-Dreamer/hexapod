import numpy as np
from typing import List, Optional

class HexapodKinematics:

    def __init__(self, segment_lengths, hip_positions, joint_limits):

        self.segment_lengths = segment_lengths
        self.hip_positions = hip_positions
        self.joint_limits = joint_limits
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        
        # Pre-calculate the base angle for each hip from its position
        self.hip_base_angles = np.arctan2(np.array(self.hip_positions)[:, 1], np.array(self.hip_positions)[:, 0])
        
    def leg_ik(self, coordinate: np.ndarray) -> Optional[List[float]]:
        """
        Calculates the inverse kinematics for a single hexapod leg.
        The coordinate should be in the leg's local reference frame, where
        the coxa joint is at the origin (0,0,0).

        Args:
            coordinate (np.ndarray): A 3-element numpy array (x, y, z) for the
                                     target foot tip position.

        Returns:
            Optional[List[float]]: A list of 3 joint angles [gamma, alpha, beta]
                                   in radians, or None if the position is unreachable.
        """
        x, y, z = coordinate

        # Calculating Coxa Angle (gamma)
        # HOW: Angle from a top down view perspective
        gamma = np.arctan2(y, x)

        l_coxa, l_femur, l_tibia = self.segment_lengths
        
        # --- Femur/Tibia Angle Calculation (alpha, beta) ---
        # To simplify the 2D math, we can rotate the coordinate system by gamma.
        # This makes the leg's horizontal component align with the X-axis,
        # removing the complexity of handling negative x/y values.
        x_rotated = np.sqrt(x**2 + y**2) - l_coxa
        y_rotated = 0 # By definition after rotation
        z_rotated = z
        
        # Direct distance from the femur joint to the foot tip in the rotated 2D plane
        femur_to_foot_dist = np.sqrt(x_rotated**2 + z_rotated**2)

        # --- Handle Special Case: Fully Extended Leg ---
        # This avoids numerical instability in the Law of Cosines calculation when the
        # leg is nearly straight.
        if np.isclose(femur_to_foot_dist, l_femur + l_tibia, atol=1e-6):
            # In the rotated frame, the horizontal distance is simply x_rotated
            alpha = np.arctan2(z_rotated, x_rotated)
            beta = 0.0
            return [gamma, alpha, beta]

        # Check if the target is reachable
        # Use a small tolerance for floating point issues at full extension
        if femur_to_foot_dist > (l_femur + l_tibia) or femur_to_foot_dist < abs(l_femur - l_tibia):
            return None

        # --- Determine Knee Bend Direction ---
        # The knee should bend away from the body's horizontal plane.
        # If the target is below the plane (z < 0, normal walking), the knee bends down (standard).
        # If the target is above the plane (z > 0, upside-down walking), the knee bends up.
        knee_direction = -1 if z_rotated > 0 else 1

        # Angle of the hypotenuse from the horizontal plane
        phi = np.arctan2(z_rotated, x_rotated)
        
        # Use the Law of Cosines to find the angle at the knee (femur-tibia joint)
        # and the angle at the femur joint.
        cos_beta_arg = (l_femur**2 + l_tibia**2 - femur_to_foot_dist**2) / (2 * l_femur * l_tibia)
        
        # Clip to handle floating point inaccuracies at the boundaries
        beta_inner = np.arccos(np.clip(cos_beta_arg, -1.0, 1.0))
        # The angle inside the triangle at the tibia joint
        # The final tibia angle is measured from the extension of the femur.
        # For a standard "knee down" bend (knee_direction=1), this angle is negative.
        beta = -(np.pi - beta_inner) * knee_direction

        cos_alpha_arg = (femur_to_foot_dist**2 + l_femur**2 - l_tibia**2) / (2 * femur_to_foot_dist * l_femur)
        
        # Clip to handle floating point inaccuracies at the boundaries
        alpha_inner = np.arccos(np.clip(cos_alpha_arg, -1.0, 1.0))
        
        # The angle inside the triangle at the femur joint
        # The final femur angle is the sum of the hypotenuse angle and this inner angle
        # The knee_direction determines if we add or subtract the inner angle from phi.
        alpha = phi + (alpha_inner * knee_direction)

        # --- Check Joint Limits ---
        # Note: The beta angle from this calculation is always positive (0 to pi).
        # If your servo can bend both ways, you might need a different convention.
        if not (abs(gamma) <= self.joint_limits[0] and abs(alpha) <= self.joint_limits[1] and abs(beta) <= self.joint_limits[2]):
            return None

        return [gamma, alpha, beta]

    def body_ik(self, translation, rotation, coxa_positions, default_foot_positions):
        """
        Calculates the new foot tip coordinates for all six legs to achieve
        a desired body position and orientation.

        Args:
            translation (np.ndarray): A 3-element numpy array (tx, ty, tz) for
                                    body translation.
            rotation (np.ndarray): A 3-element numpy array (roll, pitch, yaw) for
                                body rotation in radians.
            coxa_positions (np.ndarray): A 6x3 array where each row is the (x, y, z)
                                        position of a coxa joint relative to the
                                        body's center.
            default_foot_positions (np.ndarray): A 6x3 array where each row is the
                                                default (x, y, z) world position of a
                                                foot tip when the body is at origin.

        Returns:
            np.ndarray: A 6x3 array containing the new target coordinates (x, y, z)
                        for each foot tip, relative to their respective coxa joints.
        """
        # Create rotation matrices
        roll, pitch, yaw = rotation
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
        
        # Combined rotation matrix (Yaw -> Pitch -> Roll)
        R = Rz @ Ry @ Rx
        
        new_foot_positions = np.zeros_like(default_foot_positions)

        for i in range(6):
            # The goal is to find the new foot position vector relative to the *new* hip position.
            # 1. Get the default foot position in the world frame.
            foot_pos_world = default_foot_positions[i]

            # 2. Calculate the new position of the hip joint in the world frame after body transformation.
            hip_pos_new = R @ coxa_positions[i] + translation

            # 3. Calculate the vector from the new hip position to the static world foot position.
            # This vector is in the world frame.
            foot_vector_world = foot_pos_world - hip_pos_new
            
            # 4. Rotate this world-frame vector back into the body's coordinate frame.
            foot_vector_body = R.T @ foot_vector_world

            # 5. Rotate the body-frame vector into the leg's local coordinate frame.
            # This is the final, crucial step.
            hip_base_angle = self.hip_base_angles[i]
            R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                                  [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                                  [0,                       0,                      1]])
            new_foot_positions[i] = R_hip_inv @ foot_vector_body
            
        return new_foot_positions
