import numpy as np

class HexapodKinematics:

    def __init__(self, leg_lengths, hip_positions):

        self.leg_lengths = leg_lengths
        self.hip_positions = hip_positions
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        
    import numpy as np

    def leg_ik(point, l_coxa, l_femur, l_tibia):
        """
        Calculates the inverse kinematics for a single hexapod leg.

        Args:
            point (np.ndarray): A 3-element numpy array (x, y, z) representing the
                                target coordinates for the foot tip relative to the
                                coxa joint's origin.
            l_coxa (float): The length of the coxa link.
            l_femur (float): The length of the femur link.
            l_tibia (float): The length of the tibia link.

        Returns:
            tuple[float, float, float] | None: A tuple containing the calculated
            angles (gamma, alpha, beta) in radians. Returns None if the point
            is unreachable.
        """
        x, y, z = point
        
        # --- Calculate Coxa Angle (gamma) ---
        # Top-down view
        gamma = np.arctan2(y, x)

        # --- Calculate Femur and Tibia Angles (alpha, beta) ---
        # Side view of the leg in its 2D plane
        
        # Horizontal distance from coxa joint to foot tip
        l_horizontal = np.sqrt(x**2 + y**2)
        
        # Horizontal distance from femur joint to foot tip
        x_prime = l_horizontal - l_coxa
        
        # Direct distance from femur joint to foot tip
        l_hyp = np.sqrt(x_prime**2 + z**2)

        # Check if the target is reachable
        if l_hyp >= (l_femur + l_tibia) or l_hyp <= abs(l_femur - l_tibia):
            # print("Warning: Target point is unreachable.")
            return None

        # Angle of the hypotenuse 'l_hyp' from the horizontal plane
        phi = np.arctan2(z, x_prime)
        
        # Use the Law of Cosines to find the angle at the knee (femur-tibia joint)
        # and the angle at the femur joint.
        cos_beta_arg = (l_femur**2 + l_tibia**2 - l_hyp**2) / (2 * l_femur * l_tibia)
        # The angle inside the triangle at the tibia joint
        beta_inner = np.arccos(np.clip(cos_beta_arg, -1.0, 1.0))
        # The final tibia angle is typically measured from the extension of the femur
        beta = np.pi - beta_inner

        cos_alpha_arg = (l_hyp**2 + l_femur**2 - l_tibia**2) / (2 * l_hyp * l_femur)
        # The angle inside the triangle at the femur joint
        alpha_inner = np.arccos(np.clip(cos_alpha_arg, -1.0, 1.0))
        # The final femur angle is the sum of the hypotenuse angle and this inner angle
        alpha = phi + alpha_inner

        return (gamma, alpha, beta)

    def body_ik(translation, rotation, coxa_positions, default_foot_positions):
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
                                                default (x, y, z) position of a
                                                foot tip relative to its coxa joint.

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
            # Calculate the total vector from the body center to the foot tip in the default state
            total_vector = coxa_positions[i] + default_foot_positions[i]
            
            # Apply the inverse transformation to find the new foot position relative to the transformed body
            # Equation: P_new = R.T * (P_old - T)
            # where P_new is the new position relative to the new body frame,
            # P_old is the old position relative to the old body frame,
            # R is the rotation matrix, and T is the translation vector.
            
            new_total_vector = R.T @ (total_vector - translation)
            
            # The new foot position relative to the coxa is the difference
            new_foot_positions[i] = new_total_vector - coxa_positions[i]
            
        return new_foot_positions
