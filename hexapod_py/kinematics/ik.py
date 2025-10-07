import numpy as np

class HexapodKinematics:

    def __init__(self, leg_lengths, hip_positions):

        self.leg_lengths = leg_lengths
        self.hip_positions = hip_positions
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        # Pre-calculate the base angle for each hip from its position
        self.hip_base_angles = np.arctan2(np.array(self.hip_positions)[:, 1], np.array(self.hip_positions)[:, 0])
        
    import numpy as np

    @staticmethod
    def leg_ik(point, l_coxa, l_femur, l_tibia, knee_direction=1):
        """
        Calculates the inverse kinematics for a single hexapod leg.

        Args:
            point (np.ndarray): A 3-element numpy array (x, y, z) representing the
                                target coordinates for the foot tip relative to the
                                coxa joint's origin.
            l_coxa (float): The length of the coxa link.
            l_femur (float): The length of the femur link.
            l_tibia (float): The length of the tibia link.
            knee_direction (int): The desired knee bend direction. -1 for "down" (standard),
                                  1 for "up".

        Returns:
            tuple[float, float, float] | None: A tuple containing the calculated
            angles (gamma, alpha, beta) in radians. Returns None if the point
            is unreachable.
        """
        x, y, z = point
        
        # --- Define Joint Angle Limits in Radians ---
        # Coxa: ±90°, Femur: ±110°, Tibia: ±120°
        gamma_limit = np.deg2rad(90)
        alpha_limit = np.deg2rad(110)
        beta_limit = np.deg2rad(120)

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
        # Use a small tolerance for floating point issues at full extension
        if l_hyp > (l_femur + l_tibia) or l_hyp < abs(l_femur - l_tibia):
            return None

        # Angle of the hypotenuse 'l_hyp' from the horizontal plane
        phi = np.arctan2(z, x_prime)
        
        # Use the Law of Cosines to find the angle at the knee (femur-tibia joint)
        # and the angle at the femur joint.
        cos_beta_arg = (l_femur**2 + l_tibia**2 - l_hyp**2) / (2 * l_femur * l_tibia)
        # Clip to handle floating point inaccuracies at the boundaries
        beta_inner = np.arccos(np.clip(cos_beta_arg, -1.0, 1.0))
        # The angle inside the triangle at the tibia joint
        # The final tibia angle is measured from the extension of the femur.
        # For a standard "knee down" bend (knee_direction=-1), this angle is negative.
        # The formula is beta = beta_inner - pi.
        beta = (beta_inner - np.pi) * knee_direction

        cos_alpha_arg = (l_hyp**2 + l_femur**2 - l_tibia**2) / (2 * l_hyp * l_femur)
        # Clip to handle floating point inaccuracies at the boundaries
        alpha_inner = np.arccos(np.clip(cos_alpha_arg, -1.0, 1.0))
        # The angle inside the triangle at the femur joint
        # The final femur angle is the sum of the hypotenuse angle and this inner angle
        alpha = phi + (alpha_inner * knee_direction)

        # --- Check Joint Limits ---
        # Note: The beta angle from this calculation is always positive (0 to pi).
        # If your servo can bend both ways, you might need a different convention.
        if not (abs(gamma) <= gamma_limit and abs(alpha) <= alpha_limit and abs(beta) <= beta_limit):
            return None

        return (gamma, alpha, beta)

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
