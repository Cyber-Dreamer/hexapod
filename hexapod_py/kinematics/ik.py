import numpy as np

class HexapodKinematics:

    def __init__(self, leg_lengths, hip_positions):

        self.leg_lengths = leg_lengths
        self.hip_positions = hip_positions
        self.optimal_stance = np.deg2rad([0.0, 0.0, -120.0])
        
    def inverse_kinematics(self, P_target, flip_factor=-1):
        """
        Calculates optimized 3-DOF hexapod leg IK, prioritizing a posture close to theta_home.
        """
        L1, L2, L3 = self.leg_lengths
        x, y, z = P_target
        
        # 1. Solve for Hip/Coxa Joint (theta1) and planar distance
        r_hip = np.sqrt(x**2 + y**2)
        if r_hip < L1: return None # Unreachable: target inside Coxa length
        theta1 = np.arctan2(y, x)
        x_prime = r_hip - L1
        z_prime = z
        
        # 2. Planar IK Setup
        D_sq = x_prime**2 + z_prime**2
        D = np.sqrt(D_sq)
        if D > L2 + L3 or D < abs(L2 - L3): return None # Unreachable: target too far or too close
        
        # 3. Calculate internal angles (Law of Cosines)
        cos_B = np.clip((L2**2 + L3**2 - D_sq) / (2 * L2 * L3), -1, 1)
        beta_ik = np.arccos(cos_B)
        cos_A = np.clip((D_sq + L2**2 - L3**2) / (2 * D * L2), -1, 1)
        alpha_ik = np.arccos(cos_A)
        gamma = np.arctan2(z_prime, x_prime)
        
        # 4. Generate two solutions (Knee-Down and Knee-Up)
        solutions = []
        solutions.append((theta1, gamma - alpha_ik, -(np.pi - beta_ik))) # Solution 1: Knee-Down
        solutions.append((theta1, gamma + alpha_ik, (np.pi - beta_ik)))  # Solution 2: Knee-Up
        
        # 5. Optimize with cost function (selecting the solution closest to theta_home)
        best_cost = float('inf')
        t1_h, t2_h, t3_h = self.optimal_stance
        w2, w3 = 1.0, 2.0 # Weights for Femur (t2) and Tibia (t3)
        
        # Apply flip_factor to theta_home to prioritize the correct posture when robot is upside down
        t2_h_effective = t2_h * flip_factor
        t3_h_effective = t3_h * flip_factor
        
        for t1, t2, t3 in solutions:
            d2 = (t2 - t2_h_effective + np.pi) % (2 * np.pi) - np.pi
            d3 = (t3 - t3_h_effective + np.pi) % (2 * np.pi) - np.pi
            cost = w2 * (d2**2) + w3 * (d3**2)
            if cost < best_cost:
                best_cost = cost
                best_solution = (t1, t2, t3)
                
        return best_solution

    def body_ik(self, translation, rotation, foot_positions, knee_direction=1):
        """
        Calculates the inverse kinematics for the entire body.

        This function computes the required joint angles for all legs to achieve
        a desired body posture, assuming the feet are planted on the ground.

        :param translation: Desired [x, y, z] translation of the body in the world frame.
        :param rotation: Desired [roll, pitch, yaw] rotation of the body.
        :param foot_positions: A list of the current (x, y, z) positions of the feet
                               in the world frame.
        :param knee_direction: The desired direction of the knee (1 for up, -1 for down).
        :return: A list of lists, where each inner list contains the joint angles
                 for a leg.
        """
        joint_angles_list = []
        roll, pitch, yaw = rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        T = np.array(translation)
        for i in range(6):
            p_coxa_body = np.array(self.hip_positions[i])
            p_coxa_world = R @ p_coxa_body + T
            v_foot_world = np.array(foot_positions[i]) - p_coxa_world
            v_foot_local = R.T @ v_foot_world
            joint_angles = self.inverse_kinematics(v_foot_local, knee_direction)
            joint_angles_list.append(joint_angles)
        return joint_angles_list

    def forward_kinematics(self, leg_index, joint_angles):
        """
        Computes the foot position for a single leg using forward kinematics.
        Uses a standard Z-up, CCW angle convention.
        :param leg_index: The index of the leg (0-5).
        :param joint_angles: [coxa, femur, tibia] angles in radians.
        :return: The (x, y, z) position of the foot in the body frame.
        """
        coxa_len, femur_len, tibia_len = self.leg_lengths
        theta1, theta2, theta3 = joint_angles
        coxa_pos = self.hip_positions[leg_index]
        l_horiz = femur_len * np.cos(theta2) + tibia_len * np.cos(theta2 + theta3)
        l_total = coxa_len + l_horiz
        x_local = l_total * np.cos(theta1)
        y_local = l_total * np.sin(theta1)
        z_local = femur_len * np.sin(theta2) + tibia_len * np.sin(theta2 + theta3)
        foot_pos_body = np.array(coxa_pos) + np.array([x_local, y_local, z_local])
        return foot_pos_body.tolist()
