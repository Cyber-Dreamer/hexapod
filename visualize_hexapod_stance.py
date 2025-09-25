import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- HexapodKinematics Class (copied from hexapod_kinematics/kinematics.py) ---
class HexapodKinematics:
    def __init__(self, leg_lengths, leg_positions, coxa_initial_rpys=None):
        self.leg_lengths = leg_lengths
        self.leg_positions = leg_positions
        self.coxa_initial_rpys = coxa_initial_rpys if coxa_initial_rpys is not None else [[0.0, 0.0, 0.0]] * len(leg_positions)

    def inverse_kinematics(self, foot_pos_local, knee_direction=1):
        # This method is not strictly needed for just visualizing the initial stance,
        # but included for completeness if further visualization of joint angles is desired.
        coxa_len, femur_len, tibia_len = self.leg_lengths
        x, y, z = foot_pos_local

        theta1 = np.arctan2(y, x)

        l1 = np.sqrt(x**2 + y**2)
        l_eff = l1 - coxa_len

        l3_sq = l_eff**2 + z**2
        l3 = np.sqrt(l3_sq)

        if l3 > femur_len + tibia_len or l3 < abs(femur_len - tibia_len):
            return None

        cos_beta = (femur_len**2 + tibia_len**2 - l3_sq) / (2 * femur_len * tibia_len)
        beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))
        theta3 = knee_direction * (np.pi - beta)

        cos_alpha = (femur_len**2 + l3_sq - tibia_len**2) / (2 * femur_len * l3)
        alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
        phi = np.arctan2(z, l_eff)
        theta2 = phi - knee_direction * alpha

        COXA_LIMIT = np.deg2rad(90)
        FEMUR_LIMIT = np.deg2rad(100)
        TIBIA_LIMIT = np.deg2rad(120)

        if not (-COXA_LIMIT <= theta1 <= COXA_LIMIT): return None
        if not (-FEMUR_LIMIT <= theta2 <= FEMUR_LIMIT): return None
        if not (-TIBIA_LIMIT <= theta3 <= TIBIA_LIMIT): return None

        return [theta1, theta2, theta3]

    def body_ik(self, translation, rotation, foot_positions, knee_direction=1):
        # This method is not strictly needed for just visualizing the initial stance.
        pass

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
        
        coxa_pos = self.leg_positions[leg_index]

        # Horizontal distance covered by femur and tibia
        l_horiz = femur_len * np.cos(theta2) + tibia_len * np.cos(theta2 + theta3)
        
        # Total horizontal distance from coxa axis
        l_total = coxa_len + l_horiz
        
        # Position of the foot relative to the coxa attachment point
        x_local = l_total * np.cos(theta1)
        y_local = l_total * np.sin(theta1)
        z_local = femur_len * np.sin(theta2) + tibia_len * np.sin(theta2 + theta3)
        
        # Position of the foot in the body frame
        # This is the foot position relative to the body frame origin, not relative to coxa_pos
        foot_pos_body = np.array([coxa_pos[0] + x_local, coxa_pos[1] + y_local, coxa_pos[2] + z_local])
        
        return foot_pos_body.tolist()

# --- Simplified HexapodLocomotion for initial stance calculation ---
class HexapodStanceVisualizer:
    def __init__(self):
        # --- Robot Dimensions (copied from locomotion.py) ---
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

    def get_zero_angle_joint_coordinates(self):
        all_leg_coords = []
        zero_angles = [0.0, 0.0, 0.0] # Coxa, Femur, Tibia angles at 0

        for i in range(len(self.kinematics.leg_positions)):
            coxa_origin_pos = np.array(self.kinematics.leg_positions[i])

            coxa_len, femur_len, tibia_len = self.kinematics.leg_lengths

            # Apply coxa yaw rotation to get the direction of the leg segments
            yaw = self.kinematics.coxa_initial_rpys[i][2]
            Rz_coxa = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0,            0,           1]
            ])

            # Coxa joint position (same as coxa_origin_pos)
            coxa_pos = coxa_origin_pos

            # Femur joint position: coxa_len along the leg's x-axis from coxa_origin_pos
            femur_joint_pos = coxa_origin_pos + (Rz_coxa @ np.array([coxa_len, 0, 0]))

            # Tibia joint position: femur_len along the leg's x-axis from femur_joint_pos
            tibia_joint_pos = femur_joint_pos + (Rz_coxa @ np.array([femur_len, 0, 0]))

            # Foot position: tibia_len along the leg's x-axis from tibia_joint_pos
            foot_pos = tibia_joint_pos + (Rz_coxa @ np.array([tibia_len, 0, 0]))

            all_leg_coords.append({
                'coxa': coxa_pos,
                'femur_joint': femur_joint_pos,
                'tibia_joint': tibia_joint_pos,
                'foot': foot_pos
            })
        return all_leg_coords

# --- Visualization ---
if __name__ == "__main__":
    visualizer = HexapodStanceVisualizer()
    all_leg_coords = visualizer.get_zero_angle_joint_coordinates()

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot body origin (0,0,0)
    ax.scatter(0, 0, 0, color='black', s=200, label='Body Origin', marker='o')

    for i, leg_coords in enumerate(all_leg_coords):
        coxa = leg_coords['coxa']
        femur_joint = leg_coords['femur_joint']
        tibia_joint = leg_coords['tibia_joint']
        foot = leg_coords['foot']

        # Plot joints
        ax.scatter(coxa[0], coxa[1], coxa[2], color='red', s=80, label=f'Coxa {i}' if i == 0 else "", marker='o')
        ax.scatter(femur_joint[0], femur_joint[1], femur_joint[2], color='purple', s=80, label=f'Femur Joint {i}' if i == 0 else "", marker='o')
        ax.scatter(tibia_joint[0], tibia_joint[1], tibia_joint[2], color='orange', s=80, label=f'Tibia Joint {i}' if i == 0 else "", marker='o')
        ax.scatter(foot[0], foot[1], foot[2], color='blue', s=80, label=f'Foot {i}' if i == 0 else "", marker='x')

        # Draw links
        # Coxa to Femur Joint
        ax.plot([coxa[0], femur_joint[0]],
                [coxa[1], femur_joint[1]],
                [coxa[2], femur_joint[2]],
                color='green', linestyle='-', linewidth=2)
        # Femur Joint to Tibia Joint
        ax.plot([femur_joint[0], tibia_joint[0]],
                [femur_joint[1], tibia_joint[1]],
                [femur_joint[2], tibia_joint[2]],
                color='green', linestyle='-', linewidth=2)
        # Tibia Joint to Foot
        ax.plot([tibia_joint[0], foot[0]],
                [tibia_joint[1], foot[1]],
                [tibia_joint[2], foot[2]],
                color='green', linestyle='-', linewidth=2)

    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Hexapod with All Joint Angles at 0 (Body Frame)')
    ax.legend()

    # Set equal aspect ratio
    all_x = []
    all_y = []
    all_z = []
    for leg_coords in all_leg_coords:
        all_x.extend([leg_coords['coxa'][0], leg_coords['femur_joint'][0], leg_coords['tibia_joint'][0], leg_coords['foot'][0]])
        all_y.extend([leg_coords['coxa'][1], leg_coords['femur_joint'][1], leg_coords['tibia_joint'][1], leg_coords['foot'][1]])
        all_z.extend([leg_coords['coxa'][2], leg_coords['femur_joint'][2], leg_coords['tibia_joint'][2], leg_coords['foot'][2]])
    
    all_x = np.array(all_x)
    all_y = np.array(all_y)
    all_z = np.array(all_z)

    max_range = np.array([
        all_x.max() - all_x.min(),
        all_y.max() - all_y.min(),
        all_z.max() - all_z.min()
    ]).max() / 2.0

    mid_x = (all_x.max() + all_x.min()) * 0.5
    mid_y = (all_y.max() + all_y.min()) * 0.5
    mid_z = (all_z.max() + all_z.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.grid(True)
    plt.show()