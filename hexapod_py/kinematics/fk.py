import numpy as np
from typing import List, Tuple

class HexapodForwardKinematics:
    """
    Calculates the 3D world positions of all joints of the hexapod.
    """

    def __init__(self, leg_lengths, hip_positions):
        """
        Initializes the forward kinematics calculator.
        """
        self.l_coxa, self.l_femur, self.l_tibia = leg_lengths
        self.hip_positions = np.array(hip_positions)

        # Pre-calculate the base angle for each hip from its position
        self.hip_base_angles = np.arctan2(self.hip_positions[:, 1], self.hip_positions[:, 0])

    def leg_fk(self, angles: Tuple[float, float, float]) -> np.ndarray:
        """
        Calculates the local positions of the joints for a single leg.
        """
        coxa_angle, femur_angle, tibia_angle = angles

        # Setting coxa as the origin
        coxa_joint_pos = np.array([0.0, 0.0, 0.0])

        # Calculaing femur joint position
        femur_joint_pos = coxa_joint_pos + np.array([self.l_coxa * np.cos(coxa_angle),
                                                    self.l_coxa * np.sin(coxa_angle),
                                                     0])

        # Calculating tibia joint position
        tibia_joint_pos = femur_joint_pos + np.array([self.l_femur * np.cos(femur_angle) * np.cos(coxa_angle),
                                                      self.l_femur * np.cos(femur_angle) * np.sin(coxa_angle),
                                                      self.l_femur * np.sin(femur_angle)])

        # Calculating foot tip position
        foot_tip_pos = tibia_joint_pos + np.array([self.l_tibia * np.cos(femur_angle + tibia_angle) * np.cos(coxa_angle),
                                                   self.l_tibia * np.cos(femur_angle + tibia_angle) * np.sin(coxa_angle),
                                                   self.l_tibia * np.sin(femur_angle + tibia_angle)])
        
        return np.array([coxa_joint_pos, femur_joint_pos, tibia_joint_pos, foot_tip_pos])

    def body_fk(self, all_leg_angles: List[Tuple[float, float, float]],
                body_translation: np.ndarray = np.zeros(3),
                body_rotation: np.ndarray = np.zeros(3)) -> List[np.ndarray]:
        """
        Calculates the world positions of all joints for all six legs.
        """
        # Creating body rotation matrix from the roll, pitch, and yaw angles.
        roll, pitch, yaw = body_rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R_body = Rz @ Ry @ Rx

        world_leg_points = []
        for i in range(6):
            # Calculate the hip position in world coordinates
            hip_pos_world = R_body @ self.hip_positions[i] + body_translation

            # If no angles are provided for this leg, skip FK calculation.
            if not all_leg_angles or all_leg_angles[i] is None:
                world_leg_points.append(np.array([hip_pos_world]))
                continue

            angles = all_leg_angles[i]

            # Get joint positions in the leg's local frame
            local_points = self.leg_fk(angles)

            # Get the hip base angle for this leg
            hip_base_angle = self.hip_base_angles[i]
            R_hip = np.array([[np.cos(hip_base_angle), -np.sin(hip_base_angle), 0],
                              [np.sin(hip_base_angle),  np.cos(hip_base_angle), 0],
                              [0,                      0,                     1]])

            # Transform local leg points to world coordinates
            leg_world = np.zeros_like(local_points)
            leg_world[0] = hip_pos_world
            for j in range(1, local_points.shape[0]):
                leg_world[j] = hip_pos_world + R_body @ (R_hip @ local_points[j])
            
            world_leg_points.append(leg_world)
            
        return world_leg_points