#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from hexapod_kinematics.msg import MultiLegTargets, LegTarget, MultiLegJoints, LegJoints

class KinematicsEngine(Node):

    def __init__(self):
        super().__init__('kinematics_engine')
        
        # Declare and get parameters with default values that match the YAML structure
        self.declare_parameter('leg_lengths.coxa', 152.017)
        self.declare_parameter('leg_lengths.hip', 92.500)
        self.declare_parameter('leg_lengths.femur', 191.8)
        self.declare_parameter('leg_lengths.tibia', 284.969)

        self.coxa_len = self.get_parameter('leg_lengths.coxa').get_parameter_value().double_value
        self.hip_len = self.get_parameter('leg_lengths.hip').get_parameter_value().double_value
        self.femur_len = self.get_parameter('leg_lengths.femur').get_parameter_value().double_value
        self.tibia_len = self.get_parameter('leg_lengths.tibia').get_parameter_value().double_value

        # Declare and get joint limit parameters with default values
        self.declare_parameter('joint_limits.hip_min', -90.0)
        self.declare_parameter('joint_limits.hip_max', 90.0)
        self.declare_parameter('joint_limits.femur_min', -100.0)
        self.declare_parameter('joint_limits.femur_max', 100.0)
        self.declare_parameter('joint_limits.tibia_min', -120.0)
        self.declare_parameter('joint_limits.tibia_max', 120.0)

        self.hip_min = np.deg2rad(self.get_parameter('joint_limits.hip_min').get_parameter_value().double_value)
        self.hip_max = np.deg2rad(self.get_parameter('joint_limits.hip_max').get_parameter_value().double_value)
        self.femur_min = np.deg2rad(self.get_parameter('joint_limits.femur_min').get_parameter_value().double_value)
        self.femur_max = np.deg2rad(self.get_parameter('joint_limits.femur_max').get_parameter_value().double_value)
        self.tibia_min = np.deg2rad(self.get_parameter('joint_limits.tibia_min').get_parameter_value().double_value)
        self.tibia_max = np.deg2rad(self.get_parameter('joint_limits.tibia_max').get_parameter_value().double_value)

        # Create subscription and publisher
        self.subscription = self.create_subscription(
            MultiLegTargets,
            'leg_targets',
            self.ik_callback,
            10)
        self.publisher = self.create_publisher(MultiLegJoints, 'joint_commands', 10)

        self.get_logger().info('Kinematics Engine has been started.')

    def ik_callback(self, msg):
        multi_leg_joints = MultiLegJoints()
        
        for target in msg.targets:
            leg_joints = self.solve_ik_for_leg(target)
            multi_leg_joints.joints.append(leg_joints)
            
        self.publisher.publish(multi_leg_joints)

    def solve_ik_for_leg(self, target):
        leg_joints = LegJoints()
        leg_joints.leg_name = target.leg_name
        
        target_pos = target.pose.position
        x, y, z = target_pos.x, target_pos.y, target_pos.z

        try:
            # Solve for coxa angle (alpha)
            alpha = np.arctan2(y, x)

            # Project to the leg's XY plane (2D problem)
            l_horizontal = np.sqrt(x**2 + y**2) - self.hip_len
            
            # Distance from femur joint to foot tip
            d = np.sqrt(l_horizontal**2 + z**2)

            if d > (self.femur_len + self.tibia_len) or d < abs(self.femur_len - self.tibia_len):
                raise ValueError("Target position is unreachable")

            # Solve for tibia angle (gamma)
            cos_gamma = (d**2 - self.femur_len**2 - self.tibia_len**2) / (2 * self.femur_len * self.tibia_len)
            gamma = np.arccos(cos_gamma)

            # Solve for femur angle (beta)
            beta = np.arctan2(-z, l_horizontal) - np.arctan2(self.tibia_len * np.sin(gamma), self.femur_len + self.tibia_len * np.cos(gamma))
            
            if not (self.hip_min <= alpha <= self.hip_max and
                      self.femur_min <= beta <= self.femur_max and
                      self.tibia_min <= gamma <= self.tibia_max):
                raise ValueError("Resulting joint angles are out of limits")

            # Collision check
            femur_joint_z = 0 
            tibia_joint_z = -self.femur_len * np.sin(beta)
            
            if femur_joint_z < z or tibia_joint_z < z:
                raise ValueError("Collision detected: Leg would be lower than the foot tip.")

            leg_joints.coxa_angle = alpha
            leg_joints.femur_angle = beta
            leg_joints.tibia_angle = gamma
            
        except ValueError as e:
            self.get_logger().warn(f'IK Error for {target.leg_name}: {e}')
            leg_joints.coxa_angle = float('nan')
            leg_joints.femur_angle = float('nan')
            leg_joints.tibia_angle = float('nan')
        
        return leg_joints

def main(args=None):
    rclpy.init(args=args)
    kinematics_engine = KinematicsEngine()
    rclpy.spin(kinematics_engine)
    kinematics_engine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
