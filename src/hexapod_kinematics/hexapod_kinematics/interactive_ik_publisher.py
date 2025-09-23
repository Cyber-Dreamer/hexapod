#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hexapod_kinematics.msg import LegTarget, MultiLegTargets, LegJoints, MultiLegJoints
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pynput import keyboard

# --- Instructions ---
# Use the following keys to control the target position of the foot tip:
#
# - Arrow Up/Down:    Move target in Y direction
# - Arrow Left/Right: Move target in X direction
# - Page Up/Down:     Move target in Z direction
# - Q or ESC:         Quit the program
#
# The 3D plot will update in real-time to show the leg's posture.
# --------------------

class InteractiveIKPublisher(Node):
    def __init__(self):
        super().__init__('interactive_ik_publisher')
        
        # Declare and get parameters for leg lengths from the kinematics_params.yaml
        # Ensure these names match the parameter names in the YAML file and the kinematics_engine
        self.declare_parameter('leg_lengths.hip', 92.500)
        self.declare_parameter('leg_lengths.femur', 191.8)
        self.declare_parameter('leg_lengths.tibia', 284.969)

        self.hip_len = self.get_parameter('leg_lengths.hip').get_parameter_value().double_value
        self.femur_len = self.get_parameter('leg_lengths.femur').get_parameter_value().double_value
        self.tibia_len = self.get_parameter('leg_lengths.tibia').get_parameter_value().double_value

        # Publisher for leg targets
        self.target_publisher = self.create_publisher(MultiLegTargets, 'leg_targets', 10)
        
        # Subscriber for joint commands
        self.joint_subscriber = self.create_subscription(
            MultiLegJoints,
            'joint_commands',
            self.joint_command_callback,
            10)

        self.target_pos = [250.0, 0.0, -150.0]  # Initial target position [x, y, z]
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Timer to periodically publish the target
        self.publish_timer = self.create_timer(0.05, self.publish_target)
        
        self.get_logger().info("Interactive IK Publisher started.")
        self.get_logger().info("Use arrow keys and PageUp/Down to move the target.")
        self.get_logger().info("Press 'q' or 'ESC' to quit.")

    def publish_target(self):
        msg = MultiLegTargets()
        
        # We control one leg for this test, let's call it 'right_front'
        target = LegTarget()
        target.leg_name = 'right_front' 
        target.target_pose.position.x = self.target_pos[0]
        target.target_pose.position.y = self.target_pos[1]
        target.target_pose.position.z = self.target_pos[2]
        # Orientation can be left as default (0,0,0,1) for now
        target.target_pose.orientation.w = 1.0

        msg.targets.append(target)
        self.target_publisher.publish(msg)

    def joint_command_callback(self, msg):
        # Find the leg we are interested in and plot it
        for leg_joints in msg.joints:
            if leg_joints.leg_name == 'right_front':
                self.plot_leg(leg_joints.coxa_angle, leg_joints.femur_angle, leg_joints.tibia_angle)
                break

    def plot_leg(self, alpha, beta, gamma):
        # Forward Kinematics to get all joint positions for plotting
        p0 = np.array([0, 0, 0]) # Base
        p1 = np.array([self.hip_len * np.cos(alpha), self.hip_len * np.sin(alpha), 0]) # Coxa -> Femur joint
        
        # Femur -> Tibia joint
        x2 = np.cos(alpha) * (self.hip_len + self.femur_len * np.cos(beta))
        y2 = np.sin(alpha) * (self.hip_len + self.femur_len * np.cos(beta))
        z2 = -self.femur_len * np.sin(beta)
        p2 = np.array([x2, y2, z2])

        # Tibia -> Foot tip
        x3 = np.cos(alpha) * (self.hip_len + self.femur_len * np.cos(beta) + self.tibia_len * np.cos(beta + gamma))
        y3 = np.sin(alpha) * (self.hip_len + self.femur_len * np.cos(beta) + self.tibia_len * np.cos(beta + gamma))
        z3 = -self.femur_len * np.sin(beta) - self.tibia_len * np.sin(beta + gamma)
        p3 = np.array([x3, y3, z3])

        points = np.array([p0, p1, p2, p3])
        
        self.ax.clear()
        self.ax.plot(points[:, 0], points[:, 1], points[:, 2], "o-", c='b', markersize=8, label="Leg")
        self.ax.scatter(self.target_pos[0], self.target_pos[1], self.target_pos[2], c='r', marker='x', s=100, label="Target")
        
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Hexapod Leg IK Visualization')
        
        # Set consistent axis limits for better visualization
        limit = max(self.coxa_len + self.femur_len, self.tibia_len) * 1.1
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-limit, limit])
        self.ax.legend()
        plt.draw()

def on_press(key, node):
    step = 10.0 # mm
    try:
        if key == keyboard.Key.up:
            node.target_pos[1] += step
        elif key == keyboard.Key.down:
            node.target_pos[1] -= step
        elif key == keyboard.Key.left:
            node.target_pos[0] -= step
        elif key == keyboard.Key.right:
            node.target_pos[0] += step
        elif key == keyboard.Key.page_up:
            node.target_pos[2] += step
        elif key == keyboard.Key.page_down:
            node.target_pos[2] -= step
    except AttributeError:
        pass # Ignore other keys

def on_release(key, node):
    if key == keyboard.Key.esc or (hasattr(key, 'char') and key.char == 'q'):
        node.get_logger().info("Shutdown requested.")
        plt.close(node.fig) # Close the plot window
        rclpy.shutdown()
        return False # Stop the listener

def main(args=None):
    rclpy.init(args=args)
    ik_publisher_node = InteractiveIKPublisher()

    # Setup keyboard listener in a separate thread
    listener = keyboard.Listener(
        on_press=lambda key: on_press(key, ik_publisher_node),
        on_release=lambda key: on_release(key, ik_publisher_node)
    )
    listener.start()
    
    plt.ion() # Turn on interactive mode for matplotlib
    plt.show()

    try:
        # rclpy.spin() is not used here because the plot and keyboard listener
        # need to run in the main thread. The node spins implicitly.
        while rclpy.ok() and listener.is_alive():
            plt.pause(0.1) # Allows plot to update and messages to be processed
    except KeyboardInterrupt:
        pass
    finally:
        if listener.is_alive():
            listener.stop()
        ik_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
