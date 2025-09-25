"""
Gait Demo with Tkinter GUI
==========================

This script provides a simple graphical user interface (GUI) to control the
hexapod's gait and visualize it in RViz. It uses Tkinter for the GUI elements
and publishes JointState messages to a ROS2 topic.
"""

import tkinter as tk
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import threading
import numpy as np
from .locomotion import HexapodLocomotion
from rcl_interfaces.msg import ParameterDescriptor

class GaitDemoGUI(Node):
    def __init__(self):
        super().__init__('gait_demo_gui')
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.joint_names = [
            'hip_2', 'ties_2', 'foot_2',  # Front-Right
            'hip_3', 'ties_3', 'foot_3',  # Middle-Right
            'hip_4', 'ties_4', 'foot_4',  # Rear-Right
            'hip_1', 'ties_1', 'foot_1',  # Front-Left
            'hip_6', 'ties_6', 'foot_6',  # Middle-Left
            'hip_5', 'ties_5', 'foot_5',  # Rear-Left
        ]

        # Declare parameters
        from rcl_interfaces.msg import ParameterDescriptor # Import needed for ParameterDescriptor
        self.declare_parameter('standoff_distance', 0.25, ParameterDescriptor(description='Standoff distance for the feet from the body.'))
        self.declare_parameter('body_height', 0.20, ParameterDescriptor(description='Height of the body from the ground.'))

        # --- GUI Setup ---
        self.root = tk.Tk()
        self.root.title("Hexapod Control")

        # Main frame to hold everything
        main_frame = tk.Frame(self.root)
        main_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Initialize Tkinter variables and locomotion AFTER root is created
        self.gait_type_var = tk.StringVar(value='tripod')
        self.locomotion = HexapodLocomotion(self, gait_type=self.gait_type_var.get())

        # Gait Control Frame
        gait_frame = tk.LabelFrame(main_frame, text="Gait Control")
        gait_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        row_idx = 0
        self.vx_slider = self._create_slider(gait_frame, "Forward/Backward (vx)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1
        self.vy_slider = self._create_slider(gait_frame, "Sideways (vy)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1
        self.omega_slider = self._create_slider(gait_frame, "Turn (omega)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1

        self.standoff_slider = self._create_slider(gait_frame, "Standoff Distance", 0.1, 0.5, 0.01, row=row_idx)
        self.standoff_slider.set(self.get_parameter('standoff_distance').get_parameter_value().double_value)
        row_idx += 1
        
        self.body_height_slider = self._create_slider(gait_frame, "Body Height", 0.05, 0.25, 0.01, row=row_idx)
        self.body_height_slider.set(self.get_parameter('body_height').get_parameter_value().double_value)
        row_idx += 1

        # Gait Type Selection
        gait_type_frame = tk.Frame(gait_frame)
        gait_type_frame.grid(row=row_idx, column=0, columnspan=2, pady=5)
        tk.Label(gait_type_frame, text="Gait Type:").pack(side=tk.LEFT)
        tk.Radiobutton(gait_type_frame, text="Tripod", variable=self.gait_type_var, value='tripod', command=self.switch_gait_type).pack(side=tk.LEFT)
        tk.Radiobutton(gait_type_frame, text="Ripple", variable=self.gait_type_var, value='ripple', command=self.switch_gait_type).pack(side=tk.LEFT)
        row_idx += 1

        self.running = False
        self.start_button = tk.Button(gait_frame, text="Start Gait", command=self.start_gait)
        self.start_button.grid(row=row_idx, column=0, columnspan=2, pady=2)
        row_idx += 1
        self.stop_button = tk.Button(gait_frame, text="Stop Gait", command=self.stop_gait, state=tk.DISABLED)
        self.stop_button.grid(row=row_idx, column=0, columnspan=2, pady=2)
        row_idx += 1
        self.knee_button = tk.Button(gait_frame, text="Toggle Knee Direction", command=self.toggle_knee_direction)
        self.knee_button.grid(row=row_idx, column=0, columnspan=2, pady=10)
        row_idx += 1

        # Body Pose Control Frame
        body_pose_frame = tk.LabelFrame(main_frame, text="Body Pose Control")
        body_pose_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        row_idx = 0
        self.body_x_slider = self._create_slider(body_pose_frame, "Body X (m)", -0.1, 0.1, 0.001, initial_value=0.0, row=row_idx)
        row_idx += 1
        self.body_y_slider = self._create_slider(body_pose_frame, "Body Y (m)", -0.1, 0.1, 0.001, initial_value=0.0, row=row_idx)
        row_idx += 1
        self.body_z_slider = self._create_slider(body_pose_frame, "Body Z (m)", -0.05, 0.05, 0.001, initial_value=0.0, row=row_idx)
        row_idx += 1
        self.body_roll_slider = self._create_slider(body_pose_frame, "Body Roll (deg)", -30, 30, 1, initial_value=0.0, row=row_idx)
        row_idx += 1
        self.body_pitch_slider = self._create_slider(body_pose_frame, "Body Pitch (deg)", -30, 30, 1, initial_value=0.0, row=row_idx)
        row_idx += 1
        self.body_yaw_slider = self._create_slider(body_pose_frame, "Body Yaw (deg)", -30, 30, 1, initial_value=0.0, row=row_idx)
        row_idx += 1

        self.apply_body_pose_button = tk.Button(body_pose_frame, text="Apply Body Pose", command=self.apply_body_pose)
        self.apply_body_pose_button.grid(row=row_idx, column=0, columnspan=2, pady=5)
        row_idx += 1

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Configure grid weights for resizing
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)

    def _create_slider(self, parent, label, from_, to, resolution, initial_value=None, row=0):
        frame = tk.Frame(parent)
        frame.grid(row=row, column=0, columnspan=2, pady=2, sticky="ew")
        tk.Label(frame, text=label).pack(side=tk.TOP)
        slider = tk.Scale(frame, from_=from_, to=to, resolution=resolution, orient=tk.HORIZONTAL, length=300)
        if initial_value is not None:
            slider.set(initial_value)
        slider.pack(side=tk.BOTTOM, fill="x", expand=True)
        return slider

    def start_gait(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.apply_body_pose_button.config(state=tk.DISABLED) # Disable body pose control during gait
        self.run_gait_loop()

    def stop_gait(self):
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.apply_body_pose_button.config(state=tk.NORMAL) # Enable body pose control after gait

    def toggle_knee_direction(self):
        self.locomotion.knee_direction *= -1
        direction = "Up" if self.locomotion.knee_direction == 1 else "Down"
        self.get_logger().info(f"Knee direction set to: {direction}")
        joint_angles_list = self.locomotion.update_knee_direction(self.locomotion.knee_direction)
        if all(angles is not None for angles in joint_angles_list):
            joint_positions = [angle for leg_angles in joint_angles_list for angle in leg_angles]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = joint_positions
            self.publisher_.publish(msg)

    def switch_gait_type(self):
        # Stop current gait if running
        if self.running:
            self.stop_gait()
        
        new_gait_type = self.gait_type_var.get()
        self.get_logger().info(f"Switching gait type to: {new_gait_type}")
        self.locomotion = HexapodLocomotion(self, gait_type=new_gait_type)
        # Reset gait phase and state for the new gait
        self.locomotion.gait_phase = 0.0

    def apply_body_pose(self):
        if self.running:
            self.get_logger().warn("Cannot apply body pose while gait is running. Please stop gait first.")
            return

        tx = self.body_x_slider.get()
        ty = self.body_y_slider.get()
        tz = self.body_z_slider.get()
        roll_deg = self.body_roll_slider.get()
        pitch_deg = self.body_pitch_slider.get()
        yaw_deg = self.body_yaw_slider.get()

        translation = [tx, ty, tz]
        rotation = [np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg)]

        self.get_logger().info(f"Applying body pose: Translation={translation}, Rotation={rotation}")
        joint_angles_list = self.locomotion.set_body_pose(translation, rotation)

        if joint_angles_list and all(angles is not None for angles in joint_angles_list):
            joint_positions = [angle for leg_angles in joint_angles_list for angle in leg_angles]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = joint_positions
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Failed to apply body pose: Unreachable or invalid joint angles.")

    def run_gait_loop(self):
        if not self.running or not rclpy.ok():
            return
            
        # Update parameters from sliders
        standoff = self.standoff_slider.get()
        body_height = self.body_height_slider.get()
        self.set_parameters([
            Parameter('standoff_distance', Parameter.Type.DOUBLE, standoff),
            Parameter('body_height', Parameter.Type.DOUBLE, body_height)
        ])

        vx = self.vx_slider.get()
        vy = self.vy_slider.get()
        omega = self.omega_slider.get()

        joint_angles_list = self.locomotion.run_gait(vx, vy, omega)
        
        if all(angles is not None for angles in joint_angles_list):
            joint_positions = [angle for leg_angles in joint_angles_list for angle in leg_angles]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = joint_positions
            self.publisher_.publish(msg)
        
        self.root.after(20, self.run_gait_loop) # ~50 Hz

    def on_closing(self):
        self.stop_gait()
        self.destroy_node()
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui_node = GaitDemoGUI()
    gui_node.root.mainloop()

if __name__ == '__main__':
    main()