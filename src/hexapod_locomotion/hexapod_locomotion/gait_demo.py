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

        # --- Initialize Tkinter variables and locomotion AFTER root is created ---
        self.gait_type_var = tk.StringVar(value='tripod') 
        self.moving_mode_enabled = tk.BooleanVar(value=False) # False=Stationary, True=Moving
        self.locomotion = HexapodLocomotion(self, gait_type=self.gait_type_var.get())

        # --- Create UI Frames ---
        mode_frame = tk.LabelFrame(main_frame, text="Mode")
        mode_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        self.moving_frame = tk.LabelFrame(main_frame, text="Moving Mode Controls")
        self.moving_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.stationary_frame = tk.LabelFrame(main_frame, text="Stationary Mode Controls")
        self.stationary_frame.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        self.shared_frame = tk.LabelFrame(main_frame, text="Shared Controls")
        self.shared_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="ew")

        # --- Populate Frames ---
        self._create_mode_controls(mode_frame)
        self._create_moving_controls(self.moving_frame)
        self._create_stationary_controls(self.stationary_frame)
        self._create_shared_controls(self.shared_frame)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Configure grid weights for resizing
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)

        # Set initial UI state
        self.switch_mode()

        # Flag to control the ROS2 spin thread
        self.ros_thread_running = True
        # Start ROS2 spinning in a separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin_thread)
        self.ros_thread.daemon = True # Allow the main program to exit even if this thread is still running
        self.ros_thread.start()

    def _ros_spin_thread(self):
        """
        Function to run rclpy.spin() in a separate thread.
        """
        while rclpy.ok() and self.ros_thread_running:
            rclpy.spin_once(self, timeout_sec=0.1) # Process ROS2 events periodically
        self.get_logger().info("ROS2 spin thread stopped.")

    def _create_slider(self, parent, label, from_, to, resolution, initial_value=None, row=0, command=None):
        frame = tk.Frame(parent)
        frame.grid(row=row, column=0, columnspan=2, pady=2, sticky="ew")
        tk.Label(frame, text=label).pack(side=tk.TOP)
        slider = tk.Scale(frame, from_=from_, to=to, resolution=resolution, orient=tk.HORIZONTAL, length=300)
        if initial_value is not None:
            slider.set(initial_value)
        if command:
            slider.config(command=command)
        slider.pack(side=tk.BOTTOM, fill="x", expand=True)
        return slider

    def _create_mode_controls(self, parent):
        tk.Checkbutton(parent, text="Enable Moving Mode", variable=self.moving_mode_enabled, command=self.switch_mode, onvalue=True, offvalue=False).pack(side=tk.LEFT, padx=10)

    def _create_moving_controls(self, parent):
        row_idx = 0
        self.vx_slider = self._create_slider(parent, "Forward/Backward (vx)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1
        self.vy_slider = self._create_slider(parent, "Sideways (vy)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1
        self.omega_slider = self._create_slider(parent, "Turn (omega)", -1.0, 1.0, 0.1, row=row_idx)
        row_idx += 1
        self.moving_pitch_slider = self._create_slider(parent, "Body Pitch (deg)", -30, 30, 1, initial_value=0.0, row=row_idx)
        row_idx += 1

        self.running = False
        self.start_button = tk.Button(parent, text="Start Gait", command=self.start_gait)
        self.start_button.grid(row=row_idx, column=0, columnspan=2, pady=5)
        row_idx += 1
        self.stop_button = tk.Button(parent, text="Stop Gait", command=self.stop_gait, state=tk.DISABLED)
        self.stop_button.grid(row=row_idx, column=0, columnspan=2, pady=5)

    def _create_stationary_controls(self, parent):
        row_idx = 0
        self.body_x_slider = self._create_slider(parent, "Body X (m)", -0.1, 0.1, 0.001, initial_value=0.0, row=row_idx, command=self.apply_body_pose)
        row_idx += 1
        self.body_y_slider = self._create_slider(parent, "Body Y (m)", -0.1, 0.1, 0.001, initial_value=0.0, row=row_idx, command=self.apply_body_pose)
        row_idx += 1
        self.body_roll_slider = self._create_slider(parent, "Body Roll (deg)", -30, 30, 1, initial_value=0.0, row=row_idx, command=self.apply_body_pose)
        row_idx += 1
        self.body_pitch_slider = self._create_slider(parent, "Body Pitch (deg)", -30, 30, 1, initial_value=0.0, row=row_idx, command=self.apply_body_pose)
        row_idx += 1
        self.body_yaw_slider = self._create_slider(parent, "Body Yaw (deg)", -30, 30, 1, initial_value=0.0, row=row_idx, command=self.apply_body_pose)
        row_idx += 1

        # Add a reset button
        self.reset_pose_button = tk.Button(parent, text="Reset Pose", command=self.reset_stationary_pose)
        self.reset_pose_button.grid(row=row_idx, column=0, columnspan=2, pady=10)


    def _create_shared_controls(self, parent):
        # Frame for sliders
        slider_frame = tk.Frame(parent)
        slider_frame.pack(fill='x', expand=True)

        self.standoff_slider = self._create_slider(slider_frame, "Standoff Distance", 0.1, 0.5, 0.01, row=0, command=self._update_shared_params)
        self.standoff_slider.set(self.get_parameter('standoff_distance').get_parameter_value().double_value)
        
        self.body_height_slider = self._create_slider(slider_frame, "Body Height", 0.05, 0.25, 0.01, row=1, command=self._update_shared_params)
        self.body_height_slider.set(self.get_parameter('body_height').get_parameter_value().double_value)

        # Frame for buttons
        button_frame = tk.Frame(parent)
        button_frame.pack(pady=5)

        # Gait Type Selection
        gait_type_frame = tk.Frame(button_frame)
        gait_type_frame.pack(side=tk.LEFT, padx=10)
        tk.Label(gait_type_frame, text="Gait Type:").pack(side=tk.LEFT)
        tk.Radiobutton(gait_type_frame, text="Tripod", variable=self.gait_type_var, value='tripod', command=self.switch_gait_type).pack(side=tk.LEFT)
        tk.Radiobutton(gait_type_frame, text="Ripple", variable=self.gait_type_var, value='ripple', command=self.switch_gait_type).pack(side=tk.LEFT)

        self.knee_button = tk.Button(button_frame, text="Toggle Knee Direction", command=self.toggle_knee_direction)
        self.knee_button.pack(side=tk.LEFT, padx=10)

    def start_gait(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.run_gait_loop()

    def stop_gait(self):
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

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

    def switch_mode(self):
        is_moving_mode = self.moving_mode_enabled.get()
        if self.running:
            self.stop_gait()

        if is_moving_mode:
            self._set_frame_widgets_state(self.moving_frame, tk.NORMAL)
            self._set_frame_widgets_state(self.stationary_frame, tk.DISABLED)
        else: # stationary
            self._set_frame_widgets_state(self.moving_frame, tk.DISABLED)
            self._set_frame_widgets_state(self.stationary_frame, tk.NORMAL)
            self.apply_body_pose() # Apply a neutral pose when switching to stationary

    def _set_frame_widgets_state(self, frame, state):
        """
        Recursively sets the state of interactive widgets within a frame.
        Handles sliders (in sub-frames) and buttons directly.
        """
        for widget in frame.winfo_children():
            # This handles the tk.Scale widgets which are the second child of their container frame.
            if isinstance(widget, tk.Frame) and len(widget.winfo_children()) > 1:
                # Check if the second child is a Scale widget before configuring
                if isinstance(widget.winfo_children()[1], tk.Scale):
                    widget.winfo_children()[1].config(state=state)
            # This handles widgets like tk.Button that are direct children of the main frame.
            elif isinstance(widget, (tk.Button, tk.Scale)):
                widget.config(state=state)
        # Special case for the reset button, which should only be enabled in stationary mode
        if frame == self.stationary_frame:
            self.reset_pose_button.config(state=state)

    
    def switch_gait_type(self):
        # Stop current gait if running
        if self.running:
            self.stop_gait()
        
        new_gait_type = self.gait_type_var.get()
        self.get_logger().info(f"Switching gait type to: {new_gait_type}")
        self.locomotion.set_gait(new_gait_type)
        # If in stationary mode, re-apply the current pose to ensure it's up-to-date
        if not self.moving_mode_enabled.get():
            self.apply_body_pose()

    def _update_shared_params(self, _=None):
        """
        Updates the shared ROS parameters from their sliders and refreshes the pose.
        """
        standoff = self.standoff_slider.get()
        body_height = self.body_height_slider.get()
        self.set_parameters([
            Parameter('standoff_distance', Parameter.Type.DOUBLE, standoff),
            Parameter('body_height', Parameter.Type.DOUBLE, body_height)
        ])
        # If in stationary mode, re-apply the pose to reflect the change
        if not self.moving_mode_enabled.get():
            self.apply_body_pose()

    def reset_stationary_pose(self):
        """Resets all stationary pose sliders to their initial (zero) value."""
        for slider in [self.body_x_slider, self.body_y_slider, self.body_roll_slider, self.body_pitch_slider, self.body_yaw_slider]:
            slider.set(0.0)

    def apply_body_pose(self, _=None): # Accept an argument from the slider command, but ignore it
        tx = self.body_x_slider.get()
        ty = self.body_y_slider.get()
        tz = 0.0 # Body Z is now controlled by the Body Height slider
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
            
        # Update shared parameters from sliders (they are already linked via command)
        self._update_shared_params()

        vx = self.vx_slider.get()
        vy = self.vy_slider.get()
        omega = self.omega_slider.get()
        pitch_deg = self.moving_pitch_slider.get()

        joint_angles_list = self.locomotion.run_gait(vx, vy, omega, pitch=np.deg2rad(pitch_deg), speed=0.02)
        
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
        self.ros_thread_running = False # Signal the ROS2 thread to stop
        # The main function will handle joining the thread and rclpy.shutdown()
        self.destroy_node()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    gui_node = GaitDemoGUI()
    try:
        gui_node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure cleanup and ROS2 shutdown
        gui_node.on_closing()
        rclpy.shutdown()

if __name__ == '__main__':
    main()