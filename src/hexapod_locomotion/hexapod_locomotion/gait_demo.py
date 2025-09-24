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
from .locomotion import HexapodLocomotion

class GaitDemoGUI(Node):
    def __init__(self):
        super().__init__('gait_demo_gui')
        self.locomotion = HexapodLocomotion(self)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        self.joint_names = [
            'hip_2', 'ties_2', 'foot_2',  # Front-Right
            'hip_3', 'ties_3', 'foot_3',  # Middle-Right
            'hip_4', 'ties_4', 'foot_4',  # Rear-Right
            'hip_1', 'ties_1', 'foot_1',  # Front-Left
            'hip_6', 'ties_6', 'foot_6',  # Middle-Left
            'hip_5', 'ties_5', 'foot_5',  # Rear-Left
        ]

        # --- GUI Setup ---
        self.root = tk.Tk()
        self.root.title("Hexapod Gait Control")

        self.vx_slider = self._create_slider("Forward/Backward (vx)", -1.0, 1.0, 0.1)
        self.vy_slider = self._create_slider("Sideways (vy)", -1.0, 1.0, 0.1)
        self.omega_slider = self._create_slider("Turn (omega)", -1.0, 1.0, 0.1)

        self.standoff_slider = self._create_slider("Standoff Distance", 0.1, 0.5, 0.01)
        self.standoff_slider.set(self.get_parameter('standoff_distance').get_parameter_value().double_value)
        
        self.body_height_slider = self._create_slider("Body Height", 0.05, 0.25, 0.01)
        self.body_height_slider.set(self.get_parameter('body_height').get_parameter_value().double_value)

        self.running = False
        self.start_button = tk.Button(self.root, text="Start Gait", command=self.start_gait)
        self.start_button.pack()
        self.stop_button = tk.Button(self.root, text="Stop Gait", command=self.stop_gait, state=tk.DISABLED)
        self.stop_button.pack()
        self.knee_button = tk.Button(self.root, text="Toggle Knee Direction", command=self.toggle_knee_direction)
        self.knee_button.pack(pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_slider(self, label, from_, to, resolution):
        frame = tk.Frame(self.root)
        frame.pack(pady=5)
        tk.Label(frame, text=label).pack()
        slider = tk.Scale(frame, from_=from_, to=to, resolution=resolution, orient=tk.HORIZONTAL, length=300)
        slider.pack()
        return slider

    def start_gait(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.gait_thread = threading.Thread(target=self.run_gait_loop)
        self.gait_thread.start()

    def stop_gait(self):
        self.running = False
        if hasattr(self, 'gait_thread') and self.gait_thread.is_alive():
            self.gait_thread.join()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def toggle_knee_direction(self):
        self.locomotion.knee_direction *= -1
        direction = "Up" if self.locomotion.knee_direction == 1 else "Down"
        self.get_logger().info(f"Knee direction set to: {direction}")

    def run_gait_loop(self):
        while self.running and rclpy.ok():
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
            
            self.root.after(20) # ~50 Hz

    def on_closing(self):
        self.stop_gait()
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui_node = GaitDemoGUI()
    gui_node.root.mainloop()

if __name__ == '__main__':
    main()