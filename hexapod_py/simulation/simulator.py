import pybullet as p
import time
import pybullet_data
import os
import numpy as np

class HexapodSimulator:
    def __init__(self, gui=True):
        self.gui = gui
        self.physics_client = None
        self.robot_id = None
        self.joint_name_to_id = {}
        self.debug_param_ids = {}
        # This mapping assumes the order in your locomotion code (0-5)
        # and the joint names in your URDF.
        # The URDF uses hip_#, ties_#, and foot_# for coxa, femur, and tibia.
        self.leg_joint_names = [
            ["hip_3", "ties_3", "foot_3"], # Leg 0: Front-Right (FR) -> URDF _3
            ["hip_4", "ties_4", "foot_4"], # Leg 1: Middle-Right (MR) -> URDF _4
            ["hip_5", "ties_5", "foot_5"], # Leg 2: Rear-Right (RR) -> URDF _5
            ["hip_2", "ties_2", "foot_2"], # Leg 3: Front-Left (FL) -> URDF _2
            ["hip_1", "ties_1", "foot_1"], # Leg 4: Middle-Left (ML) -> URDF _1
            ["hip_6", "ties_6", "foot_6"], # Leg 5: Rear-Left (RL) -> URDF _6
        ]
        self.package_path = os.path.dirname(__file__) # Path to the 'simulation' directory

    def start(self):
        self.physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)

        # Add pybullet_data to the search path for built-in assets
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Add a floor
        p.loadURDF("plane.urdf")

        # Add the 'simulation' directory to the search path.
        # This allows PyBullet to find 'robot.urdf' and resolve the 'package://' paths for meshes.
        p.setAdditionalSearchPath(self.package_path)

        # Load URDF model
        self.robot_id = p.loadURDF("robot.urdf", basePosition=[0, 0, 0.2])
        self._map_joint_names_to_ids()
        print(f"Simulation started. Robot ID: {self.robot_id}")

    def _map_joint_names_to_ids(self):
        """Creates a map from joint names to their PyBullet IDs."""
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('UTF-8')
            self.joint_name_to_id[joint_name] = joint_info[0]

    def set_joint_angles(self, joint_angles):
        """
        Sets the target position for each joint.
        :param joint_angles: A list of 6 lists, where each inner list contains
                             the [coxa, femur, tibia] angles for a leg.
        """
        for leg_idx, leg_angles in enumerate(joint_angles):
            if leg_angles is not None:
                for joint_idx, angle in enumerate(leg_angles):
                    joint_name = self.leg_joint_names[leg_idx][joint_idx]
                    if joint_name in self.joint_name_to_id:
                        p.setJointMotorControl2(self.robot_id, self.joint_name_to_id[joint_name],
                                                p.POSITION_CONTROL, targetPosition=angle)
                    else:
                        print(f"Warning: Joint '{joint_name}' not found in URDF.")

    def add_ui_controls(self):
        """Adds debug sliders to the GUI for real-time control."""
        if not self.gui:
            print("Cannot add UI controls in non-GUI mode.")
            return

        self.debug_param_ids['vx'] = p.addUserDebugParameter("vx (m/s)", -0.2, 0.2, 0.0)
        self.debug_param_ids['vy'] = p.addUserDebugParameter("vy (m/s)", -0.2, 0.2, 0.0)
        self.debug_param_ids['omega'] = p.addUserDebugParameter("omega (rad/s)", -1.0, 1.0, 0.0)
        self.debug_param_ids['speed'] = p.addUserDebugParameter("gait_speed", 0.01, 0.05, 0.02)
        self.debug_param_ids['body_height'] = p.addUserDebugParameter("body_height (m)", 0.15, 0.25, 0.20)
        self.debug_param_ids['pitch'] = p.addUserDebugParameter("pitch (rad)", -0.5, 0.5, 0.0)
        self.debug_param_ids['step_height'] = p.addUserDebugParameter("step_height (m)", 0.0, 0.1, 0.05)

    def read_ui_controls(self):
        """Reads the current values from the GUI sliders."""
        if not self.gui or not self.debug_param_ids:
            # Return default values if no GUI or controls are present
            return {'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'speed': 0.02, 'body_height': 0.20, 'pitch': 0.0, 'step_height': 0.05}

        return {
            'vx': p.readUserDebugParameter(self.debug_param_ids['vx']),
            'vy': p.readUserDebugParameter(self.debug_param_ids['vy']),
            'omega': p.readUserDebugParameter(self.debug_param_ids['omega']),
            'speed': p.readUserDebugParameter(self.debug_param_ids['speed']),
            'body_height': p.readUserDebugParameter(self.debug_param_ids['body_height']),
            'pitch': p.readUserDebugParameter(self.debug_param_ids['pitch']),
            'step_height': p.readUserDebugParameter(self.debug_param_ids['step_height'])
        }

    def step(self):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    def stop(self):
        p.disconnect()
        print("Simulation stopped.")

# Example of running the simulation with locomotion control
if __name__ == "__main__":
    # This main block is now for demonstration.
    # A separate script will handle the control loop.
    sim = HexapodSimulator(gui=True)
    sim.start()

    # Example: Set legs to a "zero" position (may not be stable)
    # This demonstrates how set_joint_angles would be used.
    zero_angles = [[0.0, np.deg2rad(-30), np.deg2rad(30)]] * 6
    sim.set_joint_angles(zero_angles)

    try:
        # Run for a few seconds
        for _ in range(1000):
            sim.step()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
