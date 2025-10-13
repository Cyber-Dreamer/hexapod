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
        self.link_name_to_id = {}
        self.debug_param_ids = {}
        self.gait_selector_id = None

        # This mapping translates the locomotion leg order (0-5) to the URDF joint names.
        # Locomotion Order: 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
        # The URDF uses hip_#, ties_#, and foot_# for coxa, femur, and tibia joints.
        self.leg_joint_names = [
            ["hip_6", "ties_6", "foot_6"], # Leg 0: Rear-Left (RL) -> URDF _6
            ["hip_1", "ties_1", "foot_1"], # Leg 1: Middle-Left (ML) -> URDF _1
            ["hip_2", "ties_2", "foot_2"], # Leg 2: Front-Left (FL) -> URDF _2
            ["hip_3", "ties_3", "foot_3"], # Leg 3: Front-Right (FR) -> URDF _3
            ["hip_4", "ties_4", "foot_4"], # Leg 4: Middle-Right (MR) -> URDF _4
            ["hip_5", "ties_5", "foot_5"], # Leg 5: Rear-Right (RR) -> URDF _5
        ]
        self.package_path = os.path.dirname(__file__) # Path to the 'simulation' directory

    def start(self):
        self.physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)

        # Add pybullet_data to the search path for built-in assets
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Add a floor
        plane_id = p.loadURDF("plane.urdf")
        # Set friction to simulate rubber on rubber (kinetic friction coeff ~1.0)
        p.changeDynamics(plane_id, -1, lateralFriction=2.0)

        # Add the 'simulation' directory to the search path.
        # This allows PyBullet to find 'robot.urdf' and resolve the 'package://' paths for meshes.
        p.setAdditionalSearchPath(self.package_path)

        # Load URDF model
        self.robot_id = p.loadURDF("robot.urdf", basePosition=[0, 0, 0.4])
        self._map_joint_names_to_ids()
        self._set_foot_friction()
        print(f"Simulation started. Robot ID: {self.robot_id}")

    def _map_joint_names_to_ids(self):
        """Creates maps from joint/link names to their PyBullet IDs."""
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('UTF-8')
            self.joint_name_to_id[joint_name] = joint_info[0]
            link_name = joint_info[12].decode('UTF-8')
            # The link index is the same as the joint index it's a child of.
            self.link_name_to_id[link_name] = joint_info[0]

    def _set_foot_friction(self, lateral_friction=2.0, spinning_friction=0.1, rolling_friction=0.1):
        """Increases the friction of the foot links."""
        # The links that make contact with the ground are the "tibia" links in this URDF.
        foot_links = [name for name in self.link_name_to_id.keys() if 'tibia' in name]

        for link_name in foot_links:
            link_id = self.link_name_to_id[link_name]
            p.changeDynamics(
                self.robot_id, 
                link_id, 
                lateralFriction=lateral_friction,
                spinningFriction=spinning_friction,
                rollingFriction=rolling_friction
            )
            print(f"Set friction for link '{link_name}' (ID: {link_id}) to lateral={lateral_friction}, spinning={spinning_friction}, rolling={rolling_friction}")

        if not foot_links:
            print("Warning: No 'tibia' links found to set friction.")

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
                        p.setJointMotorControl2(
                            bodyIndex=self.robot_id,
                            jointIndex=self.joint_name_to_id[joint_name],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,      # The desired angle for the joint
                            force=50,                 # Realistic torque for an 80kg-cm servo (~7.85 Nm)
                            positionGain=1.0,          # High gain for stiff, responsive control
                            velocityGain=0.05,          # Reduced damping to mimic aggressive servo behavior
                            maxVelocity=5.236)         # 60 deg / 0.2s = 300 deg/s = 5.236 rad/s
                    else:
                        print(f"Warning: Joint '{joint_name}' not found in URDF.")

    def add_ui_controls(self):
        """Adds debug sliders to the GUI for real-time control."""
        if not self.gui:
            print("Cannot add UI controls in non-GUI mode.")
            return

        self.debug_param_ids['vx'] = p.addUserDebugParameter("Vx (fwd/bwd)", -1.0, 1.0, 0)
        self.debug_param_ids['vy'] = p.addUserDebugParameter("Vy (strafe)", -1.0, 1.0, 0)
        self.debug_param_ids['omega'] = p.addUserDebugParameter("Omega (turn)", -1.0, 1.0, 0)
        self.debug_param_ids['body_height'] = p.addUserDebugParameter("Body Height (m)", 0.15, 0.3, 0.20)
        self.debug_param_ids['step_height'] = p.addUserDebugParameter("Step Height (m)", 0.01, 0.08, 0.04)
        self.debug_param_ids['standoff'] = p.addUserDebugParameter("Standoff (m)", 0.2, 0.5, 0.28)
        self.debug_param_ids['roll'] = p.addUserDebugParameter("Roll (rad)", -0.5, 0.5, 0.0)
        self.debug_param_ids['pitch'] = p.addUserDebugParameter("Pitch (rad)", -0.5, 0.5, 0.0)

    def add_gait_selection_ui(self, gait_options):
        """Adds a radio button UI to select a gait."""
        if not self.gui:
            return
        self.gait_selector_id = p.addUserDebugParameter("Gait", 0, len(gait_options) - 1, 0)
        # This is a trick to label the radio buttons in PyBullet
        p.setDebugObjectColor(self.gait_selector_id, -1, objectDebugColorRGB=[0,0,0]) # Hide the slider
        for i, option in enumerate(gait_options):
            p.addUserDebugText(option.capitalize(), [0, -1.5 - i*0.5, 0], textColorRGB=[1,1,0], parentObjectUniqueId=self.gait_selector_id, parentLinkIndex=i)

    def read_ui_controls(self):
        """Reads the current values from the GUI sliders."""
        if not self.gui or not self.debug_param_ids:
            # Return default values if no GUI or controls are present
            return {'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'body_height': 0.20, 'pitch': 0.0, 'roll': 0.0, 'step_height': 0.04, 'standoff': 0.28}

        controls = {
            'vx': p.readUserDebugParameter(self.debug_param_ids['vx']),
            'vy': p.readUserDebugParameter(self.debug_param_ids['vy']),
            'omega': p.readUserDebugParameter(self.debug_param_ids['omega']),
            'body_height': p.readUserDebugParameter(self.debug_param_ids['body_height']),
            'pitch': p.readUserDebugParameter(self.debug_param_ids['pitch']),
            'step_height': p.readUserDebugParameter(self.debug_param_ids['step_height']),
            'standoff': p.readUserDebugParameter(self.debug_param_ids['standoff'])
        }
        if 'roll' in self.debug_param_ids:
            controls['roll'] = p.readUserDebugParameter(self.debug_param_ids['roll'])
        return controls

    def read_gait_selection_ui(self):
        """Reads the selected gait index from the radio button UI."""
        if self.gait_selector_id is not None:
            return int(p.readUserDebugParameter(self.gait_selector_id))
        return 0

    def step(self):
        p.stepSimulation()

    def stop(self):
        p.disconnect()
        print("Simulation stopped.")

    def get_camera_image(self, camera_id, width=640, height=480):
        """
        Gets an image from a camera attached to the robot.
        :param camera_id: 0 for front camera, 1 for rear camera.
        :param width: Image width.
        :param height: Image height.
        :return: An RGB image as a numpy array.
        """
        if self.robot_id is None:
            return None

        # Get the position and orientation of the robot's base
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        rot_matrix = p.getMatrixFromQuaternion(base_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Define camera properties relative to the robot's body
        if camera_id == 0:  # Front camera
            # Positioned at the front, looking forward
            camera_offset = [0.2, 0, 0.05]
            target_offset = [1.0, 0, 0.0]
        elif camera_id == 1:  # Rear camera
            # Positioned at the back, looking backward
            camera_offset = [-0.2, 0, 0.05]
            target_offset = [-1.0, 0, 0.0]
        else:
            print(f"Warning: Invalid camera_id: {camera_id}")
            return None

        # Transform camera position and target to world coordinates
        camera_pos_world = base_pos + rot_matrix.dot(camera_offset)
        target_pos_world = base_pos + rot_matrix.dot(target_offset)
        
        # The camera's "up" vector is the Z-axis of the robot's body frame
        camera_up_vector_world = rot_matrix.dot([0, 0, 1])

        # Compute the view matrix
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos_world,
            cameraTargetPosition=target_pos_world,
            cameraUpVector=camera_up_vector_world
        )

        # Compute the projection matrix
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60.0, aspect=width/height, nearVal=0.1, farVal=10.0
        )

        # Get the camera image
        _, _, rgba_img, _, _ = p.getCameraImage(
            width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Convert RGBA to RGB
        return rgba_img[:, :, :3]

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
