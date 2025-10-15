import pybullet as p
import time
import pybullet_data
import os
import numpy as np
import threading
import sys

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from hexapod_py.platform.hexapod_platform import HexapodPlatform

class HexapodSimulator(HexapodPlatform):
    def __init__(self, gui=True):
        super().__init__() # Initialize the base class (sets up target_joint_angles)
        self.gui = gui
        self.physics_client = None
        self.robot_id = None
        self.joint_name_to_id = {}
        self.link_name_to_id = {}
        self.debug_param_ids = {}
        self.gait_selector_id = None
        self._is_running = False
        self._simulation_thread = None
        self._latest_imu_data = None # Cache for sensor data

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
        p.changeDynamics(plane_id, -1, lateralFriction=2.0)

        # Add the 'simulation' directory to the search path.
        p.setAdditionalSearchPath(self.package_path)

        # Load URDF model
        self.robot_id = p.loadURDF("robot.urdf", basePosition=[0, 0, 0.4])
        self._map_joint_names_to_ids()
        self._set_foot_friction()

        # Start the simulation loop in a separate thread
        self._is_running = True
        self._simulation_thread = threading.Thread(target=self._simulation_loop)
        self._simulation_thread.start()

        print(f"Simulation started. Robot ID: {self.robot_id}")

    def _simulation_loop(self):
        """
        The main loop for the simulation, running in a separate thread.
        It applies motor commands and steps the physics engine at a fixed rate.
        """
        while self._is_running:
            # Update motors and sensors, then step the simulation
            self._apply_joint_angles()
            self._update_sensors()
            p.stepSimulation()
            time.sleep(1./240.)

    def _update_sensors(self):
        """
        Internal method to fetch sensor data from PyBullet at each simulation step
        and cache it.
        """
        if self.robot_id is None:
            return

        # Update IMU data
        try:
            linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
            self._latest_imu_data = {
                'accel': {'x': linear_velocity[0], 'y': linear_velocity[1], 'z': linear_velocity[2]},
                'gyro':  {'x': angular_velocity[0], 'y': angular_velocity[1], 'z': angular_velocity[2]}
            }
        except p.error:
            # If there's an error, set data to None to indicate it's stale
            self._latest_imu_data = None

    def _map_joint_names_to_ids(self):
        """Creates maps from joint/link names to their PyBullet IDs."""
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('UTF-8')
            self.joint_name_to_id[joint_name] = joint_info[0]
            link_name = joint_info[12].decode('UTF-8')
            # The link index is the same as the joint index it's a child of.
            self.link_name_to_id[link_name] = joint_info[0]

    def _set_foot_friction(self, lateral_friction=2.0, spinning_friction=0.5, rolling_friction=0.1):
        """
        Increases the friction of the foot links to prevent slipping.
        The URDF uses 'foot_#' for tibia joints, so the links are named 'foot_link_#'.
        """
        if not self.robot_id:
            print("Error: Robot not loaded, cannot set foot friction.")
            return

        foot_links = [name for name in self.link_name_to_id.keys() if 'foot_link' in name]
        for link_name in foot_links:
            link_id = self.link_name_to_id[link_name]
            p.changeDynamics(
                self.robot_id, 
                link_id, 
                lateralFriction=lateral_friction,
                spinningFriction=spinning_friction,
                rollingFriction=rolling_friction
            )
        if not foot_links:
            print("Warning: No 'foot_link' links found to set friction. Robot may be unstable.")

    def _apply_joint_angles(self):
        """
        Applies the stored `self.target_joint_angles` to the PyBullet simulation
        motors. This is the concrete implementation for the simulator.
        """
        # --- YOUR SERVO'S PROFILE ---
        STALL_TORQUE_NM = 7.85      # CORRECTED: 80 kg-cm converted to Nm
        MAX_VELOCITY_RAD_S = 10.0   # A reasonable guess for a powerful motor (tune if you know the speed spec)
        KP = 0.2                    # Proportional gain (start lower for high-torque motors)
        KD = 0.8                    # Derivative gain (high damping is good for stability)
        # ----------------------------------------

        for leg_idx, leg_angles in enumerate(self.target_joint_angles):
            if leg_angles is not None:
                for joint_idx, angle in enumerate(leg_angles):
                    joint_name = self.leg_joint_names[leg_idx][joint_idx]
                    if joint_name in self.joint_name_to_id:
                        p.setJointMotorControl2(
                            bodyIndex=self.robot_id,
                            jointIndex=self.joint_name_to_id[joint_name],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=STALL_TORQUE_NM,
                            maxVelocity=MAX_VELOCITY_RAD_S,
                            positionGain=KP,
                            velocityGain=KD
                        )
                    else:
                        print(f"Warning: Joint '{joint_name}' not found in URDF.")

    # The set_joint_angles method is now inherited from HexapodPlatform and
    # simply updates self.target_joint_angles.

    def stop(self):
        if self._is_running:
            self._is_running = False
            if self._simulation_thread:
                self._simulation_thread.join() # Wait for the thread to finish
            self._simulation_thread = None
            p.disconnect()
            print("Simulation stopped.")

    def get_camera_image(self, camera_id, width=640, height=480):
        return self._capture_camera_image(camera_id, width, height)

    def get_front_camera_image(self, width=640, height=480):
        return self.get_camera_image(0, width, height)

    def get_rear_camera_image(self, width=640, height=480):
        return self.get_camera_image(1, width, height)

    def _capture_camera_image(self, camera_id, width=640, height=480):
        """
        Internal method to capture an image from a virtual camera.
        
        :param camera_id: 0 for front, 1 for rear.
        :param width: Image width.
        :param height: Image height.
        :return: A numpy array of the image or None.
        """
        if self.robot_id is None:
            return None
        
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        rot_matrix = p.getMatrixFromQuaternion(base_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        if camera_id == 0:  # Front camera
            camera_offset = [0.2, 0, 0.05]
            target_offset = [1.0, 0, 0.0]
        elif camera_id == 1:  # Rear camera
            camera_offset = [-0.2, 0, 0.05]
            target_offset = [-1.0, 0, 0.0]
        else:
            print(f"Warning: Invalid camera_id: {camera_id}")
            return None

        camera_pos_world = base_pos + rot_matrix.dot(camera_offset)
        target_pos_world = base_pos + rot_matrix.dot(target_offset)
        camera_up_vector_world = rot_matrix.dot([0, 0, 1])

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos_world,
            cameraTargetPosition=target_pos_world,
            cameraUpVector=camera_up_vector_world
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60.0, aspect=width/height, nearVal=0.1, farVal=10.0
        )
        _, _, rgba_img, _, _ = p.getCameraImage(
            width, height, view_matrix, projection_matrix, renderer=p.ER_TINY_RENDERER
        )
        return rgba_img[:, :, :3]

    def get_imu_data(self):
        """
        Retrieves the latest cached IMU data.

        The data is updated in the background by the simulation loop. This method
        provides a non-blocking way to access the most recent sensor readings.
        """
        return self._latest_imu_data

# Example of running the simulator in isolation
if __name__ == "__main__":
    sim = HexapodSimulator(gui=True)
    sim.start()

    # Example: Set legs to a "stand" position
    # The angles are [coxa, femur, tibia] for each of the 6 legs
    stand_angles = [
        [0.0, np.deg2rad(30), np.deg2rad(-90)],  # Leg 0
        [0.0, np.deg2rad(30), np.deg2rad(-90)],  # Leg 1
        [0.0, np.deg2rad(30), np.deg2rad(-90)],  # Leg 2
        [0.0, np.deg2rad(30), np.deg2rad(-90)],  # Leg 3
        [0.0, np.deg2rad(30), np.deg2rad(-90)],  # Leg 4
        [0.0, np.deg2rad(30), np.deg2rad(-90)]   # Leg 5
    ]
    sim.set_joint_angles(stand_angles) # Set the target state

    try:
        # Run simulation indefinitely until the user closes the window or presses Ctrl+C
        print("Simulator running. Holding stand pose. Press Ctrl+C to stop.")
        while True:
            # The simulation now runs in a background thread.
            # This main loop is only for keeping the script alive.
            # In a real application, this loop would contain control logic.
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()