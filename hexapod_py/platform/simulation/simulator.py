
import pybullet as p
import time
import pybullet_data
import os
import numpy as np
import threading
import sys
import zmq
import msgpack

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
from hexapod_py.platform.hexapod_platform import HexapodPlatform

# Define TCP socket addresses
JOINT_ANGLES_SOCKET_PATH = "tcp://127.0.0.1:5555"
IMU_DATA_SOCKET_PATH = "tcp://127.0.0.1:5556"

class HexapodSimulator(HexapodPlatform):
    def __init__(self, gui=True):
        super().__init__()
        self.gui = gui
        self.physics_client = None
        self.robot_id = None
        self.joint_name_to_id = {}
        self.link_name_to_id = {}
        self._is_running = False
        self._simulation_thread = None
        self._latest_imu_data = None
        self._last_sensor_update_time = 0

        self.leg_joint_names = [
            ["hip_6", "ties_6", "foot_6"], # RL
            ["hip_1", "ties_1", "foot_1"], # ML
            ["hip_2", "ties_2", "foot_2"], # FL
            ["hip_3", "ties_3", "foot_3"], # FR
            ["hip_4", "ties_4", "foot_4"], # MR
            ["hip_5", "ties_5", "foot_5"], # RR
        ]
        self.package_path = os.path.dirname(__file__)

        # Set the initial pose to be flat (all joints at 0).
        # The power-on sequence from the UI will handle standing up.
        self.target_joint_angles = [[0.0, 0.0, 0.0]] * 6

    def start(self):
        self.physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=2.0)
        p.setAdditionalSearchPath(self.package_path)
        self.robot_id = p.loadURDF("robot.urdf", basePosition=[0, 0, 0.4])
        self._map_joint_names_to_ids()
        self._set_foot_friction()

        self._is_running = True
        self._simulation_thread = threading.Thread(target=self._simulation_and_sensor_loop)
        self._simulation_thread.start()
        print(f"Simulation started. Robot ID: {self.robot_id}")

    def _simulation_and_sensor_loop(self):
        """
        Runs the simulation and publishes sensor data in a dedicated thread.
        """
        context = zmq.Context()
        imu_socket = context.socket(zmq.PUB)
        imu_socket.bind(IMU_DATA_SOCKET_PATH)
        print(f"Sensor publisher thread started, publishing on {IMU_DATA_SOCKET_PATH}")

        try:
            while self._is_running:
                self._apply_joint_angles()
                self._update_sensors()
                p.stepSimulation()

                if self._latest_imu_data:
                    topic = b"sensor.imu"
                    message = msgpack.packb(self._latest_imu_data)
                    imu_socket.send_multipart([topic, message])

                time.sleep(1./240.)
        finally:
            imu_socket.close()
            context.term()
            print("Sensor publisher thread stopped.")

    def run_command_server(self):
        """
        Runs the ZMQ server for receiving commands in the main thread.
        """
        context = zmq.Context()
        joint_socket = context.socket(zmq.REP)
        joint_socket.bind(JOINT_ANGLES_SOCKET_PATH)
        print(f"Command server started, listening on {JOINT_ANGLES_SOCKET_PATH}")

        try:
            while self._is_running:
                # Block and wait for a command
                message = joint_socket.recv()
                command_message = msgpack.unpackb(message, raw=False)

                # Process the command
                if command_message.get('command') == 'set_joint_angles':
                    # The client now sends angles in degrees. The simulation needs radians.
                    angles_deg = command_message.get('angles')
                    if angles_deg:
                        angles_rad = [[np.deg2rad(angle) for angle in leg] for leg in angles_deg]
                        self.set_joint_angles(angles_rad)
                    joint_socket.send(b"OK")
                else:
                    # Respond with an error if the command is unknown
                    joint_socket.send(b"Unknown command")

        except zmq.error.ContextTerminated:
            print("Command server context terminated.")
        finally:
            joint_socket.close()
            # Don't terminate context here if it's still needed by the other thread
            print("Command server stopped.")

    def stop(self):
        if self._is_running:
            self._is_running = False
            # The command server will exit its loop. We need to unblock it.
            # A clean way is to connect a client and send a final message,
            # but for simplicity, we'll rely on the process being killed.
            if self._simulation_thread:
                self._simulation_thread.join()
            p.disconnect()
            print("Simulation stopped.")

    def _map_joint_names_to_ids(self):
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('UTF-8')
            self.joint_name_to_id[joint_name] = joint_info[0]
            link_name = joint_info[12].decode('UTF-8')
            self.link_name_to_id[link_name] = joint_info[0]

    def get_imu_data(self):
        """
        Returns the latest IMU data.
        This method is required by the HexapodPlatform abstract base class
        for in-process use cases.
        """
        return self._latest_imu_data

    def _update_sensors(self):
        if self.robot_id is None: return
        try:
            current_time = time.time()
            dt = current_time - self._last_sensor_update_time if self._last_sensor_update_time > 0 else 0.0
            self._last_sensor_update_time = current_time

            linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
            self._latest_imu_data = {
                'accel': {'x': linear_velocity[0], 'y': linear_velocity[1], 'z': linear_velocity[2]},
                'gyro':  {'x': angular_velocity[0], 'y': angular_velocity[1], 'z': angular_velocity[2]},
                'dt': dt
            }
        except p.error:
            self._latest_imu_data = None

    def _set_foot_friction(self, lateral_friction=2.0, spinning_friction=0.5, rolling_friction=0.1):
        if not self.robot_id: return
        foot_links = [name for name in self.link_name_to_id.keys() if 'foot_link' in name]
        for link_name in foot_links:
            link_id = self.link_name_to_id[link_name]
            p.changeDynamics(self.robot_id, link_id, lateralFriction=lateral_friction,
                             spinningFriction=spinning_friction, rollingFriction=rolling_friction)

    def _apply_joint_angles(self):
        STALL_TORQUE_NM = 7.85
        MAX_VELOCITY_RAD_S = 10.0
        KP = 0.2
        KD = 0.8
        for leg_idx, leg_angles in enumerate(self.target_joint_angles):
            if leg_angles is not None:
                for joint_idx, angle in enumerate(leg_angles):
                    joint_name = self.leg_joint_names[leg_idx][joint_idx]
                    if joint_name in self.joint_name_to_id:
                        p.setJointMotorControl2(
                            bodyIndex=self.robot_id,
                            jointIndex=self.joint_name_to_id[joint_name],
                            controlMode=p.POSITION_CONTROL, targetPosition=angle,
                            force=STALL_TORQUE_NM, maxVelocity=MAX_VELOCITY_RAD_S,
                            positionGain=KP, velocityGain=KD)

    def get_camera_image(self, camera_id, width=640, height=480):
        return self._capture_camera_image(camera_id, width, height)

    def get_front_camera_image(self, width=640, height=480):
        return self.get_camera_image(0, width, height)

    def get_rear_camera_image(self, width=640, height=480):
        return self.get_camera_image(1, width, height)

    def _capture_camera_image(self, camera_id, width=640, height=480):
        if self.robot_id is None: return None
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        rot_matrix = np.array(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)
        if camera_id == 0: # Front
            camera_offset, target_offset = [0.2, 0, 0.05], [1.0, 0, 0.0]
        elif camera_id == 1: # Rear
            camera_offset, target_offset = [-0.2, 0, 0.05], [-1.0, 0, 0.0]
        else: return None
        camera_pos_world = base_pos + rot_matrix.dot(camera_offset)
        target_pos_world = base_pos + rot_matrix.dot(target_offset)
        camera_up_vector_world = rot_matrix.dot([0, 0, 1])
        view_matrix = p.computeViewMatrix(cameraEyePosition=camera_pos_world,
                                          cameraTargetPosition=target_pos_world,
                                          cameraUpVector=camera_up_vector_world)
        projection_matrix = p.computeProjectionMatrixFOV(fov=60.0, aspect=width/height, nearVal=0.1, farVal=10.0)
        _, _, rgba_img, _, _ = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_TINY_RENDERER)
        return rgba_img[:, :, :3]

if __name__ == "__main__":
    sim = HexapodSimulator(gui=True)
    try:
        # Start the simulation and sensor publishing in a background thread
        sim.start()
        # Run the command server in the main thread
        sim.run_command_server()
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, shutting down...")
    finally:
        sim.stop()
