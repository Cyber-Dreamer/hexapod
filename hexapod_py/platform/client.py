
import zmq
import threading
import msgpack
import numpy as np

# Define TCP socket addresses (must match the server)
JOINT_ANGLES_SOCKET_PATH = "tcp://127.0.0.1:5555"
IMU_DATA_SOCKET_PATH = "tcp://127.0.0.1:5556"
CAMERA_SOCKET_PATH = "tcp://127.0.0.1:5557"

class PlatformClient:
    """
    A client that connects to the standalone platform server process.
    It mimics the HexapodPlatform interface for compatibility with controllers.
    """
    def __init__(self):
        self.cam_socket_lock = threading.Lock()
        self.context = zmq.Context()
        
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(JOINT_ANGLES_SOCKET_PATH)

        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(IMU_DATA_SOCKET_PATH)
        # Subscribe to all sensor topics
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "sensor.imu")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "sensor.gps")

        self.cam_socket = self.context.socket(zmq.SUB)
        self.cam_socket.connect(CAMERA_SOCKET_PATH)
        self.cam_socket.setsockopt_string(zmq.SUBSCRIBE, "camera.front")
        self.cam_socket.setsockopt_string(zmq.SUBSCRIBE, "camera.rear")
        
        # --- Data Caching ---
        self._latest_sensor_data = {}
        self._latest_camera_frames = {}
        self._sensor_data_lock = threading.Lock()
        self._camera_frame_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._camera_thread = threading.Thread(target=self._camera_listener_thread, daemon=True)
        self._camera_thread.start()
        self._sensor_thread = threading.Thread(target=self._sensor_listener_thread, daemon=True)
        self._sensor_thread.start()

        print(f"PlatformClient connected to services.")

    def _sensor_listener_thread(self):
        """Listens for all subscribed sensor topics and caches the latest data."""
        poller = zmq.Poller()
        poller.register(self.sub_socket, zmq.POLLIN)

        while not self._stop_event.is_set():
            try:
                socks = dict(poller.poll(100)) # Poll with a timeout
                if self.sub_socket in socks:
                    # Drain the socket to get the most recent message if multiple are queued
                    while self.sub_socket in dict(poller.poll(0)):
                        topic_bytes, message_bytes = self.sub_socket.recv_multipart()
                        topic = topic_bytes.decode('utf-8')
                        data = msgpack.unpackb(message_bytes, raw=False)
                        with self._sensor_data_lock:
                            self._latest_sensor_data[topic] = data
            except zmq.error.ContextTerminated:
                break # Exit cleanly on shutdown

    def _camera_listener_thread(self):
        """Listens for camera frames and caches the latest one for each topic."""
        poller = zmq.Poller()
        poller.register(self.cam_socket, zmq.POLLIN)

        while not self._stop_event.is_set():
            try:
                socks = dict(poller.poll(100)) # Poll with a timeout
                if self.cam_socket in socks:
                    # Drain the socket to get the most recent frame if multiple are queued
                    while self.cam_socket in dict(poller.poll(0)):
                        topic_bytes, frame_bytes = self.cam_socket.recv_multipart()
                        with self._camera_frame_lock:
                            self._latest_camera_frames[topic_bytes] = frame_bytes
            except zmq.error.ContextTerminated:
                break # Exit cleanly on shutdown

    def set_joint_angles(self, joint_angles):
        """Sends joint angles to the platform server and waits for confirmation."""
        try:
            # Convert angles from radians to degrees for the servo controller
            angles_deg = [[np.rad2deg(angle) for angle in leg] for leg in joint_angles]

            # Construct the command message as a dictionary
            command_message = {
                'command': 'set_joint_angles',
                'angles': angles_deg
            }
            message = msgpack.packb(command_message)
            self.req_socket.send(message, zmq.NOBLOCK)
            if self.req_socket.poll(1000):
                self.req_socket.recv()
            else:
                print("Warning: No reply from platform server within 1 second.")
        except zmq.error.ZMQError as e:
            print(f"Error communicating with platform server: {e}")

    def angles_to_dict(self, joint_angles_list: list) -> dict:
        """
        Converts a list of joint angles into a dictionary format suitable for the UI.
        """
        joint_names = [
            'coxa', 'femur', 'tibia'
        ]
        joint_angles_dict = {}
        if joint_angles_list:
            for i, leg_angles in enumerate(joint_angles_list):
                for j, angle in enumerate(leg_angles):
                    joint_angles_dict[f'leg_{i}_{joint_names[j]}_joint'] = angle
        return joint_angles_dict

    def get_imu_data(self):
        """Performs a non-blocking read for the latest IMU data."""
        with self._sensor_data_lock:
            return self._latest_sensor_data.get("sensor.imu")

    def get_gps_data(self):
        """Performs a non-blocking read for the latest GPS data."""
        with self._sensor_data_lock:
            return self._latest_sensor_data.get("sensor.gps")
            
    def get_camera_image(self, camera_id):
        """
        Performs a non-blocking read for the latest cached camera frame.
        Returns the raw JPEG bytes.
        """
        target_topic = b"camera.front" if camera_id == 0 else b"camera.rear"
        with self._camera_frame_lock:
            return self._latest_camera_frames.get(target_topic)

    def start(self):
        pass

    def stop(self):
        self._stop_event.set()
        if self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=1)
        if self._camera_thread.is_alive():
            self._camera_thread.join(timeout=1)
        self.context.term()
