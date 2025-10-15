
import zmq
import threading
import msgpack

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
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "sensor.imu")

        self.cam_socket = self.context.socket(zmq.SUB)
        self.cam_socket.connect(CAMERA_SOCKET_PATH)
        self.cam_socket.setsockopt_string(zmq.SUBSCRIBE, "camera.front")
        self.cam_socket.setsockopt_string(zmq.SUBSCRIBE, "camera.rear")
        
        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)
        self.poller.register(self.cam_socket, zmq.POLLIN)

        print(f"PlatformClient connected to services.")

    def set_joint_angles(self, joint_angles):
        """Sends joint angles to the platform server and waits for confirmation."""
        try:
            message = msgpack.packb(joint_angles)
            self.req_socket.send(message, zmq.NOBLOCK)
            if self.req_socket.poll(1000):
                self.req_socket.recv()
            else:
                print("Warning: No reply from platform server within 1 second.")
        except zmq.error.ZMQError as e:
            print(f"Error communicating with platform server: {e}")

    def get_imu_data(self):
        """Performs a non-blocking read for the latest IMU data."""
        socks = dict(self.poller.poll(1))
        if self.sub_socket in socks:
            topic, message = self.sub_socket.recv_multipart()
            return msgpack.unpackb(message, raw=False)
        return None

    def get_camera_image(self, camera_id):
        """
        Performs a non-blocking read for the latest camera frame.
        Returns the raw JPEG bytes.
        """
        with self.cam_socket_lock:
            target_topic = b"camera.front" if camera_id == 0 else b"camera.rear"
            latest_frame = None

            # Check for new messages without blocking forever
            socks = dict(self.poller.poll(1))
            if self.cam_socket in socks:
                # We have at least one message. Drain the socket to get the most recent one.
                # The `poll(0)` will check for messages that are already queued.
                while self.cam_socket in dict(self.poller.poll(0)):
                    msg_topic, msg_data = self.cam_socket.recv_multipart()
                    # Keep the frame if it matches our target topic
                    if msg_topic == target_topic:
                        latest_frame = msg_data

            # Return the last frame found for the topic, or None if no new frame was available.
            return latest_frame

    def start(self):
        pass

    def stop(self):
        self.context.term()
