
import zmq
import msgpack

# Define TCP socket addresses (must match the server)
JOINT_ANGLES_SOCKET_PATH = "tcp://127.0.0.1:5555"
IMU_DATA_SOCKET_PATH = "tcp://127.0.0.1:5556"

class PlatformClient:
    """
    A client that connects to the standalone platform server process.
    It mimics the HexapodPlatform interface for compatibility with controllers.
    """
    def __init__(self):
        self.context = zmq.Context()
        
        self.req_socket = self.context.socket(zmq.REQ)
        self.req_socket.connect(JOINT_ANGLES_SOCKET_PATH)

        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(IMU_DATA_SOCKET_PATH)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "sensor.imu")
        
        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)

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

    def start(self):
        pass

    def stop(self):
        self.req_socket.close()
        self.sub_socket.close()
        self.context.term()
