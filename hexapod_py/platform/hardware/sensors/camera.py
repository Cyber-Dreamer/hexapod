
import cv2
import time
from threading import Thread, Lock
try:
    from picamera2 import Picamera2
except ImportError:
    # This will allow the code to be imported on non-RPi systems,
    # though it will fail if you try to instantiate the class.
    Picamera2 = None

class CameraSensor:
    def __init__(self, camera_id=0):
        if Picamera2 is None:
            raise RuntimeError("picamera2 library not found. Please install it for Raspberry Pi.")

        self.camera_id = camera_id
        self.picam2 = Picamera2(camera_num=self.camera_id)
        
        # Configure the camera for video streaming for better FPS
        # The main stream is used for the high-res capture, lores for a smaller preview if needed.
        # We can also control the buffer count to prevent stale frames.
        config = self.picam2.create_video_configuration(main={"size": (640, 480)}, controls={"FrameRate": 60}, buffer_count=4)
        self.picam2.configure(config)
        
        print(f"Starting camera {self.camera_id}...")
        self.picam2.start()
        # Give the camera a moment to adjust to light levels
        time.sleep(2)

        # Threading for continuous capture
        self.frame = None
        self.lock = Lock()
        self.running = True
        self.thread = Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        """Continuously captures frames from the camera in a background thread."""
        while self.running:
            # capture_array() is the blocking call that gets a new frame.
            # By running this in a loop, we always have the latest frame ready.
            captured_frame = self.picam2.capture_array()
            with self.lock:
                self.frame = captured_frame

    def capture_image(self):
        """Returns the most recent frame captured by the background thread."""
        with self.lock:
            if self.frame is None:
                return None # Not ready yet
            # OpenCV uses BGR, but picamera2 gives RGB. Convert color space.
            # We do the conversion here so the capture loop is as fast as possible.
            frame_bgr = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        return frame_bgr

    def save_image(self, frame, path):
        cv2.imwrite(path, frame)

    def encode_to_jpeg(self, frame, quality=90):
        """
        Encodes a captured frame (NumPy array) into a JPEG byte string.
        Requires opencv-python to be installed (`pip install opencv-python`).

        :param frame: The image frame (NumPy array) to encode.
        :param quality: The JPEG quality (0-100). Lower is more compressed.
        """
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        is_success, buffer = cv2.imencode(".jpg", frame, encode_param)
        if not is_success:
            raise RuntimeError("Could not encode frame to JPEG")
        return buffer.tobytes()

    def release(self):
        self.running = False
        self.thread.join()
        self.picam2.stop()
        self.picam2.close()
        print(f"Camera {self.camera_id} released.")
