"""
Hardware Camera Sensor Module for Raspberry Pi

This module provides a CameraSensor class that interfaces with the PiCamera2
library to capture and handle images from the Raspberry Pi's camera modules.
"""

import cv2
import time
from threading import Thread, Lock

try:
    from picamera2 import Picamera2
except ImportError:
    # This allows the code to be imported on non-Pi systems without crashing.
    # Any attempt to instantiate the class will fail gracefully.
    print("Warning: picamera2 library not found. Camera will not be available.")
    Picamera2 = None

class CameraSensor:
    def __init__(self, camera_id=0, width=1280, height=720):
        if Picamera2 is None:
            raise RuntimeError("Cannot initialize CameraSensor: picamera2 library is not installed.")

        self.camera_id = camera_id
        self.picam2 = Picamera2(camera_num=self.camera_id)
        
        config = self.picam2.create_video_configuration(
            main={"size": (width, height)},
            controls={"FrameRate": 30},
            buffer_count=4 # Use a small buffer to reduce latency
        )
        self.picam2.configure(config)
        
        print(f"Starting camera {self.camera_id} at {width}x{height}...")
        self.picam2.start()
        time.sleep(2) # Allow camera to warm up

        # Threading for continuous capture
        self.frame = None
        self.lock = Lock()
        self.running = True
        self.thread = Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        """Continuously captures frames from the camera in a background thread."""
        while self.running:
            captured_frame = self.picam2.capture_array()
            with self.lock:
                self.frame = captured_frame

    def capture_image(self):
        """Captures a single frame and returns it as a NumPy array (BGR format)."""
        with self.lock:
            if self.frame is None:
                return None # Not ready yet
            # OpenCV uses BGR, but picamera2 gives RGB. Convert color space.
            frame_bgr = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        return frame_bgr

    def encode_to_jpeg(self, frame, quality=90):
        """Encodes a NumPy array frame to a JPEG byte string."""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        is_success, buffer = cv2.imencode(".jpg", frame, encode_param)
        if not is_success:
            print(f"Warning: Could not encode frame to JPEG for camera {self.camera_id}")
            return None
        return buffer.tobytes()

    def release(self):
        """Stops the camera and releases resources."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.picam2:
            self.picam2.stop()
            self.picam2.close()
        print(f"Camera {self.camera_id} released.")