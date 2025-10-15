
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from sensors.camera import CameraSensor
from datetime import datetime

if __name__ == "__main__":
    os.makedirs("hexapod_py/pictures", exist_ok=True)
    for cam_id in [0, 1]:
        cam = CameraSensor(camera_id=cam_id)
        frame = cam.capture_image()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"hexapod_py/pictures/camera{cam_id}_{timestamp}.jpg"
        cam.save_image(frame, filename)
        print(f"Saved image from camera {cam_id} to {filename}")
        cam.release()
