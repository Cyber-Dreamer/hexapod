import os
import sys
from datetime import datetime

# Add the project root to the Python path to allow importing from 'sensors'
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from hardware.sensors.camera import CameraSensor
except (ImportError, RuntimeError) as e:
    print(f"Could not import CameraSensor: {e}")
    print("Please ensure picamera2 is installed (`sudo apt install python3-picamera2`) and you are running on a Raspberry Pi.")
    sys.exit(1)

if __name__ == "__main__":
    # Create a directory to store pictures if it doesn't exist
    output_dir = "pictures"
    os.makedirs(output_dir, exist_ok=True)

    # Test both cameras (0 and 1 for a dual-camera setup)
    for cam_id in [0, 1]:
        try:
            print(f"--- Testing Camera {cam_id} ---")
            cam = CameraSensor(camera_id=cam_id)
            frame = cam.capture_image()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(output_dir, f"camera{cam_id}_{timestamp}.jpg")

            cam.save_image(frame, filename)
            print(f"Successfully saved image from camera {cam_id} to {filename}")
            cam.release()
        except Exception as e:
            print(f"Error with camera {cam_id}: {e}")
        print("-" * (20 + len(str(cam_id))))
