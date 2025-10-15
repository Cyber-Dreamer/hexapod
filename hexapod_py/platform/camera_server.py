import zmq
import cv2
import time
import argparse
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

try:
    # Attempt to import the hardware camera sensor
    from hexapod_py.platform.hardware.sensors.camera import CameraSensor
except (ImportError, RuntimeError):
    # If it fails (e.g., not on a Pi), set it to None
    CameraSensor = None

CAMERA_SOCKET_PATH = "tcp://*:5557"

def run_simulation_camera(socket):
    """Captures from a local webcam and publishes frames."""
    print("Starting camera server in SIMULATION mode (using local webcam).")
    cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cam.isOpened():
        print("!!! ERROR: Could not open webcam. Camera server will not stream.")
        return

    print("Webcam opened successfully. Streaming frames...")
    try:
        while True:
            success, frame = cam.read()
            if not success:
                time.sleep(0.1)
                continue

            # Encode the frame as JPEG
            is_success, jpeg_bytes = cv2.imencode('.jpg', frame)
            if not is_success:
                continue

            # Publish on both front and rear topics
            socket.send_multipart([b"camera.front", jpeg_bytes])

            # For the rear camera, send a flipped version
            flipped_frame = cv2.flip(frame, 1)
            is_success_flipped, jpeg_bytes_flipped = cv2.imencode('.jpg', flipped_frame)
            if is_success_flipped:
                socket.send_multipart([b"camera.rear", jpeg_bytes_flipped])

            time.sleep(1 / 30)  # Limit to ~30 FPS
    finally:
        cam.release()
        print("Webcam released.")

def run_physical_camera(socket):
    """Captures from PiCamera and publishes frames."""
    print("Starting camera server in PHYSICAL mode.")
    if CameraSensor is None:
        print("!!! ERROR: PiCamera library not found. Cannot run physical camera server.")
        return

    # This is a simplified example. You would expand this to handle two cameras.
    try:
        cam0 = CameraSensor(camera_id=0)
        # cam1 = CameraSensor(camera_id=1) # If you have a second camera
    except Exception as e:
        print(f"!!! ERROR: Failed to initialize physical cameras: {e}")
        return

    print("Physical cameras initialized. Streaming frames...")
    try:
        while True:
            frame0 = cam0.capture_image()
            if frame0 is not None:
                _, jpeg_bytes0 = cv2.imencode('.jpg', frame0)
                socket.send_multipart([b"camera.front", jpeg_bytes0])

            # Add logic for cam1 here if it exists

            time.sleep(1 / 30)
    finally:
        cam0.release()
        # cam1.release()
        print("Physical cameras released.")

def main():
    parser = argparse.ArgumentParser(description="Hexapod Camera Server")
    parser.add_argument("--platform", type=str, choices=["simulation", "physical"], required=True)
    args = parser.parse_args()

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(CAMERA_SOCKET_PATH)
    print(f"Camera server publishing on {CAMERA_SOCKET_PATH}")

    try:
        if args.platform == "simulation":
            run_simulation_camera(socket)
        elif args.platform == "physical":
            run_physical_camera(socket)
    except KeyboardInterrupt:
        print("Camera server shutting down...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()