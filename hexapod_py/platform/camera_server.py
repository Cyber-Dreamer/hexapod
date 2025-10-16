import zmq
import cv2
import time
import argparse
import sys
import os

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

try:
    # Attempt to import the hardware camera sensor
    from hexapod_py.platform.hardware.camera import CameraSensor
except (ImportError, RuntimeError):
    # If it fails (e.g., not on a Pi), set it to None
    CameraSensor = None

CAMERA_SOCKET_PATH = "tcp://*:5557"

def run_simulation_camera(socket):
    """Captures from a local webcam and publishes frames."""
    print("Starting camera server in SIMULATION mode.")
    from hexapod_py.platform.simulation.simulator import HexapodSimulator
    sim = HexapodSimulator(gui=False)
    sim.start()

    capture_func = lambda cam_id: sim.get_camera_image(cam_id, 1280, 720)

    print("Simulator camera client started. Streaming frames...")
    try:
        while True:
            for cam_id, topic in [(0, b'camera.front'), (1, b'camera.rear')]:
                frame = capture_func(cam_id)
                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    socket.send_multipart([topic, buffer.tobytes()])
            time.sleep(1 / 30)
    finally:
        sim.stop()
        print("Simulation for camera server stopped.")

def run_physical_camera(socket, width, height, quality):
    """Captures from PiCamera and publishes frames."""
    print("Starting camera server in HARDWARE mode.")
    if CameraSensor is None:
        print("!!! ERROR: PiCamera library not found. Cannot run physical camera server.")
        return

    cams = {}
    try:
        cams[0] = CameraSensor(camera_id=0, width=width, height=height)
        print(" - Front camera (ID 0) initialized.")
    except Exception as e:
        print(f"!!! WARNING: Could not initialize front camera (ID 0): {e}")

    try:
        cams[1] = CameraSensor(camera_id=1, width=width, height=height)
        print(" - Rear camera (ID 1) initialized.")
    except Exception as e:
        print(f"!!! WARNING: Could not initialize rear camera (ID 1): {e}")
    
    if not cams:
        print("!!! ERROR: No physical cameras could be initialized. Server is stopping.")
        return

    print("Physical cameras initialized. Streaming frames...")
    try:
        frame_count = 0
        while True:
            for cam_id, topic in [(0, b'camera.front'), (1, b'camera.rear')]:
                if cam_id in cams:
                    frame = cams[cam_id].capture_image()
                    if frame is not None:
                        jpeg_bytes = cams[cam_id].encode_to_jpeg(frame, quality=quality)
                        frame_count += 1
                        if frame_count % 100 == 0:
                            # This acts as a heartbeat to show the server is alive and sending
                            print(f"[CameraServer] Published frame #{frame_count}...")
                        if jpeg_bytes:
                            socket.send_multipart([topic, jpeg_bytes])

            time.sleep(1 / 30)
    finally:
        for cam in cams.values():
            cam.release()
        print("Physical cameras released.")

def main():
    parser = argparse.ArgumentParser(description="Hexapod Camera Server")
    parser.add_argument("--platform", type=str, choices=["simulation", "physical"], required=True)
    parser.add_argument("--width", type=int, default=1280, help="Capture width for physical cameras")
    parser.add_argument("--height", type=int, default=720, help="Capture height for physical cameras")
    parser.add_argument("--quality", type=int, default=75, help="JPEG quality for compression (1-100)")
    args = parser.parse_args()
    print(f"Camera server starting for platform: {args.platform}")

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(CAMERA_SOCKET_PATH)
    print(f"Camera server publishing on {CAMERA_SOCKET_PATH}")

    try:
        if args.platform == "simulation":
            run_simulation_camera(socket)
        elif args.platform == "physical":
            run_physical_camera(socket, args.width, args.height, args.quality)
    except KeyboardInterrupt:
        print("Camera server shutting down...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()