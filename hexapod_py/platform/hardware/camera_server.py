"""
Hexapod Hardware Camera Server
==============================

This script runs on the Raspberry Pi and is responsible for capturing images
from the physical PiCamera modules and broadcasting them over a ZMQ PUB socket.
"""

import zmq
import time
import argparse
import sys
import os

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

try:
    from hexapod_py.platform.hardware.camera import CameraSensor
except (ImportError, RuntimeError):
    CameraSensor = None

CAMERA_SOCKET_PATH = "tcp://*:5557"

def main():
    parser = argparse.ArgumentParser(description="Hexapod Hardware Camera Server")
    parser.add_argument("--width", type=int, default=1280, help="Capture width for physical cameras")
    parser.add_argument("--height", type=int, default=720, help="Capture height for physical cameras")
    parser.add_argument("--quality", type=int, default=75, help="JPEG quality for compression (1-100)")
    args = parser.parse_args()

    if CameraSensor is None:
        print("!!! ERROR: PiCamera library not found. Cannot run physical camera server.", file=sys.stderr)
        sys.exit(1)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(CAMERA_SOCKET_PATH)
    print(f"Hardware camera server publishing on {CAMERA_SOCKET_PATH}")

    cams = {}
    try:
        cams[0] = CameraSensor(camera_id=0, width=args.width, height=args.height)
        cams[1] = CameraSensor(camera_id=1, width=args.width, height=args.height)
        print("Physical cameras initialized. Streaming frames...")
    except Exception as e:
        print(f"!!! ERROR: Could not initialize all physical cameras: {e}", file=sys.stderr)
        if not cams:
            sys.exit(1)

    try:
        while True:
            for cam_id, topic in [(0, b'camera.front'), (1, b'camera.rear')]:
                if cam_id in cams:
                    frame = cams[cam_id].capture_image()
                    if frame is not None:
                        jpeg_bytes = cams[cam_id].encode_to_jpeg(frame, quality=args.quality)
                        if jpeg_bytes:
                            socket.send_multipart([topic, jpeg_bytes])
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        print("Hardware camera server shutting down...")
    finally:
        for cam in cams.values():
            cam.release()
        socket.close()
        context.term()
        print("Physical cameras released.")

if __name__ == "__main__":
    main()