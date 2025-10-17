"""
Hexapod Webcam Camera Server (for Simulation)
================================

This script captures video from a local webcam and broadcasts the frames over a
ZMQ PUB socket. It acts as a lightweight camera source for simulation mode,
allowing UI and streaming tests without running the full PyBullet simulation.
"""

import zmq
import cv2
import time
import sys
import os

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

CAMERA_SOCKET_PATH = "tcp://*:5557"

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(CAMERA_SOCKET_PATH)
    print(f"Simulation camera server publishing on {CAMERA_SOCKET_PATH}")

    # Initialize webcam capture
    cap = cv2.VideoCapture(0) # Use the first available webcam
    if not cap.isOpened():
        print("!!! ERROR: Could not open webcam.", file=sys.stderr)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("Webcam capture started for simulation streaming.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Warning: Failed to grab frame from webcam.")
                time.sleep(0.5)
                continue

            # --- Front Camera (Original) ---
            # Encode the original frame and publish on the 'front' topic
            _, buffer_front = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            frame_bytes_front = buffer_front.tobytes()
            socket.send_multipart([b'camera.front', frame_bytes_front])

            # --- Rear Camera (Inverted) ---
            # Invert the frame colors to simulate a different camera
            inverted_frame = cv2.bitwise_not(frame)
            # Encode the inverted frame and publish on the 'rear' topic
            _, buffer_rear = cv2.imencode('.jpg', inverted_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            frame_bytes_rear = buffer_rear.tobytes()
            socket.send_multipart([b'camera.rear', frame_bytes_rear])

            time.sleep(1 / 30)
    except KeyboardInterrupt:
        print("Webcam camera server shutting down...")
    finally:
        cap.release()
        socket.close()
        context.term()
        print("Webcam released and server stopped.")

if __name__ == "__main__":
    main()