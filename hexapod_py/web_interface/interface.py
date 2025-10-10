#!/usr/bin/env python3

import os
import sys
import time
import atexit

from flask import Flask, render_template, Response, jsonify
from gpiozero import Robot

# Add the project root to the Python path to allow importing from 'sensors'
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from sensors.gps import GPS
    from sensors.gyro import Gyro
    from sensors.camera import CameraSensor
except (ImportError, RuntimeError, ModuleNotFoundError) as e:
    print(f"Error importing sensor modules: {e}")
    print("This script requires the custom sensor classes (GPS, Gyro, CameraSensor).")
    # Create dummy classes for the app to run without hardware
    class GPS:
        def read(self): return (None, None)
    class Gyro:
        def read(self): return None
    class CameraSensor:
        def __init__(self, **kwargs): raise RuntimeError("Camera not available")

app = Flask(__name__, static_folder='static', template_folder='templates')

# --- Sensor Initialization ---
try:
    gps = GPS()
except Exception as e:
    print(f"Could not initialize GPS: {e}")
    gps = GPS() # Use dummy class

try:
    gyro = Gyro()
except Exception as e:
    print(f"Could not initialize Gyro: {e}")
    gyro = Gyro() # Use dummy class

cameras = {}
for cam_id in [0, 1]:
    try:
        cameras[cam_id] = CameraSensor(camera_id=cam_id)
        print(f"Camera {cam_id} initialized.")
    except Exception as e:
        print(f"Could not initialize Camera {cam_id}: {e}")

# --- Camera Streaming Setup ---
def generate_frames(cam_id):
    """Video streaming generator function using the CameraSensor class."""
    camera = cameras.get(cam_id)
    if not camera:
        return
    while True:
        time.sleep(1/30) # Limit framerate to avoid overwhelming the network
        frame = camera.capture_image()
        if frame is None:
            continue
        jpeg_bytes = camera.encode_to_jpeg(frame, quality=70)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --- Robot Control ---

# Define the GPIO pins for the robot motors
# Update these pins if your robot's wiring is different
robby = Robot(left=(20, 21), right=(19, 26))

@app.route('/')
def index():
    # The initial data is now just a placeholder, it will be filled by JavaScript
    return render_template('index.html')

@app.route('/move/<direction>')
def move(direction):
    if direction == 'forward': robby.forward()
    elif direction == 'backward': robby.backward()
    elif direction == 'left': robby.left()
    elif direction == 'right': robby.right()
    elif direction == 'stop': robby.stop()
    else: return ('Invalid command', 400)
    return f"Moved {direction}"

@app.route('/sensor_data')
def sensor_data():
    """Endpoint to fetch live sensor data as JSON."""
    gps_data, _ = gps.read() if gps else (None, None)
    gyro_data = gyro.read() if gyro else None
    return jsonify(gps_data=gps_data, gyro_data=gyro_data)

@app.route('/video_feed/<int:camera_id>')
def video_feed(camera_id):
    """Video streaming route for a given camera."""
    if camera_id not in cameras:
        return f"Camera {camera_id} not available", 503
    return Response(generate_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def cleanup():
    """Release resources on exit."""
    print("Shutting down... Releasing resources.")
    for cam_id, cam in cameras.items():
        cam.release()

if __name__ == '__main__':
    atexit.register(cleanup)
    app.run(host='0.0.0.0', port=80, debug=False, threaded=True)