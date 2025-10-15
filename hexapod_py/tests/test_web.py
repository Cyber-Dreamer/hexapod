import os
import sys
import time
from flask import Flask, render_template_string, Response, jsonify
import atexit

# Add the project root to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from hardware.sensors.gps import GPS
    from hardware.sensors.gyro import Gyro
    from hardware.sensors.camera import CameraSensor
except (ImportError, RuntimeError, ModuleNotFoundError) as e:
    print(f"Error importing sensor modules: {e}")
    print("Please ensure all sensor dependencies are installed and you are on a Raspberry Pi.")
    sys.exit(1)

app = Flask(__name__)

# --- Sensor Initialization ---
try:
    gps = GPS()
except Exception as e:
    print(f"Could not initialize GPS: {e}")
    gps = None

try:
    gyro = Gyro()
except Exception as e:
    print(f"Could not initialize Gyro: {e}")
    gyro = None

cameras = {}
for cam_id in [0, 1]:
    try:
        cameras[cam_id] = CameraSensor(camera_id=cam_id)
        print(f"Camera {cam_id} initialized.")
    except Exception as e:
        print(f"Could not initialize Camera {cam_id}: {e}")

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Hexapod Sensor Web Test</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 2em; background-color: #f4f4f4; color: #333; }
        h1 { color: #0056b3; }
        .container { display: flex; flex-wrap: wrap; gap: 2em; }
        .sensor-card { background: white; padding: 1em; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); min-width: 300px; }
        .camera-feed { width: 640px; height: 480px; border: 1px solid #ddd; background: #eee; }
        .label { font-weight: bold; color: #0056b3; }
    </style>
    <script>
        function updateSensorData() {
            fetch('/sensor_data')
                .then(response => response.json())
                .then(data => {
                    const gpsDiv = document.getElementById('gps-data');
                    if (data.gps_data) {
                        gpsDiv.innerHTML = `
                            Latitude: ${data.gps_data.lat}<br>
                            Longitude: ${data.gps_data.lon}<br>
                            Altitude: ${data.gps_data.alt} m<br>
                        `;
                    } else {
                        gpsDiv.innerHTML = 'No GPS fix or GPS not available.';
                    }

                    const gyroDiv = document.getElementById('gyro-data');
                    if (data.gyro_data) {
                        gyroDiv.innerHTML = `
                            Accel: X=${data.gyro_data.accel.x} Y=${data.gyro_data.accel.y} Z=${data.gyro_data.accel.z}<br>
                            Gyro: X=${data.gyro_data.gyro.x} Y=${data.gyro_data.gyro.y} Z=${data.gyro_data.gyro.z}<br>
                        `;
                    } else {
                        gyroDiv.innerHTML = 'Gyro not available.';
                    }
                });
        }
        setInterval(updateSensorData, 2000); // Update every 2 seconds
    </script>
</head>
<body>
    <h1>Hexapod Sensor Web Test</h1>
    <div class="container">
        <div class="sensor-card">
            <span class="label">GPS:</span><br>
            {% if gps_data %}
                <div id="gps-data">Loading GPS data...</div>
            {% else %}
                <div id="gps-data">No GPS fix or GPS not available.</div>
            {% endif %}
        </div>
        <div class="sensor-card">
            <span class="label">Gyro/Accel (MPU6050):</span><br>
            <div id="gyro-data">{% if gyro_data %}Loading Gyro data...{% else %}Gyro not available.{% endif %}</div>
        </div>
    </div>
    <h2>Camera Feeds</h2>
    <div class="container">
        {% for cam_id in cameras %}
        <div class="sensor-card">
             <span class="label">Camera {{ cam_id }}:</span><br>
             <img class="camera-feed" src="{{ url_for('video_feed', cam_id=cam_id) }}">
        </div>
        {% endfor %}
    </div>
</body>
</html>
"""

def gen_frames(cam_id):
    """Video streaming generator function."""
    camera = cameras.get(cam_id)
    if not camera:
        return
    while True:
        time.sleep(1/60) # Limit to ~60fps to avoid overwhelming the client/network
        frame = camera.capture_image()
        if frame is None:
            continue
        # Encode with 70% quality for a good balance of compression and quality
        jpeg_bytes = camera.encode_to_jpeg(frame, quality=70)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')

@app.route('/')
def index():
    gps_data, _ = gps.read() if gps else (None, None)
    gyro_data = gyro.read() if gyro else None
    return render_template_string(HTML, gps_data=gps_data, gyro_data=gyro_data, cameras=cameras.keys())

@app.route('/sensor_data')
def sensor_data():
    """Endpoint to fetch sensor data as JSON."""
    gps_data, _ = gps.read() if gps else (None, None)
    gyro_data = gyro.read() if gyro else None
    return jsonify(gps_data=gps_data, gyro_data=gyro_data)

@app.route('/video_feed/<int:cam_id>')
def video_feed(cam_id):
    return Response(gen_frames(cam_id), mimetype='multipart/x-mixed-replace; boundary=frame')

def cleanup():
    """Release resources on exit."""
    print("Shutting down... Releasing resources.")
    for cam_id, cam in cameras.items():
        cam.release()

if __name__ == "__main__":
    atexit.register(cleanup)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
