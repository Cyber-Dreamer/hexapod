import os
import sys
import time
import threading
from flask import Flask, render_template_string, Response, jsonify
import atexit
import lgpio

# Add the project root to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, project_root)

try:
    from hexapod_py.platform.hardware.sensors.gps import GPS
    from hexapod_py.platform.hardware.sensors.gyro import Gyro
    from hexapod_py.platform.hardware.sensors.camera import CameraSensor
except (ImportError, RuntimeError, ModuleNotFoundError) as e:
    print(f"Error importing sensor modules: {e}")
    # Non-fatal, the web server can run without them

# --- HX711 Weight Sensor Code ---
# GPIO pins
DATA_PIN = 12
CLOCK_PIN = 21

# lgpio handles
try:
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(h, DATA_PIN)
    lgpio.gpio_claim_output(h, CLOCK_PIN)
    print("lgpio initialized for HX711.")
except Exception as e:
    h = None
    print(f"Could not initialize lgpio for HX711: {e}")

class HX711:
    def __init__(self, dout, pd_sck):
        self.dout = dout
        self.pd_sck = pd_sck
        self.offset = 0
        self.scale = 1
        if h:
            lgpio.gpio_write(h, self.pd_sck, 0)

    def is_ready(self):
        if not h: return False
        return lgpio.gpio_read(h, self.dout) == 0

    def read(self):
        if not h: return 0
        while not self.is_ready():
            pass
        count = 0
        for i in range(24):
            lgpio.gpio_write(h, self.pd_sck, 1)
            lgpio.gpio_write(h, self.pd_sck, 0)
            count = count << 1
            if lgpio.gpio_read(h, self.dout):
                count += 1
        lgpio.gpio_write(h, self.pd_sck, 1)
        lgpio.gpio_write(h, self.pd_sck, 0)
        if count & 0x800000:
            count = ~count & 0xFFFFFF
            count += 1
            count *= -1
        return count

    def tare(self, times=10):
        if not h: return
        total = 0
        for _ in range(times):
            total += self.read()
        self.offset = total / times

    def get_weight(self, times=1):
        if not h: return 0
        total = 0
        for _ in range(times):
            total += self.read()
        reading = (total / times) - self.offset
        return reading / self.scale

    def set_scale(self, scale):
        self.scale = scale

# --- Global variables for sensor data ---
latest_weight_data = {"weight": 0, "on_ground": False}
data_lock = threading.Lock()

# --- Weight Sensor Thread ---
def weight_sensor_thread():
    if not h:
        print("Weight sensor not initialized, thread exiting.")
        return

    hx = HX711(dout=DATA_PIN, pd_sck=CLOCK_PIN)
    print("Taring weight sensor...")
    hx.tare()
    print("Tare done.")
    hx.set_scale(92)  # You should calibrate this!
    CONTACT_THRESHOLD = 100 # Grams

    while True:
        val = hx.get_weight(5)
        is_on_ground = val > CONTACT_THRESHOLD
        with data_lock:
            latest_weight_data["weight"] = f"{val:.2f}"
            latest_weight_data["on_ground"] = is_on_ground
        time.sleep(0.5)

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

                    const weightDiv = document.getElementById('weight-data');
                    if (data.weight_data) {
                        weightDiv.innerHTML = `
                            On Ground: ${data.weight_data.on_ground}<br>
                            Weight: ${data.weight_data.weight} g
                        `;
                    } else {
                        weightDiv.innerHTML = 'Weight sensor not available.';
                    }
                });
        }
        setInterval(updateSensorData, 1000); // Update every second
    </script>
</head>
<body>
    <h1>Hexapod Sensor Web Test</h1>
    <div class="container">
        <div class="sensor-card">
            <span class="label">GPS:</span><br>
            <div id="gps-data">Loading GPS data...</div>
        </div>
        <div class="sensor-card">
            <span class="label">Gyro/Accel (MPU6050):</span><br>
            <div id="gyro-data">Loading Gyro data...</div>
        </div>
        <div class="sensor-card">
            <span class="label">Leg Weight Sensor:</span><br>
            <div id="weight-data">Loading weight data...</div>
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
        time.sleep(1/60)
        frame = camera.capture_image()
        if frame is None: continue
        jpeg_bytes = camera.encode_to_jpeg(frame, quality=70)
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template_string(HTML, cameras=cameras.keys())

@app.route('/sensor_data')
def sensor_data():
    """Endpoint to fetch sensor data as JSON."""
    gps_data, _ = gps.read() if gps else (None, None)
    gyro_data = gyro.read() if gyro else None
    with data_lock:
        weight_data_copy = latest_weight_data.copy()
    return jsonify(gps_data=gps_data, gyro_data=gyro_data, weight_data=weight_data_copy)

@app.route('/video_feed/<int:cam_id>')
def video_feed(cam_id):
    return Response(gen_frames(cam_id), mimetype='multipart/x-mixed-replace; boundary=frame')

def cleanup():
    """Release resources on exit."""
    print("Shutting down... Releasing resources.")
    for cam_id, cam in cameras.items():
        cam.release()
    if h:
        lgpio.gpiochip_close(h)

if __name__ == "__main__":
    # Start the weight sensor thread
    w_thread = threading.Thread(target=weight_sensor_thread, daemon=True)
    w_thread.start()

    atexit.register(cleanup)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
