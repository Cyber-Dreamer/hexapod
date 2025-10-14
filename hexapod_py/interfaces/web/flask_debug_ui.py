from flask import Flask, render_template_string
from sensors.gps import GPS
from sensors.gyro import Gyro
from sensors.ir_sensor import IRSensor

app = Flask(__name__)

gps = GPS()
gyro = Gyro()
ir = IRSensor()

HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Hexapod Sensor Debug UI</title>
    <meta http-equiv="refresh" content="2">
    <style>
        body { font-family: Arial, sans-serif; margin: 2em; }
        h1 { color: #333; }
        .sensor { margin-bottom: 1.5em; }
        .label { font-weight: bold; }
    </style>
</head>
<body>
    <h1>Hexapod Sensor Debug UI</h1>
    <div class="sensor">
        <span class="label">GPS:</span><br>
        {% if gps %}
            Latitude: {{ gps.lat }}<br>
            Longitude: {{ gps.lon }}<br>
            Altitude: {{ gps.alt }} m<br>
        {% else %}
            No GPS fix.
        {% endif %}
    </div>
    <div class="sensor">
        <span class="label">Gyro/Accel (MPU6050):</span><br>
        Accel: X={{ gyro.accel.x }} Y={{ gyro.accel.y }} Z={{ gyro.accel.z }}<br>
        Gyro: X={{ gyro.gyro.x }} Y={{ gyro.gyro.y }} Z={{ gyro.gyro.z }}<br>
    </div>
    <div class="sensor">
        <span class="label">IR Sensor:</span><br>
        {% if ir %}Obstacle detected!{% else %}No obstacle detected.{% endif %}
    </div>
</body>
</html>
'''

@app.route("/")
def index():
    gps_data = gps.read()
    gyro_data = gyro.read()
    ir_data = ir.read()
    return render_template_string(HTML, gps=gps_data, gyro=gyro_data, ir=ir_data)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
