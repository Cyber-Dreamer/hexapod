"""
Sensor Worker Processes for the Hardware Platform

Each function in this module is designed to be run in its own separate process.
It initializes a specific sensor, reads data from it in a loop, and publishes
the data to a ZMQ PUB socket. This ensures that slow I/O from one sensor
does not block others or the main command server.
"""

import time
import zmq
import msgpack
import serial
from smbus2 import SMBus
import lgpio

# --- GPS Worker ---
def run_gps_sensor(push_socket_path, port='/dev/ttyAMA0', baudrate=38400):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect(push_socket_path)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"[GPS_WORKER] Error opening serial port {port}: {e}")
        return

    while True:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$GNGGA'):
            parts = line.split(',')
            if len(parts) > 9 and parts[2] and parts[4] and parts[9]:
                lat_raw, lat_dir = parts[2], parts[3]
                lon_raw, lon_dir = parts[4], parts[5]
                lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60
                if lat_dir == 'S': lat = -lat
                lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60
                if lon_dir == 'W': lon = -lon
                alt = float(parts[9])

                data = {'lat': lat, 'lon': lon, 'alt': alt}
                socket.send_multipart([b'sensor.gps', msgpack.packb(data)])
        time.sleep(0.5) # Don't spam the bus

# --- IMU (MPU6050) Worker ---
def run_imu_sensor(push_socket_path, i2c_bus=1, addr=0x68):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect(push_socket_path)

    try:
        bus = SMBus(i2c_bus)
        bus.write_byte_data(addr, 0x6B, 0) # Wake up MPU6050
    except (FileNotFoundError, OSError) as e:
        print(f"[IMU_WORKER] Error initializing MPU6050 on bus {i2c_bus}: {e}")
        return

    def read_raw(reg):
        high = bus.read_byte_data(addr, reg)
        low = bus.read_byte_data(addr, reg + 1)
        val = (high << 8) | low
        return val - 65536 if val > 32767 else val

    while True:
        try:
            data = {
                'accel': {
                    'x': read_raw(0x3B), 'y': read_raw(0x3B + 2), 'z': read_raw(0x3B + 4)
                },
                'gyro': {
                    'x': read_raw(0x43), 'y': read_raw(0x43 + 2), 'z': read_raw(0x43 + 4)
                }
            }
            socket.send_multipart([b'sensor.imu', msgpack.packb(data)])
        except OSError as e:
            # This can happen if there's a temporary I2C communication issue.
            # Log the error but don't crash the worker.
            print(f"[IMU_WORKER] Warning: Could not read from MPU6050: {e}")
        time.sleep(0.1) # 10 Hz update rate

# --- Leg Contact Sensor (HX711) Worker ---
def run_leg_contact_sensor(push_socket_path, leg_index, data_pin=12, clock_pin=21):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect(push_socket_path)

    try:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, data_pin)
        lgpio.gpio_claim_output(h, clock_pin)
    except Exception as e:
        print(f"[CONTACT_WORKER_LEG_{leg_index}] Error initializing lgpio: {e}")
        return

    def read_hx711():
        while lgpio.gpio_read(h, data_pin) != 0: pass
        count = 0
        for _ in range(24):
            lgpio.gpio_write(h, clock_pin, 1)
            lgpio.gpio_write(h, clock_pin, 0)
            count <<= 1
            if lgpio.gpio_read(h, data_pin): count += 1
        lgpio.gpio_write(h, clock_pin, 1)
        lgpio.gpio_write(h, clock_pin, 0)
        return count - 0x1000000 if count & 0x800000 else count

    CONTACT_THRESHOLD_RAW = 50000 # Example raw value threshold

    while True:
        raw_value = read_hx711()
        on_ground = raw_value > CONTACT_THRESHOLD_RAW

        data = {'leg_index': leg_index, 'on_ground': on_ground, 'raw_value': raw_value}
        topic = f'sensor.contact.{leg_index}'.encode('utf-8')
        socket.send_multipart([topic, msgpack.packb(data)])
        time.sleep(0.2)

    lgpio.gpiochip_close(h)