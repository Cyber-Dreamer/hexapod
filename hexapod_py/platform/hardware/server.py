"""
Hexapod Hardware Platform Server
================================

This script runs on the Raspberry Pi and acts as the bridge between the high-level
control software and the robot's actual hardware (servos, sensors).

It operates on two main ZeroMQ sockets:
1. A REP (Reply) socket for receiving commands and sending back confirmations.
   This is used for actions like moving legs.
2. A PUB (Publish) socket for broadcasting sensor data continuously. This allows
   any part of the system to subscribe to sensor feeds without blocking the
   command server.

Each major sensor (GPS, IMU, etc.) is run in its own process to prevent I/O
blocking and ensure real-time performance.
"""

import sys
import os
import time
import zmq
import multiprocessing
import threading
import msgpack

# Add project root to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from hexapod_py.platform.hardware.servos import ServoController
from hexapod_py.platform.hardware.sensor_workers import run_gps_sensor, run_imu_sensor

# --- ZMQ Configuration ---
COMMAND_SOCKET_PATH = "tcp://*:5555"
SENSOR_SOCKET_PATH = "tcp://*:5556"
WORKER_SOCKET_PATH = "inproc://sensor_workers"

def sensor_data_forwarder(context):
    """
    Pulls data from worker processes and publishes it on the main sensor socket.
    """
    pull_socket = context.socket(zmq.PULL)
    pull_socket.bind(WORKER_SOCKET_PATH)

    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(SENSOR_SOCKET_PATH)
    print(f"Broadcasting sensor data on {SENSOR_SOCKET_PATH}")
    zmq.proxy(pull_socket, pub_socket)

def main():
    """
    Initializes and runs the hardware platform server.
    """
    print("--- Hexapod Hardware Platform Server ---")

    # --- Initialize ZMQ Sockets ---
    context = zmq.Context()

    # Command socket (REP)
    command_socket = context.socket(zmq.REP)
    command_socket.bind(COMMAND_SOCKET_PATH)
    print(f"Listening for commands on {COMMAND_SOCKET_PATH}")

    # Start the sensor data forwarder in a separate thread
    # This proxy will handle pulling data from workers and publishing it
    forwarder_thread = threading.Thread(target=sensor_data_forwarder, args=(context,), daemon=True)
    forwarder_thread.start()

    # --- Initialize Hardware Controllers ---
    try:
        servo_controller = ServoController()
        print("Servo controller initialized.")
    except Exception as e:
        print(f"FATAL: Could not initialize servo controller: {e}", file=sys.stderr)
        print("The platform server cannot run without servos. Exiting.", file=sys.stderr)
        sys.exit(1)

    # --- Start Sensor Processes ---
    sensor_processes = []
    print("Starting sensor processes...")

    # GPS Sensor Process
    try:
        gps_process = multiprocessing.Process(target=run_gps_sensor, args=(WORKER_SOCKET_PATH,), daemon=True)
        gps_process.start()
        sensor_processes.append(gps_process)
        print(" - GPS sensor process started.")
    except Exception as e:
        print(f"Warning: Could not start GPS sensor process: {e}", file=sys.stderr)

    # IMU (Gyro/Accel) Sensor Process - using msgpack
    try:
        imu_process = multiprocessing.Process(target=run_imu_sensor, args=(WORKER_SOCKET_PATH,), daemon=True)
        imu_process.start()
        sensor_processes.append(imu_process)
        print(" - IMU sensor process started.")
    except Exception as e:
        print(f"Warning: Could not start IMU sensor process: {e}", file=sys.stderr)

    # # Leg Contact Sensor Process (HX711) - Disabled
    # try:
    #     contact_process = multiprocessing.Process(target=run_leg_contact_sensor, args=(WORKER_SOCKET_PATH, 0), daemon=True)
    #     contact_process.start()
    #     sensor_processes.append(contact_process)
    #     print(" - Leg contact sensor process started for leg 0.")
    # except Exception as e:
    #     print(f"Warning: Could not start leg contact sensor process: {e}", file=sys.stderr)

    # --- Main Command Loop ---
    print("\nServer is running. Waiting for commands...")
    try:
        while True:
            try:
                # Commands now use msgpack for consistency
                message_bytes = command_socket.recv(flags=zmq.NOBLOCK)
                message = msgpack.unpackb(message_bytes, raw=False)

                command = message.get('command')
                response = {'status': 'error', 'message': 'Unknown command'}

                if command == 'set_joint_angles': # Match client command
                    angles_deg = message.get('angles')
                    if angles_deg is not None:
                        servo_controller.set_all_leg_angles(angles_deg)
                        response = {'status': 'ok', 'message': 'Leg angles set.'}
                    else:
                        response = {'status': 'error', 'message': 'Missing "angles" data.'}
                
                # Send confirmation back
                command_socket.send(b"OK")

            except zmq.Again:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, shutting down server...")

    finally:
        # --- Cleanup ---
        print("Cleaning up resources...")

        for p in sensor_processes:
            if p.is_alive():
                print(f"Terminating process {p.name} (PID: {p.pid})...")
                p.terminate()
                p.join(timeout=2)

        if 'servo_controller' in locals() and servo_controller:
            servo_controller.deinit()
            print("Servos de-energized.")

        command_socket.close()
        context.term()
        print("Server shutdown complete.")

if __name__ == "__main__":
    main()