"""
Test script to set all servos to their center (neutral) position and hold.

This is useful for initial hardware assembly, calibration, and ensuring all
servos are responding correctly before running complex locomotion.
"""

import time
import board
import sys
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration (mirrored from platform/hardware/servos.py for consistency) ---
FREQUENCY = 50
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500
ACTUATION_RANGE = 270
PCA9685_ADDRESSES = [0x40, 0x41]
CHANNEL_COUNT = 16

def hold_all_servos_at_center():
    """
    Initializes all PCA9685 boards and sets all servos to their center angle,
    holding them there until the script is stopped.
    """
    pcas = {}
    all_servos = {}
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)

        # Initialize all PCA9685 boards
        for addr in PCA9685_ADDRESSES:
            try:
                pca = PCA9685(i2c_bus, address=addr)
                pca.frequency = FREQUENCY
                pcas[addr] = pca
                print(f"Initialized PCA9685 at address {hex(addr)}.")
            except (ValueError, OSError) as e:
                print(f"Warning: Could not initialize PCA9685 at address {hex(addr)}. Skipping. Error: {e}", file=sys.stderr)

        if not pcas:
            raise RuntimeError("No PCA9685 boards found. Check wiring, power, and I2C addresses.")

        # Initialize servo objects for all channels on each board
        for addr, pca in pcas.items():
            board_servos = []
            for channel_idx in range(CHANNEL_COUNT):
                board_servos.append(servo.Servo(
                    pca.channels[channel_idx],
                    min_pulse=SERVO_MIN_PULSE,
                    max_pulse=SERVO_MAX_PULSE,
                    actuation_range=ACTUATION_RANGE
                ))
            all_servos[addr] = board_servos
            print(f"Initialized {CHANNEL_COUNT} servo objects for board at {hex(addr)}.")

        center_angle = ACTUATION_RANGE / 2
        print(f"\nSetting all servos to center position ({center_angle} degrees)...")
        for addr, board_servos in all_servos.items():
            for current_servo in board_servos:
                # First, set angle to None to ensure the next command is sent.
                current_servo.angle = None
                current_servo.angle = center_angle

        print(f"All servos set to {center_angle} degrees. Holding position. Press Ctrl+C to stop and de-energize.")
        while True:
            time.sleep(1)

    except (ValueError, OSError, RuntimeError) as e:
        print(f"\nError: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        for pca in pcas.values():
            pca.deinit()
        print("\nAll PCA9685 boards de-initialized. Servos are now de-energized.")

if __name__ == "__main__":
    hold_all_servos_at_center()