"""
Test script to de-energize all servos, making the robot go limp ("relax").

This is useful for safely powering down the robot or for manual manipulation
of the legs without fighting against the servo motors. It finds all configured
PCA9685 boards and calls deinit() on them, which releases all servo channels.
"""

import board
import sys
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration (mirrored from platform/hardware/servos.py for consistency) ---
PCA9685_ADDRESSES = [0x40, 0x41]
CHANNEL_COUNT = 16 # All PCA9685 boards have 16 channels

# These values are needed to instantiate adafruit_motor.servo.Servo objects
# even if we just set angle=None. They are mirrored from servos.py.
SERVO_MIN_PULSE = 500   # Microseconds
SERVO_MAX_PULSE = 2500  # Microseconds
ACTUATION_RANGE = 270   # Degrees

def release_all_servos():
    """
    Initializes all PCA9685 boards, explicitly de-energizes all channels
    by setting their angle to None, and then de-initializes the boards.
    """
    pcas = [] # List of initialized PCA objects
    all_servos = [] # List of lists, one for each board's servo objects
    print("Attempting to release all servos...")
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)

        # Find all PCA9685 boards
        for addr in PCA9685_ADDRESSES:
            try:
                pca = PCA9685(i2c_bus, address=addr)
                # No need to set frequency if we're just de-energizing
                pcas.append(pca)
                print(f"Found PCA9685 at address {hex(addr)}.")

                # Create servo objects for all channels on this board
                board_servos = []
                for channel_idx in range(CHANNEL_COUNT):
                    board_servos.append(
                        servo.Servo(
                            pca.channels[channel_idx],
                            min_pulse=SERVO_MIN_PULSE,
                            max_pulse=SERVO_MAX_PULSE,
                            actuation_range=ACTUATION_RANGE
                        )
                    )
                all_servos.append(board_servos)
                print(f"Prepared {CHANNEL_COUNT} servo objects for board at {hex(addr)}.")
            except (ValueError, OSError):
                # This is not an error, just means a board isn't connected at that address
                print(f"No PCA9685 found at address {hex(addr)}. Skipping.")

        if not pcas:
            print("No PCA9685 boards found to release.", file=sys.stderr)
            return

        print("\nExplicitly de-energizing all servo channels...")
        for board_servos in all_servos:
            for current_servo in board_servos:
                current_servo.angle = None # This is the key change
        print("All servo channels commanded to de-energize.")

    except Exception as e:
        print(f"\nAn error occurred during I2C or servo initialization: {e}", file=sys.stderr)
    finally:
        # Ensure all PCA boards are de-initialized to release I2C resources
        for pca in pcas:
            pca.deinit()
        print("\nAll found PCA9685 boards de-initialized. Servos should now be completely relaxed.")

if __name__ == "__main__":
    release_all_servos()