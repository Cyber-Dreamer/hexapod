import time
import board
import sys
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# PCA9685 Configuration
CHANNEL_COUNT = 16  # PCA9685 has 16 channels (0-15)
FREQUENCY = 50  # Standard frequency for servos

# List of I2C addresses for your PCA9685 boards.
# The default is 0x40. Solder the address jumpers on your second board
# to set a unique address (e.g., 0x41).
PCA9685_ADDRESSES = [0x40, 0x41]

# Servo pulse width configuration (adjust these values for your specific servos)
SERVO_MIN_PULSE = 500  # Microseconds
SERVO_MAX_PULSE = 2500 # Microseconds

# Set the actuation range in degrees based on the servo's specification sheet.
ACTUATION_RANGE = 270  # For RDS5180SG servos

def test_all_servos_center():
    """
    Initializes all PCA9685 boards and wiggles each servo +-10 degrees
    around its center position, one by one.
    """
    pcas = {}  # Use a dictionary to store pca objects with their addresses
    all_servos = {} # Use a dictionary to store servo objects, keyed by address
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)

        # Initialize all PCA9685 boards
        for addr in PCA9685_ADDRESSES:
            try:
                pca = PCA9685(i2c_bus, address=addr)
                pca.frequency = FREQUENCY
                pcas[addr] = pca
                print(f"Initialized PCA9685 at address {hex(addr)} with {FREQUENCY}Hz frequency.")
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

        print("Beginning servo centering test. Press Ctrl+C to stop.")

        center_angle = ACTUATION_RANGE / 2
        wiggle = 10

        while True:
            for board_idx, (addr, board_servos) in enumerate(all_servos.items()):
                for channel_idx, current_servo in enumerate(board_servos):
                    print(f"\rTesting Board {board_idx} (Addr: {hex(addr)}), Channel {channel_idx}...  ", end="", flush=True)
                    current_servo.angle = center_angle - wiggle
                    time.sleep(0.5)
                    current_servo.angle = center_angle + wiggle
                    time.sleep(0.5)
                    current_servo.angle = center_angle # Return to center before next servo
                    time.sleep(1.0) # Pause before moving to the next servo

    except (ValueError, OSError, RuntimeError) as e:
        print(f"\nError: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        for pca in pcas.values():
            pca.deinit()
        print("\nAll PCA9685 boards de-initialized. Servos are now de-energized.")

if __name__ == "__main__":
    test_all_servos_center()