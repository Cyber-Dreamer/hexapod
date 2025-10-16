import time
import board
import sys
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# PCA9685 Configuration
CHANNEL_COUNT = 16  # PCA9685 has 16 channels (0-15)
FREQUENCY = 50  # Standard frequency for servos

# Servo pulse width configuration (adjust these values for your specific servos)
# These are typical values for many hobby servos.
# Min pulse: 500us (0 degrees), Max pulse: 2500us (180 degrees)
# For a 50Hz frequency (20ms period), this means:
# 0 degrees: 0.5ms / 20ms = 0.025 duty cycle
# 180 degrees: 2.5ms / 20ms = 0.125 duty cycle
# The PCA9685 takes 16-bit values for duty cycle, so 0-65535.
# 0.025 * 65535 = 1638
# 0.125 * 65535 = 8192
# adafruit_motor.servo handles this conversion if you provide min_pulse and max_pulse in microseconds.
SERVO_MIN_PULSE = 500  # Microseconds
SERVO_MAX_PULSE = 2500 # Microseconds

# Set the actuation range in degrees based on the servo's specification sheet.
ACTUATION_RANGE = 270  # For RDS5180SG servos

def find_pca9685(i2c_bus):
    """Scans the I2C bus for a PCA9685 device."""
    print("Scanning for PCA9685 on the I2C bus...")
    addresses = i2c_bus.scan()
    # Common PCA9685 addresses are 0x40-0x7F
    for addr in addresses:
        if 0x40 <= addr <= 0x7F:
            try:
                # Attempt to initialize the device at the found address
                PCA9685(i2c_bus, address=addr)
                print(f"Found PCA9685 at address: {hex(addr)}")
                return addr
            except (ValueError, OSError):
                # This address is not a PCA9685, continue scanning
                pass
    return None

def test_servo_channel_0():
    """
    Initializes the PCA9685 and sweeps the servo on channel 0.
    """
    CHANNEL_TO_TEST = 0
    pca = None
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        
        pca_address = find_pca9685(i2c_bus)
        if pca_address is None:
            raise RuntimeError("Could not find a PCA9685 on the I2C bus. Check wiring and power.")

        pca = PCA9685(i2c_bus, address=pca_address)
        pca.frequency = FREQUENCY
        print(f"PCA9685 initialized. Frequency: {FREQUENCY}Hz")
        print(f"Beginning servo sweep for channel {CHANNEL_TO_TEST}. Press Ctrl+C to stop.")

        # Initialize the servo object for the specified channel
        my_servo = servo.Servo(
            pca.channels[CHANNEL_TO_TEST],
            min_pulse=SERVO_MIN_PULSE,
            max_pulse=SERVO_MAX_PULSE,
            actuation_range=ACTUATION_RANGE
        )
        print(f"Initialized servo object on channel {CHANNEL_TO_TEST}.")

        while True:
            print(f"\n--- Testing Channel {CHANNEL_TO_TEST} ---")

            # Sweep from 0 to ACTUATION_RANGE degrees
            print(f"\rSweeping to {ACTUATION_RANGE} degrees...", end="", flush=True)
            for angle in range(0, ACTUATION_RANGE + 1, 5): # Step by 5 for faster sweep
                my_servo.angle = angle
                time.sleep(0.01)
            # Sweep from ACTUATION_RANGE to 0 degrees
            print(f"\rSweeping to 0 degrees...      ", end="", flush=True)
            for angle in range(ACTUATION_RANGE, -1, -5):
                my_servo.angle = angle
                time.sleep(0.01)

    except (ValueError, OSError, RuntimeError) as e:
        print(f"\nError: Could not initialize PCA9685. Is it connected? {e}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        if pca:
            pca.deinit()
            print("PCA9685 de-initialized. Servos are now de-energized.")

if __name__ == "__main__":
    test_servo_channel_0()