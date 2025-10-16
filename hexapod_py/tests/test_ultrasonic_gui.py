import time
import sys

try:
    from gpiozero import DistanceSensor
    from gpiozero.exc import GPIOZeroError
except ImportError:
    print("Error: gpiozero library not found.", file=sys.stderr)
    print("Please install it using: pip install gpiozero", file=sys.stderr)
    sys.exit(1)

# --- Configuration ---
ECHO_PIN = 20
TRIGGER_PIN = 21
MAX_DISTANCE_M = 4  # Max distance for HC-SR04 is ~4 meters

def measure_and_print(sensor):
    """Measures distance and prints the result to the terminal."""
    distance_cm = sensor.distance * 100

    if distance_cm < 30:  # 1 foot is ~30 cm
        message = "-> Too close!"
    else:
        message = "-> Clear"

    # Print to the same line using carriage return '\r' and flush=True
    print(f"\rDistance: {distance_cm:6.2f} cm {message:10}", end="", flush=True)

if __name__ == "__main__":
    sensor = None
    try:
        # max_distance implicitly sets the echo timeout.
        # queue_len averages readings for stability, which helps with longer distances.
        sensor = DistanceSensor(
            echo=ECHO_PIN,
            trigger=TRIGGER_PIN,
            max_distance=MAX_DISTANCE_M,
            queue_len=5,
            partial=True
        )
        print("Starting distance measurement... Press Ctrl+C to stop.")
        print("Waiting for sensor to settle...")
        time.sleep(2)

        while True:
            measure_and_print(sensor)
            time.sleep(0.2)  # Update 5 times per second

    except GPIOZeroError as e:
        print(f"\nError initializing sensor: {e}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    finally:
        if sensor:
            sensor.close()
            print("Sensor resources released.")