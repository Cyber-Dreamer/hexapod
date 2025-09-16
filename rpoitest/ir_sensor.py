import time
import RPi.GPIO as GPIO

# Use BCM numbering as in the video (GPIO 22 = physical pin 15)
IR_SENSOR_PIN = 15

def setup():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    GPIO.setup(IR_SENSOR_PIN, GPIO.IN)
    print(f"GPIO setup complete. IR_SENSOR_PIN={IR_SENSOR_PIN} set as INPUT (BCM mode).")

def main():
    setup()
    print("IR Obstacle Sensor Test (Press Ctrl+C to exit)")
    try:
        while True:
            if GPIO.input(IR_SENSOR_PIN) == 0:
                print("Obstacle detected!")
            else:
                print("No obstacle detected.")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()