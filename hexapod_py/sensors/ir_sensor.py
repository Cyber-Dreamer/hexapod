import time
import RPi.GPIO as GPIO

class IRSensor:
    def __init__(self, pin=15):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)

    def read(self):
        return GPIO.input(self.pin) == 0  # True if obstacle detected

    def cleanup(self):
        GPIO.cleanup()
