import unittest
import time
from gpiozero import DistanceSensor
from gpiozero.exc import GPIOPinInUse

class TestUltrasonicRangeSensor(unittest.TestCase):
    sensor = None

    @classmethod
    def setUpClass(cls):
        try:
            # gpiozero handles all the pin setup automatically.
            # It uses BCM pin numbering by default.
            cls.sensor = DistanceSensor(echo=20, trigger=21)
            print("Waiting for sensor to settle...")
            time.sleep(2) # Give sensor time to settle
        except GPIOPinInUse:
            raise RuntimeError("GPIO pins 20 or 21 are already in use.")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize DistanceSensor: {e}")

    @classmethod
    def tearDownClass(cls):
        if cls.sensor:
            cls.sensor.close()

    @unittest.skipIf(sensor is None, "DistanceSensor could not be initialized.")
    def test_distance_measurement(self):
        print("Taking a distance measurement...")
        # .distance returns meters, so we multiply by 100 for cm
        distance = self.sensor.distance * 100
        print(f"Measured distance: {distance:.2f} cm")
        self.assertGreater(distance, 2)  # Should be >2cm (sensor min range)
        self.assertLess(distance, 400)   # Should be <400cm (sensor max range)

if __name__ == "__main__":
    unittest.main()
