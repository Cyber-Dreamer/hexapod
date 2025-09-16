import time
import sys

try:
    import RPi.GPIO as GPIO
    from hx711 import HX711
except ImportError:
    print("This script must be run on a Raspberry Pi with the HX711 library installed.")
    sys.exit(1)

# Pin configuration (update if needed)
DT_PIN = 5   # HX711 data pin (DT)
SCK_PIN = 6  # HX711 clock pin (SCK)

def cleanAndExit():
    print("Cleaning up...")
    GPIO.cleanup()
    sys.exit()

def main():
    hx = HX711(DT_PIN, SCK_PIN)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(1)  # You must calibrate this value for your setup

    hx.reset()
    hx.tare()
    print("Tare done. Place weight on the scale.")

    try:
        while True:
            val = hx.get_weight(5)
            print("Weight: {} g".format(val))
            hx.power_down()
            hx.power_up()
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        cleanAndExit()

if __name__ == "__main__":
    main()
