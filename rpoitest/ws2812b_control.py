import time
from rpi_ws281x import PixelStrip, Color

# LED strip configuration:
LED_COUNT = 8         # Number of LED pixels.
LED_PIN = 14          # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10          # DMA channel to use for generating signal
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0

# Create PixelStrip object
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

def set_color(r, g, b):
    """Set all LEDs to the specified RGB color."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(r, g, b))
    strip.show()

if __name__ == '__main__':
    try:
        while True:
            set_color(255, 0, 0)   # Red
            time.sleep(1)
            set_color(0, 255, 0)   # Green
            time.sleep(1)
            set_color(0, 0, 255)   # Blue
            time.sleep(1)
    except KeyboardInterrupt:
        set_color(0, 0, 0)        # Turn off on exit
