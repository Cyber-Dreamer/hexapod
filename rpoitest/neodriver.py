import board
import busio
from adafruit_seesaw.seesaw import Seesaw
from adafruit_seesaw.neopixel import NeoPixel

# The default pin on the NeoDriver for the NeoPixel data line is 15
NEOPIXEL_PIN = 15

class NeoDriver:
    """
    A class to control the Adafruit NeoDriver using the adafruit-circuitpython-seesaw library.
    """

    def __init__(self, num_pixels, address=0x60, auto_write=False):
        """
        Initializes the NeoDriver controller.

        :param num_pixels: The number of NeoPixels in the strip.
        :param address: The I2C address of the NeoDriver.
        :param auto_write: Whether to automatically call show() after each pixel change.
        """
        # Use Blinka to get the correct I2C bus device.
        # This automatically uses the bus specified by the BLINKA_I2C environment variable.
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        
        # Initialize the seesaw object
        seesaw = Seesaw(i2c_bus, addr=address)

        # Initialize the NeoPixel object
        self.pixels = NeoPixel(seesaw, NEOPIXEL_PIN, num_pixels, auto_write=auto_write)

    @property
    def brightness(self):
        """
        Gets or sets the overall brightness of the strip (0.0 to 1.0).
        """
        return self.pixels.brightness
    
    @brightness.setter
    def brightness(self, value):
        self.pixels.brightness = value

    def set_pixel_color(self, n, r, g, b):
        """
        Sets the color of a single pixel. Call show() to make the change visible.
        """
        self.pixels[n] = (r, g, b)

    def show(self):
        """
        Updates the physical LED strip with the colors set previously.
        """
        self.pixels.show()

    def fill(self, r, g, b):
        """
        Fills the entire strip with a single color.
        """
        self.pixels.fill((r, g, b))