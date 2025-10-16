import time
import sys
import lgpio

# GPIO pins
DATA_PIN = 12
CLOCK_PIN = 21

# lgpio handles
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, DATA_PIN)
lgpio.gpio_claim_output(h, CLOCK_PIN)

class HX711:
    def __init__(self, dout, pd_sck):
        self.dout = dout
        self.pd_sck = pd_sck
        self.offset = 0
        self.scale = 1

        lgpio.gpio_write(h, self.pd_sck, 0)

    def is_ready(self):
        return lgpio.gpio_read(h, self.dout) == 0

    def read(self):
        while not self.is_ready():
            pass

        count = 0
        for i in range(24):
            lgpio.gpio_write(h, self.pd_sck, 1)
            lgpio.gpio_write(h, self.pd_sck, 0)
            count = count << 1
            if lgpio.gpio_read(h, self.dout):
                count += 1

        lgpio.gpio_write(h, self.pd_sck, 1)
        lgpio.gpio_write(h, self.pd_sck, 0)

        # Invert the 24-bit value
        if count & 0x800000:
            count = ~count & 0xFFFFFF
            count += 1
            count *= -1

        return count

    def tare(self, times=10):
        total = 0
        for _ in range(times):
            total += self.read()
        self.offset = total / times

    def get_weight(self, times=1):
        total = 0
        for _ in range(times):
            total += self.read()
        reading = (total / times) - self.offset
        return reading / self.scale

    def set_scale(self, scale):
        self.scale = scale

    def power_down(self):
        lgpio.gpio_write(h, self.pd_sck, 1)

    def power_up(self):
        lgpio.gpio_write(h, self.pd_sck, 0)

def cleanAndExit():
    print("Cleaning up...")
    lgpio.gpiochip_close(h)
    print("Bye!")
    sys.exit()

hx = HX711(dout=DATA_PIN, pd_sck=CLOCK_PIN)

print("Taring... please wait.")
hx.tare()
print("Tare done. Add weight now.")

# To set the reference unit, place a known weight on the scale and uncomment the following lines:
# known_weight_grams = 1000  # e.g., 1 kg
# raw_value = hx.get_weight(15) # get an average of 15 readings
# scale = raw_value / known_weight_grams
# hx.set_scale(scale)
# print(f"Scale set to: {scale}")

# A default scale for testing, you should calibrate this
hx.set_scale(92) # This is just an example value from your old script

# Set a threshold to determine if the leg is on the ground.
# You will need to experiment to find a suitable value for your setup.
# This value should be greater than the weight of the leg itself when lifted.
CONTACT_THRESHOLD = 100  # in grams

while True:
    try:
        val = hx.get_weight(5)
        is_on_ground = val > CONTACT_THRESHOLD
        print(f"On Ground: {is_on_ground}, Weight: {val:.2f} g")
        time.sleep(0.5) # Reduced sleep time for faster feedback

    except (KeyboardInterrupt, SystemExit):
        cleanAndExit()
