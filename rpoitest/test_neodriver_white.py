import time
import sys
from neodriver import NeoDriver

# --- Configuration ---
NUM_PIXELS = 10 # The number of NeoPixels in your strip

def main():
    """
    Initializes the NeoDriver, sets all pixels to 100% white,
    and holds them there.
    """
    print("Starting NeoDriver test: 100% White")
    try:
        # Initialize the NeoDriver for your strip
        driver = NeoDriver(num_pixels=NUM_PIXELS)

        # Set brightness to maximum (1.0 = 100%)
        driver.brightness = 1.0
        print(f"Brightness set to {driver.brightness * 100}%")

        # Fill the entire strip with white (R=255, G=255, B=255)
        print("Setting all pixels to white...")
        driver.fill(255, 255, 255)
        driver.show()

        print("\nPixels are now white. Press Ctrl+C to turn off and exit.")
        while True:
            time.sleep(1)

    except (KeyboardInterrupt, SystemExit):
        print("\nExiting and turning off pixels.")
    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()