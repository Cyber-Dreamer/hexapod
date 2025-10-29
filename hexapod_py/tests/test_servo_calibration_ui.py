"""
Interactive UI for calibrating individual servos on PCA9685 boards.

This script provides a graphical interface to:
1. Select which PCA9685 board to control (if multiple are present).
2. Enter a specific servo channel (0-15).
3. Activate/deactivate the servo control.
4. Use a slider to sweep the servo's angle, helping to identify the physical motor.

Press Ctrl+C in the terminal or close the matplotlib window to stop the script
and de-energize all servos.
"""

import time
import sys
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox, RadioButtons

# --- Configuration (mirrored from platform/hardware/servos.py for consistency) ---
FREQUENCY = 50  # Standard for most analog servos
SERVO_MIN_PULSE = 500   # Microseconds
SERVO_MAX_PULSE = 2500  # Microseconds
ACTUATION_RANGE = 270   # Degrees (e.g., for RDS5180SG servos)

# List of I2C addresses for your PCA9685 boards.
# The default is 0x40. Solder the address jumpers on your second board
# to set a unique address (e.g., 0x41).
PCA9685_ADDRESSES = [0x40, 0x41] # Default addresses to scan

# --- Global State ---
i2c_bus = None
pcas = {}  # Dictionary to store initialized PCA9685 objects: {address: PCA9685_object}
selected_pca_addr = None
current_servo = None
is_servo_active = False
active_channel_idx = -1

# --- UI Elements (global references for callbacks) ---
slider_angle = None
button_toggle = None
text_channel = None
radio_pca_addr = None
status_text_artist = None # Matplotlib Text artist for status messages

def setup_pca_boards():
    """Initializes PCA9685 boards based on configured addresses."""
    global i2c_bus, pcas, selected_pca_addr

    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        print("I2C bus initialized.")
    except Exception as e:
        print(f"Error initializing I2C bus: {e}", file=sys.stderr)
        return False

    for addr in PCA9685_ADDRESSES:
        try:
            pca = PCA9685(i2c_bus, address=addr)
            pca.frequency = FREQUENCY
            pcas[addr] = pca
            print(f"Initialized PCA9685 at address {hex(addr)}.")
        except (ValueError, OSError) as e:
            print(f"Warning: Could not initialize PCA9685 at address {hex(addr)}. Skipping. Error: {e}", file=sys.stderr)

    if not pcas:
        print("No PCA9685 boards found. Check wiring, power, and I2C addresses.", file=sys.stderr)
        return False
    
    # Set the first found PCA as the initially selected one
    selected_pca_addr = list(pcas.keys())[0]
    print(f"Initially selected PCA: {hex(selected_pca_addr)}")
    return True

def update_status(message):
    """Updates the status text in the UI."""
    if status_text_artist:
        status_text_artist.set_text(message)
        plt.draw()
    print(f"STATUS: {message}")

def on_pca_select(label):
    """Callback for PCA address radio buttons."""
    global selected_pca_addr
    
    new_addr = int(label, 16) # Convert hex string back to int
    if new_addr != selected_pca_addr:
        selected_pca_addr = new_addr
        # If a servo was active, de-activate it as the board changed
        if is_servo_active:
            deactivate_servo()
            update_ui_state()
    
    update_status(f"Selected PCA: {hex(selected_pca_addr)}. Channel: {active_channel_idx}. Ready to Start.")

def on_channel_submit(text):
    """Callback for channel text box."""
    global active_channel_idx
    try:
        channel = int(text)
        if 0 <= channel < 16:
            active_channel_idx = channel
            update_status(f"Channel set to: {active_channel_idx}. Ready to Start.")
        else:
            update_status("Invalid channel number. Must be between 0 and 15.")
    except ValueError:
        update_status("Invalid channel input. Please enter a number.")

def on_angle_change(val):
    """Callback for angle slider."""
    global current_servo
    if is_servo_active and current_servo:
        current_servo.angle = val
        update_status(f"Channel {active_channel_idx} angle: {val:.1f}°")

def activate_servo():
    """Activates the selected servo."""
    global current_servo, is_servo_active

    if selected_pca_addr is None or selected_pca_addr not in pcas:
        update_status("Error: No PCA9685 board selected or initialized.")
        return False

    if not (0 <= active_channel_idx < 16):
        update_status("Error: Please enter a valid channel number (0-15).")
        return False

    try:
        pca = pcas[selected_pca_addr]
        current_servo = servo.Servo(
            pca.channels[active_channel_idx],
            min_pulse=SERVO_MIN_PULSE,
            max_pulse=SERVO_MAX_PULSE,
            actuation_range=ACTUATION_RANGE
        )
        # Set to center initially
        current_servo.angle = ACTUATION_RANGE / 2
        slider_angle.set_val(ACTUATION_RANGE / 2)
        is_servo_active = True
        update_status(f"Servo on PCA {hex(selected_pca_addr)}, Channel {active_channel_idx} ACTIVATED. Angle: {ACTUATION_RANGE/2:.1f}°")
        return True
    except Exception as e:
        update_status(f"Error activating servo: {e}")
        return False

def deactivate_servo():
    """Deactivates the current servo."""
    global current_servo, is_servo_active
    if current_servo:
        current_servo.angle = None # De-energize
        current_servo = None
    is_servo_active = False
    update_status("Servo DEACTIVATED.")

def on_toggle_button_click(event):
    """Callback for Start/Stop button."""
    global is_servo_active
    if is_servo_active:
        deactivate_servo()
    else:
        activate_servo()
    update_ui_state()

def update_ui_state():
    """Updates UI elements based on servo active state."""
    if is_servo_active:
        button_toggle.label.set_text("Stop")
        slider_angle.eventson = True
        text_channel.eventson = False
        radio_pca_addr.eventson = False
    else:
        button_toggle.label.set_text("Start")
        slider_angle.eventson = False
        text_channel.eventson = True
        radio_pca_addr.eventson = True

def on_close(event):
    """Cleanup function when the plot window is closed."""
    print("\nClosing UI. De-initializing PCA9685 boards.")
    deactivate_servo() # Ensure current servo is de-energized
    for pca in pcas.values():
        pca.deinit()
    if i2c_bus:
        try:
            # Some busio.I2C objects might not have deinit, handle gracefully
            if hasattr(i2c_bus, 'deinit'):
                i2c_bus.deinit()
        except Exception as e:
            print(f"Warning during I2C bus deinitialization: {e}", file=sys.stderr)
    print("PCA9685 boards de-initialized. Bye!")

def main():
    global slider_angle, button_toggle, text_channel, radio_pca_addr, status_text_artist

    if not setup_pca_boards():
        print("Exiting due to PCA9685 initialization failure.")
        sys.exit(1)

    # --- UI Setup ---
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(8, 6))
    fig.patch.set_facecolor('#2E2E2E') # Dark grey background
    ax.set_facecolor('#2E2E2E')
    ax.set_title("Hexapod Servo Calibration UI", color='white')
    ax.set_xticks([]) # Hide axes ticks
    ax.set_yticks([])
    ax.spines['top'].set_visible(False) # Hide axes spines
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)

    # Define positions for UI elements
    ax_pca_radio = plt.axes([0.1, 0.8, 0.2, 0.15], facecolor='#444444')
    ax_channel_text = plt.axes([0.4, 0.85, 0.2, 0.05], facecolor='#444444')
    ax_button_toggle = plt.axes([0.65, 0.85, 0.15, 0.05], facecolor='#444444')
    ax_slider_angle = plt.axes([0.1, 0.7, 0.8, 0.05], facecolor='#444444')
    ax_status_display = plt.axes([0.1, 0.05, 0.8, 0.1], facecolor='#2E2E2E') # For status messages

    # PCA Address Radio Buttons
    pca_addr_labels = [hex(addr) for addr in pcas.keys()]
    radio_pca_addr = RadioButtons(ax_pca_radio, pca_addr_labels, active=0)
    radio_pca_addr.on_clicked(on_pca_select)
    ax_pca_radio.set_title("Select PCA Board", color='white', fontsize=10)

    # Channel Text Box
    text_channel = TextBox(
        ax_channel_text,
        'Channel (0-15): ',
        initial="0",
        color='#666666',          # Background color of the text box
        hovercolor='#777777',     # Background color when hovered
        label_pad=0.02            # Padding for the label
    )
    # The text color itself is inherited from the theme (white), so we just need to change the box color.
    text_channel.on_submit(on_channel_submit)
    active_channel_idx = int(text_channel.text) # Set initial channel

    # Start/Stop Button
    button_toggle = Button(ax_button_toggle, 'Start', color='#666666', hovercolor='#777777')
    # Set the button text color explicitly to ensure readability
    button_toggle.label.set_color('white')
    button_toggle.on_clicked(on_toggle_button_click)

    # Angle Slider
    slider_angle = Slider(ax_slider_angle, 'Angle (deg)', 0, ACTUATION_RANGE, valinit=ACTUATION_RANGE / 2, valstep=1)
    slider_angle.on_changed(on_angle_change)
    slider_angle.eventson = False # Initially disabled

    # Status Text Display
    status_text_artist = ax_status_display.text(0.5, 0.5, "",
                                      horizontalalignment='center', verticalalignment='center',
                                      fontsize=12, color='white', wrap=True)
    ax_status_display.set_xticks([]) # Hide axes ticks for status display
    ax_status_display.set_yticks([])
    ax_status_display.spines['top'].set_visible(False) # Hide axes spines for status display
    ax_status_display.spines['right'].set_visible(False)
    ax_status_display.spines['bottom'].set_visible(False)
    ax_status_display.spines['left'].set_visible(False)

    # Initial UI state update and status message
    update_ui_state()
    update_status(f"Ready. Selected PCA: {hex(selected_pca_addr)}. Channel: {active_channel_idx}. Enter channel and click Start.")

    # Register cleanup on window close
    fig.canvas.mpl_connect('close_event', on_close)

    plt.show()

if __name__ == "__main__":
    main()