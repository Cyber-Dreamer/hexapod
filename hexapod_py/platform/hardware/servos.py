"""
Servo Controller for the Hexapod's Hardware Platform

This module abstracts the control of all 18 servos for the hexapod legs
using the Adafruit PCA9685 library. It assumes a single 16-channel board
is used for the first 16 servos and provides a structure to add more boards.
"""

import board
import busio
import threading
import time
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration ---
I2C_BUS = busio.I2C(board.SCL, board.SDA)
FREQUENCY = 50  # Standard for most analog servos

# These values are based on your test script. Fine-tune them for your specific servos.
UPDATE_RATE_HZ = 200    # How many times per second to update servo positions
SERVO_MIN_PULSE = 500   # Microseconds
SERVO_MAX_PULSE = 2500  # Microseconds
ACTUATION_RANGE = 270   # Degrees

# --- Multi-board Configuration ---
# List of I2C addresses for your PCA9685 boards.
# The default is 0x40. Solder the address jumpers on your second board
# to set a unique address (e.g., 0x41).
PCA9685_ADDRESSES = [0x40, 0x41]

# --- Kinematic Joint Limits (in degrees) ---
# This defines the safe travel range for each joint type relative to its
# calibrated center position (kinematic zero).
# Format: [coxa_limit, femur_limit, tibia_limit]
# e.g., a coxa_limit of 90 means the coxa can move from -90 to +90 degrees.
KINEMATIC_JOINT_LIMITS = [90, 110, 120]

# --- Servo Centering/Calibration ---
#
# !!! CRITICAL HARDWARE CONFIGURATION !!!
#
# This is the most important calibration step for your robot.
#
# `SERVO_CENTER_DEGREES` defines the raw angle (0-270) for each servo that
# corresponds to its kinematic "zero" position. A value of 135 is a theoretical
# midpoint for a 270-degree servo, but you MUST adjust these values for each
# individual servo to correct for mechanical misalignments during assembly.
#
# HOW TO CALIBRATE:
# 1. Run the `test_servo_calibration_ui.py` script on your Raspberry Pi.
# 2. For each joint, use the UI to find the angle value that makes the joint
#    physically centered in its neutral kinematic pose.
# 3. Update the corresponding value in the array below with the value you found.
#
# Example: If you find that the Rear-Left coxa servo is centered at an angle of
# 138.5 degrees, you would change `SERVO_CENTER_DEGREES[0][0]` to `138.5`.
#
# Structure: [leg][joint] -> center_angle
# joint_index: 0=coxa, 1=femur, 2=tibia
# Leg Numbering (matches locomotion logic):
# 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
SERVO_CENTER_DEGREES = [
    [135, 135, 135],    # Leg 0 (Rear-Left)
    [135, 135, 135],    # Leg 1 (Middle-Left)
    [135, 135, 135],    # Leg 2 (Front-Left)
    [135, 135, 135],    # Leg 3 (Front-Right)
    [135, 135, 135],    # Leg 4 (Middle-Right)
    [135, 135, 135],    # Leg 5 (Rear-Right)
]
# Mapping of leg and joint to a (board_index, channel_index) tuple.
# The board_index corresponds to the order in PCA9685_ADDRESSES.
# board_index 0 -> 0x40
# board_index 1 -> 0x41
# joint_index: 0=coxa, 1=femur, 2=tibia
# Leg Numbering (matches locomotion logic):
#
# !!! CRITICAL HARDWARE CONFIGURATION !!!
#
# This map defines your physical wiring. Double-check that each leg joint
# is connected to the correct channel on the correct PCA9685 board.
# An incorrect mapping will cause the wrong leg to move.
# 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
LEG_CHANNEL_MAP = [
    [(0, 4), (0, 10), (0, 6)],   # Leg 0 (Rear-Left) -> Board 0 (0x40)
    [(1, 7), (1, 9), (1, 11)],   # Leg 1 (Middle-Left) -> Board 1 (0x41)
    [(1, 15), (1, 13), (1, 14)], # Leg 2 (Front-Left) -> Board 1 (0x41)
    [(1, 5), (1, 6), (1, 10)],   # Leg 3 (Front-Right) -> Board 1 (0x41)
    [(0, 2), (0, 1), (0, 9)],    # Leg 4 (Middle-Right) -> Board 0 (0x40)
    [(0, 5), (0, 8), (0, 0)],    # Leg 5 (Rear-Right) -> Board 0 (0x40)
]

class ServoController:
    def __init__(self):
        self.boards = []
        for addr in PCA9685_ADDRESSES:
            try:
                board = PCA9685(I2C_BUS, address=addr)
                board.frequency = FREQUENCY
                self.boards.append(board)
                print(f"Initialized PCA9685 at address {hex(addr)}")
            except (ValueError, OSError) as e:
                raise RuntimeError(f"Could not initialize PCA9685 at address {hex(addr)}. Check wiring and address jumpers.") from e

        # Create a nested list of servo objects, one for each board
        self.servos = [[] for _ in self.boards]
        for board_idx, board in enumerate(self.boards):
            for channel_idx in range(16): # Initialize all 16 channels on each board
                self.servos[board_idx].append(
                    servo.Servo(
                        board.channels[channel_idx],
                        min_pulse=SERVO_MIN_PULSE,
                        max_pulse=SERVO_MAX_PULSE,
                        actuation_range=ACTUATION_RANGE
                    )
                )

        # --- Threading for non-blocking updates ---
        # Target angles are stored in a thread-safe way.
        # The `set_all_leg_angles` method will update this array, and a background
        # thread will read from it to command the physical servos.
        self._target_angles_deg = np.zeros((6, 3))
        self._lock = threading.Lock()
        self._is_running = True
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        print(f"Servo update thread started (rate: {UPDATE_RATE_HZ} Hz).")

    def _set_physical_leg_angle(self, leg_index, joint_index, angle_deg):
        """
        PRIVATE: Directly sets the angle of a single physical servo.
        This is called by the internal update loop.
        """
        # 1. Apply kinematic limits to the requested angle.
        #    This ensures we don't command the joint beyond its safe mechanical range.
        limit = KINEMATIC_JOINT_LIMITS[joint_index]
        limited_angle_deg = max(-limit, min(limit, angle_deg))

        # 2. Get the mapping and calibration values for the specific servo.
        board_idx, channel_idx = LEG_CHANNEL_MAP[leg_index][joint_index]
        center_angle = SERVO_CENTER_DEGREES[leg_index][joint_index]

        # 3. The final angle sent to the servo is the limited kinematic angle plus
        #    the calibrated center position.
        servo_angle = center_angle + limited_angle_deg

        if board_idx < len(self.servos) and channel_idx < len(self.servos[board_idx]):
            # 4. Clamp the final angle to the servo's absolute actuation range (e.g., 0-270).
            safe_angle = max(0, min(ACTUATION_RANGE, servo_angle))
            self.servos[board_idx][channel_idx].angle = safe_angle
        else:
            print(f"Warning: Board/Channel ({board_idx}, {channel_idx}) is out of range.")

    def set_all_leg_angles(self, angles_deg):
        """
        Updates the target angles for all joints of all legs.
        This method is now very fast as it only updates an array and does not
        wait for I2C communication.

        :param angles_deg: A list of 6 lists, where each inner list contains
                           the [coxa, femur, tibia] angles in degrees for a leg.
        """
        with self._lock:
            # np.asarray is used to handle both lists and numpy arrays gracefully
            self._target_angles_deg = np.asarray(angles_deg)

    def _update_loop(self):
        """The main loop for the background update thread."""
        while self._is_running:
            with self._lock:
                # Make a local copy of the target angles to avoid holding the lock
                # during I2C communication.
                current_targets = self._target_angles_deg.copy()

            for leg_idx in range(6):
                for joint_idx in range(3):
                    self._set_physical_leg_angle(leg_idx, joint_idx, current_targets[leg_idx, joint_idx])
            
            time.sleep(1.0 / UPDATE_RATE_HZ)

    def deinit(self):
        """Stops the update thread and de-energizes all servos."""
        self._is_running = False
        self._update_thread.join() # Wait for the thread to finish
        for board in self.boards:
            board.deinit()