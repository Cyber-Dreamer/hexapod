"""
Servo Controller for the Hexapod's Hardware Platform

This module abstracts the control of all 18 servos for the hexapod legs
using the Adafruit PCA9685 library. It assumes a single 16-channel board
is used for the first 16 servos and provides a structure to add more boards.
"""

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Configuration ---
I2C_BUS = busio.I2C(board.SCL, board.SDA)
FREQUENCY = 50  # Standard for most analog servos

# These values are based on your test script. Fine-tune them for your specific servos.
SERVO_MIN_PULSE = 500   # Microseconds
SERVO_MAX_PULSE = 2500  # Microseconds
ACTUATION_RANGE = 270   # Degrees

# --- Multi-board Configuration ---
# List of I2C addresses for your PCA9685 boards.
# The default is 0x40. Solder the address jumpers on your second board
# to set a unique address (e.g., 0x41).
PCA9685_ADDRESSES = [0x40, 0x41]

# Mapping of leg and joint to a (board_index, channel_index) tuple.
# This configuration splits the legs across two boards.
# Board 0 (at address 0x40) controls legs 0, 1, 2.
# Board 1 (at address 0x41) controls legs 3, 4, 5.
# joint_index: 0=coxa, 1=femur, 2=tibia
LEG_CHANNEL_MAP = [
    # Board 0
    [(0, 0), (0, 1), (0, 2)],    # Leg 0 (Right Front)
    [(0, 3), (0, 4), (0, 5)],    # Leg 1 (Right Middle)
    [(0, 6), (0, 7), (0, 8)],    # Leg 2 (Right Rear)
    # Board 1
    [(1, 0), (1, 1), (1, 2)],    # Leg 3 (Left Rear)
    [(1, 3), (1, 4), (1, 5)],    # Leg 4 (Left Middle)
    [(1, 6), (1, 7), (1, 8)],    # Leg 5 (Left Front)
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

    def set_leg_angle(self, leg_index, joint_index, angle_deg):
        """Sets the angle of a single joint on a specific leg."""
        board_idx, channel_idx = LEG_CHANNEL_MAP[leg_index][joint_index]

        if board_idx < len(self.servos) and channel_idx < len(self.servos[board_idx]):
            # Clamp angle to a safe range (e.g., 0-180) to prevent servo damage
            # The actuation range is 270, but it's good practice to have a clamp.
            # Let's use the full actuation range.
            safe_angle = max(0, min(ACTUATION_RANGE, angle_deg))
            self.servos[board_idx][channel_idx].angle = safe_angle
        else:
            print(f"Warning: Board/Channel ({board_idx}, {channel_idx}) is out of range.")

    def set_all_leg_angles(self, angles_deg):
        """
        Sets the angles for all joints of all legs.
        :param angles_deg: A list of 6 lists, where each inner list contains
                           the [coxa, femur, tibia] angles in degrees for a leg.
        """
        for leg_idx, joint_angles in enumerate(angles_deg):
            for joint_idx, angle in enumerate(joint_angles):
                self.set_leg_angle(leg_idx, joint_idx, angle)

    def deinit(self):
        """De-energizes all servos."""
        for board in self.boards:
            board.deinit()