"""
Main Simulation Control Loop
============================

This script orchestrates the hexapod simulation by connecting the high-level
locomotion controller with the PyBullet simulator.
"""

import sys
import os

# Add the project root to the Python path
project_root = os.path.abspath(os.path.dirname(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import time
from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.simulation.simulator import HexapodSimulator

def main():
    """
    Initializes and runs the hexapod simulation control loop.
    """
    # 1. Initialize the simulator (the "subscriber")
    simulator = HexapodSimulator(gui=True)
    simulator.start()

    # 2. Initialize the locomotion controller (the "publisher")
    locomotion = HexapodLocomotion(gait_type='tripod')

    try:
        print("Starting control loop. Press Ctrl+C to exit.")
        while True:
            # Define robot's desired movement (vx, vy, omega)
            # For this example, let's make it walk forward.
            vx, vy, omega = 0.1, 0.0, 0.0

            # 3. "Publish": The controller calculates the next set of joint angles.
            joint_angles = locomotion.run_gait(vx, vy, omega)

            # 4. "Subscribe": The simulator receives and applies the joint angles.
            simulator.set_joint_angles(joint_angles)

            simulator.step()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        simulator.stop()

if __name__ == "__main__":
    main()