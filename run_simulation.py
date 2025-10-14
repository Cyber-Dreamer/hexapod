"""
Main Hexapod Simulation Runner
============================

This script initializes and runs the hexapod gait simulation with an
interactive UI. It serves as a clean entry point that wires together the
major components of the system.
"""

import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.interfaces.simple_ui.gait_demo_controller import GaitDemoController

def main():
    # 1. Initialize the platform (the simulator in this case)
    platform = HexapodSimulator(gui=True)

    # 2. Initialize the locomotion controller
    locomotion = HexapodLocomotion(gait_type='tripod')

    # 3. Initialize the UI controller and run it
    controller = GaitDemoController(platform, locomotion)
    controller.run()

if __name__ == "__main__":
    main()