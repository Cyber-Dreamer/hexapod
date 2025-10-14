"""
Main Simulation Control Loop with UI
====================================

This script orchestrates the hexapod simulation, providing real-time
control over its movement via GUI sliders.
"""

import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.interfaces.simple_ui.gait_demo_controller import GaitDemoController

def main():
    """
    Main function to initialize and run the hexapod gait simulation with a UI.

    This script wires together the major components:
    1. The simulation platform (`HexapodSimulator`).
    2. The high-level locomotion logic (`HexapodLocomotion`).
    3. The user interface controller (`GaitDemoController`).
    """
    # 1. Initialize the platform (the simulator in this case)
    platform = HexapodSimulator(gui=True)

    # 2. Initialize the locomotion controller
    locomotion = HexapodLocomotion(gait_type='tripod')

    # 3. Initialize the UI controller and run it
    controller = GaitDemoController(platform, locomotion)
    controller.run()

if __name__ == "__main__":
    main()