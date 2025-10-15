import sys
import os

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

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

if __name__ == '__main__':
    main()