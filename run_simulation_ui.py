"""
Main Simulation Control Loop with UI
====================================

This script orchestrates the hexapod simulation, providing real-time
control over its movement via GUI sliders.
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
    Initializes and runs the hexapod simulation control loop with a UI.
    """
    # 1. Initialize the simulator
    simulator = HexapodSimulator(gui=True)
    simulator.start()

    # 2. Add UI controls (sliders) to the simulation window
    simulator.add_ui_controls()

    # 3. Initialize the locomotion controller
    # We can initialize with default values; they will be updated from the UI.
    locomotion = HexapodLocomotion(gait_type='tripod')

    try:
        print("Starting control loop with UI. Press Ctrl+C in terminal to exit.")
        while True:
            # 4. Read control values from the UI sliders
            controls = simulator.read_ui_controls()

            # Update locomotion parameters that can change at runtime
            locomotion.body_height = controls['body_height']
            locomotion.step_height = controls['step_height']

            # 5. "Publish": The controller calculates joint angles based on UI input.
            joint_angles = locomotion.run_gait(
                vx=controls['vx'], vy=controls['vy'], omega=controls['omega'],
                pitch=controls['pitch'], speed=controls['speed']
            )

            # 6. "Subscribe": The simulator applies the calculated joint angles.
            simulator.set_joint_angles(joint_angles)

            simulator.step()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        simulator.stop()

if __name__ == "__main__":
    main()