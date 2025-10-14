"""
Hexapod Master Runner
=====================

This is the main entry point for the hexapod control software.
It allows you to launch different interfaces (like the simple UI or a web server)
and connect them to different platforms (like the simulator or a physical robot).

Examples:
  # Run the simple PyBullet UI with the simulator (default)
  python run.py

  # Run the web interface with the simulator
  python run.py --interface web --platform simulation

  # Run the simple UI, preparing for a physical robot
  python run.py --interface simple_ui --platform physical
"""

import argparse
import os
import sys

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

# Import necessary components
from hexapod_py.platform.hexapod_platform import HexapodPlatform
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.interfaces.simple_ui.gait_demo_controller import GaitDemoController

# The web server needs to be imported carefully to be launched programmatically
import uvicorn
from hexapod_py.interfaces.web.server import app as web_app
from hexapod_py.interfaces.web.server import platform as web_platform_global, locomotion as web_locomotion_global

def main():
    parser = argparse.ArgumentParser(description="Run the Hexapod control system.")
    parser.add_argument("--interface", type=str, choices=["simple_ui", "web"],
                        help="The user interface to run.")
    parser.add_argument("--platform", type=str, choices=["simulation", "physical"],
                        help="The platform to control (simulation or physical robot).")
    args = parser.parse_args()

    # --- Fallback to interactive prompt if arguments are not provided ---
    try:
        if not args.interface:
            choice = input("Select interface (1: Simple UI, 2: Web UI) [1]: ").strip()
            if choice == '2':
                args.interface = 'web'
            else:
                args.interface = 'simple_ui'

        if not args.platform:
            choice = input("Select platform (1: Simulation, 2: Physical) [1]: ").strip()
            if choice == '2':
                args.platform = 'physical'
            else:
                args.platform = 'simulation'

    except KeyboardInterrupt:
        print("\nSelection cancelled. Exiting.")
        sys.exit(0)

    # --- 1. Initialize Platform ---
    platform: HexapodPlatform
    if args.platform == 'simulation':
        print("Initializing platform: HexapodSimulator")
        platform = HexapodSimulator(gui=True)
    elif args.platform == 'physical':
        print("Initializing platform: PhysicalHexapod (Not Implemented)")
        # from hexapod_py.platform.physical_robot import PhysicalHexapod
        # platform = PhysicalHexapod()
        print("Error: PhysicalHexapod is not yet implemented. Exiting.")
        sys.exit(1)

    # --- 2. Initialize Locomotion Controller ---
    locomotion = HexapodLocomotion(gait_type='tripod')

    # --- 3. Launch Selected Interface ---
    if args.interface == 'simple_ui':
        print(f"Launching interface: Simple UI Controller for {args.platform} platform.")
        controller = GaitDemoController(platform, locomotion)
        controller.run()
    elif args.interface == 'web':
        print(f"Launching interface: Web Server for {args.platform} platform.")
        # Assign the created platform and locomotion to the web server's globals
        globals()['web_platform_global'] = platform
        globals()['web_locomotion_global'] = locomotion
        platform.start() # The web server expects the platform to be started
        uvicorn.run(web_app, host="127.0.0.1", port=8000)

if __name__ == "__main__":
    main()
