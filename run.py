
import argparse
import os
import sys
import subprocess
import time
import zmq

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.interfaces.simple_ui.gait_demo_controller import GaitDemoController
from hexapod_py.platform.client import PlatformClient

def main():
    parser = argparse.ArgumentParser(description="Run the Hexapod control system.")
    parser.add_argument("--interface", type=str, choices=["simple_ui", "web"],
                        default="simple_ui", help="The user interface to run.")
    parser.add_argument("--platform", type=str, choices=["simulation", "physical"],
                        default="simulation", help="The platform to control.")
    args = parser.parse_args()

    platform_process = None
    platform_client = None
    try:
        # --- 1. Launch the selected platform server ---
        if args.platform == 'simulation':
            print("Launching Simulation Server...")
            platform_process = subprocess.Popen(
                [sys.executable, "-m", "hexapod_py.platform.simulation.simulator"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            # Give the server a moment to start and bind sockets
            time.sleep(2)
        elif args.platform == 'physical':
            print("Error: Physical platform server is not yet implemented.")
            sys.exit(1)

        # --- 2. Initialize client and controllers ---
        platform_client = PlatformClient()
        locomotion = HexapodLocomotion(gait_type='tripod')

        # --- 3. Launch the selected interface ---
        if args.interface == 'simple_ui':
            print("Launching interface: Simple UI Controller")
            controller = GaitDemoController(platform_client, locomotion)
            controller.run()
        elif args.interface == 'web':
            print("Warning: The web interface has not been migrated yet.")

    except zmq.error.ZMQError as e:
        print(f"\nError: Could not connect to the platform server. {e}")
        print("Please make sure the simulator process started correctly.")
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, shutting down...")
    finally:
        print("\nCleaning up...")
        if platform_client:
            platform_client.stop()
        if platform_process:
            print("Terminating platform server process...")
            platform_process.terminate()
            stdout, stderr = platform_process.communicate(timeout=5)
            if stdout:
                print(f"[Server STDOUT]:\n{stdout.decode()}")
            if stderr:
                print(f"[Server STDERR]:\n{stderr.decode()}")
        print("Cleanup complete.")

if __name__ == "__main__":
    main()
