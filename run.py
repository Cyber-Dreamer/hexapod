
import argparse
import os
import subprocess
import time
import zmq
import sys

# Add the project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from hexapod_py.locomotion.locomotion import HexapodLocomotion # noqa: E402
from hexapod_py.interfaces.simple_ui.gait_demo_controller import GaitDemoController # noqa: E402
from hexapod_py.platform.client import PlatformClient # noqa: E402
from hexapod_py.interfaces.web.server import setup_server # noqa: E402
import uvicorn # noqa: E402

def main():
    # If run without arguments, prompt the user for choices.
    if len(sys.argv) == 1:
        print("No arguments provided. Please choose from the options below.")

        # Platform choice
        p_choice = input("Choose platform (1: simulation, 2: physical) [default: 1]: ").strip()
        platform = 'physical' if p_choice == '2' else 'simulation'

        # Create a simple namespace object to hold the choices
        class Args:
            pass
        args = Args()

        # Interface choice
        i_choice = input("Choose interface (1: simple_ui, 2: web) [default: 2]: ").strip()
        # Set the interface on the args object
        args.interface = 'simple_ui' if i_choice == '1' else 'web'

        # Set the platform on the args object
        args.platform = platform
        print(f"\nStarting with: platform='{args.platform}', interface='{args.interface}'\n")

    else:
        parser = argparse.ArgumentParser(
            description="Run the Hexapod control system.",
            formatter_class=argparse.RawTextHelpFormatter
        )
        parser.add_argument("--interface", type=str, choices=["simple_ui", "web"],
                            default="web", help="The user interface to run.")
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
            print("Launching interface: Web UI")
            mode_name = "Simulation" if args.platform == 'simulation' else "Physical Robot"
            # Configure the web server with the platform client and locomotion controller
            setup_server(platform_client, locomotion, mode=mode_name)
            # Run the FastAPI server using uvicorn
            # Note: This will block the main thread, which is fine for the web UI.
            # The platform runs in a separate process.
            uvicorn.run("hexapod_py.interfaces.web.server:app", host="0.0.0.0", port=8000, log_level="info")

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
