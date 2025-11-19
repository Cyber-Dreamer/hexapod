
import argparse
import os
import subprocess
import time
import zmq
import sys


sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from hexapod_py.locomotion.locomotion import HexapodLocomotion # noqa: E402
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
        args.platform = platform
        print(f"\nStarting with: platform='{args.platform}', interface='web'\n")

    else:
        parser = argparse.ArgumentParser(
            description="Run the Hexapod control system.",
            formatter_class=argparse.RawTextHelpFormatter
        )
        parser.add_argument("--platform", type=str, choices=["simulation", "physical"],
                            default="simulation", help="The platform to control.")
        args = parser.parse_args()
        print(f"\nStarting with: platform='{args.platform}', interface='web'\n")

    platform_process = None
    camera_process = None
    platform_client = None
    project_root_dir = os.path.abspath(os.path.dirname(__file__))

    try:
        # --- 1. Launch the selected platform server ---
        if args.platform == 'simulation':
            print("Launching Simulation Server...")
            platform_process = subprocess.Popen(
                [sys.executable, "-m", "hexapod_py.platform.simulation.simulator"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=project_root_dir)
            # Give the server a moment to start and bind sockets
            time.sleep(2)
        elif args.platform == 'physical':
            print("Launching Hardware Platform Server...")
            platform_process = subprocess.Popen(
                [sys.executable, "-m", "hexapod_py.platform.hardware.server"],
                cwd=project_root_dir)
            time.sleep(3) # Give it time to initialize hardware

        # --- 1.5 Launch the correct camera server for the selected platform ---
        print(f"Launching Camera Server for {args.platform} platform...")
        if args.platform == 'simulation':
            camera_module = "hexapod_py.platform.simulation.camera_server"
            camera_args = []
        else: # physical
            camera_module = "hexapod_py.platform.hardware.camera_server"
            camera_args = ["--width", "1280", "--height", "720", "--quality", "75"]

        camera_process = subprocess.Popen(
            [sys.executable, "-m", camera_module] + camera_args,
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            cwd=project_root_dir)

        time.sleep(2) # Give it a moment to start

        # --- 2. Initialize client and controllers ---
        platform_client = PlatformClient()
        # Start the client's background threads to listen for sensor data
        platform_client.start()

        locomotion = HexapodLocomotion(gait_type='tripod')

        # --- 3. Launch the web interface ---
        print("Launching interface: Web UI")
        mode_name = "Simulation" if args.platform == 'simulation' else "Physical Robot"
        
        # The web server runs in the main thread. The platform client runs in the background.
        
        # Configure the web server with the platform client and locomotion controller
        setup_server(platform_client, locomotion, mode=mode_name)
        # Run the FastAPI server using uvicorn
        uvicorn.run("hexapod_py.interfaces.web.server:app", host="0.0.0.0", port=8001, log_level="info")

    except zmq.error.ZMQError as e:
        print(f"\nError: Could not connect to the platform server. {e}")
        print("Please make sure the simulator process started correctly.")
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt, shutting down...")
    finally:
        print("\nCleaning up...")
        if platform_client:
            platform_client.stop() # This will stop the client's background threads
        if camera_process:
            print("Terminating camera server process...")
            camera_process.terminate()
            stdout, stderr = camera_process.communicate(timeout=5)
            if stdout: print(f"[CamServer STDOUT]:\n{stdout.decode()}")
            if stderr: print(f"[CamServer STDERR]:\n{stderr.decode()}")

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
