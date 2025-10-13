import argparse
import asyncio
import os
import sys

# Add the project root to the Python path
project_root = os.path.abspath(os.path.dirname(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.web.server import run_web_server
from hexapod_py.core.main import main as core_main

async def main():
    parser = argparse.ArgumentParser(description="Run the Hexapod control system.")
    parser.add_argument("--mode", type=str, choices=["robot", "simulation"],
                        help="The mode to run the hexapod in (robot or simulation).")
    args = parser.parse_args()

    mode = args.mode
    if not mode:
        try:
            mode = input("Please select a mode (robot/simulation): ").strip().lower()
            while mode not in ["robot", "simulation"]:
                print("Invalid mode. Please choose 'robot' or 'simulation'.")
                mode = input("Please select a mode (robot/simulation): ").strip().lower()
        except KeyboardInterrupt:
            print("\nSelection cancelled by user. Exiting.")
            return

    tasks = []

    if mode == 'robot':
        print("Running in robot mode")
        tasks.append(asyncio.create_task(core_main()))
        tasks.append(asyncio.create_task(run_web_server(mode='robot')))
    elif mode == 'simulation':
        print("Running in simulation mode")
        tasks.append(asyncio.create_task(run_web_server(mode='simulation')))

    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())

