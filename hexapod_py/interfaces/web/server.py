from fastapi import FastAPI, Request, Response
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
import asyncio
import uvicorn
import os
import sys
from typing import Optional, Dict
import numpy as np
import cv2

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.platform.hexapod_platform import HexapodPlatform
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion

# --- Global Variables ---
# These will be initialized in the main execution block
platform: Optional[HexapodPlatform] = None
locomotion: Optional[HexapodLocomotion] = None
control_values: Dict[str, float] = {'vx': 0.0, 'vy': 0.0, 'omega': 0.0}

app = FastAPI()

# --- Path Setup for Static Files and Templates ---
# Get the directory where this server.py file is located
interface_dir = os.path.dirname(__file__)
web_dir = os.path.abspath(os.path.join(interface_dir, '..')) # Go up one level to hexapod_py/interfaces

# Mount static files
app.mount("/static", StaticFiles(directory=os.path.join(web_dir, "web", "static")), name="static")

# Templates
templates = Jinja2Templates(directory=os.path.join(web_dir, "web", "templates"))

async def control_loop():
    """
    The main control loop that runs in the background.
    It reads control values, runs the gait logic, and updates the platform.
    """
    while True:
        if platform and locomotion:
            # 1. Run gait logic to get target joint angles
            joint_angles = locomotion.run_gait(
                vx=control_values['vx'],
                vy=control_values['vy'],
                omega=control_values['omega']
            )

            # 2. Set the target angles on the platform
            platform.set_joint_angles(joint_angles)

            # 3. If the platform has a 'step' method (like the simulator), call it.
            #    A physical robot might have its own threaded update loop.
            if hasattr(platform, 'step') and callable(platform.step):
                platform.step()

        # Run the loop at a consistent rate (e.g., 100Hz)
        await asyncio.sleep(1/100.)

@app.on_event("startup")
async def startup_event():
    """Starts the background control loop when the server starts."""
    asyncio.create_task(control_loop())

@app.get("/")
async def index(request: Request):
    """Serves the main control page."""
    mode = "Simulation" if isinstance(platform, HexapodSimulator) else "Physical Robot"
    return templates.TemplateResponse("index.html", {"request": request, "mode": mode})

@app.post("/move")
async def move(request: Request):
    """Receives movement commands from the web UI."""
    global control_values
    data = await request.json()
    control_values['vx'] = data.get('vx', 0.0)
    control_values['vy'] = data.get('vy', 0.0)
    control_values['omega'] = data.get('omega', 0.0)
    return {"status": "success", "received": control_values}

@app.get("/sensor_data")
async def sensor_data():
    """Streams sensor data to the client (e.g., IMU)."""
    if platform and hasattr(platform, 'get_imu_data'):
        imu_data = platform.get_imu_data()
        return {"imu": imu_data if imu_data else {}}
    return {"imu": {}}

@app.get("/video_feed/{camera_id}")
async def video_feed(camera_id: int):
    """Streams video from a specified camera on the platform."""
    if platform and hasattr(platform, 'get_camera_image'):
        async def generate():
            while True:
                img_arr = platform.get_camera_image(camera_id)
                if img_arr is not None:
                    _, jpeg = cv2.imencode('.jpg', img_arr)
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                await asyncio.sleep(1/30)
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return HTMLResponse(content="<h1>Camera not available on this platform.</h1>", status_code=404)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run Hexapod Web Server")
    parser.add_argument(
        '--mode',
        type=str,
        default='simulation',
        choices=['simulation', 'physical'],
        help='The platform to run the web server with.'
    )
    args = parser.parse_args()

    # --- Initialize Platform and Locomotion ---
    if args.mode == 'simulation':
        print("Starting in SIMULATION mode.")
        platform = HexapodSimulator(gui=True)
    elif args.mode == 'physical':
        print("Starting in PHYSICAL mode. (PhysicalHexapod class not yet implemented)")
        # from hexapod_py.platform.physical_robot import PhysicalHexapod
        # platform = PhysicalHexapod() # This is where you'd instantiate the real robot
        # For now, we'll exit if physical is chosen but not implemented.
        print("Error: PhysicalHexapod not implemented. Exiting.")
        sys.exit(1)

    platform.start()
    locomotion = HexapodLocomotion(gait_type='tripod')

    uvicorn.run(app, host="127.0.0.1", port=8000)