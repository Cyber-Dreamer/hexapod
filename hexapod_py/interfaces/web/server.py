from fastapi import FastAPI, Request, Response
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
import asyncio
import uvicorn
import os
import sys
import time
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
locomotion_enabled: bool = False

app = FastAPI()

# --- Path Setup for Static Files and Templates ---
# Get the directory where this server.py file is located
server_dir = os.path.dirname(__file__)
static_dir = os.path.join(server_dir, "static")
templates_dir = os.path.join(server_dir, "templates")

# Mount static files
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Templates
templates = Jinja2Templates(directory=templates_dir)

def setup_server(p: HexapodPlatform, l: HexapodLocomotion):
    """
    Initializes the web server with the necessary platform and locomotion objects.
    This function is called by the main runner script before starting the server.
    """
    global platform, locomotion
    platform = p
    locomotion = l
    print("Web server configured with platform and locomotion instances.")

async def control_loop():
    """
    The main control loop that runs in the background.
    It reads control values, runs the gait logic, and updates the platform.
    """
    while True:
        if platform and locomotion:
            if locomotion_enabled:
                # 1. If enabled, run gait logic with current control values
                joint_angles = locomotion.run_gait(
                    vx=control_values['vx'],
                    vy=control_values['vy'],
                    omega=control_values['omega']
                )
                # Set the target angles on the platform
                platform.set_joint_angles(joint_angles)
            else:
                # 2. If disabled, do nothing. This allows the robot to hold its last
                # commanded position. On startup, this will be the simulator's
                # initial standing pose, preventing the "collapse" behavior.
                # We also reset control values to prevent sudden movement on re-enabling.
                control_values.update({'vx': 0.0, 'vy': 0.0, 'omega': 0.0})

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
    print(f"Received move command: {data}, locomotion_enabled: {locomotion_enabled}")
    control_values['vx'] = data.get('vx', 0.0)
    control_values['vy'] = data.get('vy', 0.0)
    control_values['omega'] = data.get('omega', 0.0)
    return {"status": "success", "received": control_values}

@app.post("/toggle_locomotion")
async def toggle_locomotion():
    """Toggles the locomotion system between enabled and disabled (emergency stop)."""
    global locomotion_enabled
    locomotion_enabled = not locomotion_enabled
    status = "enabled" if locomotion_enabled else "disabled"
    print(f"Locomotion has been {status}.")
    return {"status": f"locomotion_{status}"}

@app.get("/sensor_data")
async def sensor_data():
    """Streams sensor data to the client (e.g., IMU)."""
    if platform and hasattr(platform, 'get_imu_data'):
        imu_data = platform.get_imu_data()
        return {"imu": imu_data if imu_data else {}, "locomotion_enabled": locomotion_enabled}
    return {"imu": {}, "locomotion_enabled": locomotion_enabled}

@app.get("/video_feed/{camera_id}")
async def video_feed(camera_id: int):
    """Streams video from a specified camera on the platform."""
    if platform and hasattr(platform, 'get_camera_image'):
        # This function is now synchronous. FastAPI will run it in a separate
        # thread from the main async event loop, which is the correct way to
        # handle a blocking, long-running response. We use time.sleep()
        # to pace the stream.
        def generate():
            while True:
                frame_bytes = None
                if isinstance(platform, HexapodSimulator):
                    # --- Generate a dummy frame for simulation to save resources ---
                    width, height = 640, 480
                    # Create a black image
                    img = np.zeros((height, width, 3), dtype=np.uint8)
                    
                    # Add text indicating which camera it is
                    text = f"Dummy Feed: Cam {camera_id}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text_size = cv2.getTextSize(text, font, 1, 2)[0]
                    text_x = (width - text_size[0]) // 2
                    text_y = (height + text_size[1]) // 2
                    cv2.putText(img, text, (text_x, text_y), font, 1, (255, 255, 255), 2)

                    _, jpeg = cv2.imencode('.jpg', img)
                    frame_bytes = jpeg.tobytes()
                else:
                    # --- Capture real frame from platform (physical robot or otherwise) ---
                    img_arr = platform.get_camera_image(camera_id)
                    if img_arr is not None:
                        bgr_img = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)
                        _, jpeg = cv2.imencode('.jpg', bgr_img)
                        frame_bytes = jpeg.tobytes()

                if frame_bytes:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                time.sleep(1/15.) # Pace the stream to ~15 FPS

        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return HTMLResponse(content="<h1>Camera not available on this platform.</h1>", status_code=404)