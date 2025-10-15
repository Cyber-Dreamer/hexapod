from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles 
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
import asyncio
import uvicorn
import os
import sys
import time
from typing import Optional, Dict
import logging
import numpy as np
import cv2

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.platform.hexapod_platform import HexapodPlatform
from hexapod_py.locomotion.locomotion import HexapodLocomotion

# --- Global Variables ---
# These will be initialized in the main execution block
platform: Optional[HexapodPlatform] = None
locomotion: Optional[HexapodLocomotion] = None 
control_values: Dict[str, float] = {'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'body_height': 150.0}
locomotion_enabled: bool = False
last_joint_angles: Optional[Dict[str, float]] = None
server_mode: str = "Unknown"

class CameraStreamer:
    """Manages broadcasting camera frames to multiple WebSocket clients."""
    def __init__(self):
        self.connections: Dict[int, list[WebSocket]] = {}
        self.locks: Dict[int, asyncio.Lock] = {}
        self.broadcasting_tasks: Dict[int, asyncio.Task] = {}

    async def add_client(self, camera_id: int, websocket: WebSocket, fps: int = 30):
        if camera_id not in self.connections:
            self.connections[camera_id] = []
            self.locks[camera_id] = asyncio.Lock()
        
        async with self.locks[camera_id]:
            self.connections[camera_id].append(websocket)

        if camera_id not in self.broadcasting_tasks or self.broadcasting_tasks[camera_id].done():
            self.broadcasting_tasks[camera_id] = asyncio.create_task(self._broadcast_frames(camera_id, fps))

    async def remove_client(self, camera_id: int, websocket: WebSocket):
        async with self.locks[camera_id]:
            self.connections[camera_id].remove(websocket)
        
        if not self.connections[camera_id]:
            if camera_id in self.broadcasting_tasks:
                self.broadcasting_tasks[camera_id].cancel()
                del self.broadcasting_tasks[camera_id]

    async def _broadcast_frames(self, camera_id: int, fps: int):
        """Continuously fetches frames and sends them to connected clients."""
        logging.info(f"Starting frame broadcast for camera {camera_id} at {fps} FPS.")
        try:
            delay = 1.0 / fps
            while True:
                frame_bytes = await get_frame_bytes(camera_id)
                if frame_bytes:
                    # Use a copy of the list to avoid issues if a client disconnects
                    # during iteration.
                    async with self.locks[camera_id]:
                        connections_to_send = self.connections[camera_id][:]

                    for connection in connections_to_send:
                        try:
                            await connection.send_bytes(frame_bytes)
                        except (WebSocketDisconnect, ConnectionResetError):
                            # These are expected when a client closes the connection.
                            # The remove_client logic will handle cleanup.
                            logging.info(f"Client disconnected during send on camera {camera_id}.")
                await asyncio.sleep(delay)
        except asyncio.CancelledError:
            logging.info(f"Broadcast task for camera {camera_id} was cancelled.")
        except Exception as e:
            logging.error(f"!!! Unhandled exception in broadcast task for camera {camera_id}: {e}", exc_info=True)

camera_streamer = CameraStreamer()
app = FastAPI()

# --- Path Setup for Static Files and Templates ---
# Get the directory where this server.py file is located
server_dir = os.path.dirname(__file__)
static_dir = os.path.join(server_dir, "static")
templates_dir = os.path.join(server_dir, "templates")

# Mount static files
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Also mount the simulation's meshes directory so the URDF loader can find the STL files
simulation_dir = os.path.join(project_root, "hexapod_py", "platform", "simulation")
app.mount("/simulation_assets", StaticFiles(directory=simulation_dir), name="simulation_assets")

# Templates
templates = Jinja2Templates(directory=templates_dir)

def setup_server(p: HexapodPlatform, l: HexapodLocomotion, mode: str):
    """
    Initializes the web server with the necessary platform and locomotion objects.
    This function is called by the main runner script before starting the server.
    """
    global platform, locomotion, server_mode
    platform = p
    locomotion = l
    server_mode = mode
    print("Web server configured with platform and locomotion instances.")

async def control_loop():
    """
    The main control loop that runs in the background.
    It reads control values, runs the gait logic, and updates the platform.
    """
    while True:
        global last_joint_angles, locomotion_enabled
        if platform and locomotion:
            if locomotion_enabled:
                # Update locomotion parameters that can be changed live
                # Convert meters from UI to millimeters for the locomotion module
                if 'body_height' in control_values:
                    locomotion.body_height = control_values['body_height']
                    locomotion.recalculate_stance()
                # 1. If enabled, run gait logic with current control values
                joint_angles = locomotion.run_gait(
                    vx=control_values['vx'],
                    vy=control_values['vy'],
                    omega=control_values['omega']
                )
                # Set the target angles on the platform
                platform.set_joint_angles(joint_angles)
                last_joint_angles = joint_angles
            else:
                # 2. If disabled, do nothing. This allows the robot to hold its last
                # commanded position. On startup, this will be the simulator's
                # initial standing pose, preventing the "collapse" behavior.
                # We also reset control values to prevent sudden movement on re-enabling.
                control_values.update({'vx': 0.0, 'vy': 0.0, 'omega': 0.0}) # Keep height

        # Run the loop at a consistent rate (e.g., 100Hz)
        await asyncio.sleep(1/100.)

@app.on_event("startup")
async def startup_event():
    """Starts the background control loop when the server starts."""
    asyncio.create_task(control_loop())

@app.on_event("shutdown")
def shutdown_event():
    """Releases the webcam when the server shuts down."""

@app.get("/")
async def index(request: Request):
    """Serves the main control page."""
    return templates.TemplateResponse("index.html", {"request": request, "mode": server_mode})

@app.post("/move")
async def move(request: Request):
    """Receives movement commands from the web UI."""
    global control_values
    data = await request.json()
    print(f"Received move command: {data}, locomotion_enabled: {locomotion_enabled}")
    control_values['vx'] = data.get('vx', 0.0)
    control_values['vy'] = data.get('vy', 0.0)
    control_values['omega'] = data.get('omega', 0.0)
    control_values['body_height'] = data.get('body_height', 150.0)
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
        return {
            "imu": imu_data if imu_data else {}, 
            "locomotion_enabled": locomotion_enabled,
            "joint_angles": last_joint_angles
        }
    return {"imu": {}, "locomotion_enabled": locomotion_enabled, "joint_angles": None}

@app.websocket("/ws/video/{camera_id}")
async def websocket_video_feed(websocket: WebSocket, camera_id: int, fps: int = 30):
    await websocket.accept()
    await camera_streamer.add_client(camera_id, websocket, fps)
    try:
        while True:
            await websocket.receive_text() # Keep connection alive
    except WebSocketDisconnect:
        print(f"Client disconnected from camera {camera_id}")
        await camera_streamer.remove_client(camera_id, websocket)

async def get_frame_bytes(camera_id: int) -> Optional[bytes]:
    """Helper function to get encoded frame bytes for a camera."""
    # The platform client now handles all communication (ZMQ) and returns
    # the raw JPEG bytes directly from the dedicated camera_server.
    if platform and hasattr(platform, 'get_camera_image'):
        # Run the blocking ZMQ call in a thread to not block the event loop
        frame_bytes = await asyncio.to_thread(platform.get_camera_image, camera_id)
        return frame_bytes
    return None