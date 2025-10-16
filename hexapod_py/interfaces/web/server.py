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
control_values: Dict[str, float] = {
    'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 
    'body_height': 150.0, 
    'standoff': 400.0,
    'step_height': 40.0,
    'pitch': 0.0,
    'roll': 0.0
}
locomotion_enabled: bool = False # Gait movement vs. body posing
ai_vision_enabled: bool = False  # AI object detection toggle
object_detector: Optional['ObjectDetector'] = None
last_joint_angles: Optional[Dict[str, float]] = None
server_mode: str = "Unknown"

class CameraStreamer:
    """Manages broadcasting camera frames to multiple WebSocket clients."""
    def __init__(self):
        self.connections: Dict[int, list[WebSocket]] = {}
        self.locks: Dict[int, asyncio.Lock] = {}
        self.broadcast_tasks: Dict[int, asyncio.Task] = {}

    async def add_client(self, camera_id: int, websocket: WebSocket):
        if camera_id not in self.connections:
            self.connections[camera_id] = []
            self.locks[camera_id] = asyncio.Lock()
        
        async with self.locks[camera_id]:
            self.connections[camera_id].append(websocket)

        if camera_id not in self.broadcast_tasks or self.broadcast_tasks[camera_id].done():
            self.broadcast_tasks[camera_id] = asyncio.create_task(self._broadcast_frames(camera_id))

    async def remove_client(self, camera_id: int, websocket: WebSocket):
        async with self.locks[camera_id]:
            self.connections[camera_id].remove(websocket)
        
        if not self.connections[camera_id]:
            if camera_id in self.broadcast_tasks:
                self.broadcast_tasks[camera_id].cancel()
                del self.broadcast_tasks[camera_id]

    async def _broadcast_frames(self, camera_id: int):
        """Continuously fetches frames and sends them to connected clients."""
        logging.info(f"Starting frame broadcast for camera {camera_id}.")
        try:
            delay = 1.0 / 30 # Aim for 30 FPS
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
                            pass # Noisy log, can be disabled.
                
                # Wait for the next frame.
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
    global platform, locomotion, server_mode, object_detector
    platform = p
    locomotion = l
    server_mode = mode
    print("Web server configured with platform and locomotion instances.")
    # Initialize the object detector only if on the physical robot
    if server_mode == "Physical Robot":
        from hexapod_py.platform.hardware.ai_vision import ObjectDetector
        # object_detector = ObjectDetector() # Temporarily disabled to avoid model loading issues.
        print("!!! AI Vision initialization is temporarily disabled in server.py !!!")

async def control_loop():
    """
    The main control loop that runs in the background.
    It reads control values, runs the gait logic, and updates the platform.
    """
    while True:
        global last_joint_angles, locomotion_enabled
        if platform and locomotion:
            # Update locomotion parameters that can be changed live from the UI
            new_body_height = control_values.get('body_height', locomotion.body_height)
            new_standoff = control_values.get('standoff', locomotion.standoff_distance)
            new_step_height = control_values.get('step_height', locomotion.step_height)

            # Recalculate stance if body height or standoff has changed
            if new_body_height != locomotion.body_height or new_standoff != locomotion.standoff_distance:
                locomotion.body_height = new_body_height
                locomotion.recalculate_stance(standoff_distance=new_standoff)
            
            # Update step height for the current gait
            if new_step_height != locomotion.step_height:
                locomotion.step_height = new_step_height

            if locomotion_enabled:
                # 1. If enabled, run gait logic with current control values
                joint_angles = locomotion.run_gait(
                    vx=control_values['vx'],
                    vy=control_values['vy'],
                    omega=control_values['omega'],
                    roll=control_values.get('roll', 0.0),
                    pitch=control_values.get('pitch', 0.0),
                    step_height=locomotion.step_height
                )
            else:
                # 2. If disabled, switch to stationary body IK mode.
                # The left joystick controls body translation (tx, ty)
                # The right joystick controls body orientation (roll, pitch)
                max_translation = 50.0 # mm
                max_rotation = np.deg2rad(20.0) # radians

                translation = np.array([
                    control_values['vy'] * max_translation, # vy is forward/backward -> maps to body tx
                    control_values['vx'] * max_translation, # vx is strafe -> maps to body ty
                    0.0 # tz is not controlled by joystick
                ])
                rotation = np.array([
                    control_values['roll'] * max_rotation,
                    control_values['pitch'] * max_rotation,
                    0.0 # yaw is not controlled in this mode
                ])

                # Calculate the required joint angles for the body pose
                joint_angles = locomotion.set_body_pose(translation, rotation)

            # Convert joint angles (which may be numpy arrays) to a list of lists
            # for serialization, as msgpack cannot handle numpy arrays directly.
            joint_angles_list = []
            if joint_angles is not None:
                joint_angles_list = [arr.tolist() if isinstance(arr, np.ndarray) else arr for arr in joint_angles]

            # Set the target angles on the platform
            platform.set_joint_angles(joint_angles_list)
            
            # Convert the list of lists into a dictionary for the UI's 3D model
            # The URDF loader expects joint names.
            joint_names = [
                'coxa', 'femur', 'tibia'
            ]
            last_joint_angles = {}
            if joint_angles is not None:
                for i, leg_angles in enumerate(joint_angles):
                    for j, angle in enumerate(leg_angles):
                        last_joint_angles[f'leg_{i}_{joint_names[j]}_joint'] = angle
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
    control_values['vx'] = data.get('vx', 0.0)
    control_values['vy'] = data.get('vy', 0.0)
    control_values['omega'] = data.get('omega', 0.0)
    control_values['body_height'] = data.get('body_height', control_values['body_height'])
    control_values['standoff'] = data.get('standoff', control_values['standoff'])
    control_values['step_height'] = data.get('step_height', control_values['step_height'])
    control_values['pitch'] = data.get('pitch', 0.0)
    control_values['roll'] = data.get('roll', 0.0)
    return {"status": "success", "received": control_values}

@app.post("/toggle_locomotion")
async def toggle_locomotion():
    """Toggles the locomotion system between enabled and disabled (emergency stop)."""
    global locomotion_enabled
    locomotion_enabled = not locomotion_enabled
    status = "enabled" if locomotion_enabled else "disabled"
    print(f"Locomotion has been {status}.")
    # When disabling, reset movement values to prevent sudden lurching
    if not locomotion_enabled:
        control_values.update({
            'vx': 0.0, 
            'vy': 0.0, 
            'omega': 0.0,
            'pitch': 0.0,
            'roll': 0.0
        })
    return {"status": f"locomotion_{status}"}

@app.post("/toggle_ai_vision")
async def toggle_ai_vision():
    """Toggles the AI vision processing for the front camera."""
    global ai_vision_enabled
    ai_vision_enabled = not ai_vision_enabled
    status = "enabled" if ai_vision_enabled else "disabled"
    return {"status": f"ai_vision_{status}"}

@app.get("/sensor_data")
async def sensor_data():
    """Streams sensor data to the client (e.g., IMU)."""
    if platform:
        imu_data = platform.get_imu_data()
        gps_data = platform.get_gps_data() if hasattr(platform, 'get_gps_data') else None
        return {
            "imu": imu_data if imu_data else {}, 
            "gps": gps_data if gps_data else {},
            "locomotion_enabled": locomotion_enabled,
            "ai_vision_enabled": ai_vision_enabled,
            "joint_angles": last_joint_angles
        }
    return {"imu": {}, "gps": {}, "locomotion_enabled": locomotion_enabled, "ai_vision_enabled": ai_vision_enabled, "joint_angles": None}

@app.websocket("/ws/video/{camera_id}")
async def websocket_video_feed(websocket: WebSocket, camera_id: int):
    await websocket.accept()
    await camera_streamer.add_client(camera_id, websocket)
    try:
        while True:
            # Keep the connection alive. The server is pushing frames.
            # We can add client-to-server messages here later if needed.
            await websocket.receive_text() # This will block until a message is received or the client disconnects
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

        # If AI vision is on for the front camera on the physical robot, process the frame
        if ai_vision_enabled and camera_id == 0 and object_detector:
            try:
                if frame_bytes:
                    # 1. Decode JPEG bytes to a numpy array
                    np_arr = np.frombuffer(frame_bytes, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    # 2. Run detection
                    processed_frame = object_detector.detect(frame)

                    # 3. Re-encode to JPEG bytes
                    _, new_frame_bytes = cv2.imencode('.jpg', processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    frame_bytes = new_frame_bytes.tobytes()
            except Exception as e:
                logging.error(f"AI vision processing failed: {e}. Falling back to original frame.")

        return frame_bytes
    return None