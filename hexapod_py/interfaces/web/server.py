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
import logging, json
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
    'body_height': 200.0, 
    'standoff': 200.0,
    'step_height': 40.0,
    'pitch': 0.0,
    'roll': 0.0
}
current_gait: str = "tripod"
power_on: bool = False
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

class SensorDataStreamer:
    """Manages broadcasting sensor data to multiple WebSocket clients."""
    def __init__(self):
        self.connections: list[WebSocket] = []
        self.lock = asyncio.Lock()
        self.broadcast_task: Optional[asyncio.Task] = None

    async def add_client(self, websocket: WebSocket):
        async with self.lock:
            self.connections.append(websocket)
        if not self.broadcast_task or self.broadcast_task.done():
            self.broadcast_task = asyncio.create_task(self._broadcast_data())
            logging.info("Started sensor data broadcast task.")

    async def remove_client(self, websocket: WebSocket):
        async with self.lock:
            if websocket in self.connections:
                self.connections.remove(websocket)
        
        if not self.connections and self.broadcast_task:
            self.broadcast_task.cancel()
            self.broadcast_task = None
            logging.info("Stopped sensor data broadcast task (no clients).")

    async def _broadcast_data(self):
        """Continuously gathers sensor data and sends it to connected clients."""
        last_broadcast_time = time.time()
        try:
            while True:
                if platform:
                    imu_data = platform.get_imu_data()
                    gps_data = platform.get_gps_data() if hasattr(platform, 'get_gps_data') else {}

                    # Calculate pitch and roll and add it to the IMU data
                    if imu_data and imu_data.get("accel"):
                        orientation = _calculate_pitch_roll_from_accel(imu_data["accel"])
                        imu_data.update(orientation)
                    
                    data_packet = {
                        "imu": imu_data if imu_data else {}, 
                        "gps": gps_data if gps_data else {},
                        "power_on": power_on,
                        "locomotion_enabled": locomotion_enabled,
                        "ai_vision_enabled": ai_vision_enabled,
                        "joint_angles": last_joint_angles,
                        "gait": current_gait
                    }
                    message = json.dumps(data_packet)
                    
                    # Ensure 'dt' is present for the frontend IMU calculation.
                    # If the platform doesn't provide it, calculate it here as a fallback.
                    if data_packet["imu"] and "dt" not in data_packet["imu"]:
                        current_time = time.time()
                        data_packet["imu"]["dt"] = current_time - last_broadcast_time
                        last_broadcast_time = current_time

                    async with self.lock:
                        connections_to_send = self.connections[:]
                    
                    for connection in connections_to_send:
                        await connection.send_text(message)
                else:
                    # If platform is not ready, send a minimal status update
                    message = json.dumps({
                        "power_on": power_on,
                        "locomotion_enabled": locomotion_enabled
                    })
                    await _broadcast_to_all_sensor_clients(message)
                
                await asyncio.sleep(1.0 / 50) # Broadcast at 50Hz
        except asyncio.CancelledError:
            logging.info("Sensor data broadcast task was cancelled.")

camera_streamer = CameraStreamer()
sensor_streamer = SensorDataStreamer()

async def _broadcast_to_all_sensor_clients(message: str):
    """Helper to broadcast a message to all connected sensor clients."""
    async with sensor_streamer.lock:
        connections_to_send = sensor_streamer.connections[:]
    for connection in connections_to_send:
        await connection.send_text(message)

app = FastAPI()

def _calculate_pitch_roll_from_accel(accel_data: Dict[str, float]) -> Dict[str, float]:
    """
    Calculates pitch and roll from accelerometer data.
    This is a simple approach and is susceptible to noise from linear acceleration.
    A proper sensor fusion filter (Kalman, Madgwick) would be more robust.
    """
    if not accel_data or 'x' not in accel_data or 'y' not in accel_data or 'z' not in accel_data:
        return {}
    accel_x, accel_y, accel_z = accel_data['x'], accel_data['y'], accel_data['z']
    pitch = np.arctan2(-accel_x, np.sqrt(accel_y * accel_y + accel_z * accel_z))
    roll = np.arctan2(accel_y, accel_z)
    return {"pitch": pitch, "roll": roll} # in radians

# --- Path Setup for Static Files and Templates ---
# Get the directory where this server.py file is located
server_dir = os.path.dirname(__file__)
static_dir = os.path.join(server_dir, "static")
templates_dir = os.path.join(server_dir, "templates")

# Mount static files
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Templates
templates = Jinja2Templates(directory=templates_dir)

def setup_server(p: HexapodPlatform, l: HexapodLocomotion, mode: str):
    """
    Initializes the web server with the necessary platform and locomotion objects.
    This function is called by the main runner script before starting the server.
    """
    global platform, locomotion, server_mode, object_detector, current_gait
    platform = p
    locomotion = l
    current_gait = l.gait_type
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
    while True: # Loop forever, but only act if powered on
        global last_joint_angles, locomotion_enabled
        if platform and locomotion and power_on:
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
                    step_height=new_step_height
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

async def power_on_sequence():
    """
    Runs a graceful power-on and startup sequence for the robot.
    0. Re-initialize servos if they were de-energized.
    1. Go to home position (all joints at 0 degrees).
    2. Go to a "sitting" position with legs tucked under.
    3. Stand up to the default body height and enable controls.
    """
    if not platform or not locomotion:
        print("Cannot run initialization sequence: platform or locomotion not available.")
        return

    # Wait a moment to ensure the locomotion object has finished its own init,
    # especially the recalculate_stance() which calculates default_joint_angles.
    while not hasattr(locomotion, 'default_joint_angles') or len(locomotion.default_joint_angles) != 6:
        await asyncio.sleep(0.1)


    print("--- Starting Robot Power-On Sequence ---")
    global last_joint_angles, power_on

    # Step 0: Ensure servos are energized.
    # The `set_joint_angles` command will implicitly re-energize them if they were off.
    print("Energizing servos...")

    # Define the sequence steps
    # Each step is (description, target_angles_function, delay_after)
    sequence = [
        (
            "Homing: Moving to zero-angle position...",
            lambda: [[0, 0, 0]] * 6,
            2.0
        ),
        (
            "Sitting: Tucking legs for standing...",
            lambda: locomotion.calculate_sit_angles(),
            2.0
        ),
        (
            "Standing: Moving to default ride height...",
            lambda: locomotion.default_joint_angles,
            1.0
        )
    ]

    for description, get_angles_func, delay in sequence:
        print(description)
        target_angles = get_angles_func()
        platform.set_joint_angles(target_angles)
        # This is a simple way to update the UI during init.
        # We'll broadcast the angles over the sensor websocket.
        last_joint_angles = platform.angles_to_dict(target_angles)
        await _broadcast_to_all_sensor_clients(json.dumps({
            "joint_angles": last_joint_angles
        }))
        await asyncio.sleep(delay)

    globals()['power_on'] = True
    print("--- Power-On Sequence Complete. Robot is active. ---")

async def _smooth_lower_body(duration=2.0, steps=50):
    """
    Helper function to smoothly lower the robot's body to a safe minimum height.
    """
    if not platform or not locomotion:
        return

    print("Lowering body to minimum height...")
    start_height = locomotion.body_height
    # A safe minimum height, e.g., half the femur length.
    # This ensures the legs are still bent and can support the body.
    end_height = locomotion.kinematics.segment_lengths[1] * 0.5 
    delay = duration / steps

    for i in range(steps + 1):
        t = i / steps
        current_height = start_height * (1 - t) + end_height * t
        
        # Use set_body_pose to calculate angles for the new height.
        # We keep translation and rotation at zero.
        locomotion.body_height = current_height
        locomotion.recalculate_stance() # This updates foot positions for the new height
        target_angles = locomotion.set_body_pose(np.zeros(3), np.zeros(3))

        platform.set_joint_angles(target_angles)
        await asyncio.sleep(delay)


async def power_off_sequence():
    """
    Runs a graceful shutdown sequence for the robot.
    1. Go to a "sitting" position.
    2. Go to home position (all joints at 0 degrees).
    3. De-energize servos to release them.
    """
    if not platform or not locomotion:
        return

    print("--- Starting Robot Power-Off Sequence ---")
    global last_joint_angles, power_on, locomotion_enabled

    # Disable locomotion and reset controls first
    power_on = False
    locomotion_enabled = False
    control_values.update({'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'pitch': 0.0, 'roll': 0.0})

    # New Step: Smoothly lower the body before sitting.
    await _smooth_lower_body()

    # Sequence: Sit -> Home -> De-energize
    sequence = [
        ("Sitting down...", lambda: locomotion.calculate_sit_angles(), 2.0),
        ("Homing legs...", lambda: [[0, 0, 0]] * 6, 1.0)
    ]

    for description, get_angles_func, delay in sequence:
        print(description)
        target_angles = get_angles_func()
        platform.set_joint_angles(target_angles)
        last_joint_angles = platform.angles_to_dict(target_angles)
        await _broadcast_to_all_sensor_clients(json.dumps({"joint_angles": last_joint_angles}))
        await asyncio.sleep(delay)

    print("De-energizing servos to release legs...")
    platform.deinit()
    last_joint_angles = platform.angles_to_dict([[0, 0, 0]] * 6)
    await _broadcast_to_all_sensor_clients(json.dumps({"joint_angles": last_joint_angles}))

    print("--- Power-Off Sequence Complete. Robot is safe. ---")

@app.on_event("startup")
async def startup_event():
    """Starts the background control loop when the server starts."""
    # The control loop now starts immediately, but it will be idle
    # until the robot is powered on via the UI.
    # The old initialization_sequence is replaced by the power_on_sequence.
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

@app.post("/power")
async def power_toggle():
    """Toggles the robot's power state and runs the appropriate sequence."""
    if power_on:
        asyncio.create_task(power_off_sequence())
        return {"status": "powering_off"}
    else:
        asyncio.create_task(power_on_sequence())
        return {"status": "powering_on"}

@app.post("/toggle_locomotion")
async def toggle_locomotion():
    """Toggles the locomotion system between enabled and disabled (emergency stop)."""
    global locomotion_enabled
    locomotion_enabled = not locomotion_enabled
    status = "enabled" if locomotion_enabled else "disabled"
    print(f"Locomotion has been {status}.")
    # When disabling, or if robot is not powered on, reset movement values
    if not locomotion_enabled:
        control_values.update({
            'vx': 0.0, 
            'vy': 0.0, 
            'omega': 0.0,
            'pitch': 0.0,
            'roll': 0.0
        })
    return {"status": f"locomotion_{status}"}

@app.post("/set_gait")
async def set_gait(request: Request):
    """Sets the locomotion gait."""
    global current_gait
    data = await request.json()
    gait_name = data.get('gait')
    if locomotion and gait_name in locomotion.available_gaits:
        locomotion.set_gait(gait_name)
        current_gait = gait_name
        print(f"Gait changed to: {gait_name}")
        return {"status": "success", "gait": gait_name}
    return {"status": "error", "message": "Invalid gait name or locomotion not ready."}

@app.post("/reinit")
async def reinit_sequence():
    """Triggers the robot's initialization sequence again."""
    asyncio.create_task(power_on_sequence())
    return {"status": "success", "message": "Initialization sequence triggered."}

@app.post("/toggle_ai_vision")
async def toggle_ai_vision():
    """Toggles the AI vision processing for the front camera."""
    global ai_vision_enabled
    ai_vision_enabled = not ai_vision_enabled
    status = "enabled" if ai_vision_enabled else "disabled"
    return {"status": f"ai_vision_{status}"}

@app.websocket("/ws/sensors")
async def websocket_sensor_feed(websocket: WebSocket):
    """Handles WebSocket connections for the main sensor data stream."""
    await websocket.accept()
    await sensor_streamer.add_client(websocket)
    try:
        while True:
            # Keep the connection alive. The server is pushing data.
            # We can listen for client messages here if needed in the future.
            await websocket.receive_text()
    except WebSocketDisconnect:
        print("Sensor data client disconnected.")
        await sensor_streamer.remove_client(websocket)
        
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