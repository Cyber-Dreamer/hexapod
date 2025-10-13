from fastapi import FastAPI, Request, Response
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import asyncio
import uvicorn
import os
import sys
import numpy as np
import cv2

# Add the project root to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion

try:
    from gpiozero import Robot
except (ImportError, RuntimeError, ModuleNotFoundError):
    print("Could not import gpiozero. Robot functionality will not be available.")
    class Robot:
        def __init__(self, left, right):
            pass
        def forward(self):
            pass
        def backward(self):
            pass
        def left(self):
            pass
        def right(self):
            pass
        def stop(self):
            pass

app = FastAPI()

# Mount static files
app.mount("/static", StaticFiles(directory="hexapod_py/web/static"), name="static")

# Templates
templates = Jinja2Templates(directory="hexapod_py/web/templates")

# Global variables
hexapod = None
locomotion = None
control_values = {'vx': 0, 'vy': 0, 'omega': 0}

async def simulation_loop():
    while True:
        if isinstance(hexapod, HexapodSimulator) and locomotion is not None:
            joint_angles = locomotion.run_gait(
                vx=control_values['vx'],
                vy=control_values['vy'],
                omega=control_values['omega'],
                pitch=0,
                speed=1
            )
            hexapod.set_joint_angles(joint_angles)
            hexapod.step()
        await asyncio.sleep(1/100)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(simulation_loop())

@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request, "mode": "simulation" if isinstance(hexapod, HexapodSimulator) else "robot"})

@app.get("/move/{direction}")
async def move(direction: str):
    global control_values
    if direction == 'forward':
        control_values = {'vx': 0.1, 'vy': 0, 'omega': 0}
    elif direction == 'backward':
        control_values = {'vx': -0.1, 'vy': 0, 'omega': 0}
    elif direction == 'left':
        control_values = {'vx': 0, 'vy': 0.1, 'omega': 0}
    elif direction == 'right':
        control_values = {'vx': 0, 'vy': -0.1, 'omega': 0}
    elif direction == 'stop':
        control_values = {'vx': 0, 'vy': 0, 'omega': 0}
    else:
        return {"status": "Invalid command"}

    if isinstance(hexapod, Robot):
        if direction == 'forward': hexapod.forward()
        elif direction == 'backward': hexapod.backward()
        elif direction == 'left': hexapod.left()
        elif direction == 'right': hexapod.right()
        elif direction == 'stop': hexapod.stop()

    return {"status": f"Moved {direction}"}

@app.get("/sensor_data")
async def sensor_data():
    # Mock data for now
    return {"gps_data": {"lat": 0, "lon": 0}, "gyro_data": {"x": 0, "y": 0, "z": 0}}

@app.get("/video_feed/{camera_id}")
async def video_feed(camera_id: int):
    if isinstance(hexapod, HexapodSimulator):
        async def generate():
            while True:
                img_arr = hexapod.get_camera_image(camera_id)
                if img_arr is not None:
                    _, jpeg = cv2.imencode('.jpg', img_arr)
                    frame = jpeg.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                await asyncio.sleep(1/30)
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return Response(status_code=404)

async def run_web_server(mode: str = 'robot'):
    global hexapod, locomotion
    if mode == 'robot':
        hexapod = Robot(left=(20, 21), right=(19, 26))
    elif mode == 'simulation':
        hexapod = HexapodSimulator(gui=True)
        hexapod.start()
        locomotion = HexapodLocomotion(gait_type='tripod')
    else:
        raise ValueError(f"Invalid mode: {mode}")

    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)
    await server.serve()