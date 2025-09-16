import pybullet as p
import pybullet_data
import time

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and robot
robot_id = p.loadURDF("hexpod robot.urdf", [0, 0, 0.2])

# Set gravity
p.setGravity(0, 0, -9.81)

# Add cameras (using PyBullet's debug visualizer camera)
# Camera 1: Front view
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.2])

# Optionally, you can simulate multiple cameras by capturing images from different viewpoints
def capture_camera(yaw, pitch, dist, target):
    width, height, view_matrix, proj_matrix, _, _, _, _, _, _, rgb, _, _ = p.getCameraImage(
        width=320,
        height=240,
        viewMatrix=p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2),
        projectionMatrix=p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=3.1)
    )
    return rgb

# Main simulation loop
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

    # Example: capture images from two camera angles
    if i % 240 == 0:
        img_front = capture_camera(0, -30, 1.5, [0, 0, 0.2])
        img_side = capture_camera(90, -30, 1.5, [0, 0, 0.2])

p.disconnect()