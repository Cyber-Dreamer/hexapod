import pybullet as p
import pybullet_data
import time
import os

class HexapodSimulator:
    def __init__(self, gui=True):
        self.gui = gui
        self.physics_client = None
        self.model_path = os.path.join(os.path.dirname(__file__), 'robot.sdf')
        self.meshes_path = os.path.join(os.path.dirname(__file__), 'meshes')

    def start(self):
        self.physics_client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # Add simulation/meshes to search path for custom meshes
        p.setAdditionalSearchPath(self.meshes_path)
        p.setGravity(0, 0, -9.81)
        plane_id = p.loadURDF("plane.urdf")
        # Load SDF model
        robot_ids = p.loadSDF(self.model_path)
        print(f"Simulation started. Robot IDs: {robot_ids}")
        self.robot_ids = robot_ids

    def step(self):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    def stop(self):
        p.disconnect()
        print("Simulation stopped.")

if __name__ == "__main__":
    sim = HexapodSimulator(gui=True)
    sim.start()
    try:
        for _ in range(2400):  # 10 seconds at 240Hz
            sim.step()
    finally:
        sim.stop()
