import sys
import os
import time
import numpy as np

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion

def main():
    """
    Main function to run the hexapod gait simulation using the HexapodSimulator class.
    This version runs the simulation without the live kinematic plot for better performance.
    """
    # --- Initialize Simulator and Locomotion ---
    sim = HexapodSimulator(gui=True)
    locomotion = HexapodLocomotion(gait_type='tripod')

    try:
        # --- Start Simulation and Add Controls ---
        sim.start()
        sim.add_ui_controls()

        # --- Simulation Loop ---
        while True:
            # 1. Read values from UI controls
            controls = sim.read_ui_controls()
            vx, vy = controls['vx'], controls['vy']

            # Normalize the direction vector if its magnitude is > 1.0
            dir_vec = np.array([vx, vy])
            if np.linalg.norm(dir_vec) > 1.0:
                dir_vec_normalized = dir_vec / np.linalg.norm(dir_vec)
                vx, vy = dir_vec_normalized

            # Update parameters that might change during simulation
            locomotion.body_height = controls['body_height'] * 1000 # m to mm
            standoff_mm = controls['standoff'] * 1000 # m to mm
            locomotion.recalculate_stance(standoff_distance=standoff_mm) # Recalculate foot positions

            # 2. Run the gait logic to get target joint angles
            all_angles_rad = locomotion.run_gait(
                vx=vx,
                vy=vy,
                omega=controls['omega'],
                roll=controls['roll'],
                pitch=controls['pitch'],
                step_height=controls['step_height'] * 1000 # m to mm
            )

            # 3. Set joint motor targets in PyBullet
            sim.set_joint_angles(all_angles_rad)

            # 4. Step the simulation
            sim.step()
            
            # Pace the simulation to run in real-time
            time.sleep(1./240.)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if sim.physics_client is not None:
            sim.stop()

if __name__ == '__main__':
    main()