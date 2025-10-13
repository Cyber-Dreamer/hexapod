import sys
import os
import time
import numpy as np

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.kinematics.fk import HexapodForwardKinematics
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.simulation.live_plotter import LivePlotter

def main():
    """
    Main function to run the hexapod gait simulation using the HexapodSimulator class.
    """
    # --- Initialize Simulator and Locomotion ---
    sim = HexapodSimulator(gui=True)
    locomotion = HexapodLocomotion(gait_type='tripod')
    
    # Initialize a forward kinematics calculator to get 3D points from angles
    fk_calculator = HexapodForwardKinematics(
        leg_lengths=locomotion.kinematics.segment_lengths,
        hip_positions=locomotion.kinematics.hip_positions
    )

    # Initialize the live 3D plotter
    plotter = LivePlotter(hip_positions=locomotion.kinematics.hip_positions)

    try:
        # --- Start Simulation and Add Controls ---
        sim.start()
        sim.add_ui_controls()

        # --- Timing for Plot Updates ---
        plot_update_interval = 1.0 / 20.0  # Target 20 FPS for the plot
        last_plot_update_time = time.time()

        # --- Simulation Loop ---
        while True:
            # 1. Read values from UI controls
            controls = sim.read_ui_controls()            
            vx, vy = controls['vx'], controls['vy']

            # Normalize the direction vector if its magnitude is > 1.0
            # This is consistent with the gait_test_ui.py logic.
            dir_vec = np.array([vx, vy])
            if np.linalg.norm(dir_vec) > 1.0:
                dir_vec_normalized = dir_vec / np.linalg.norm(dir_vec)
                vx, vy = dir_vec_normalized

            # Update parameters that might change during simulation
            locomotion.body_height = controls['body_height'] * 1000 # m to mm
            standoff_mm = controls['standoff'] * 1000 # m to mm
            # Only recalculate stance if the standoff distance has changed to avoid unnecessary computation
            if locomotion.standoff_distance != standoff_mm:
                locomotion.recalculate_stance(standoff_distance=standoff_mm)
 

            # 3. Run the gait logic to get target joint angles
            # Note: The URDF joint names are different from the locomotion leg order.
            # The HexapodSimulator class handles this mapping.
            all_angles_rad = locomotion.run_gait(
                vx=vx,
                vy=vy,
                omega=controls['omega'],
                roll=controls['roll'],
                pitch=controls['pitch'],
                step_height=controls['step_height'] * 1000 # m to mm
            )

            # Update the 3D plot at a limited frame rate to improve performance.
            current_time = time.time()
            if all_angles_rad is not None:
                if (current_time - last_plot_update_time) > plot_update_interval:
                    # Perform FK and update the plot only when it's time
                    all_leg_points = fk_calculator.body_fk(all_angles_rad)
                    plotter.update(all_leg_points)
                    last_plot_update_time = current_time

            # 4. Set joint motor targets in PyBullet
            sim.set_joint_angles(all_angles_rad)

            # 5. Step the simulation
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